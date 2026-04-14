/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.google.ar.imp.core.editor;

import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;
import androidx.annotation.VisibleForTesting;
import com.google.common.base.Splitter;
import com.google.common.io.ByteStreams;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

/**
 * A thread that runs a simple HTTP server to serve static assets for the Impress remote editor.
 *
 * <p>This server listens on a specified port and handles GET requests. It is designed to serve
 * files such as `index.html`, JavaScript (`.js`), and CSS (`.css`) from the application's assets.
 * It also dynamically generates a `config.json` file containing WebSocket port information for
 * connecting clients.
 *
 * <ul>
 *   <li><b>Starting the Server:</b> The server is started by calling {@link #startServer()}, which
 *       launches this thread.
 *   <li><b>Handling Requests:</b> The {@link #run()} method contains the main loop where the server
 *       socket accepts incoming connections. For each connection, it reads the HTTP GET request and
 *       attempts to serve the requested file or dynamic content.
 *   <li><b>Serving Files:</b> The {@link #sendResponseToClient(String, OutputStream)} method parses
 *       the GET request. It supports serving `index.html` (for requests to "/"), files with `.js`
 *       and `.css` extensions from the app's {@link AssetManager}, and dynamically serves
 *       `config.json` for requests to "/config.json".
 *   <li><b>Error Handling:</b> If a requested file is not found or the request is not a supported
 *       GET request for a known file type, a "404 Not Found" response is sent.
 * </ul>
 */
public final class RemoteEditorHttpServer implements Runnable {
  private static final String TAG = RemoteEditorHttpServer.class.getSimpleName();
  private final int httpPort;
  private final int scriptApiBridgePort;
  private final int uiStreamingPort;
  private volatile ServerSocket serverSocket;
  private volatile boolean running = false;
  private Thread serverThread;
  private final AssetManager assetManager;
  private static final String NOT_FOUND_RESPONSE =
      """
      HTTP/1.1 404 Not Found
      Content-Length: 0

      """;
  private static final String INDEX_HTML_FILENAME = "index.html";
  private static final float BYTES_PER_KB = 1024.0f;
  private String indexFilename = INDEX_HTML_FILENAME;

  RemoteEditorHttpServer(
      Context context, int httpPort, int scriptApiBridgePort, int uiStreamingPort) {
    this.assetManager = context.getAssets();
    this.httpPort = httpPort;
    this.scriptApiBridgePort = scriptApiBridgePort;
    this.uiStreamingPort = uiStreamingPort;
  }

  // Sets the filename of the custom index file to serve.
  void setIndexFilename(String filename) {
    indexFilename = filename;
  }

  // Attempts to start this thread which runs an HTTP server listening on the specified port.
  void startServer() {
    if (running) {
      return;
    }
    running = true;
    serverThread = new Thread(this);
    serverThread.start();
  }

  /** Returns the port this server is listening on. */
  int getPort() {
    ServerSocket socket = serverSocket;
    return (socket == null) ? httpPort : socket.getLocalPort();
  }

  // Returns the thread running the server. For testing only.
  
  Thread getServerThread() {
    return serverThread;
  }

  // Stops this thread and its HTTP server.
  void stopServer() {
    running = false;

    if (serverSocket != null) {
      try {
        serverSocket.close();
        Log.i(TAG, "HTTP server stopped on [port = " + httpPort + "]");
      } catch (IOException e) {
        Log.e(TAG, "Error closing server socket: " + e.getMessage());
      } finally {
        serverSocket = null;
      }
    }
  }

  // The tick for this thread when isAlive(). If the thread is running, it listens on the specified
  // port for requests.
  @Override
  public void run() {
    try {
      ServerSocket serverSocket = new ServerSocket(httpPort);
      this.serverSocket = serverSocket;
      Log.i(TAG, "HTTP server started on [port = " + httpPort + "]");

      while (running) {
        // This is a blocking call, waits for a client connection.
        try (Socket clientSocket = serverSocket.accept();
            BufferedReader in =
                new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
            OutputStream out = clientSocket.getOutputStream()) {

          String request = in.readLine();
          sendResponseToClient(request, out);
        } catch (Exception e) {
          if (running) {
            Log.e(TAG, "Error awaiting connection - " + e.getMessage());
          }

          // If this is an InterruptedException, preserve the interrupt status.
          if (e instanceof InterruptedException) {
            Thread.currentThread().interrupt();
          }
          // The exception is expected when serverSocket.close() is called,
          // so we break the loop if the server is no longer running.
          break;
        }
      }
    } catch (IOException | SecurityException e) {
      Log.e(TAG, "Error encountered while running HTTP server: " + e.getMessage());
    } finally {
      stopServer();
    }
  }

  private void sendResponseToClient(String request, OutputStream out) {
    if (request == null) {
      return;
    }

    List<String> requestParts = Splitter.on(' ').splitToList(request);
    // Check for a known GET request structure: "GET /path/to/resource HTTP/1.1"
    if ((requestParts.size() < 2) || !requestParts.get(0).equals("GET")) {
      send404(out, request);
      return;
    }

    String resourcePath = null;
    try {
      // Using URI to parse the resource path ensures proper handling of encodings and structure.
      URI uri = new URI(requestParts.get(1));
      resourcePath = uri.getPath();
    } catch (URISyntaxException e) {
      Log.e(TAG, "Invalid URI in request: " + requestParts.get(1) + " - " + e.getMessage());
      send404(out, request);
      return;
    }

    String filename = null;
    if (resourcePath != null) {
      // Determine the filename to serve based on the resource path.
      if (resourcePath.equals("/")) {
        filename = indexFilename;
      } else if (resourcePath.equals("/config.json")) {
        String json =
            "{\"scriptApiBridgePort\": "
                + scriptApiBridgePort
                + ", \"uiStreamingPort\": "
                + uiStreamingPort
                + "}";
        byte[] bytes = json.getBytes();
        try {
          out.write("HTTP/1.1 200 OK\r\n".getBytes());
          out.write("Content-Type: application/json\r\n".getBytes());
          out.write(("Content-Length: " + bytes.length + "\r\n").getBytes());
          out.write("\r\n".getBytes());
          out.write(bytes);
          out.flush();
          Log.i(TAG, "Served dynamic config.json");
        } catch (IOException e) {
          Log.e(TAG, "Error serving config.json: " + e.getMessage());
        }
        return;
      } else if (resourcePath.endsWith(".js") || resourcePath.endsWith(".css")) {
        // Remove the leading slash to get the filename relative to the assets directory.
        filename = resourcePath.substring(1);
      }
    }

    if (filename == null) {
      send404(out, request);
      return;
    }

    // Security Note: assetManager.open() is used here to access files. The Android
    // AssetManager only allows access to files bundled within the application's APK
    // in the 'assets/' directory. This prevents path traversal vulnerabilities, as
    // requests cannot access arbitrary files on the device's file system (e.g., via "../").
    InputStream fileStream;
    try {
      fileStream = assetManager.open(filename);
    } catch (IOException e) {
      Log.e(TAG, "Asset not found: " + filename + ". Sending 404.");
      send404(out, request);
      return;
    }

    try (fileStream) {
      byte[] bytes = ByteStreams.toByteArray(fileStream);

      out.write("HTTP/1.1 200 OK\r\n".getBytes());
      if (filename.endsWith(".html")) {
        out.write("Content-Type: text/html\r\n".getBytes());
      } else if (filename.endsWith(".js")) {
        out.write("Content-Type: text/javascript\r\n".getBytes());
      } else if (filename.endsWith(".css")) {
        out.write("Content-Type: text/css\r\n".getBytes());
      }
      out.write(("Content-Length: " + bytes.length + "\r\n").getBytes());
      out.write("\r\n".getBytes()); // Blank line separates headers from body.
      out.write(bytes);
      out.flush();

      Log.i(
          TAG,
          "Received GET request and sending [file = "
              + filename
              + "] [size = "
              + String.format("%.2f", (float) bytes.length / BYTES_PER_KB)
              + " KB]");
    } catch (IOException e) {
      Log.e(TAG, "Error serving file " + filename + ": " + e.getMessage());
    }
  }

  private void send404(OutputStream out, String request) {
    try {
      out.write(NOT_FOUND_RESPONSE.getBytes());
      out.flush();
      Log.i(TAG, "Received unsupported request. Sent 404 [request = " + request + "]");
    } catch (IOException e) {
      Log.e(TAG, "Error sending 404 response - " + e.getMessage());
    }
  }
}
