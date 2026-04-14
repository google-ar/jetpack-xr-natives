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

import android.util.Log;
import androidx.annotation.Nullable;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

/**
 * A generic WebSocket server used by the Impress remote editor.
 *
 * <p>This server provides a communication channel for remote clients (e.g., a web-based UI). It
 * notifies registered {@link WebSocketListener}s of incoming messages and connection events,
 * allowing components to handle the application logic.
 *
 * <ul>
 *   <li><b>Incoming Messages:</b> When a message is received from a connected client, it is
 *       dispatched to all registered {@link WebSocketListener}s.
 *   <li><b>Sending Responses:</b> Messages can be sent back to connected clients using the {@link
 *       #send(String)} or {@link #send(ByteBuffer)} methods.
 * </ul>
 */
public class RemoteEditorWebSocketServer extends WebSocketServer {

  private static final String TAG = RemoteEditorWebSocketServer.class.getSimpleName();
  private final List<WebSocketListener> listeners = new CopyOnWriteArrayList<>();

  /** Interface for listening to WebSocket events. */
  public interface WebSocketListener {
    /** Called when a new client connects to the WebSocket server. */
    void onConnected(WebSocket conn);

    /** Called when the currently connected client disconnects. */
    void onDisconnected();

    /**
     * Called when a string message is received from the client.
     *
     * @param message The string message received from the client.
     */
    void onStringMessage(String message);

    /**
     * Called when a binary message is received from the client.
     *
     * @param blob The binary message data received from the client.
     */
    void onBinaryMessage(ByteBuffer blob);
  }

  public RemoteEditorWebSocketServer(int port) {
    super(new InetSocketAddress(port));
    setReuseAddr(true);
  }

  public void addListener(WebSocketListener listener) {
    if (!listeners.contains(listener)) {
      listeners.add(listener);
    }
  }

  public void removeListener(WebSocketListener listener) {
    listeners.remove(listener);
  }

  public void startServer() {
    start();
  }

  @Override
  public void onOpen(WebSocket conn, ClientHandshake handshake) {
    Log.i(TAG, "New connection from [address = " + conn.getRemoteSocketAddress() + "]");
    for (WebSocketListener listener : listeners) {
      listener.onConnected(conn);
    }
  }

  @Override
  public void onClose(WebSocket conn, int code, String reason, boolean remote) {
    Log.i(
        TAG,
        "Connection closed from "
            + conn.getRemoteSocketAddress()
            + " with exit code "
            + code
            + " reason: "
            + reason);
    for (WebSocketListener listener : listeners) {
      listener.onDisconnected();
    }
  }

  @Override
  public void onMessage(WebSocket conn, ByteBuffer blob) {
    for (WebSocketListener listener : listeners) {
      blob.rewind(); // Reset position before each listener
      listener.onBinaryMessage(blob);
    }
  }

  @Override
  public void onMessage(WebSocket conn, String message) {
    for (WebSocketListener listener : listeners) {
      listener.onStringMessage(message);
    }
  }

  @Override
  public void onError(@Nullable WebSocket conn, Exception ex) {
    if (conn != null) {
      Log.e(TAG, "An error occurred on connection " + conn.getRemoteSocketAddress() + ":" + ex);
    } else {
      Log.e(TAG, "An error occurred: " + ex);
    }
  }

  @Override
  public void onStart() {
    Log.i(TAG, "WebSocket server started on [port = " + getPort() + "]");
  }

  public void send(String message) {
    broadcast(message);
  }

  public void send(ByteBuffer blob) {
    broadcast(blob);
  }

  public void send(byte[] bytes) {
    broadcast(bytes);
  }
}
