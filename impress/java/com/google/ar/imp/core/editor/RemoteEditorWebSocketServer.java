/*
 * Copyright 2025 Google LLC
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
import com.google.android.filament.proguard.UsedByNative;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

/**
 * A WebSocket server used for the Impress remote editor.
 *
 * <p>This server facilitates communication between a remote client (e.g., a web-based UI) and the
 * native Impress engine. It extends {@link WebSocketServer} to handle WebSocket connections.
 *
 * <ul>
 *   <li><b>Incoming Messages:</b> When a binary message is received from a connected client via
 *       {@link #onMessage(WebSocket, ByteBuffer)}, the data is forwarded to the native layer (C++)
 *       using the {@code nPostMessage} native method. This allows the Impress engine to process
 *       requests from the remote editor.
 *   <li><b>Sending Responses:</b> The native layer can send responses back to the client by calling
 *       the {@link #postMessageToScript(String)} method. This method sends the provided string
 *       message over the WebSocket connection to the currently connected client.
 * </ul>
 */
@UsedByNative("remote_editor_server.cc")
public class RemoteEditorWebSocketServer extends WebSocketServer {

  private static final String TAG = RemoteEditorWebSocketServer.class.getSimpleName();
  private final long viewHandle;
  private final long executorHandle;
  private WebSocket connection;

  @UsedByNative("remote_editor_server.cc")
  public RemoteEditorWebSocketServer(long viewHandle, long executorHandle, int port) {
    super(new InetSocketAddress(port));
    Log.i(TAG, "RemoteEditorWebSocketServer()");

    this.viewHandle = viewHandle;
    this.executorHandle = executorHandle;
  }

  @UsedByNative("remote_editor_server.cc")
  public void startServer() {
    Log.i(TAG, "startServer");
    setReuseAddr(true);
    start();
  }

  @Override
  public void onOpen(WebSocket conn, ClientHandshake handshake) {
    Log.i(TAG, "New connection from " + conn.getRemoteSocketAddress());
    connection = conn;
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
    connection = null;
  }

  @Override
  public void onMessage(WebSocket conn, ByteBuffer blob) {
    Log.i(TAG, "onMessage - message received from client.");
    if (connection == null) {
      Log.e(TAG, "onMessage - connection is null! Cannot deliver message.");
      return;
    }
    byte[] requestByteArray = new byte[blob.remaining()];
    blob.get(requestByteArray);
    nPostMessageToNative(this, viewHandle, executorHandle, requestByteArray);
  }

  @Override
  public void onMessage(WebSocket conn, String message) {
    Log.e(TAG, "Wrong onMessage method called! Expected ByteBuffer, got String.");
  }

  @Override
  public void onError(WebSocket conn, Exception ex) {
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

  // LINT.IfChange(remote_editor_websocket_server)

  /** Posts a MessageToScript from native to the Java scripting interface. */
  @UsedByNative("remote_editor_jni.cc")
  public void postMessageToScript(String response) {
    if (connection == null) {
      Log.e(TAG, "postMessageFromNative - connection is null! Cannot deliver response.");
      return;
    }
    connection.send(response);
  }

  protected static native void nPostMessageToNative(
      Object self, long viewHandle, long executorHandle, byte[] requestBytes);

  // LINT.ThenChange(
  //
  // //depot/google3/third_party/impress/core/editor/remote_editor/remote_editor_jni.cc:remote_editor_websocket_server
  // )
}
