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
import androidx.annotation.VisibleForTesting;
import com.google.android.filament.proguard.UsedByNative;
import java.io.IOException;
import java.nio.ByteBuffer;
import org.java_websocket.WebSocket;

/**
 * Bridge between the Web UI Script API and the native Impress engine.
 *
 * <p>This bridge facilitates the exchange of Script API messages between the browser-based UI and
 * the native Impress engine. It acts as a {@link RemoteEditorWebSocketServer.WebSocketListener} to
 * receive incoming messages from the WebSocket and routes them to native code via JNI.
 *
 * <p><b>Message Flow:</b>
 *
 * <ul>
 *   <li><b>Web to Native:</b> The browser sends binary-encoded {@code MessageToNative} protocol
 *       buffers over the WebSocket. This bridge receives them in {@link #onBinaryMessage}, extracts
 *       the byte array, and posts it to the native engine via {@code nativePostMessageToNative}.
 *   <li><b>Native to Web:</b> The native engine processes the request and serializes a {@code
 *       MessageToScript} response or event into a Base64-encoded string. This string is passed back
 *       to Java via {@link #postMessageToScript}, which then sends it to the browser as a text
 *       message over the WebSocket.
 * </ul>
 */
@UsedByNative("android_remote_editor_script_api_bridge_wrapper.cc")
public class RemoteEditorScriptApiBridge implements RemoteEditorWebSocketServer.WebSocketListener {

  private static final String TAG = RemoteEditorScriptApiBridge.class.getSimpleName();
  private final long viewHandle;
  private final long executorHandle;
  private long nativeScriptApiBridgeWrapperPtr;
  private RemoteEditorWebSocketServer webSocketServer;
  private NativeInterface nativeInterface =
      new NativeInterface() {
        @Override
        public void postMessageToNative(
            long nativeScriptApiBridgeWrapperPtr,
            long viewHandle,
            long executorHandle,
            byte[] requestBytes) {
          if (nativeScriptApiBridgeWrapperPtr == 0) {
            return;
          }
          nativePostMessageToNative(
              nativeScriptApiBridgeWrapperPtr, viewHandle, executorHandle, requestBytes);
        }
      };

  /**
   * Constructs a new RemoteEditorScriptApiBridge.
   *
   * @param viewHandle The handle to the native View.
   * @param executorHandle The handle to the native Executor.
   */
  public RemoteEditorScriptApiBridge(long viewHandle, long executorHandle) {
    this.viewHandle = viewHandle;
    this.executorHandle = executorHandle;
  }

  /**
   * Starts the Script API bridge and its underlying WebSocket server.
   *
   * @param webSocketPort The port to listen on.
   * @param nativeScriptApiBridgeWrapperPtr The pointer to the native RemoteEditorScriptApiBridge
   *     wrapper.
   */
  public void startBridge(int webSocketPort, long nativeScriptApiBridgeWrapperPtr) {
    stopBridge();
    this.nativeScriptApiBridgeWrapperPtr = nativeScriptApiBridgeWrapperPtr;
    this.webSocketServer = new RemoteEditorWebSocketServer(webSocketPort);
    this.webSocketServer.addListener(this);
    this.webSocketServer.startServer();
  }

  /** Stops the bridge and shuts down the underlying WebSocket server. */
  public void stopBridge() {
    if (webSocketServer != null) {
      webSocketServer.removeListener(this);
      try {
        webSocketServer.stop();
      } catch (IOException | InterruptedException e) {
        if (e instanceof InterruptedException) {
          Thread.currentThread().interrupt();
        }
        Log.e(TAG, "Error stopping Script API WebSocket server", e);
      }
      webSocketServer = null;
    }
  }

  @Override
  public void onConnected(WebSocket conn) {}

  @Override
  public void onDisconnected() {}

  @Override
  public void onStringMessage(String message) {}

  /**
   * Forwards binary messages from the Web UI to the native engine.
   *
   * <p>Receives binary-encoded {@code MessageToNative} protobufs from the WebSocket client and
   * posts them to the native engine via {@code nativePostMessageToNative}.
   *
   * @param blob The {@link ByteBuffer} containing the binary message data.
   */
  @Override
  public void onBinaryMessage(ByteBuffer blob) {
    byte[] messageByteArray = new byte[blob.remaining()];
    blob.get(messageByteArray);
    nativeInterface.postMessageToNative(
        nativeScriptApiBridgeWrapperPtr, viewHandle, executorHandle, messageByteArray);
  }

  
  RemoteEditorWebSocketServer getWebSocketServer() {
    return webSocketServer;
  }

  
  void setWebSocketServer(RemoteEditorWebSocketServer webSocketServer) {
    this.webSocketServer = webSocketServer;
  }

  
  void setNativeInterface(NativeInterface nativeInterface) {
    this.nativeInterface = nativeInterface;
  }

  
  interface NativeInterface {
    void postMessageToNative(
        long nativeScriptApiBridgeWrapperPtr,
        long viewHandle,
        long executorHandle,
        byte[] requestBytes);
  }

  // LINT.IfChange(postMessageToScript)
  /**
   * Sends a message from the native engine to the connected Web UI script API.
   *
   * <p>Called from native C++ code via JNI, this method forwards a message (typically a
   * Base64-encoded {@code MessageToScript} protobuf) to the browser client as a WebSocket text
   * message.
   *
   * @param response The message from native to send to the script API.
   */
  @UsedByNative("android_remote_editor_script_api_bridge_wrapper.cc")
  public void postMessageToScript(String response) {
    if (webSocketServer != null) {
      webSocketServer.send(response);
    }
  }

  // LINT.ThenChange(//depot/google3/third_party/impress/core/editor/remote_editor/android_remote_editor_script_api_bridge_wrapper.cc:postMessageToScript)

  // LINT.IfChange(nativePostMessageToNative)
  /**
   * JNI method to post a binary message to the native RemoteEditorScriptApiBridge.
   *
   * @param nativeScriptApiBridgeWrapperPtr The pointer to the native RemoteEditorScriptApiBridge
   *     wrapper.
   * @param viewHandle The handle to the native View.
   * @param executorHandle The handle to the native Executor.
   * @param requestBytes The binary-encoded {@code MessageToNative} protobuf.
   */
  private native void nativePostMessageToNative(
      long nativeScriptApiBridgeWrapperPtr,
      long viewHandle,
      long executorHandle,
      byte[] requestBytes);
  // LINT.ThenChange(//depot/google3/third_party/impress/core/editor/remote_editor/remote_editor_script_api_bridge_jni.cc:nativePostMessageToNative)
}
