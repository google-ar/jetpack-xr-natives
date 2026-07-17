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
import android.util.Log;
import com.google.android.filament.proguard.UsedByNative;
import java.io.IOException;

// LINT.IfChange(remote_editor)

/**
 * Manages the services for the Impress Remote Editor.
 *
 * <p>This class is responsible for orchestrating the lifecycle of the remote editor's components:
 *
 * <ul>
 *   <li><b>HTTP Server:</b> Serves the static web assets (HTML, JavaScript, CSS) that comprise the
 *       remote editor UI, including a dynamic `config.json` that provides WebSocket port
 *       information to the client.
 *   <li><b>Script API Bridge:</b> Facilitates bidirectional communication between the web UI's
 *       JavaScript API and the native C++ engine via a WebSocket connection. Incoming
 *       binary-encoded {@code MessageToNative} protocol buffers are received and routed to the
 *       native engine via JNI. Outgoing {@code MessageToScript} protocol buffers are serialized by
 *       native code and sent back to the browser.
 *   <li><b>UI Video Streamer:</b> (Optional) Captures and streams the Impress editor UI in
 *       real-time to the web UI via a WebSocket connection, enabling a fully interactive remote
 *       editing experience.
 * </ul>
 *
 * <p>It is instantiated and controlled from native code via JNI.
 */
@UsedByNative("android_remote_editor_wrapper.cc")
public final class RemoteEditor {
  private static final String TAG = RemoteEditor.class.getSimpleName();

  private RemoteEditorHttpServer httpServer;
  private final RemoteEditorScriptApiBridge scriptApiBridge;
  private final RemoteEditorVideoStreamer videoStreamer;

  private final Context context;

  /**
   * Constructs a new RemoteEditor.
   *
   * @param context The application context.
   * @param viewHandle The native handle to the Impress View.
   * @param executorHandle The native handle to the Executor.
   */
  @UsedByNative("android_remote_editor_wrapper.cc")
  public RemoteEditor(Context context, long viewHandle, long executorHandle) {
    this.context = context;
    scriptApiBridge = new RemoteEditorScriptApiBridge(viewHandle, executorHandle);
    videoStreamer = new RemoteEditorVideoStreamer();
  }

  /**
   * Starts the remote editor services.
   *
   * @param nativeRemoteEditorWrapperPtr The pointer to the native RemoteEditor C++ wrapper.
   * @param nativeScriptApiBridgeWrapperPtr The pointer to the native RemoteEditorScriptApiBridge
   *     C++ wrapper.
   * @param httpPort The port for the HTTP server.
   * @param scriptApiBridgePort The port for the Script API bridge WebSocket server.
   * @param uiStreamingPort The port for the UI Streaming WebSocket server.
   */
  @UsedByNative("android_remote_editor_wrapper.cc")
  public void start(
      long nativeRemoteEditorWrapperPtr,
      long nativeScriptApiBridgeWrapperPtr,
      int httpPort,
      int scriptApiBridgePort,
      int uiStreamingPort) {
    // Defensive call: ensure any previously running services are stopped and their ports freed.
    stop();

    scriptApiBridge.startBridge(scriptApiBridgePort, nativeScriptApiBridgeWrapperPtr);

    httpServer =
        new RemoteEditorHttpServer(context, httpPort, scriptApiBridgePort, uiStreamingPort);
    httpServer.startServer();

    if (uiStreamingPort != 0) {
      httpServer.setIndexFilename("ui_stream/index.html");
      try {
        videoStreamer.startStreaming(uiStreamingPort, nativeRemoteEditorWrapperPtr);
      } catch (IOException | RuntimeException e) {
        Log.e(TAG, "Failed to initialize media codec", e);
      }
    }
  }

  /** Stops the remote editor services. */
  @UsedByNative("android_remote_editor_wrapper.cc")
  public void stop() {
    scriptApiBridge.stopBridge();
    if (httpServer != null) {
      httpServer.stopServer();
      httpServer = null;
    }
    videoStreamer.stopStreaming();
  }

  /** Final release of all resources. Cannot be restarted after this. */
  @UsedByNative("android_remote_editor_wrapper.cc")
  public void release() {
    stop();
    if (videoStreamer != null) {
      videoStreamer.release();
    }
  }

  @UsedByNative("android_remote_editor_wrapper.cc")
  public RemoteEditorScriptApiBridge getScriptApiBridge() {
    return scriptApiBridge;
  }
}

// LINT.ThenChange(
//
// //depot/google3/third_party/impress/core/editor/remote_editor/android_remote_editor_wrapper.cc:remote_editor)
