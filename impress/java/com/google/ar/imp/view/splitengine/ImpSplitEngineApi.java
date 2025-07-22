/*
 * Copyright 2024 Google LLC
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

package com.google.ar.imp.view.splitengine;

import android.content.Context;
import android.os.IBinder;
import android.os.StrictMode;
import android.os.StrictMode.ThreadPolicy;
import android.util.Log;
import androidx.annotation.Nullable;
import com.android.extensions.xr.XrExtensions;
import com.google.ar.imp.view.View;
import com.google.imp.splitengine.extensions.IRendererConnection;
import java.util.concurrent.Executor;

/** Provides a JNI API for running Impress in Split Engine mode. */
public final class ImpSplitEngineApi {
  private static final String TAG = ImpSplitEngineApi.class.getSimpleName();
  // Default library to load if none is provided. Matches the name from imp.bzl
  // returned by imp_default_jni_binary_name()
  // LINT.IfChange
  private static final String DEFAULT_LIBRARY_NAME = "imp_view_split_engine_jni";
  private static final int DEFAULT_BRIDGE_BUFFER_SIZE_KB = 10000;

  // LINT.ThenChange(//depot/google3/third_party/impress/build_tools/imp.bzl)

  @SuppressWarnings("NonFinalStaticField")
  private static volatile boolean libraryLoaded = false;

  private final Context context;
  private IRendererConnection mConnection;
  private final int mBridgeBufferSizeKb;
  private View mView;
  private volatile Boolean isViewSetup = false;
  private volatile Boolean pendingResume = false;

  /** Initialize Impress for Split Engine */
  static ImpSplitEngineApi create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable ImpSplitEngine.ScreenSize screenSize,
      Executor frameSchedulerExecutor,
      IBinder serviceBinder,
      @Nullable XrExtensions xrExtensions) {
    // If no native library name has been specified, use the basic one.
    @Nullable String nativeLibrary = null;
    int bridgeBufferSizeKb = 0;
    @Nullable String viewIdentifier = null;
    if (setupParams != null) {
      nativeLibrary = setupParams.jniLibraryName;
      bridgeBufferSizeKb = setupParams.bridgeBufferSizeKb;
      viewIdentifier = setupParams.viewIdentifier;
    }
    if (nativeLibrary == null || nativeLibrary.isEmpty()) {
      nativeLibrary = DEFAULT_LIBRARY_NAME;
    }
    if (bridgeBufferSizeKb == 0) {
      bridgeBufferSizeKb = DEFAULT_BRIDGE_BUFFER_SIZE_KB;
    }

    // Temporarily allow disk reads to load the library in createView and initBridge.
    ThreadPolicy oldPolicy = StrictMode.getThreadPolicy();
    try {
      StrictMode.setThreadPolicy(new ThreadPolicy.Builder(oldPolicy).permitDiskReads().build());

      // Create the standard Impress View Jni object.
      View view = View.createView(nativeLibrary, viewIdentifier, context);
      // Create the bridge service.
      ImpSplitEngineApi api = new ImpSplitEngineApi(context, view, bridgeBufferSizeKb);
      api.initBridge(nativeLibrary, frameSchedulerExecutor, serviceBinder, xrExtensions);

      if (screenSize != null) {
        view.resize(screenSize.getWidthPixels(), screenSize.getHeightPixels(), 1.0f, 1.0f);
      }
      return api;
    } finally {
      StrictMode.setThreadPolicy(oldPolicy);
    }
  }

  private ImpSplitEngineApi(Context context, View view, int bridgeBufferSizeKb) {
    this.context = context;
    this.mView = view;
    this.mBridgeBufferSizeKb = bridgeBufferSizeKb;
  }

  public void onResume() {
    if (!isViewSetup) {
      pendingResume = true;
      return;
    }
    this.mView.onResume();
  }

  public void onPause() {
    if (!isViewSetup) {
      pendingResume = false;
      return;
    }
    this.mView.onPause();
  }

  /** Returns the Impress view associated with this Split Engine API. */
  public View getView() {
    return this.mView;
  }

  /** Returns the renderer connection associated with this Split Engine API. */
  public IRendererConnection getRendererConnection() {
    return this.mConnection;
  }

  /** Initializes the Split Engine bridge service. */
  private void initBridge(
      String nativeLibrary,
      Executor frameSchedulerExecutor,
      IBinder serviceBinder,
      XrExtensions xrExtensions) {
    Log.d(TAG, "Initialize the SplitEngineSharedMemoryBridgeService.");

    loadLibrary(nativeLibrary);

    Log.i(TAG, "Initializing bridge service provider.");
    RendererConnectionServiceProvider bridgeServiceProvider =
        new RendererConnectionServiceProvider(frameSchedulerExecutor, xrExtensions);
    bridgeServiceProvider.initializeService(context, serviceBinder);

    // This will immediately happen via directExecutor on XROS, but will be delayed on the
    // phone.
    bridgeServiceProvider.onBridgeReady(
        (bridge) -> {
          mConnection = bridge;
          nSetup(this.mView.getViewHostHandle(), mConnection, 1024 * mBridgeBufferSizeKb);
          isViewSetup = true;
        });
  }

  /** Renders the next frame. Returns -1 if successful, 0 if skipped. */
  public long renderNextFrame(
      long lastFrameTimeNanos, long frameTimeNanos, long cameraUpdateParamsHandle) {
    // This check is only necessary for phone. On XROS, the view is setup immediately.
    if (!isViewSetup) {
      return 0;
    }
    // Handle any pending lifecycle state changes.
    if (pendingResume) {
      pendingResume = false;
      this.mView.onResume();
    }
    return nRenderNextFrame(
        this.mView.getViewHostHandle(),
        lastFrameTimeNanos,
        frameTimeNanos,
        cameraUpdateParamsHandle);
  }

  /** Destroys the native Impress view. */
  public void destroy() {
    if (isViewSetup) {
      this.mView.destroy();
      isViewSetup = false;
    }
  }

  private synchronized void loadLibrary(String nativeLibraryName) {
    if (libraryLoaded) {
      return;
    }
    Log.i(TAG, "Loading native library: " + nativeLibraryName);
    try {
      System.loadLibrary(nativeLibraryName);
    } catch (UnsatisfiedLinkError e) {
      Log.e(TAG, "Unable to load " + nativeLibraryName);
      return;
    }
    libraryLoaded = true;
  }

  // LINT.IfChange(api)
  private static native void nSetup(
      long viewHandle, IRendererConnection bridge, int bridgeBufferSizeBytes);

  private static native long nRenderNextFrame(
      long viewHandle, long lastFrameTimeNanos, long frameTimeNanos, long cameraUpdateParamsHandle);
  // LINT.ThenChange(//depot/google3/third_party/impress/core/split_engine/android/view/split_engine_jni.cc:api)
}
