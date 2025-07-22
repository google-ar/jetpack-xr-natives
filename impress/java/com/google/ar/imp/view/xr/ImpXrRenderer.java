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

package com.google.ar.imp.view.xr;

import android.content.Context;
import android.util.Log;
import androidx.annotation.Nullable;
import com.google.ar.imp.view.ContinuousFrameScheduler;
import com.google.ar.imp.view.FrameScheduler;
import com.google.ar.imp.view.ImpApi;
import com.google.ar.imp.view.SetupParams;
import com.google.ar.imp.view.View;
import com.google.common.util.concurrent.FutureCallback;
import com.google.common.util.concurrent.Futures;
import com.google.common.util.concurrent.ListenableFuture;
import java.util.concurrent.ExecutionException;

/**
 * Impress renderer for OpenXR.
 *
 * <p>Used to create a native Impress View, initialize OpenXR, and drive the frame loop for OpenXR
 * builds of Impress.
 */
// TODO: Add listener for getting callbacks between frames to ImpXrRenderer.
public final class ImpXrRenderer {
  private static final String TAG = ImpXrRenderer.class.getSimpleName();

  /** Interface for callbacks that occur as part of the frame loop of the renderer. */
  public interface FrameListener {
    /** Called after each attempted frame. */
    // TODO: Add parameter to indicate if the frame succeeded or not.
    void onPostFrame();
  }

  /** Logs an error message on failure. */
  private static class LogOnErrorCallback implements FutureCallback<Void> {
    public LogOnErrorCallback(String message) {
      this.message = message;
    }

    @Override
    public void onSuccess(Void result) {}

    @Override
    public void onFailure(Throwable t) {
      Log.w(TAG, message, t);
    }

    private final String message;
  }

  private final ImpXrApi xrApi;
  private final FrameScheduler frameScheduler;

  @Nullable private FrameListener frameListener = null;

  /** Initialize Impress for OpenXR. */
  public static ImpXrRenderer create(Context context, @Nullable SetupParams setupParams) {
    if (setupParams == null) {
      setupParams = SetupParams.getDefaultInstance();
    }

    // Use a ContinuousFrameScheduler to drive the OpenXR Frame Loop.
    // For each run of the loop, xrWaitFrame is called which blocks until the OpenXR frame is ready.
    // After the frame, the frame scheduler will immediately schedule another frame using an Android
    // looper.
    //
    // This gives an opportunity for java code in the activity to run instead of running the frame
    // loop in a tight loop on the C++ side.
    //
    // Supports both ThreadMode.MAIN_DEFAULT and ThreadMode.BACKGROUND
    FrameScheduler frameScheduler =
        new ContinuousFrameScheduler.Factory()
            .create(
                ImpApi.getThreadMode(setupParams.getThreadMode()),
                setupParams.getBackgroundThreadName());

    // ImpXrApi creates the View internally, and also provides Xr specific Jni calls.
    ImpXrApi xrApi;
    final SetupParams finalParams = setupParams;
    ListenableFuture<ImpXrApi> apiFuture =
        frameScheduler.submitOnFrameThread(() -> ImpXrApi.create(context, finalParams));

    try {
      // Block until ImpXrApi has been created.
      xrApi = apiFuture.get();
    } catch (InterruptedException | ExecutionException e) {
      Log.e(TAG, "Failed to create ImpXrApi. Exception: ", e);
      throw new IllegalStateException("Unable to initialize Impress API", e);
    }
    return new ImpXrRenderer(xrApi, frameScheduler);
  }

  private ImpXrRenderer(ImpXrApi xrApi, FrameScheduler frameScheduler) {
    this.xrApi = xrApi;
    this.frameScheduler = frameScheduler;
  }

  /**
   * Returns the Impress view associated with the renderer. Used to initialize the Impress scripting
   * system or make custom direct Jni calls.
   */
  public View getView() {
    return xrApi.getView();
  }

  /**
   * Returns the FrameScheduler. Allows callers to get theframe thread's looper and to submit work.
   */
  public FrameScheduler getFrameScheduler() {
    return frameScheduler;
  }

  /** Initializes OpenXR, creates the swap chain, and resizes the Impress view. */
  public void onWindowAttached() {
    Futures.addCallback(
        frameScheduler.submitOnFrameThread(
            () -> {
              xrApi.onWindowAttached();
              return null;
            }),
        new LogOnErrorCallback("Failed to attach ImpXrApi."),
        frameScheduler.getExecutor());
  }

  public void startFrameLoop() {
    ListenableFuture<Void> future =
        frameScheduler.submitOnFrameThread(
            () -> {
              xrApi.show();
              return null;
            });

    try {
      // Block until ImpXrApi has been shown.
      future.get();
    } catch (InterruptedException | ExecutionException e) {
      throw new IllegalStateException("Unable to show ImpXrApi", e);
    }
    frameScheduler.startFrameLoop(this::advanceFrame);
  }

  public void stopFrameLoop() {
    Futures.addCallback(
        frameScheduler.submitOnFrameThread(
            () -> {
              xrApi.hide();
              return null;
            }),
        new LogOnErrorCallback("Failed to hide ImpXrApi."),
        frameScheduler.getExecutor());
    frameScheduler.stopFrameLoop();
  }

  public void destroy() {
    ListenableFuture<Void> future =
        frameScheduler.submitOnFrameThread(
            () -> {
              xrApi.destroy();
              return null;
            });

    try {
      // Block until destroy has returned.
      future.get();
    } catch (InterruptedException | ExecutionException e) {
      throw new IllegalStateException("Unable to call xrApi::destroy", e);
    }

    frameScheduler.destroy();
  }

  /** Sets listener to receive callbacks that occur as part of the frame loop of the renderer. */
  public void setFrameListener(@Nullable FrameListener frameListener) {
    this.frameListener = frameListener;
  }

  /** Gets listener to receive callbacks that occur as part of the frame loop of the renderer. */
  @Nullable
  public FrameListener getFrameListener() {
    return frameListener;
  }

  public String dump() {
    ListenableFuture<String> future = frameScheduler.submitOnFrameThread(xrApi::dump);

    try {
      // Block until dump has returned.
      return future.get();
    } catch (InterruptedException | ExecutionException e) {
      throw new IllegalStateException("Unable to call xrApi::dump", e);
    }
  }

  //  Disables rendering/display of frames but allows the frame loop to continue running.
  public void disableDisplay() {
    Futures.addCallback(
        frameScheduler.submitOnFrameThread(
            () -> {
              xrApi.disableDisplay();
              return null;
            }),
        new LogOnErrorCallback("Failed to disable display."),
        frameScheduler.getExecutor());
  }

  //  Renders/displays as normal. This is the default and only needs to be called to reenable
  // the display after a call to disableDisplay.
  public void enableDisplay() {
    Futures.addCallback(
        frameScheduler.submitOnFrameThread(
            () -> {
              xrApi.enableDisplay();
              return null;
            }),
        new LogOnErrorCallback("Failed to enable display."),
        frameScheduler.getExecutor());
  }

  public void setDrmProtectionModeEnabled(boolean enabled) {
    xrApi.setDrmProtectionModeEnabled(enabled);
  }

  private long advanceFrame(long frameTimeNanos) {
    // The time coming from the FrameScheduler is ignored because the timing comes from OpenXR in
    // native code.
    xrApi.advanceFrame();

    if (frameListener != null) {
      frameListener.onPostFrame();
    }

    // Always return "successful". The ContinuousFrameScheduler will schedule another frame
    // immediately either way.
    return -1;
  }
}
