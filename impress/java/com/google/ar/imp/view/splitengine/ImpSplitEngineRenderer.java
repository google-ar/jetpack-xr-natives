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
import android.util.Log;
import androidx.annotation.Nullable;
import androidx.xr.extensions.splitengine.SplitEngineBridge;
import com.android.extensions.xr.XrExtensions;
import com.google.ar.imp.view.ChoreographerFrameScheduler;
import com.google.ar.imp.view.FrameScheduler;
import com.google.ar.imp.view.ImpApiScuba;
import com.google.ar.imp.view.View;
import com.google.common.util.concurrent.ListenableFuture;
import java.util.concurrent.ExecutionException;

/**
 * Impress renderer for Split Engine apps.
 *
 * <p>Used to create a native Impress View and drive the frame loop for Split Engine.
 */
public class ImpSplitEngineRenderer implements ImpApiScuba {
  private static final String TAG = ImpSplitEngineRenderer.class.getSimpleName();

  /** Interface for callbacks that occur as part of the frame loop of the renderer. */
  public interface FrameListener {
    /** Called after each attempted frame. */
    // TODO: Add parameter to indicate if the frame succeeded or not.
    void onPostFrame();
  }

  private final ImpSplitEngineApi splitEngineApi;
  private final FrameScheduler frameScheduler;

  @Nullable private FrameListener frameListener = null;

  @Nullable private Long renderedFrameTimeNanos = null;

  @Nullable private ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider = null;

  private boolean isActive = false;

  /** Initialize Impress for Split Engine. */
  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable XrExtensions xrExtensions) {
    return create(context, setupParams, null, null, xrExtensions);
  }

  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable IBinder binder,
      @Nullable XrExtensions xrExtensions) {
    return create(context, setupParams, null, binder, xrExtensions);
  }

  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider,
      @Nullable XrExtensions xrExtensions) {
    return create(context, setupParams, viewParamsProvider, null, xrExtensions);
  }

  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider,
      @Nullable IBinder serviceBinder,
      @Nullable XrExtensions xrExtensions) {

    FrameScheduler frameScheduler =
        new ChoreographerFrameScheduler.Factory().create(FrameScheduler.ThreadMode.MAIN_DEFAULT);

    // ImpSplitEngineApi creates the View internally, and also provides Xr specific Jni calls.
    ImpSplitEngineApi splitEngineApi;
    final ImpSplitEngine.ScreenSize finalScreenSize =
        viewParamsProvider != null ? viewParamsProvider.getScreenSize() : null;

    ListenableFuture<ImpSplitEngineApi> apiFuture =
        frameScheduler.submitOnFrameThread(
            () ->
                ImpSplitEngineApi.create(
                    context,
                    setupParams,
                    finalScreenSize,
                    frameScheduler.getExecutor(),
                    serviceBinder,
                    xrExtensions));

    try {
      // Block until ImpSplitEngineApi has been created.
      splitEngineApi = apiFuture.get();
    } catch (InterruptedException | ExecutionException e) {
      Log.e(TAG, "Failed to create ImpXrApi. Exception: ", e);
      throw new IllegalStateException("Unable to initialize Impress API", e);
    }
    return new ImpSplitEngineRenderer(splitEngineApi, frameScheduler, viewParamsProvider);
  }

  private ImpSplitEngineRenderer(
      ImpSplitEngineApi splitEngineApi,
      FrameScheduler frameScheduler,
      @Nullable ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider) {
    this.splitEngineApi = splitEngineApi;
    this.frameScheduler = frameScheduler;
    this.viewParamsProvider = viewParamsProvider;
  }

  /**
   * Returns the Impress view associated with the renderer. Used to initialize the Impress scripting
   * system or make custom direct Jni calls.
   */
  public View getView() {
    return splitEngineApi.getView();
  }

  public SplitEngineBridge getBridge() {
    return splitEngineApi.getBridge();
  }

  /**
   * Returns the FrameScheduler. Allows callers to get the frame thread's looper and to submit work.
   */
  public FrameScheduler getFrameScheduler() {
    return frameScheduler;
  }

  /** Starts the render loop. Rendering will not begin until the bridge service is connected. */
  public void startFrameLoop() {
    isActive = true;
    frameScheduler.runOnFrameThread(splitEngineApi::onResume);
    frameScheduler.startFrameLoop(this::advanceFrame);
  }

  /** Stops the render loop. */
  public void stopFrameLoop() {
    isActive = false;
    frameScheduler.stopFrameLoop();
    frameScheduler.runOnFrameThread(splitEngineApi::onPause);
  }

  /** Destroys the renderer and Impress view. */
  public void destroy() {
    ListenableFuture<Void> future =
        frameScheduler.submitOnFrameThread(
            () -> {
              splitEngineApi.destroy();
              return null;
            });

    try {
      // Block until destroy has returned.
      future.get();
    } catch (InterruptedException | ExecutionException e) {
      throw new IllegalStateException("Unable to call splitEngineApi::destroy", e);
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

  private long advanceFrame(long frameTimeNanos) {
    frameScheduler.checkFrameThread();
    ImpSplitEngine.ViewUpdateParams viewUpdateParams =
        (viewParamsProvider != null) ? viewParamsProvider.getViewUpdateParams() : null;
    long result =
        splitEngineApi.renderNextFrame(
            (renderedFrameTimeNanos != null) ? renderedFrameTimeNanos : frameTimeNanos,
            frameTimeNanos,
            (viewUpdateParams != null) ? viewUpdateParams.getNativeHandle() : 0);
    if (viewUpdateParams != null) {
      viewUpdateParams.destroyNativeParams();
    }
    renderedFrameTimeNanos = frameTimeNanos;

    if (result != 0 && frameListener != null) {
      frameListener.onPostFrame();
    }
    return result;
  }

  @Override
  public boolean doFrame(long frameTimeNanos) {
    frameScheduler.runOnFrameThread(
        () -> {
          if (!isActive) {
            isActive = true;
            splitEngineApi.onResume();
          }
          long unused = advanceFrame(frameTimeNanos);
        });
    return true;
  }

  @Override
  public Long getPreviousFrameTimeNanos() {
    return renderedFrameTimeNanos;
  }

  @Override
  public void drainAllExecutorsForTest() {
    frameScheduler.runOnFrameThread(getView()::drainAllExecutorsForTest);
  }
}
