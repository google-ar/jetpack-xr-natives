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
import com.android.extensions.xr.XrExtensions;
import com.google.ar.imp.view.ChoreographerFrameScheduler;
import com.google.ar.imp.view.FrameScheduler;
import com.google.ar.imp.view.ImpApiScuba;
import com.google.ar.imp.view.View;
import com.google.common.util.concurrent.ListenableFuture;
import com.google.imp.splitengine.extensions.IRendererConnection;
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

  /** Builder for creating ImpSplitEngineRenderer. */
  public static class Builder {
    private final Context context;
    @Nullable private ImpSplitEngine.SplitEngineSetupParams setupParams = null;
    @Nullable private ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider = null;
    @Nullable private IBinder serviceBinder = null;
    @Nullable private XrExtensions xrExtensions = null;
    @Nullable private RendererConnectionServiceProvider rendererConnection = null;
    @Nullable private FrameScheduler.Factory frameSchedulerFactory = null;

    public Builder(Context context) {
      this.context = context;
    }

    public Builder setSetupParams(@Nullable ImpSplitEngine.SplitEngineSetupParams setupParams) {
      this.setupParams = setupParams;
      return this;
    }

    public Builder setViewParamsProvider(
        @Nullable ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider) {
      this.viewParamsProvider = viewParamsProvider;
      return this;
    }

    public Builder setServiceBinder(@Nullable IBinder serviceBinder) {
      this.serviceBinder = serviceBinder;
      return this;
    }

    public Builder setXrExtensions(@Nullable XrExtensions xrExtensions) {
      this.xrExtensions = xrExtensions;
      return this;
    }

    public Builder setRendererConnection(
        @Nullable RendererConnectionServiceProvider rendererConnection) {
      this.rendererConnection = rendererConnection;
      return this;
    }

    public Builder setFrameSchedulerFactory(
        @Nullable FrameScheduler.Factory frameSchedulerFactory) {
      this.frameSchedulerFactory = frameSchedulerFactory;
      return this;
    }

    public ImpSplitEngineRenderer create() {
      FrameScheduler.Factory factory =
          frameSchedulerFactory != null
              ? frameSchedulerFactory
              : new ChoreographerFrameScheduler.Factory();
      FrameScheduler.ThreadMode threadMode =
          (setupParams != null && setupParams.threadMode != null)
              ? setupParams.threadMode
              : FrameScheduler.ThreadMode.MAIN_DEFAULT;
      FrameScheduler frameScheduler = factory.create(threadMode);
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
                      xrExtensions,
                      rendererConnection));
      try {
        // Block until ImpSplitEngineApi has been created.
        splitEngineApi = apiFuture.get();
      } catch (InterruptedException | ExecutionException e) {
        Log.e(TAG, "Failed to create ImpXrApi. Exception: ", e);
        throw new IllegalStateException("Unable to initialize Impress API", e);
      }
      return new ImpSplitEngineRenderer(splitEngineApi, frameScheduler, viewParamsProvider);
    }
  }

  /**
   * @deprecated Use {@link Builder} instead.
   */
  @Deprecated // Use the Builder instead.
  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable XrExtensions xrExtensions) {
    return new Builder(context).setSetupParams(setupParams).setXrExtensions(xrExtensions).create();
  }

  /**
   * @deprecated Use {@link Builder} instead.
   */
  @Deprecated // Use the Builder instead.
  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable XrExtensions xrExtensions,
      @Nullable FrameScheduler.Factory frameSchedulerFactory) {
    return new Builder(context)
        .setSetupParams(setupParams)
        .setXrExtensions(xrExtensions)
        .setFrameSchedulerFactory(frameSchedulerFactory)
        .create();
  }

  /**
   * @deprecated Use {@link Builder} instead.
   */
  @Deprecated // Use the Builder instead.
  public static ImpSplitEngineRenderer create(
      Context context,
      RendererConnectionServiceProvider rendererConnection,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider,
      @Nullable IBinder binder) {
    return new Builder(context)
        .setRendererConnection(rendererConnection)
        .setSetupParams(setupParams)
        .setViewParamsProvider(viewParamsProvider)
        .setServiceBinder(binder)
        .create();
  }

  /**
   * @deprecated Use {@link Builder} instead.
   */
  @Deprecated // Use the Builder instead.
  public static ImpSplitEngineRenderer create(
      Context context,
      RendererConnectionServiceProvider rendererConnection,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider,
      @Nullable IBinder binder,
      @Nullable FrameScheduler.Factory frameSchedulerFactory) {
    return new Builder(context)
        .setRendererConnection(rendererConnection)
        .setSetupParams(setupParams)
        .setViewParamsProvider(viewParamsProvider)
        .setServiceBinder(binder)
        .setFrameSchedulerFactory(frameSchedulerFactory)
        .create();
  }

  /**
   * @deprecated Use {@link Builder} instead.
   */
  @Deprecated // Use the Builder instead.
  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable IBinder binder,
      @Nullable XrExtensions xrExtensions) {
    return new Builder(context)
        .setSetupParams(setupParams)
        .setServiceBinder(binder)
        .setXrExtensions(xrExtensions)
        .create();
  }

  /**
   * @deprecated Use {@link Builder} instead.
   */
  @Deprecated // Use the Builder instead.
  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable IBinder binder,
      @Nullable XrExtensions xrExtensions,
      @Nullable FrameScheduler.Factory frameSchedulerFactory) {
    return new Builder(context)
        .setSetupParams(setupParams)
        .setServiceBinder(binder)
        .setXrExtensions(xrExtensions)
        .setFrameSchedulerFactory(frameSchedulerFactory)
        .create();
  }

  /**
   * @deprecated Use {@link Builder} instead.
   */
  @Deprecated // Use the Builder instead.
  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider,
      @Nullable XrExtensions xrExtensions) {
    return new Builder(context)
        .setSetupParams(setupParams)
        .setViewParamsProvider(viewParamsProvider)
        .setXrExtensions(xrExtensions)
        .create();
  }

  /**
   * @deprecated Use {@link Builder} instead.
   */
  @Deprecated // Use the Builder instead.
  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider,
      @Nullable XrExtensions xrExtensions,
      @Nullable FrameScheduler.Factory frameSchedulerFactory) {
    return new Builder(context)
        .setSetupParams(setupParams)
        .setViewParamsProvider(viewParamsProvider)
        .setXrExtensions(xrExtensions)
        .setFrameSchedulerFactory(frameSchedulerFactory)
        .create();
  }

  /**
   * @deprecated Use {@link Builder} instead.
   */
  @Deprecated // Use the Builder instead.
  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider,
      @Nullable IBinder serviceBinder,
      @Nullable XrExtensions xrExtensions,
      @Nullable RendererConnectionServiceProvider rendererConnection) {
    return new Builder(context)
        .setSetupParams(setupParams)
        .setViewParamsProvider(viewParamsProvider)
        .setServiceBinder(serviceBinder)
        .setXrExtensions(xrExtensions)
        .setRendererConnection(rendererConnection)
        .create();
  }

  /**
   * @deprecated Use {@link Builder} instead.
   */
  @Deprecated // Use the Builder instead.
  public static ImpSplitEngineRenderer create(
      Context context,
      @Nullable ImpSplitEngine.SplitEngineSetupParams setupParams,
      @Nullable ImpSplitEngine.SplitEngineViewParamsProvider viewParamsProvider,
      @Nullable IBinder serviceBinder,
      @Nullable XrExtensions xrExtensions,
      @Nullable RendererConnectionServiceProvider rendererConnection,
      @Nullable FrameScheduler.Factory frameSchedulerFactory) {
    return new Builder(context)
        .setSetupParams(setupParams)
        .setViewParamsProvider(viewParamsProvider)
        .setServiceBinder(serviceBinder)
        .setXrExtensions(xrExtensions)
        .setRendererConnection(rendererConnection)
        .setFrameSchedulerFactory(frameSchedulerFactory)
        .create();
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

  public IRendererConnection getRendererConnection() {
    return splitEngineApi.getRendererConnection();
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
    // Stop the frame loop if it's not already stopped to avoid the client keep sending commands.
    if (isActive) {
      stopFrameLoop();
    }
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
