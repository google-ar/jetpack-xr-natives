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

package com.google.ar.imp.view;

import android.content.Context;
import android.util.Log;
import android.view.MotionEvent;
import android.view.Surface;
import androidx.annotation.Nullable;
import com.google.android.filament.proguard.UsedByNative;
import com.google.ar.imp.core.ViewConfig;
import com.google.ar.imp.core.scripting.ScriptEndpoint;
import com.google.ar.imp.core.web.FragmentHost;

import com.google.common.util.concurrent.FutureCallback;
import com.google.common.util.concurrent.Futures;
import com.google.common.util.concurrent.ListenableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executor;

/**
 * Imp's public API. It is always created and available immediately, but can support asynchronous
 * loading of native library. It hides this asynchronicity to allow clients to call these API
 * conveniently.
 *
 * <p>If the library is loaded asynchronously, it will use the Context's mainExecutor for setting up
 * and call the postCreatedCallback, so thereafter it expects all future calls to also be on that
 * thread.
 */
public class ImpApi implements ImpApiScuba {
  private static final String TAG = ImpApi.class.getSimpleName();

  private ImpViewController impViewController;

  /** Tracks whether {@ link #isReleased} has been called in this API. */
  private boolean released = false;

  /**
   * Interface for createAsync().
   *
   * <p>TODO Replace with java.util.function.Consumer after adding Java 8 to builds.
   */
  public interface PostCreatedCallback {
    public void accept(ImpApi createdImpApi);
  }

  /** Interface that notifies Java client on lifecycle events. */
  @UsedByNative("imp_lifecycle_callback.cc")
  public interface ImpLifeCycleCallback {
    @UsedByNative("imp_lifecycle_callback.cc")
    public void onEditorEnabled(boolean enabled);
  }

  public static FrameScheduler.ThreadMode getThreadMode(SetupParams.ThreadMode threadMode) {
    return switch (threadMode) {
      case MAIN_DEFAULT -> FrameScheduler.ThreadMode.MAIN_DEFAULT;
      case BACKGROUND -> FrameScheduler.ThreadMode.BACKGROUND;
    };
  }

  /** Builder for creating ImpApi. */
  public static class Builder {
    private final SetupParams setupParams;
    private final Context context;
    private final android.view.View androidView;
    @Nullable private FragmentHost host = null;
    private long eglContext = 0;
    @Nullable private FrameScheduler.Factory frameSchedulerFactory = null;

    public Builder(SetupParams setupParams, Context context, android.view.View androidView) {
      this.setupParams = setupParams;
      this.context = context;
      this.androidView = androidView;
    }

    /**
     * Sets the fragment host for hosting a WebView. This is only needed if the client app intends
     * to embed a webview within the Impress View.
     */
    public Builder setFragmentHost(@Nullable FragmentHost host) {
      this.host = host;
      return this;
    }

    /** Sets the EGL context for the Impress View. */
    public Builder setEglContext(long eglContext) {
      this.eglContext = eglContext;
      return this;
    }

    /**
     * Sets the factory for creating a frame scheduler for the Impress View. If unset, the default
     * used is ChoreographerFrameScheduler.Factory.
     */
    public Builder setFrameSchedulerFactory(FrameScheduler.Factory frameSchedulerFactory) {
      this.frameSchedulerFactory = frameSchedulerFactory;
      return this;
    }

    /**
     * Creates an ImpApi synchronously on the frame thread. This call will block the calling thread
     * until the ImpApi is ready to use.
     */
    public ImpApi createSync() {
      return ImpApi.createSync(
          setupParams, context, androidView, host, eglContext, frameSchedulerFactory);
    }

    /**
     * Creates an ImpApi asynchronously on the provided executor. The postCreatedCallback will be
     * called on the calling thread when the ImpApi is ready to use.
     *
     * <p>The returned ImpApi will no-op any API calls until it is done. First, it loads the library
     * on the provided Executor. Then, it switches back to the calling thread to setup and call the
     * postCreatedCallback. After this point, all API should work and need to be called from the
     * calling thread
     */
    public ImpApi createAsync(Executor executor, PostCreatedCallback postCreatedCallback) {
      return ImpApi.createAsync(
          setupParams,
          context,
          androidView,
          host,
          eglContext,
          executor,
          postCreatedCallback,
          frameSchedulerFactory);
    }

    /**
     * Creates an ImpApi asynchronously on the provided executor.
     *
     * <p>The library is loaded asynchronously on the provided Executor, then switches to the frame
     * scheduler's Executor to complete the set up of the Impress View.
     */
    public ListenableFuture<ImpApi> createAsync(Executor executor) {
      return ImpApi.createAsync(
          setupParams, context, androidView, host, executor, frameSchedulerFactory);
    }
  }

  /** ImpApi is valid for further calls immediately. */
  @Deprecated // Use the builder instead.
  public static ImpApi createSync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      @Nullable FragmentHost host) {
    return createSync(setupParams, context, androidView, host, 0, null);
  }

  /** ImpApi is valid for further calls immediately. */
  @Deprecated // Use the builder instead.
  public static ImpApi createSync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      @Nullable FragmentHost host,
      long eglContext) {
    return createSync(setupParams, context, androidView, host, eglContext, null);
  }

  /** ImpApi is valid for further calls immediately. */
  @Deprecated // Use the builder instead.
  public static ImpApi createSync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      @Nullable FragmentHost host,
      long eglContext,
      @Nullable FrameScheduler.Factory frameSchedulerFactory) {
    if (frameSchedulerFactory == null) {
      frameSchedulerFactory = new ChoreographerFrameScheduler.Factory();
    }
    FrameScheduler frameScheduler =
        frameSchedulerFactory.create(getThreadMode(setupParams.getThreadMode()));

    ViewConfig viewConfig =
        setupParams.hasViewConfig()
            ? setupParams.getViewConfig()
            : ViewConfig.newBuilder()
                .setMainViewRenderSettings(setupParams.getViewRenderSettings())
                .build();

    ListenableFuture<View> viewFuture =
        frameScheduler.submitOnFrameThread(
            () -> {
              View view;
              if (!setupParams.getSkipNativeLibraryLoad()) {
                view =
                    View.createView(
                        setupParams.getCustomNativeLibrary(),
                        setupParams.getViewIdentifier(),
                        context,
                        host,
                        frameScheduler.getExecutor(),
                        viewConfig.toByteArray());
              } else {
                view =
                    View.createViewWithPreloadedLibrary(
                        setupParams.getCustomNativeLibrary(),
                        setupParams.getViewIdentifier(),
                        context,
                        host,
                        frameScheduler.getExecutor(),
                        viewConfig.toByteArray());
              }
              view.setup(setupParams.getPlatformHandle(), eglContext);
              return view;
            });

    try {
      View view = viewFuture.get();

      return new ImpApi(
          new ImpViewController(
              frameScheduler,
              context,
              androidView,
              setupParams.getIsOpaque(),
              view,
              setupParams.getDesiredSizeScale(),
              setupParams.getSwapChainFlags(),
              setupParams.getUseSynchronousSurfaceChanges()));
    } catch (InterruptedException | ExecutionException e) {
      throw new IllegalStateException("Unable to initialize Impress API", e);
    }
  }

  @Deprecated // Use the builder instead.
  public static ListenableFuture<View> createViewAsync(
      SetupParams setupParams, Context context, FragmentHost host, Executor executor) {
    ViewConfig viewConfig =
        setupParams.hasViewConfig()
            ? setupParams.getViewConfig()
            : ViewConfig.newBuilder()
                .setMainViewRenderSettings(setupParams.getViewRenderSettings())
                .build();
    return Futures.submit(
        () ->
            View.createView(
                setupParams.getCustomNativeLibrary(),
                setupParams.getViewIdentifier(),
                context,
                host,
                executor,
                viewConfig.toByteArray()),
        executor);
  }

  /**
   * The returned ImpApi will no-op any API calls until it is done. First, it loads the library on
   * the provided Executor. Then, it switches back to the main thread to setup and call the
   * postCreatedCallback. After this point, all API should work and need to be called from the main
   * thread.
   */
  @Deprecated // Use the builder instead.
  public static ImpApi createAsync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      FragmentHost host,
      Executor executor,
      PostCreatedCallback postCreatedCallback) {
    return createAsync(setupParams, context, androidView, host, 0, executor, postCreatedCallback);
  }

  @Deprecated // Use the builder instead.
  public static ImpApi createAsync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      FragmentHost host,
      long eglContext,
      Executor executor,
      PostCreatedCallback postCreatedCallback) {
    return createAsync(
        setupParams, context, androidView, host, eglContext, executor, postCreatedCallback, null);
  }

  @Deprecated // Use the builder instead.
  public static ImpApi createAsync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      FragmentHost host,
      long eglContext,
      Executor executor,
      PostCreatedCallback postCreatedCallback,
      @Nullable FrameScheduler.Factory frameSchedulerFactory) {
    ImpApi impApi = new ImpApi();

    // Create the view on the provided executor to isolate file io from the main thread.
    ListenableFuture<View> viewFuture = createViewAsync(setupParams, context, host, executor);

    if (frameSchedulerFactory == null) {
      frameSchedulerFactory = new ChoreographerFrameScheduler.Factory();
    }

    FrameScheduler frameScheduler =
        frameSchedulerFactory.create(getThreadMode(setupParams.getThreadMode()));

    Futures.addCallback(
        viewFuture,
        new FutureCallback<View>() {
          @Override
          public void onSuccess(@Nullable View view) {
            view.setup(setupParams.getPlatformHandle(), eglContext);
            impApi.impViewController =
                new ImpViewController(
                    frameScheduler,
                    context,
                    androidView,
                    setupParams.getIsOpaque(),
                    view,
                    setupParams.getDesiredSizeScale(),
                    setupParams.getSwapChainFlags(),
                    setupParams.getUseSynchronousSurfaceChanges());
            postCreatedCallback.accept(impApi);
          }

          @Override
          public void onFailure(Throwable t) {
            Log.w(TAG, "Failed to create ImpViewController.", t);
          }
        },
        // TODO: Return ListenableFuture<ImpApi> to allow caller to decide where to run
        // the callback and prevent any APIs from being called until the future is completed.
        frameScheduler.getExecutor());
    return impApi;
  }

  @Deprecated // Use the builder instead.
  public static ListenableFuture<ImpApi> createAsync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      FragmentHost host,
      Executor executor) {
    return createAsync(
        setupParams, context, androidView, host, executor, (FrameScheduler.Factory) null);
  }

  @Deprecated // Use the builder instead.
  public static ListenableFuture<ImpApi> createAsync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      FragmentHost host,
      Executor executor,
      @Nullable FrameScheduler.Factory frameSchedulerFactory) {
    ListenableFuture<View> viewFuture =
        ImpApi.createViewAsync(setupParams, context, host, executor);

    if (frameSchedulerFactory == null) {
      frameSchedulerFactory = new ChoreographerFrameScheduler.Factory();
    }
    FrameScheduler frameScheduler =
        frameSchedulerFactory.create(getThreadMode(setupParams.getThreadMode()));

    return Futures.transform(
        viewFuture,
        view -> {
          view.setup(setupParams.getPlatformHandle());
          return new ImpApi(
              new ImpViewController(
                  frameScheduler,
                  context,
                  androidView,
                  setupParams.getIsOpaque(),
                  view,
                  setupParams.getDesiredSizeScale(),
                  setupParams.getSwapChainFlags(),
                  setupParams.getUseSynchronousSurfaceChanges()));
        },
        frameScheduler.getExecutor());
  }

  public void addLifeCycleCallback(ImpLifeCycleCallback callback) {
    getView().setLifeCycleCallback(callback);
  }

  private ImpApi() {}

  private ImpApi(ImpViewController impViewController) {
    this.impViewController = impViewController;
  }

  /** Returns the basic view jni interface. */
  public View getView() {
    return this.impViewController.getView();
  }

  @Override
  @Nullable
  public Long getPreviousFrameTimeNanos() {
    if (isReleased()) {
      Log.w(
          TAG,
          "getPreviousFrameTimeNanos() was called on ImpApi after its native resources had been"
              + " released.");
      return null;
    }
    if (impViewController != null) {
      return impViewController.getPreviousFrameTimeNanos();
    }
    return null;
  }

  /**
   * Returns the frame scheduler used to run the frame loop. When Impress is configured to run on a
   * background thread, this can be used to schedule work on frame thread.
   */
  public FrameScheduler getFrameScheduler() {
    return impViewController.getFrameScheduler();
  }

  /**
   * Starts automatically advancing and rendering the frames.
   *
   * <p>Typically, frames are advanced and rendered when the choreographer ticks. However, if the
   * app skips a frame using ViewPreFrameUpdateEvent::SkipFrame with a time_until_retry, then we
   * will post to the main looper to advance another frame instead of advancing at the next
   * choreographer tick.
   */
  public void startFrameLoop() {
    if (isReleased()) {
      Log.w(
          TAG,
          "startFrameLoop() was called on ImpApi after its native resources had been"
              + " released.");
      return;
    }
    if (impViewController != null) {
      impViewController.startFrameLoop();
    }
  }

  /** Stops advancing and rendering frames automatically. */
  public void stopFrameLoop() {
    if (isReleased()) {
      Log.w(
          TAG,
          "stopFrameLoop() was called on ImpApi after its native resources had been released.");
      return;
    }
    if (impViewController != null) {
      impViewController.stopFrameLoop();
    }
  }

  /**
   * Explicitly triggers the resume lifecycle event.
   *
   * <p>**IMPORTANT** This is called automatically as part of {@link #startFrameLoop()}, this is
   * only required to be called explicitly for apps that wish to forego the normal frame loop and
   * explicitly control when frames are rendered.
   */
  public void resume() {
    if (isReleased()) {
      Log.w(
          TAG, "resume() was called on ImpApi after its native resources had been" + " released.");
      return;
    }
    if (impViewController != null) {
      impViewController.onResume();
    }
  }

  /**
   * Explicitly triggers the pause lifecycle event.
   *
   * <p>**IMPORTANT** This is called automatically as part of {@link #stopFrameLoop()}, this is only
   * required to be called explicitly for apps that wish to forego the normal frame loop and
   * explicitly control when frames are rendered.
   */
  public void pause() {
    if (isReleased()) {
      Log.w(TAG, "pause() was called on ImpApi after its native resources had been released.");
      return;
    }
    if (impViewController != null) {
      impViewController.onPause();
    }
  }

  /**
   * Advances the frame to the timestamp passed in and renders.
   *
   * <p>Use this method to manually control when Impress renders a frame. Alternatively, you can let
   * Impress control the frame loop automatically by calling startFrameLoop and stopFrameLoopReturns
   * whether the frame was successfully submitted to Filament.
   */
  @Override
  public boolean doFrame(long frameTimeNanos) {
    if (isReleased()) {
      Log.w(
          TAG, "doFrame() was called on ImpApi after its native resources had been" + " released.");
      return false;
    }
    if (impViewController == null) {
      return false;
    }

    // -1 means the frame succeeded. See advanceFrame documentation for more details.
    return impViewController.advanceFrame(frameTimeNanos) == -1;
  }

  // Block the engine until all pending frames have been processed.
  public void synchronizePendingFrames() {
    if (isReleased()) {
      Log.w(
          TAG,
          "synchronizePendingFrames() was called on ImpApi after its native resources had been"
              + " released.");
      return;
    }

    if (impViewController != null) {
      impViewController.synchronizePendingFrames();
    }
  }

  
  public void staticRenderForTest() {
    if (isReleased()) {
      Log.w(
          TAG,
          "staticRenderForTest() was called on ImpApi after its native resources had been "
              + " released.");
      return;
    }
    if (impViewController != null) {
      impViewController.staticRenderForTest();
    }
  }

  @Override
  public void drainAllExecutorsForTest() {
    if (isReleased()) {
      Log.w(
          TAG,
          "drainAllExecutorsForTest() was called on ImpApi after its native resources had been"
              + " released.");
      return;
    }
    if (impViewController != null) {
      impViewController.drainAllExecutorsForTest();
    }
  }

  /**
   * Releases the native resources backing this ImpApi instance. If the client is using a custom C++
   * imp::View class registered through the use of the imp::client_api::SetCreateViewFn API, and
   * specified by its identifier when this ImpApi was constructed, the native instance of that class
   * backing this ImpApi will be deleted as well.
   *
   * <p>Once the call to this method is made, it is the client responsibility to avoid making calls
   * to ImpApi methods, calls to objects obtained from this ImpApi (such as {@link getView()}, and
   * JNI calls to the custom C++ imp::View object whose handle was obtained through {@link
   * #getNativeHandle}.
   */
  public void releaseResources() {
    if (isReleased()) {
      Log.w(
          TAG,
          "releaseResources() was called on ImpApi after its native resources had been released.");
      return;
    }
    if (impViewController != null) {
      impViewController.releaseResources();
      released = true;
    }
  }

  /**
   * Returns whether {@link #isReleased} has been invoked on this instance, releasing the native
   * resources backing it, including any custom C++ imp::View registered by the client. Helpful to
   * avoid issuing calls to native objects that have been destroyed.
   */
  public boolean isReleased() {
    return released;
  }

  /** Returns the handle to the native imp::ViewHost instance. */
  public long getViewHostHandle() {
    if (isReleased()) {
      Log.w(
          TAG,
          "getViewHostHandle() was called on ImpApi after its native resources had been"
              + " released.");
      return 0;
    }
    if (impViewController != null) {
      return impViewController.getViewHostHandle();
    }
    return 0;
  }

  /** Returns the handle to the native imp::View instance. */
  public long getNativeHandle() {
    if (isReleased()) {
      Log.w(
          TAG,
          "getNativeHandle() was called on ImpApi after its native resources had been released.");
      return 0;
    }
    if (impViewController != null) {
      return impViewController.getNativeHandle();
    }
    return 0;
  }

  public boolean onTouchEvent(MotionEvent e) {
    if (isReleased()) {
      Log.w(
          TAG,
          "onTouchEvent() was called on ImpApi after its native resources had been" + " released.");
      return false;
    }
    if (impViewController != null) {
      return impViewController.onTouchEvent(e);
    }
    return false;
  }

  public void setDisplayRotation(int rotation) {
    if (isReleased()) {
      Log.w(
          TAG,
          "setDisplayRotation() was called on ImpApi after its native resources had been"
              + " released.");
      return;
    }
    if (impViewController != null) {
      impViewController.setDisplayRotation(rotation);
    }
  }

  /** Sends the script endpoint from native to enable Java<-->C++ communication. */
  public void setScriptEndpoint(ScriptEndpoint scriptEndpoint) {
    if (isReleased()) {
      Log.w(
          TAG,
          "setScriptEndpoint() was called on ImpApi after its native resources had been"
              + " released.");
      return;
    }
    if (impViewController != null) {
      impViewController.setScriptEndpoint(scriptEndpoint);
    }
  }

  /**
   * Setup SwapChainRenderer with the given surface. visibilityGroup specifies which nodes will be
   * rendered. Use "Main" to specify the default group. If cameraName is set, it will try to find a
   * CameraComponent that's attached to a node with that name and will render with that camera if it
   * managed to find one.
   */
  public void setupSurfaceRenderer(Surface surface, String visibilityGroup, String cameraName) {
    if (isReleased()) {
      Log.w(
          TAG,
          "setupSurfaceRenderer() was called on ImpApi after its native resources had been"
              + " released.");
      return;
    }
    if (impViewController != null) {
      impViewController.setupSurfaceRenderer(surface, visibilityGroup, cameraName);
    }
  }

  /**
   * Setup SwapChainRenderer with the given surface. visibilityGroup specifies which nodes will be
   * rendered. Use "Main" to specify the default group. The main camera will be used.
   */
  public void setupSurfaceRenderer(Surface surface, String visibilityGroup) {
    setupSurfaceRenderer(surface, visibilityGroup, null);
  }
}
