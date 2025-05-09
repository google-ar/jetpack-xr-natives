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

  static FrameScheduler.Factory frameSchedulerFactory = new ChoreographerFrameScheduler.Factory();

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
    switch (threadMode) {
      case MAIN_DEFAULT:
        return FrameScheduler.ThreadMode.MAIN_DEFAULT;
      case BACKGROUND:
        return FrameScheduler.ThreadMode.BACKGROUND;
    }
    throw new IllegalArgumentException("Unknown thread mode: " + threadMode.getNumber());
  }

  /** ImpApi is valid for further calls immediately. */
  public static ImpApi createSync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      @Nullable FragmentHost host) {
    return createSync(setupParams, context, androidView, host, 0);
  }

  public static ImpApi createSync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      @Nullable FragmentHost host,
      long eglContext) {
    FrameScheduler frameScheduler =
        frameSchedulerFactory.create(getThreadMode(setupParams.getThreadMode()));

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
                        host);
              } else {
                view =
                    View.createViewWithPreloadedLibrary(
                        setupParams.getCustomNativeLibrary(),
                        setupParams.getViewIdentifier(),
                        context,
                        host);
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
              setupParams.getSwapChainFlags()));
    } catch (InterruptedException | ExecutionException e) {
      throw new IllegalStateException("Unable to initialize Impress API", e);
    }
  }

  public static ListenableFuture<View> createViewAsync(
      SetupParams setupParams, Context context, FragmentHost host, Executor executor) {
    return Futures.submit(
        () ->
            View.createView(
                setupParams.getCustomNativeLibrary(),
                setupParams.getViewIdentifier(),
                context,
                host),
        executor);
  }

  /**
   * The returned ImpApi will no-op any API calls until it is done. First, it loads the library on
   * the provided Executor. Then, it switches back to the main thread to setup and call the
   * postCreatedCallback. After this point, all API should work and need to be called from the main
   * thread.
   */
  public static ImpApi createAsync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      FragmentHost host,
      Executor executor,
      PostCreatedCallback postCreatedCallback) {
    return createAsync(setupParams, context, androidView, host, 0, executor, postCreatedCallback);
  }

  public static ImpApi createAsync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      FragmentHost host,
      long eglContext,
      Executor executor,
      PostCreatedCallback postCreatedCallback) {
    ImpApi impApi = new ImpApi();

    // Create the view on the provided executor to isolate file io from the main thread.
    ListenableFuture<View> viewFuture = createViewAsync(setupParams, context, host, executor);

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
                    setupParams.getSwapChainFlags());
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

  public static ListenableFuture<ImpApi> createAsync(
      SetupParams setupParams,
      Context context,
      android.view.View androidView,
      FragmentHost host,
      Executor executor) {
    ListenableFuture<View> viewFuture =
        ImpApi.createViewAsync(setupParams, context, host, executor);
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
                  setupParams.getSwapChainFlags()));
        },
        frameScheduler.getExecutor());
  }

  // The following two functions are only required in a test setting when determining the value of
  // the platform handle requires ImpViewController to already be instantiated.
  // TODO Clean this up.
  
  public static ImpApi createForTesting(
      Context context,
      String customNativeLibrary,
      @Nullable String identifier,
      android.view.View androidView,
      boolean isOpaque,
      FragmentHost host,
      float desiredSizeScale,
      long swapChainFlags) {
    View view = View.createView(customNativeLibrary, identifier, context, host);
    return new ImpApi(
        new ImpViewController(
            null, context, androidView, isOpaque, view, desiredSizeScale, swapChainFlags));
  }

  public void addLifeCycleCallback(ImpLifeCycleCallback callback) {
    getView().setLifeCycleCallback(callback);
  }

  
  public void setupForTesting(long platformHandle) {
    impViewController.setupForTesting(platformHandle);
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

  /**
   * Select a factory implementation matching the desired FrameScheduler. This must be called prior
   * to createAsync. By default, frame scheduling is done by the Choreographer.
   */
  public static void setFrameSchedulerFactory(FrameScheduler.Factory factory) {
    frameSchedulerFactory = factory;
  }
}
