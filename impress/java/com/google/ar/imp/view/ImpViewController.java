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

import static java.lang.Math.max;
import static java.lang.Math.min;

import android.content.Context;
import android.util.DisplayMetrics;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.SurfaceView;
import android.view.TextureView;
import androidx.annotation.Nullable;
import com.google.android.filament.android.UiHelper;
import com.google.ar.imp.core.scripting.ScriptEndpoint;

import com.google.common.base.Preconditions;
import com.google.common.util.concurrent.ListenableFuture;
import java.util.concurrent.ExecutionException;

/**
 * Java layer for ImpView functionality. Any Java-specific logic for ImpView goes here. It is
 * created with a fully instantiated View instance.
 *
 * <p>ImpApi exposes this API publicly and handles any asynchronicity.
 */
public class ImpViewController {
  // TODO: replace with MonotonicNonNull
  @Nullable public final UiHelper uiHelper;
  private final View view;
  private final FrameScheduler frameScheduler;
  @Nullable private final android.view.View androidView;

  public ImpViewController(
      FrameScheduler frameScheduler,
      Context context,
      android.view.View androidView,
      boolean isOpaque,
      View view,
      float desiredSizeScale,
      long swapChainFlags) {
    this.androidView = androidView;

    Preconditions.checkState(
        androidView instanceof TextureView || androidView instanceof SurfaceView);

    this.frameScheduler = frameScheduler;
    this.view = view;

    uiHelper = new UiHelper(UiHelper.ContextErrorPolicy.DONT_CHECK);
    uiHelper.setOpaque(isOpaque);

    // If the view has already been laid out, we need its size.
    desiredSizeScale = max(0.1f, min(1.0f, desiredSizeScale));
    int desiredWidth = (int) ((float) androidView.getWidth() * desiredSizeScale);
    int desiredHeight = (int) ((float) androidView.getHeight() * desiredSizeScale);

    DisplayMetrics dm = androidView.getContext().getResources().getDisplayMetrics();
    float displayDensity = dm.density;

    uiHelper.setDesiredSize(desiredWidth, desiredHeight);
    uiHelper.setRenderCallback(
        new UiHelper.RendererCallback() {
          @Override
          public void onNativeWindowChanged(Surface surface) {
            if (getNativeHandle() == 0) {
              return;
            }

            frameScheduler.runOnFrameThread(
                () -> {
                  if (view.hasSwapChain()) {
                    view.destroySwapChain();
                  }
                  view.createSwapChain(surface, uiHelper.getSwapChainFlags() | swapChainFlags);
                });
          }

          @Override
          public void onDetachedFromSurface() {
            if (getNativeHandle() == 0) {
              return;
            }

            frameScheduler.runOnFrameThread(
                () -> {
                  if (view.hasSwapChain()) {
                    view.destroySwapChain();
                    view.flushAndWait();
                  }
                });
          }

          @Override
          public void onResized(int width, int height) {
            if (getNativeHandle() == 0) {
              return;
            }

            // When android inflates a layout it converts from dp into pixels.
            // We want to convert back into dp to pass the size into Impress, and pass the density
            // of the display along as the ratio of virtual pixels to physical pixels.
            // This behavior is more consistent with other platforms (i.e. iOS, web) and allows
            // developers to write logic using a density independent unit.
            frameScheduler.runOnFrameThread(
                () -> view.resize(width, height, displayDensity, displayDensity));
          }
        });
    // If the androidView is already valid, the callback will be called immediately. Therefore, we
    // need to make sure the callback is set before the view.
    if (androidView instanceof TextureView) {
      uiHelper.attachTo((TextureView) androidView);
    } else {
      uiHelper.attachTo((SurfaceView) androidView);
    }
  }

  /** Returns the basic view jni interface. */
  public View getView() {
    return view;
  }

  // TODO Clean this up.
  
  public void setupForTesting(long platformHandle) {
    view.setup(platformHandle);
  }

  public void setDisplayRotation(int rotation) {
    frameScheduler.runOnFrameThread(() -> view.setSurfaceRotation(rotation));
  }

  @Nullable
  public Long getPreviousFrameTimeNanos() {

    return view.getPreviousFrameTimeNanos();
  }

  /** Returns the frame scheduler used to run the frame loop. */
  public FrameScheduler getFrameScheduler() {
    return frameScheduler;
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
    onResume();
    frameScheduler.startFrameLoop(this::advanceFrame);
  }

  /** Stops advancing and rendering frames automatically. */
  public void stopFrameLoop() {
    frameScheduler.stopFrameLoop();
    onPause();
  }

  /**
   * Advances to the timestamp passed in and renders the frame.
   *
   * <p>If -1 is returned, then the frame was successful. If 0 is returned, then the frame was
   * skipped. If > 0 is returned, then the value indicates how long we should wait before advancing
   * another frame. These values are not intuitive, but this allows us to avoid allocating a java
   * object on the heap every frame to represent the result, which is acceptable given that this
   * method is an implementation detail.
   */
  public long advanceFrame(long frameTimeNanos) {
    frameScheduler.checkFrameThread();
    if (getNativeHandle() == 0
        || !view.hasSwapChain()
        || (uiHelper != null && !uiHelper.isReadyToRender())) {
      return 0;
    }
    view.captureVsyncTime();
    return view.renderNextFrame(frameTimeNanos);
  }

  public void synchronizePendingFrames() {
    frameScheduler.runOnFrameThread(view::synchronizePendingFrames);
  }

  public void releaseResources() {
    if (getNativeHandle() == 0) {
      return;
    }

    // Always detach the surface before destroying the engine
    if (uiHelper != null) {
      uiHelper.detach();
      if (androidView instanceof TextureView) {
        ((TextureView) androidView).setSurfaceTextureListener(null);
      }
    }

    ListenableFuture<Void> future =
        frameScheduler.submitOnFrameThread(
            () -> {
              view.destroy();
              return null;
            });

    try {
      // Block until destroy has returned.
      future.get();
    } catch (InterruptedException | ExecutionException e) {
      throw new IllegalStateException("Unable to call View::destroy", e);
    }

    frameScheduler.destroy();
  }

  
  public void staticRenderForTest() {
    frameScheduler.runOnFrameThread(view::staticRenderForTest);
  }

  public void drainAllExecutorsForTest() {
    frameScheduler.runOnFrameThread(view::drainAllExecutorsForTest);
  }

  /** Returns the handle to the native imp::ViewHost instance. */
  public long getViewHostHandle() {
    return view.getViewHostHandle();
  }

  /** Returns the handle to the native imp::View instance. */
  public long getNativeHandle() {
    return view.getNativeHandle();
  }

  /** Sends the script endpoint from native to enable Java<-->C++ communication. */
  public void setScriptEndpoint(ScriptEndpoint scriptEndpoint) {
    frameScheduler.runOnFrameThread(() -> view.setScriptEndpoint(scriptEndpoint));
  }

  public void setupSurfaceRenderer(Surface surface, String visibilityGroup, String cameraName) {
    frameScheduler.runOnFrameThread(
        () -> view.setupSurfaceRenderer(surface, visibilityGroup, cameraName));
  }

  public void onPause() {
    if (getNativeHandle() == 0) {
      return;
    }

    frameScheduler.runOnFrameThread(() -> view.onPause());
  }

  public void onResume() {
    if (getNativeHandle() == 0) {
      return;
    }

    frameScheduler.runOnFrameThread(() -> view.onResume());
  }

  public boolean onTouchEvent(MotionEvent e) {
    if (getNativeHandle() == 0) {
      return false;
    }

    if (!frameScheduler.isOnFrameThread()) {
      // If the frame scheduler will run on a different thread, we need to obtain() a copy of the
      // event, as it's recycled on the main thread.
      MotionEvent safeEvent = MotionEvent.obtain(e);
      frameScheduler.runOnFrameThread(
          () -> {
            view.onTouchEvent(safeEvent);
            safeEvent.recycle();
          });
    } else {
      // This will be run immediately.
      view.onTouchEvent(e);
    }

    return true;
  }
}
