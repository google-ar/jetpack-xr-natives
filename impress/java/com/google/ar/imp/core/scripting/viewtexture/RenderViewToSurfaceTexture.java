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

package com.google.ar.imp.core.scripting.viewtexture;

import android.app.Presentation;
import android.content.Context;
import android.graphics.Outline;
import android.graphics.PixelFormat;
import android.hardware.display.DisplayManager;
import android.hardware.display.VirtualDisplay;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import android.util.DisplayMetrics;
import android.view.InputDevice;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.View;
import android.view.ViewOutlineProvider;
import android.view.WindowManager;
import com.google.android.filament.proguard.UsedByNative;
import com.google.common.base.Ascii;
import com.google.common.base.Strings;
import java.util.ArrayList;

// LINT.IfChange(android_view_renderer)

/**
 * Used to render an android view to a native open GL texture that can then be rendered by open GL.
 *
 * <p>This is done by using an Android SurfaceTexture which is rendered to directly by creating a
 * VirtualDisplay that the view is presented on instead of drawing the view to the main display
 * using the activity.
 *
 * @hide
 */
@UsedByNative("android_view_renderer.cc")
public class RenderViewToSurfaceTexture implements View.OnLayoutChangeListener {
  private static final String PICO_DEVICE_MANUFACTURER = "Pico";
  private final Context context;
  private final View view;
  private final Surface surface;
  private final long nativeAndroidViewRenderer;

  private VirtualDisplay virtualDisplay;
  private Presentation presentation;

  // Track the size of the virtual display to ensure that we only resize it when we actually need
  // to since onLayout events can occur without the size actually changing, especially when the view
  // is attached for the first time and the size is initially set. This is tracked manually because
  // VirtualDisplay does not have accessors for the information.
  private int virtualDisplayWidth;
  private int virtualDisplayHeight;
  private float virtualDisplayCornerRadius;

  @UsedByNative("android_view_renderer.cc")
  public RenderViewToSurfaceTexture(
      Context context,
      View view,
      Surface surface,
      int width,
      int height,
      float cornerRadius,
      long nativeAndroidViewRenderer) {
    this.context = context;
    this.view = view;
    this.surface = surface;
    this.nativeAndroidViewRenderer = nativeAndroidViewRenderer;
    virtualDisplayWidth = width;
    virtualDisplayHeight = height;
    virtualDisplayCornerRadius = cornerRadius;
  }

  @UsedByNative("android_view_renderer.cc")
  public void initialize() {
    view.addOnLayoutChangeListener(this);

    if (virtualDisplayWidth == 0 || virtualDisplayHeight == 0) {
      // Measure the view to determine the initial size the virtual display should be.
      view.measure(0, 0);
      virtualDisplayWidth = view.getMeasuredWidth();
      virtualDisplayHeight = view.getMeasuredHeight();
    }

    setCornerRadius(virtualDisplayCornerRadius);

    nSetRenderViewSurfaceDimensions(
        nativeAndroidViewRenderer, virtualDisplayWidth, virtualDisplayHeight);

    final DisplayManager dm = context.getSystemService(DisplayManager.class);
    final DisplayMetrics metrics = context.getResources().getDisplayMetrics();

    // Create a VirtualDisplay that will render into the Surface from the SurfaceTexture based on
    // the size of the view.
    int displayFlags = DisplayManager.VIRTUAL_DISPLAY_FLAG_OWN_CONTENT_ONLY;
    if (!isPicoDevice()) {
      // This flag does not work on Pico devices.
      // Causes: PxrPresentationManager: no suitable presentation found
      displayFlags |= DisplayManager.VIRTUAL_DISPLAY_FLAG_PRESENTATION;
    }

    // Start the presentation when the virtualDisplay is added.
    DisplayManager.DisplayListener displayListener =
        new DisplayManager.DisplayListener() {
          @Override
          public void onDisplayAdded(int displayId) {
            if (virtualDisplay != null && displayId == virtualDisplay.getDisplay().getDisplayId()) {
              dm.unregisterDisplayListener(this);
              startPresentation();
            }
          }

          @Override
          public void onDisplayRemoved(int displayId) {}

          @Override
          public void onDisplayChanged(int displayId) {}
        };

    dm.registerDisplayListener(displayListener, new Handler(Looper.getMainLooper()));

    virtualDisplay =
        dm.createVirtualDisplay(
            "ImpressAndroidViewVirtualDisplay",
            virtualDisplayWidth,
            virtualDisplayHeight,
            metrics.densityDpi,
            surface,
            displayFlags);
  }

  private void startPresentation() {
    if (presentation != null) {
      return;
    }

    // Creates a Presentation which is necessary to setup a Window & Context that we can add the
    // view to.
    presentation = new Presentation(context, virtualDisplay.getDisplay());
    presentation.getWindow().setBackgroundDrawableResource(android.R.color.transparent);
    presentation.setContentView(view, createWindowLayoutParams());
    presentation.show();
  }

  private static WindowManager.LayoutParams createWindowLayoutParams() {
    WindowManager.LayoutParams params =
        new WindowManager.LayoutParams(
            WindowManager.LayoutParams.WRAP_CONTENT,
            WindowManager.LayoutParams.WRAP_CONTENT,
            WindowManager.LayoutParams.TYPE_PRIVATE_PRESENTATION,
            WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE
                | WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS
                | WindowManager.LayoutParams.FLAG_NOT_TOUCHABLE
                | WindowManager.LayoutParams.FLAG_HARDWARE_ACCELERATED,
            PixelFormat.TRANSLUCENT);
    params.setTitle("ImpressAndroidViewVirtualWindow");

    return params;
  }

  @Override
  public void onLayoutChange(
      View v,
      int left,
      int top,
      int right,
      int bottom,
      int oldLeft,
      int oldTop,
      int oldRight,
      int oldBottom) {
    // Avoid resizing the virtual display if the dimensions are unchanged or zero. Resizing to a
    // zero sized view causes a fatal exception, and can happen during initialization.
    if (view.getWidth() == 0
        || view.getHeight() == 0
        || (virtualDisplayWidth == view.getWidth() && virtualDisplayHeight == view.getHeight())) {
      return;
    }
    final DisplayMetrics metrics = context.getResources().getDisplayMetrics();
    virtualDisplay.resize(view.getWidth(), view.getHeight(), metrics.densityDpi);
    virtualDisplayWidth = view.getWidth();
    virtualDisplayHeight = view.getHeight();
    nSetRenderViewSurfaceDimensions(
        nativeAndroidViewRenderer, virtualDisplayWidth, virtualDisplayHeight);
  }

  @UsedByNative("android_view_renderer.cc")
  public void setCornerRadius(float cornerRadius) {
    this.virtualDisplayCornerRadius = cornerRadius;
    if (this.virtualDisplayCornerRadius > 0) {
      view.setOutlineProvider(
          new ViewOutlineProvider() {
            @Override
            public void getOutline(View view, Outline outline) {
              outline.setRoundRect(
                  0, 0, view.getWidth(), view.getHeight(), virtualDisplayCornerRadius);
            }
          });
      view.setClipToOutline(true);
    } else {
      view.setOutlineProvider(null);
      view.setClipToOutline(false);
    }
  }

  @UsedByNative("android_view_renderer.cc")
  public void release() {
    if (presentation != null) {
      presentation.cancel();
      presentation = null;
    }
    virtualDisplay.release();
  }

  @UsedByNative("android_view_renderer.cc")
  public void dispatchGenericMotionEventToView(MotionEvent originalMotionEvent) {
    MotionEvent motionEvent = createMouseEventWith(originalMotionEvent);
    view.dispatchGenericMotionEvent(motionEvent);
    motionEvent.recycle();
  }

  @UsedByNative("android_view_renderer.cc")
  public void dispatchTouchEventToView(MotionEvent originalMotionEvent) {
    MotionEvent motionEvent = createMouseEventWith(originalMotionEvent);
    view.dispatchTouchEvent(motionEvent);
    motionEvent.recycle();
  }

  private MotionEvent createMouseEventWith(MotionEvent motionEvent) {
    int pointerCount = motionEvent.getPointerCount();
    ArrayList<MotionEvent.PointerProperties> pointerProperties = new ArrayList<>();
    ArrayList<MotionEvent.PointerCoords> pointerCoords = new ArrayList<>();
    for (int i = 0; i < pointerCount; i++) {
      MotionEvent.PointerProperties properties = new MotionEvent.PointerProperties();
      motionEvent.getPointerProperties(i, properties);
      properties.toolType = MotionEvent.TOOL_TYPE_MOUSE;
      pointerProperties.add(properties);

      MotionEvent.PointerCoords coords = new MotionEvent.PointerCoords();
      motionEvent.getPointerCoords(i, coords);
      pointerCoords.add(coords);
    }

    return MotionEvent.obtain(
        motionEvent.getDownTime(),
        motionEvent.getEventTime(),
        motionEvent.getAction(),
        pointerCount,
        pointerProperties.toArray(new MotionEvent.PointerProperties[pointerCount]),
        pointerCoords.toArray(new MotionEvent.PointerCoords[pointerCount]),
        motionEvent.getMetaState(),
        motionEvent.getButtonState(),
        motionEvent.getXPrecision(),
        motionEvent.getYPrecision(),
        motionEvent.getDeviceId(),
        motionEvent.getEdgeFlags(),
        InputDevice.SOURCE_MOUSE,
        motionEvent.getFlags());
  }

  private static boolean isPicoDevice() {
    return !Strings.isNullOrEmpty(Build.MANUFACTURER)
        && Ascii.equalsIgnoreCase(PICO_DEVICE_MANUFACTURER, Build.MANUFACTURER);
  }

  private static native void nSetRenderViewSurfaceDimensions(
      long nativeAndroidViewRenderer, int width, int height);
}

// LINT.ThenChange(
//
// //depot/google3/third_party/impress/core/scripting/message_handlers/android/android_view_renderer.cc
// )
