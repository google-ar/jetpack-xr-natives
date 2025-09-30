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
import android.content.Intent;
import android.net.Uri;
import android.view.MotionEvent;
import android.view.Surface;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import com.google.ar.imp.core.scripting.ScriptEndpoint;
import com.google.ar.imp.core.web.FragmentHost;
import com.google.ar.imp.view.input.InputManager;
import java.util.concurrent.Executor;

/**
 * JNI bindings for the native imp::View type. Thicker than the normal view API, because we do not
 * yet have JNI bindings for FilamentHost.
 *
 * <p>TODO rename this class to not conflict with Android View.
 */
public class View {
  private static final String TAG = View.class.getSimpleName();
  // Default library to load if none is provided. Matches the name from imp.bzl
  // returned by imp_default_jni_binary_name()
  // LINT.IfChange
  public static final String DEFAULT_LIBRARY_NAME = "imp_view_jni";
  // LINT.ThenChange(//depot/google3/third_party/impress/build_tools/imp.bzl)
  private static final String DEFAULT_VIEW_IDENTIFIER = "default";

  /** Used to provide a custom subclass of imp::ViewHost. */
  public interface CustomHostProvider {
    long createCustomHost(long viewHandle);
  }

  protected long viewHostHandle;
  private Context context;
  private final InputManager inputManager;

  @Nullable private Long renderedFrameTimeNanos = null;

  // Value saved from calls to SetSurfaceRotation.
  private int previousSurfaceRotation = Surface.ROTATION_0;

  private View(long viewHostHandle, Context context, String nativeLibrary) {
    this.viewHostHandle = viewHostHandle;
    this.context = context;
    this.inputManager = new InputManager(viewHostHandle);
  }

  public static View createView(
      @Nullable String nativeLibrary, Context context, Executor callbackExecutor) {
    return createView(nativeLibrary, null, context, null, callbackExecutor, null);
  }

  public static View createView(
      @Nullable String nativeLibrary,
      @Nullable String identifier,
      Context context,
      Executor callbackExecutor) {
    return createView(nativeLibrary, identifier, context, null, callbackExecutor, null);
  }

  public static View createView(
      @Nullable String nativeLibrary,
      @Nullable String identifier,
      Context context,
      @Nullable FragmentHost host,
      Executor callbackExecutor) {
    return createView(nativeLibrary, identifier, context, host, callbackExecutor, null);
  }

  /**
   * It is safe to call this in a separate thread, and in fact you should do so to not have file IO
   * on the Java main UI thread. However, calling setup() and all other API must be on the same
   * thread because it creates the filament::Engine, which has the restriction that all API must be
   * from the same thread that created the engine.
   */
  public static View createView(
      @Nullable String nativeLibrary,
      @Nullable String identifier,
      Context context,
      @Nullable FragmentHost host,
      Executor callbackExecutor,
      // `viewRenderSettingsBytes` is a serialized `imp.render_settings.ViewRenderSettings` proto.
      // A `byte[]` is used instead of the proto message to avoid a dependency on
      // `com.google.protobuf`,
      // which would break `//third_party/impress/apibindings:impress_no_native_lib_aar`.
      @Nullable byte[] viewRenderSettingsBytes) {
    if (nativeLibrary == null || nativeLibrary.isEmpty()) {
      nativeLibrary = DEFAULT_LIBRARY_NAME;
    }
    if (identifier == null || identifier.isEmpty()) {
      identifier = DEFAULT_VIEW_IDENTIFIER;
    }
    try {
      System.loadLibrary(nativeLibrary);
    } catch (UnsatisfiedLinkError e) {
      throw new IllegalStateException("Could not load native library \"" + nativeLibrary + "\"", e);
    }
    return new View(
        nCreateView(context, identifier, host, callbackExecutor, viewRenderSettingsBytes),
        context,
        nativeLibrary);
  }

  public static View createViewWithPreloadedLibrary(
      @Nullable String nativeLibraryName,
      @Nullable String identifier,
      Context context,
      @Nullable FragmentHost host,
      Executor callbackExecutor,
      @Nullable byte[] viewRenderSettingsBytes) {
    if (nativeLibraryName == null || nativeLibraryName.isEmpty()) {
      nativeLibraryName = DEFAULT_LIBRARY_NAME;
    }
    if (identifier == null || identifier.isEmpty()) {
      identifier = DEFAULT_VIEW_IDENTIFIER;
    }

    return new View(
        nCreateView(context, identifier, host, callbackExecutor, viewRenderSettingsBytes),
        context,
        nativeLibraryName);
  }

  // Similar to View.createView, except this allows a custom subclass of imp::ViewHost to be
  // injected with additional functionality.
  public static View createViewWithCustomHost(
      @Nullable String nativeLibrary,
      @Nullable String identifier,
      Context context,
      CustomHostProvider customHostProvider) {
    if (nativeLibrary == null || nativeLibrary.isEmpty()) {
      nativeLibrary = DEFAULT_LIBRARY_NAME;
    }
    if (identifier == null || identifier.isEmpty()) {
      identifier = DEFAULT_VIEW_IDENTIFIER;
    }
    try {
      System.loadLibrary(nativeLibrary);
    } catch (UnsatisfiedLinkError e) {
      throw new IllegalStateException("Could not load native library \"" + nativeLibrary + "\"", e);
    }
    long viewHandle = nCreateViewWithoutHost(context, identifier);
    return new View(customHostProvider.createCustomHost(viewHandle), context, nativeLibrary);
  }

  /** Destroy the View's native instance. */
  public void destroy() {
    nDestroyView(viewHostHandle);
    clearViewHostHandle();
    this.context = null;
  }

  public void createSwapChain(Object surface, long flags) {
    nCreateSwapChain(viewHostHandle, surface, flags);
  }

  public void destroySwapChain() {
    nDestroySwapChain(viewHostHandle);
  }

  public boolean hasSwapChain() {
    return nHasSwapChain(viewHostHandle);
  }

  public void resize(int width, int height, float subpixelRatioX, float subpixelRatioY) {
    nResize(viewHostHandle, width, height, subpixelRatioX, subpixelRatioY);
  }

  // Returns value set by setSurfaceRotation()
  public int getSurfaceRotation() {
    return previousSurfaceRotation;
  }

  // surface_rotation is specified by @c android.view.Surface constants: @c ROTATION_0, @c
  // ROTATION_90, @c ROTATION_180 and @c ROTATION_270
  public void setSurfaceRotation(int surfaceRotation) {
    if (previousSurfaceRotation == surfaceRotation) {
      return;
    }
    previousSurfaceRotation = surfaceRotation;

    nSetDisplayRotation(viewHostHandle, surfaceRotation);
  }

  /**
   * One of the setup functions need to be called before any other API. This function and all others
   * afterward must be from the same thread.
   */
  public void setup(long eglContext) {
    nSetup(viewHostHandle, 0, eglContext);
  }

  /**
   * One of the setup functions need to be called before any other API. This function and all others
   * afterward must be from the same thread.
   */
  public void setup(long platformHandle, long eglContext) {
    nSetup(viewHostHandle, platformHandle, eglContext);
  }

  public void setLifeCycleCallback(Object callback) {
    nSetLifeCycleCallback(viewHostHandle, callback);
  }

  public void flushAndWait() {
    nFlushAndWait(viewHostHandle);
  }

  public void synchronizePendingFrames() {
    nSynchronizePendingFrames(viewHostHandle);
  }

  @Nullable
  public Long getPreviousFrameTimeNanos() {
    return renderedFrameTimeNanos;
  }

  // Call as early as possible once work on a frame has started for filament time management.
  public void captureVsyncTime() {
    nCaptureVsyncTime(viewHostHandle);
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
  public long renderNextFrame(long frameTimeNanos) {
    long result =
        nRenderNextFrame(
            viewHostHandle,
            (renderedFrameTimeNanos != null) ? renderedFrameTimeNanos : frameTimeNanos,
            frameTimeNanos);
    renderedFrameTimeNanos = frameTimeNanos;
    return result;
  }

  // If something else is controlling the Filament render, call this before
  // rendering.
  public void isolatedPreRender(long frameTimeNanos) {
    nIsolatedPreRender(
        viewHostHandle,
        (renderedFrameTimeNanos != null) ? renderedFrameTimeNanos : frameTimeNanos,
        frameTimeNanos);
    renderedFrameTimeNanos = frameTimeNanos;
  }

  // If something else is controlling the Filament render, call this after
  // rendering.
  public void isolatedPostRender() {
    nIsolatedPostRender(viewHostHandle);
  }

  public void onResume() {
    nOnResume(viewHostHandle);
  }

  public void onPause() {
    nOnPause(viewHostHandle);
  }

  public boolean onTouchEvent(MotionEvent e) {
    return inputManager.onTouchEvent(e);
  }

  public void onDragBegin() {
    nOnDragBegin(viewHostHandle);
  }

  public void onDrag(float positionX, float positionY, float travelX, float travelY) {
    nOnDrag(viewHostHandle, positionX, positionY, travelX, travelY);
  }

  public void onDragEnd() {
    nOnDragEnd(viewHostHandle);
  }

  public void onScroll(float travelX, float travelY) {
    nOnScroll(viewHostHandle, travelX, travelY);
  }

  /** Sends the script endpoint from native to enable Java<-->C++ communication. */
  public void setScriptEndpoint(ScriptEndpoint scriptEndpoint) {
    nSetScriptEndpoint(viewHostHandle, scriptEndpoint);
  }

  
  public void staticRenderForTest() {
    nStaticRenderForTest(viewHostHandle);
  }

  
  public Boolean shouldUseSrgbSwapChain() {
    return nShouldUseSrgbSwapChain(viewHostHandle);
  }

  
  public Boolean shouldUseStencilSwapChain() {
    return nShouldUseStencilSwapChain(viewHostHandle);
  }

  public void drainAllExecutorsForTest() {
    nDrainAllExecutorsForTest(viewHostHandle);
  }

  public void setupSurfaceRenderer(Surface surface, String visibilityGroup) {
    setupSurfaceRenderer(surface, visibilityGroup, null);
  }

  public void setupSurfaceRenderer(Surface surface, String visibilityGroup, String cameraName) {
    nSetupSurfaceRenderer(viewHostHandle, surface, visibilityGroup, cameraName);
  }

  public long getForegroundExecutor() {
    return nGetForegroundExecutor();
  }

  @Override
  public String toString() {
    return this.getClass().getSimpleName() + "@" + getViewHostHandle();
  }

  /**
   * Gets the handle of the native view. A value of 0 is returned if there is no valid native view
   * (e.g. it has been destroyed).
   */
  public long getNativeHandle() {
    if (viewHostHandle == 0) {
      return 0;
    }
    return nGetViewHandle(viewHostHandle);
  }

  /** Gets the view host handle. Returns a value of 0 if this View has been destroyed. */
  public long getViewHostHandle() {
    return viewHostHandle;
  }

  void clearViewHostHandle() {
    viewHostHandle = 0;
  }

  public void navigate(String url) {
    Intent navigationIntent = new Intent(Intent.ACTION_VIEW);
    navigationIntent.setData(Uri.parse(url));
    context.startActivity(navigationIntent);
  }

  // LINT.IfChange(api)
  protected static native long nCreateView(
      Object context,
      String identifier,
      Object fragmentHost,
      Object callbackExecutor,
      byte[] viewRenderSettings);

  protected static native long nCreateViewWithoutHost(Object context, String identifier);

  protected static native void nDestroyView(long viewHostHandle);

  protected static native long nGetViewHandle(long viewHostHandle);

  protected static native void nSetLifeCycleCallback(long viewHostHandle, Object callback);

  protected static native void nCreateSwapChain(long viewHostHandle, Object surface, long flags);

  protected static native void nDestroySwapChain(long viewHostHandle);

  protected static native boolean nHasSwapChain(long viewHostHandle);

  protected static native void nResize(
      long viewHostHandle, int width, int height, float subpixelRatioX, float subpixelRatioY);

  protected static native void nSetDisplayRotation(long viewHostHandle, int rotationDegrees);

  protected static native void nSetup(long viewHostHandle, long platformHandle, long eglContext);

  protected static native void nSetupShared(
      long viewHostHandle,
      long engineHandle,
      long rendererHandle,
      long filamentViewHandle,
      long sceneHandle);

  protected static native void nFlushAndWait(long viewHostHandle);

  protected static native void nSynchronizePendingFrames(long viewHostHandle);

  protected static native long nRenderNextFrame(
      long viewHostHandle, long lastVsyncNanos, long nextVsyncNanos);

  protected static native void nIsolatedPreRender(
      long viewHostHandle, long lastVsyncNanos, long nextVsyncNanos);

  protected static native void nIsolatedPostRender(long viewHostHandle);

  protected static native void nOnResume(long viewHostHandle);

  protected static native void nOnPause(long viewHostHandle);

  private static native void nOnDragBegin(long viewHostHandle);

  private static native void nOnDrag(
      long viewHostHandle, float positionX, float positionY, float travelX, float travelY);

  private static native void nOnDragEnd(long viewHostHandle);

  private static native void nOnScroll(long viewHostHandle, float travelX, float travelY);

  private static native void nSetScriptEndpoint(long viewHostHandle, Object scriptEndpoint);

  private static native void nStaticRenderForTest(long viewHostHandle);

  private static native boolean nShouldUseSrgbSwapChain(long viewHostHandle);

  private static native boolean nShouldUseStencilSwapChain(long viewHostHandle);

  private static native void nDrainAllExecutorsForTest(long viewHostHandle);

  private static native void nSetupSurfaceRenderer(
      long viewHostHandle, Object surface, String visibilityGroup, String cameraName);

  private static native long nGetForegroundExecutor();

  private static native void nCaptureVsyncTime(long viewHostHandle);

  // LINT.ThenChange(
  //     //depot/google3/third_party/impress/core/view/platforms/android/view_jni.cc:api
  // )
}
