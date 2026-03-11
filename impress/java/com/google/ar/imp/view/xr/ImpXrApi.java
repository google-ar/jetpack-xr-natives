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
import androidx.annotation.Nullable;
import com.google.ar.imp.view.SetupParams;
import com.google.ar.imp.view.View;
import com.google.common.base.Strings;

/**
 * Used to call into native code for Impress OpenXR integration.
 *
 * <p>Creates and wraps the Impress View Jni object.
 */
final class ImpXrApi {
  /**
   * Default name of the Impress .so for Xr builds of Impress
   *
   * <p>This is different from the normal default to avoid duplicate .so targets when building with
   * imp_app.bzl
   */
  // LINT.IfChange
  private static final String DEFAULT_LIBRARY_NAME = "imp_view_xr_jni";

  // LINT.ThenChange(//depot/google3/third_party/impress/build_tools/imp.bzl)

  private final Context context;
  private final View view;

  private static int intFromFoveationLevel(SetupParams.FoveationLevel level) {
    return switch (level) {
      case LOW -> 1;
      case MEDIUM -> 2;
      case HIGH -> 3;
      default -> 0;
    };
  }

  /** Initialize Impress for OpenXR. */
  static ImpXrApi create(Context context, SetupParams setupParams) {
    // If no native library name has been specified, use the correct one for Xr builds.
    @Nullable String nativeLibrary = setupParams.getCustomNativeLibrary();
    if (Strings.isNullOrEmpty(nativeLibrary)) {
      nativeLibrary = DEFAULT_LIBRARY_NAME;
    }

    // For OpenXR, we use a special subclass of ViewHost called XrSessionHost that adds OpenXR
    // specific functionality for running the frame loop and rendering.
    //
    // Create the Impress View Jni object with a special path that allows us to provide this custom
    // subclass of ViewHost.
    // TODO: (broken link) - serialize setupParams instead of passing each param individually.
    View view =
        View.createViewWithCustomHost(
            nativeLibrary,
            setupParams.getViewIdentifier(),
            context,
            (long viewHandle) ->
                nCreateSessionHost(
                    context,
                    viewHandle,
                    setupParams.getEnableCompositionLayerDepth(),
                    setupParams.getUseEnhancedStereoscopicRendering(),
                    setupParams.getUseMaxSwapchainSize(),
                    intFromFoveationLevel(setupParams.getFoveationLevel()),
                    setupParams.getUseQuadViews(),
                    setupParams.getUseMonoView(),
                    setupParams.getUseVarjoFoveatedRendering(),
                    setupParams.getMsaaSampleCount(),
                    setupParams.getOpenxrReferenceSpaceType(),
                    setupParams.getUseEyeGazeInteraction(),
                    setupParams.getUseAndroidDepthTexture(),
                    setupParams.getUseXrActionDefaults(),
                    setupParams.getUseFbColorSpace(),
                    setupParams.getEnableAndroidSystemExtensions(),
                    setupParams.getSwapchainSizeMultiplier(),
                    setupParams.getUseGlobalPassthroughDimmingExtensions(),
                    setupParams.getUseEyeTrackingCalibration()));

    // Calls XrSessionHost::Setup, which sets up Impress with the custom XrPlatform.
    // This doesn't actually initialize OpenXR yet. That happens in onWindowAttached.
    nSetup(context, view.getViewHostHandle());

    return new ImpXrApi(context, view);
  }

  private ImpXrApi(Context context, View view) {
    this.context = context;
    this.view = view;
  }

  /** Returns the underlying Impress View Jni object. */
  public View getView() {
    return view;
  }

  public void onWindowAttached() {
    nOnWindowAttached(context, view.getViewHostHandle());
  }

  public void advanceFrame() {
    if (view.hasSwapChain()) {
      nAdvanceFrame(view.getViewHostHandle());
    }
  }

  public void destroy() {
    view.destroy();
  }

  public String dump() {
    return nDump(view.getViewHostHandle());
  }

  public void show() {
    nShow(view.getViewHostHandle());
  }

  public void hide() {
    nHide(view.getViewHostHandle());
  }

  public void disableDisplay() {
    nDisableDisplay(view.getViewHostHandle());
  }

  public void enableDisplay() {
    nEnableDisplay(view.getViewHostHandle());
  }

  public void setDrmProtectionModeEnabled(boolean enabled) {
    nSetDrmProtectionModeEnabled(view.getViewHostHandle(), enabled);
  }

  // LINT.IfChange

  private static native long nCreateSessionHost(
      Object context,
      long viewHandle,
      boolean useCompositionLayerDepth,
      boolean useInstancedRendering,
      boolean useMaxSwapchainSize,
      int foveationLevel,
      boolean useQuadViews,
      boolean useMonoView,
      boolean useVarjoFoveatedRendering,
      int msaaSampleCount,
      long openXrReferenceSpaceType,
      boolean useEyeGazeInteraction,
      boolean useAndroidDepthTexture,
      boolean useXrActionDefaults,
      boolean useFbColorSpace,
      boolean enableAndroidSystemExtensions,
      float swapchainSizeMultiplier,
      boolean useGlobalPassthroughDimmingExtensions,
      boolean useEyeTrackingCalibration);

  private static native void nSetup(Object context, long viewHostHandle);

  private static native void nOnWindowAttached(Object context, long viewHostHandle);

  private static native void nAdvanceFrame(long viewHostHandle);

  private static native String nDump(long viewHostHandle);

  private static native void nHide(long viewHostHandle);

  private static native void nShow(long viewHostHandle);

  private static native void nEnableDisplay(long viewHostHandle);

  private static native void nDisableDisplay(long viewHostHandle);

  private static native void nSetDrmProtectionModeEnabled(long viewHostHandle, boolean enabled);

  // LINT.ThenChange(//depot/google3/third_party/impress/core/view/platforms/xr_android/xr_jni.cc)
}
