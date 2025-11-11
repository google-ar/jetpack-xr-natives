// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <jni.h>

#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "core/common/jni_helpers.h"
#include "core/monitor/monitor.h"
#include "core/render/content_security_level.h"
#include "core/view/framework/view.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_action_controller.h"
#include "core/view/platforms/xr_android/xr_helpers.h"
#include "core/view/platforms/xr_android/xr_session_host.h"
#include "mediapipe/framework/port/status_macros.h"

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_view_xr_ImpXrApi_##method_name

namespace {
using ::imp::JniAllowlist;
using ::imp::ThrowError;
using ::imp::View;
using ::imp::XrSessionHost;

template <class T>
inline jlong ToJava(T* p) {
  return JniAllowlist<T, XrSessionHost>::ToJava(p);
}

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, XrSessionHost, imp::View>::FromJava(n);
}

absl::Status InitializeLoader(JNIEnv* env, jobject context) {
  // This is a necessary step to initialize the OpenXR loader. Without this, it
  // won't be possible to create an XrInstance.
  PFN_xrInitializeLoaderKHR initialize_loader = nullptr;
  MP_RETURN_IF_ERROR(imp::ToStatus(
      XR_NULL_HANDLE,
      xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                            (PFN_xrVoidFunction*)(&initialize_loader))));

  JavaVM* app_vm = nullptr;
  env->GetJavaVM(&app_vm);

  const XrLoaderInitInfoAndroidKHR loader_init_info_android = {
      .type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR,
      .next = nullptr,
      .applicationVM = app_vm,
      .applicationContext = context,
  };

  return imp::ToStatus(
      XR_NULL_HANDLE,
      initialize_loader(reinterpret_cast<const XrLoaderInitInfoBaseHeaderKHR*>(
          &loader_init_info_android)));
}

XrFoveationLevelFB XrFoveationLevelFBFromInt(int foveation_level) {
  
  switch (foveation_level) {
    case 1:
      return XR_FOVEATION_LEVEL_LOW_FB;
    case 2:
      return XR_FOVEATION_LEVEL_MEDIUM_FB;
    case 3:
      return XR_FOVEATION_LEVEL_HIGH_FB;
    default:
    case 0:
      return XR_FOVEATION_LEVEL_NONE_FB;
  }
}

}  // namespace

extern "C" {
// LINT.IfChange

JNI_METHOD(jlong, nCreateSessionHost)
(JNIEnv* env, jclass /*clazz*/, jobject context, jlong view_handle,
 jboolean use_composition_layer_depth,
 jboolean use_enhanced_stereoscopic_rendering, jboolean use_max_swapchain_size,
 jint foveation_level, jboolean use_quad_views, jboolean use_mono_view,
 jboolean use_varjo_foveated_rendering, jint msaa_sample_count,
 jlong openxr_reference_space_type, jboolean use_eye_gaze_interaction,
 jboolean use_android_depth_texture, jboolean use_xr_action_defaults,
 jboolean use_fb_color_space, jboolean enable_android_system_extensions,
 jfloat swapchain_size_multiplier,
 jboolean use_global_passthrough_dimming_extensions) {
  context = env->NewGlobalRef(context);
  absl::Status status = InitializeLoader(env, context);
  if (!status.ok()) {
    ThrowError(env, status);
  }
  auto view = std::unique_ptr<View>(FromJava<View>(view_handle));

  XrSessionHost::XrSessionHostOptions options;
  options.use_composition_layer_depth = use_composition_layer_depth;
  options.use_enhanced_stereoscopic_rendering =
      use_enhanced_stereoscopic_rendering;
  options.use_max_swapchain_size = use_max_swapchain_size;
  options.foveation_level = XrFoveationLevelFBFromInt(foveation_level);
  options.use_quad_views = use_quad_views;
  options.use_mono_view = use_mono_view;
  options.use_varjo_foveated_rendering = use_varjo_foveated_rendering;
  options.msaa_sample_count = msaa_sample_count;
  options.reference_space_type =
      static_cast<XrReferenceSpaceType>(openxr_reference_space_type);
  options.use_eye_gaze_interaction = use_eye_gaze_interaction;
  options.use_android_depth_texture = use_android_depth_texture;
  options.use_fb_color_space = use_fb_color_space;
  options.enable_android_system_extensions = enable_android_system_extensions;
  options.swapchain_size_multiplier = swapchain_size_multiplier;
  options.use_global_passthrough_dimming_extensions =
      use_global_passthrough_dimming_extensions;

  auto session_host = std::make_unique<XrSessionHost>(std::move(view), options);
  auto& xr_action_controller =
      session_host->GetView()
          ->GetRegistry()
          .GetOrCreate<imp::XrActionController>(*session_host);
  if (use_xr_action_defaults == JNI_TRUE) {
    xr_action_controller.SetXrSessionActionConfig(
        imp::XrActionController::XrSessionActionConfig::kUseXrActionDefaults);
  } else if (use_xr_action_defaults == JNI_FALSE) {
    xr_action_controller.SetXrSessionActionConfig(
        imp::XrActionController::XrSessionActionConfig::kOmitXrActionDefaults);
  }
  return ToJava(session_host.release());
}

JNI_METHOD(void, nSetup)
(JNIEnv* env, jclass /*clazz*/, jobject context, jlong view_host_handle) {
  XrSessionHost* host = FromJava<XrSessionHost>(view_host_handle);

  JavaVM* app_vm = nullptr;
  env->GetJavaVM(&app_vm);

  absl::Status status = host->Setup(env, app_vm, context);
  if (!status.ok()) {
    ThrowError(env, status);
  }
}

JNI_METHOD(void, nOnWindowAttached)
(JNIEnv* env, jclass /*clazz*/, jobject context, jlong view_host_handle) {
  XrSessionHost* host = FromJava<XrSessionHost>(view_host_handle);

  absl::Status status = host->onWindowAttached();
  if (!status.ok()) {
    ThrowError(env, status);
  }
}

JNI_METHOD(void, nAdvanceFrame)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  XrSessionHost* host = FromJava<XrSessionHost>(view_host_handle);
  absl::Status status = host->AdvanceFrame();
  if (!status.ok()) {
    ThrowError(env, status);
  }
}

JNI_METHOD(jstring, nDump)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  XrSessionHost* host = FromJava<XrSessionHost>(view_host_handle);
  imp::Monitor* monitor = host->GetMonitor();
  if (monitor == nullptr) {
    ThrowError(env, absl::InternalError("Monitor is null"));
  }
  return env->NewStringUTF(
      DumpXrFrameTiming(*monitor, host->GetXrTimingSummary()).c_str());
}

JNI_METHOD(void, nHide)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  XrSessionHost* host = FromJava<XrSessionHost>(view_host_handle);
  absl::Status status =
      host->SetShownState(imp::XrSessionHost::ShownState::kHidden);
  if (!status.ok()) {
    ThrowError(env, status);
  }
}

JNI_METHOD(void, nShow)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  XrSessionHost* host = FromJava<XrSessionHost>(view_host_handle);
  absl::Status status =
      host->SetShownState(imp::XrSessionHost::ShownState::kShown);
  if (!status.ok()) {
    ThrowError(env, status);
  }
}

JNI_METHOD(void, nEnableDisplay)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  XrSessionHost* host = FromJava<XrSessionHost>(view_host_handle);
  absl::Status status =
      host->SetDisplayState(imp::XrHelpers::DisplayState::kDisplayEnabled);
  if (!status.ok()) {
    ThrowError(env, status);
  }
}

JNI_METHOD(void, nDisableDisplay)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  XrSessionHost* host = FromJava<XrSessionHost>(view_host_handle);
  absl::Status status =
      host->SetDisplayState(imp::XrHelpers::DisplayState::kDisplayDisabled);
  if (!status.ok()) {
    ThrowError(env, status);
  }
}

JNI_METHOD(void, nSetDrmProtectionModeEnabled)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jboolean enabled) {
  XrSessionHost* host = FromJava<XrSessionHost>(view_host_handle);
  host->SetContentSecurityLevel(enabled ? imp::ContentSecurityLevel::kProtected
                                        : imp::ContentSecurityLevel::kNone);
}

// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/view/xr/ImpXrApi.java)
}
