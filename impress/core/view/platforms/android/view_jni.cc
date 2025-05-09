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

#include <memory>
#include <utility>
#include <vector>

#include "absl/base/call_once.h"
#include "core/common/log.h"
#include "filament/filament/backend/include/backend/Platform.h"
#include "filament/filament/backend/include/private/backend/VirtualMachineEnv.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/async/executor.h"
#include "core/common/context.h"
#include "core/common/enum_flags.h"
#include "core/common/filament_engine_helpers.h"
#include "core/common/jni_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/platform_helpers.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/render_passes/surface_renderer.h"
#include "core/view/base_view.h"
#include "core/view/framework/view.h"
#include "core/view/platforms/android/wrappers/imp_lifecycle_callback.h"
#include "core/view/view_host.h"
#include "core/window/filament_host.h"

#if IMP_PLATFORM(ANDROID)
#include <android/native_window_jni.h>
#endif  // IMP_PLATFORM(ANDROID)

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL Java_com_google_ar_imp_view_View_##method_name

namespace {

using ::imp::BaseView;
using ::imp::Context;
using ::imp::float2;
using ::imp::float3;
using ::imp::JniAllowlist;
using ::imp::OptionalError;
using ::imp::ThrowError;
using ::imp::uint2;
using ::imp::View;
using ::imp::ViewHost;
using ::imp::window::FilamentHost;

template <class T>
inline jlong ToJava(T* p) {
  return JniAllowlist<T, BaseView, ViewHost, filament::Engine,
                      filament::Renderer, filament::View, filament::Scene,
                      imp::Executor>::ToJava(p);
}

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, ViewHost, filament::Engine, filament::Renderer,
                      filament::View, filament::Scene,
                      filament::backend::Platform>::FromJava(n);
}

std::unique_ptr<View> CreateImpressView(JNIEnv* env, jobject context,
                                        jstring identifier,
                                        jobject fragment_host) {
  IMP_TRACE();
  JavaVM* vm;
  jint status = env->GetJavaVM(&vm);
  if (status != JNI_OK) {
    IMP_LOG(imp::FATAL) << "Failed to get java VM " << status;
  }

  // We must assign the JavaVM to filament. This is required for certain
  // feature to work like creating a Filament texture from an android
  // SurfaceTexture.
  //
  // Filament normally does this by defining a global JNI_OnLoad method in its
  // android JNI library that is called automatically when the .so is loaded by
  // System.loadLibrary. Impress doesn't use this for two reasons:
  //   1. Filament's android JNI library introduces ~120kb of extra binary size
  //   that we don't need since we only call into filament from C++.
  //   2. Impress doesn't want to define a global JNI_OnLoad method to avoid
  //   causing duplicate symbol issues with other libraries that define a
  //   JNI_OnLoad.
  //
  // Instead, Impress explicitly passes the JavaVM into filament here.
  // TODO: Move this call to SharedHostState where the Filament
  // Engine is actually created. Currently, that code doesn't have access to the
  // JavaVM.
  static absl::once_flag jni_on_load_once_flag;
  absl::call_once(jni_on_load_once_flag,
                  [vm]() { ::filament::VirtualMachineEnv::JNI_OnLoad(vm); });

  std::unique_ptr<View> view = imp::View::CreateClient(
      std::make_unique<Context>(vm, context, fragment_host),
      imp::GetString(env, identifier));

  return view;
}

}  // namespace

extern "C" {
// LINT.IfChange(api)

JNI_METHOD(jlong, nCreateView)
(JNIEnv* env, jclass /*clazz*/, jobject context, jstring identifier,
 jobject fragment_host) {
  std::unique_ptr<View> view =
      CreateImpressView(env, context, identifier, fragment_host);
  auto view_host = std::make_unique<ViewHost>(std::move(view));

  return ToJava(view_host.release());
}

JNI_METHOD(jlong, nCreateViewWithoutHost)
(JNIEnv* env, jclass /*clazz*/, jobject context, jstring identifier) {
  std::unique_ptr<View> view =
      CreateImpressView(env, context, identifier, nullptr);
  return ToJava(static_cast<BaseView*>(view.release()));
}

JNI_METHOD(void, nDestroyView)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  auto view_host =
      std::unique_ptr<ViewHost>(FromJava<ViewHost>(view_host_handle));
  THROW_IF_ERROR(env, view_host->Cleanup());
  // Note: view deleted on scope exit.
}

JNI_METHOD(jlong, nGetViewHandle)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  auto* view_host = FromJava<ViewHost>(view_host_handle);
  return ToJava(view_host->GetView());
}

JNI_METHOD(void, nSetLifeCycleCallback)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jobject callback) {
#if IMP_PLATFORM(ANDROID)
  auto* view_host = FromJava<ViewHost>(view_host_handle);
  view_host->GetView()->GetRegistry().GetOrCreate<imp::ImpLifeCycleCallback>(
      *view_host->GetView(), callback);
#else
  ThrowError(env, imp::Error("Not supported"));
#endif  // IMP_PLATFORM(ANDROID)
}

JNI_METHOD(void, nCreateSwapChain)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jobject surface,
 jlong flags) {
  IMP_TRACE();
#if IMP_PLATFORM(ANDROID)
  THROW_IF_ERROR(
      env, FromJava<ViewHost>(view_host_handle)
               ->CreateSwapChain(surface != nullptr
                                     ? ANativeWindow_fromSurface(env, surface)
                                     : nullptr,
                                 flags));
#else
  ThrowError(env, imp::Error("Not supported"));
#endif  // IMP_PLATFORM(ANDROID)
}

JNI_METHOD(void, nDestroySwapChain)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  THROW_IF_ERROR(env, FromJava<ViewHost>(view_host_handle)->DestroySwapChain());
}

JNI_METHOD(jboolean, nHasSwapChain)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  return FromJava<ViewHost>(view_host_handle)->HasSwapChain();
}

JNI_METHOD(void, nSetDisplayRotation)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jint orientation) {
  IMP_TRACE();
  THROW_IF_ERROR(env, FromJava<ViewHost>(view_host_handle)
                          ->SetDisplayRotation(
                              imp::window::ToWindowRotation(orientation)));
}

JNI_METHOD(void, nResize)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jint width, jint height,
 jfloat subpixel_ratio_x, jfloat subpixel_ratio_y) {
  IMP_TRACE();
  auto pixel_dimensions = uint2{width, height};
  auto subpixel_ratio = float2{subpixel_ratio_x, subpixel_ratio_y};
  FromJava<ViewHost>(view_host_handle)
      ->Resize(pixel_dimensions, subpixel_ratio);
}

JNI_METHOD(void, nSetup)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jlong platform_handle,
 jlong egl_context) {
  IMP_TRACE();
  auto* view_host = FromJava<ViewHost>(view_host_handle);
  auto* platform = FromJava<filament::backend::Platform>(platform_handle);

#if IMP_MATERIAL_API(OPENGL)
  THROW_IF_ERROR(env, view_host->Setup(filament::Engine::Backend::OPENGL,
                                       platform, (void*)egl_context));
#elif IMP_MATERIAL_API(VULKAN)
  THROW_IF_ERROR(env, view_host->Setup(filament::Engine::Backend::VULKAN,
                                       platform, nullptr));
#else
  (void)view_host;
  (void)platform;
  ThrowError(env, imp::Error("The backend is not supported on Android"));
#endif
}

JNI_METHOD(void, nSetupShared)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jlong engine_handle,
 jlong renderer_handle, jlong filament_view_handle, jlong scene_handle) {
  IMP_TRACE();
  THROW_IF_ERROR(env,
                 FromJava<ViewHost>(view_host_handle)
                     ->Setup(FromJava<filament::Engine>(engine_handle),
                             FromJava<filament::Renderer>(renderer_handle),
                             FromJava<filament::View>(filament_view_handle),
                             FromJava<filament::Scene>(scene_handle)));
}

JNI_METHOD(void, nFlushAndWait)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  assert(FromJava<ViewHost>(view_host_handle)->GetEngine() ==
         imp::BaseView::GetSharedEngine());
  auto* engine = imp::BaseView::GetSharedEngine();
  if (!engine) {
    ThrowError(env, imp::Error("Tried to flush with no engine"));
  }
  imp::FlushEngineAndWait(engine);
}

JNI_METHOD(void, nSynchronizePendingFrames)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  assert(FromJava<ViewHost>(view_host_handle)->GetEngine() ==
         imp::BaseView::GetSharedEngine());
  auto* engine = imp::BaseView::GetSharedEngine();
  if (!engine) {
    ThrowError(env, imp::Error("Tried to synchronize frames with no engine"));
  }
  imp::SynchronizePendingFrames(engine);
}

// Called as early as possible once work on a frame has started.
JNI_METHOD(void, nCaptureVsyncTime)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  FromJava<ViewHost>(view_host_handle)->CaptureVsyncTime();
}

JNI_METHOD(jlong, nRenderNextFrame)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jlong last_vsync_nanos,
 jlong next_vsync_nanos) {
  IMP_TRACE();
  absl::StatusOr<FilamentHost::RenderResult> result =
      FromJava<ViewHost>(view_host_handle)
          ->RenderNextFrame(absl::Nanoseconds(last_vsync_nanos),
                            absl::Nanoseconds(next_vsync_nanos));
  if (!result.ok()) {
    imp::ThrowError(env, result.status());
    return 0;
  }
  if (result->flags.Test(FilamentHost::RenderResultFlags::kSkippedRender)) {
    // 0 indicates the frame was skipped with no retry time.
    // > 0 indicate how man ms to wait until retrying. These values are not
    // intuitive, but this allows us to avoid allocating a java object on the
    // heap every frame to represent the result, which is acceptable given that
    // this method is an implementation detail.
    return result->time_until_retry.has_value()
               ? absl::ToInt64Milliseconds(result->time_until_retry.value())
               : 0;
  }

  // -1 indicates the frame succeeded.
  return -1;
}

JNI_METHOD(void, nIsolatedPreRender)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jlong last_vsync_nanos,
 jlong next_vsync_nanos) {
  IMP_TRACE();
  FilamentHost::RenderResult render_result;
  THROW_IF_ERROR(env,
                 FromJava<ViewHost>(view_host_handle)
                     ->IsolatedPreRender(absl::Nanoseconds(last_vsync_nanos),
                                         absl::Nanoseconds(next_vsync_nanos),
                                         &render_result.flags,
                                         &render_result.time_until_retry));
}

JNI_METHOD(void, nIsolatedPostRender)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  imp::Flags<FilamentHost::RenderResultFlags> flags;
  THROW_IF_ERROR(
      env, FromJava<ViewHost>(view_host_handle)->IsolatedPostRender(&flags));
}

JNI_METHOD(void, nOnResume)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  THROW_IF_ERROR(env, FromJava<ViewHost>(view_host_handle)->Resume());
}

JNI_METHOD(void, nOnPause)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  THROW_IF_ERROR(env, FromJava<ViewHost>(view_host_handle)->Pause());
}

JNI_METHOD(void, nOnDragBegin)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  THROW_IF_ERROR(env, FromJava<ViewHost>(view_host_handle)
                          ->QueueMouseInput(FilamentHost::DragBegin{}));
}
JNI_METHOD(void, nOnDrag)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jfloat position_x,
 jfloat position_y, jfloat travel_x, jfloat travel_y) {
  IMP_TRACE();
  THROW_IF_ERROR(env, FromJava<ViewHost>(view_host_handle)
                          ->QueueMouseInput(FilamentHost::Drag{
                              imp::int2{position_x, position_y},
                              imp::int2{travel_x, travel_y}}));
}
JNI_METHOD(void, nOnDragEnd)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  THROW_IF_ERROR(env, FromJava<ViewHost>(view_host_handle)
                          ->QueueMouseInput(FilamentHost::DragEnd{}));
}

JNI_METHOD(void, nOnScroll)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jfloat travel_x,
 jfloat travel_y) {
  IMP_TRACE();
  const float kScale = 0.05f;
  THROW_IF_ERROR(env, FromJava<ViewHost>(view_host_handle)
                          ->QueueMouseInput(FilamentHost::Wheel{imp::int2{
                              travel_x * kScale, travel_y * kScale}}));
}

JNI_METHOD(void, nDrainAllExecutorsForTest)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  auto* view_host = FromJava<ViewHost>(view_host_handle);
  view_host->DrainAllExecutorsForTest();
}

JNI_METHOD(void, nStaticRenderForTest)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle) {
  IMP_TRACE();
  auto* view_host = FromJava<ViewHost>(view_host_handle);
  auto status = view_host->StaticRenderForTest();
  if (!status.ok()) {
    ThrowError(env, imp::Error(std::string(status.message()).c_str()));
  }
}

// TODO: it would probably take more refactoring but it would be
// nice if SetScriptEndpoint could take a ScriptEndpoint interface or
// something, and the JniWrapper for the jobject is an implementation of the
// interface that gets created in view_jni instead of web_view.cc.
JNI_METHOD(void, nSetScriptEndpoint)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle,
 jobject script_endpoint) {
  IMP_TRACE();
  auto* view_host = FromJava<ViewHost>(view_host_handle);
  view_host->GetView()->SetScriptEndpoint(script_endpoint);
}

JNI_METHOD(jboolean, nSetupSurfaceRenderer)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jobject surface,
 jstring visibility_group, jstring camera_name) {
  IMP_TRACE();
#if IMP_PLATFORM(ANDROID)
  auto* view_host = FromJava<imp::ViewHost>(view_host_handle);
  void* native_window = ANativeWindow_fromSurface(env, surface);
  imp::NodeHandle node = view_host->GetView()->CreateNode();

  std::string camera_name_string = imp::GetString(env, camera_name);

  std::optional<absl::string_view> camera_name_string_view = std::nullopt;
  if (camera_name) {
    camera_name_string_view = camera_name_string;
  }

  return node
      ->AddComponent<imp::SurfaceRenderer>(
          native_window,
          std::optional<absl::string_view>(
              imp::GetString(env, visibility_group)),
          camera_name_string_view)
      .ok();
#else
  ThrowError(env, imp::Error("Not supported"));
  return false;
#endif  // IMP_PLATFORM(ANDROID)
}

JNI_METHOD(jlong, nGetForegroundExecutor)
(JNIEnv* env, jclass /*clazz*/) {
  return ToJava(imp::Executor::ForegroundExecutor());
}

// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/view/View.java:api
// )
}  // extern "C"
