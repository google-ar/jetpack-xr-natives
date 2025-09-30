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

#include <android/binder_auto_utils.h>
#include <android/binder_ibinder_jni.h>
#include <jni.h>

#include <cassert>
#include <functional>
#include <memory>
#include <string>  // IWYU pragma: keep
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/async/executor.h"
#include "core/common/buffer_access.h"
#include "core/common/enum_flags.h"
#include "core/common/jni_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/common/registry.h"
#include "core/math/transform.h"
#include "core/split_engine/android/extensions/split_engine_bridge.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/android/split_engine_android_shared_memory_bridge.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_sender.h"
#include "core/split_engine/android/view/view_update_params.h"
#include "core/split_engine/split_engine_bridge_sender.h"
#include "core/split_engine/split_engine_serializer_impl.h"
#include "core/view/base_view.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/render/renderable_manager_wrapper.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_helpers.h"
#include "core/view/view_host.h"
#include "core/window/filament_host.h"

#if __ANDROID_API__ >= 34
#include "core/split_engine/android/split_engine_shared_memory_bridge_service.h"
#endif

// TODO: Refactor SplitEngineActivity to use existing View.java
//  class and aggregation like ImpXrRenderer.
#define JNI_METHOD_ACTIVITY(return_type, method_name) \
  IMP_JNI return_type JNICALL                         \
      Java_com_google_ar_imp_view_splitengine_ImpSplitEngineApi_##method_name

#define JNI_METHOD_SERVICE(return_type, method_name) \
  IMP_JNI return_type JNICALL                        \
      Java_com_google_ar_imp_app_splitengine_SplitEngineSharedMemoryBridgeService_##method_name  // NOLINT

using ::imp::GetThreadId;
using ::imp::GetThreadNiceness;
using ::imp::JniAllowlist;
using ::imp::split_engine::SplitEngineBridgeSender;

namespace {

template <class T>
using SplitEngineBridgeServiceAllowlist =
    JniAllowlist<T, imp::ViewHost,
                 imp::split_engine::SplitEngineAndroidSharedMemoryBridge,
                 android_xr::ViewUpdateParams>;

template <class T>
constexpr auto ToJava = &SplitEngineBridgeServiceAllowlist<T>::ToJava;

template <class T>
constexpr auto FromJava = &SplitEngineBridgeServiceAllowlist<T>::FromJava;

// This method is used to convert a ViewProjection into an XrView. The XrView
// contains the pose and FOV of the view for a single eye.
XrView ToXrView(const android_xr::ViewProjection& view_projection) {
  return XrView{.type = XR_TYPE_VIEW,
                .next = nullptr,
                .pose = {.orientation =
                             {
                                 .x = view_projection.pose.rotation.x,
                                 .y = view_projection.pose.rotation.y,
                                 .z = view_projection.pose.rotation.z,
                                 .w = view_projection.pose.rotation.w,
                             },
                         .position =
                             {
                                 .x = view_projection.pose.translation.x,
                                 .y = view_projection.pose.translation.y,
                                 .z = view_projection.pose.translation.z,
                             }},
                .fov = {
                    .angleLeft = view_projection.fov.angle_left,
                    .angleRight = view_projection.fov.angle_right,
                    .angleUp = view_projection.fov.angle_up,
                    .angleDown = view_projection.fov.angle_down,
                }};
}

// This method is used to update the camera in the view host with the new
// ViewUpdateParams. We first create a XrView for both eyes that we feed into
// the helper methods provided by xr_helpers.
void UpdateCamera(imp::ViewHost* view_host,
                  const android_xr::ViewUpdateParams& view_update_params) {
  std::vector<XrView> xr_views = {ToXrView(*view_update_params.left_eye),
                                  ToXrView(*view_update_params.right_eye)};
  filament::View* view = view_host->GetView()->GetHost()->GetView();
  // Set the eye projection matrix on the camera, and the culling matrix.
  imp::SetCustomEyeProjectionOnCamera(&view->getCamera(), xr_views);
  // Set the camera node position to the center of the eyes.
  imp::Transform<float> eye_center_transform =
      imp::GetEyeCenterTransform(xr_views[0], xr_views[1]);
  view_host->GetView()->GetCameraManager().GetCamera()->GetNode()->SetLocalTrs(
      eye_center_transform.AsMat4());
  imp::SetEyeModelMatrixOnCamera(view_host->GetEngine(), &view->getCamera(),
                                 xr_views);
}

}  // namespace

extern "C" {

// LINT.IfChange(api)
JNI_METHOD_ACTIVITY(void, nSetup)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle,
 jobject split_engine_bridge, jlong bridge_buffer_size_bytes) {
  // Make the isolated process slightly lower priority than the main thread.
  imp::SetThreadNiceness(GetThreadId(), GetThreadNiceness(GetThreadId()) + 5);

  auto view_host = FromJava<imp::ViewHost>(view_host_handle);

  imp::BaseView* view = view_host->GetView();
  std::unique_ptr<imp::split_engine::SplitEngineSharedMemoryBridgeClient>
      bridge_client = std::make_unique<imp::split_engine::SplitEngineBridge>(
          env, split_engine_bridge);

  auto bridge_sender =
      std::make_unique<imp::split_engine::SplitEngineSharedMemoryBridgeSender>(
          *bridge_client,
          /*recycle_buffers=*/true);

  auto bridge_one_shot_sender =
      std::make_unique<imp::split_engine::SplitEngineSharedMemoryBridgeSender>(
          *bridge_client,
          /*recycle_buffers=*/false);

  std::unique_ptr<imp::split_engine::SplitEngineAndroidSharedMemoryBridge>
      bridge = std::make_unique<
          imp::split_engine::SplitEngineAndroidSharedMemoryBridge>(
          std::move(bridge_client));

  view->SetRenderableManager(
      std::make_unique<imp::RenderableManagerWrapper>(*view));

  auto split_engine_serializer =
      std::make_unique<imp::split_engine::SplitEngineSerializerImpl>(
          *view, std::move(bridge), std::move(bridge_sender),
          std::move(bridge_one_shot_sender), bridge_buffer_size_bytes);

  view->SetSplitEngineSerializer(std::move(split_engine_serializer));

  THROW_IF_ERROR(env, view_host->Setup(filament::Engine::Backend::NOOP));
  THROW_IF_ERROR(env, view_host->CreateSwapChain(nullptr));
}

JNI_METHOD_ACTIVITY(jlong, nRenderNextFrame)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jlong last_vsync_nanos,
 jlong next_vsync_nanos, jlong camera_update_params_handle) {
  imp::Flags<imp::window::FilamentHost::IsolatedPreRenderFlags>
      pre_render_flags;
  pre_render_flags |=
      imp::window::FilamentHost::IsolatedPreRenderFlags::kNeverRenderDevMode;

  imp::ViewHost* view_host = FromJava<imp::ViewHost>(view_host_handle);
  if (camera_update_params_handle) {
    auto view_update_params =
        FromJava<android_xr::ViewUpdateParams>(camera_update_params_handle);
    UpdateCamera(view_host, *view_update_params);
  }
  imp::Flags<imp::window::FilamentHost::RenderResultFlags> result_flags;
  THROW_IF_ERROR(env, view_host->IsolatedPreRender(
                          absl::Nanoseconds(last_vsync_nanos),
                          absl::Nanoseconds(next_vsync_nanos), &result_flags,
                          nullptr, pre_render_flags));

  THROW_IF_ERROR(env, view_host->IsolatedPostRender(&result_flags));
  return -1;
}
// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/view/splitengine/ImpSplitEngineApi.java:api)

// LINT.IfChange(service)
JNI_METHOD_SERVICE(jobject, nCreateServiceBinder)
(JNIEnv* env, jclass /*clazz*/, jlong viewHandle, jlong executorHandle) {
#if __ANDROID_API__ >= 34
  auto view = reinterpret_cast<imp::BaseView*>(viewHandle);
  auto executor = reinterpret_cast<imp::Executor*>(executorHandle);
  
  

  static imp::split_engine::SplitEngineSharedMemoryBridgeService
      splitEngineSharedMemoryBridgeService(*view, executor);
  splitEngineSharedMemoryBridgeService.Update(*view, executor);
  return env->NewGlobalRef(AIBinder_toJavaBinder(
      env, splitEngineSharedMemoryBridgeService.asBinder().get()));
#else
  THROW_IF_ERROR(
      env, absl::FailedPreconditionError("Android API must be 34 or greater."));
#endif
  return nullptr;
}
// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/app/splitengine/SplitEngineSharedMemoryBridgeService.java:service)
}  // extern "C"
