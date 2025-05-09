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

#include <jni.h>

#include <cassert>
#include <functional>
#include <memory>
#include <string>  // IWYU pragma: keep
#include <utility>
#include <vector>

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
#include "core/view/framework/camera/camera_manager.h"

#if __ANDROID_API__ >= 34
#include <android/binder_auto_utils.h>
#include <android/binder_ibinder_jni.h>

#include "core/split_engine/android/split_engine_shared_memory_bridge_client_ndk.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_service.h"
#endif

#include "core/split_engine/android/message_group_id_mapper_impl.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/android/split_engine_android_shared_memory_bridge.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_sender.h"
#include "core/split_engine/android/view/view_update_params.h"
#include "core/split_engine/split_engine_bridge_sender.h"
#include "core/split_engine/split_engine_serializer_impl.h"
#include "core/view/base_view.h"
#include "core/view/framework/render/renderable_manager_wrapper.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_helpers.h"
#include "core/view/view_host.h"
#include "core/window/filament_host.h"

// TODO: Refactor SplitEngineActivity to use existing View.java
//  class and aggregation like ImpXrRenderer.
#define JNI_METHOD_ACTIVITY(return_type, method_name) \
  IMP_JNI return_type JNICALL                         \
      Java_com_google_ar_imp_view_splitengine_ImpSplitEngineApi_##method_name

#define JNI_METHOD_SERVICE(return_type, method_name) \
  IMP_JNI return_type JNICALL                        \
      Java_com_google_ar_imp_app_splitengine_SplitEngineSharedMemoryBridgeService_##method_name  // NOLINT

#define JNI_METHOD_PROVIDER(return_type, method_name) \
  IMP_JNI return_type JNICALL                         \
      Java_com_google_ar_imp_view_splitengine_SplitEngineBridgeServiceProvider_##method_name  // NOLINT

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

class SplitEngineBridge : public imp::JavaWrapper {
 public:
  SplitEngineBridge(JNIEnv* env, jobject split_engine_bridge)
      : JavaWrapper(env, split_engine_bridge) {
    native_handle_ = GetFieldHandle("mNativeHandle", "J");
    assert(native_handle_);
  }

  SplitEngineBridge(
      JNIEnv* env,
      std::unique_ptr<imp::split_engine::SplitEngineAndroidSharedMemoryBridge>
          split_engine_shared_memory_bridge)
      : JavaWrapper(env,
                    // TODO: Change this back to the new path:
                    // com/android/extensions/xr/splitengine/SplitEngineBridge
                    // once the JNI issue is resolved.
                    "androidx/xr/extensions/splitengine/SplitEngineBridge",
                    "()V") {
    native_handle_ = GetFieldHandle("mNativeHandle", "J");
    assert(native_handle_);
    Env()->SetLongField(
        Self(), ToFieldID(native_handle_),
        reinterpret_cast<jlong>(split_engine_shared_memory_bridge.release()));
  }

  imp::split_engine::SplitEngineAndroidSharedMemoryBridge* GetBridge() {
    jlong handle = Env()->GetLongField(Self(), ToFieldID(native_handle_));
    return FromJava<imp::split_engine::SplitEngineAndroidSharedMemoryBridge>(
        handle);
  };

  imp::JniHandle native_handle_;
};

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

  // SplitEngineBridge is used to extract the native pointer to a
  // SplitEngineAndroidSharedMemoryBridge, which was allocated by XROS or on the
  // phone. The ownership of this memory is transferred to Impress to manage.
  SplitEngineBridge bridge_wrapper(env, split_engine_bridge);

  std::unique_ptr<imp::split_engine::SplitEngineAndroidSharedMemoryBridge>
      bridge(bridge_wrapper.GetBridge());
  bridge_wrapper.Release();

  auto bridge_sender =
      std::make_unique<imp::split_engine::SplitEngineSharedMemoryBridgeSender>(
          bridge->GetSplitEngineSharedMemoryBridgeClient(),
          std::make_unique<imp::split_engine::MessageIdMapperImpl>(
              /*map_to_high_word=*/false),
          /*recycle_buffers=*/true);
  auto bridge_one_shot_sender =
      std::make_unique<imp::split_engine::SplitEngineSharedMemoryBridgeSender>(
          bridge->GetSplitEngineSharedMemoryBridgeClient(),
          std::make_unique<imp::split_engine::MessageIdMapperImpl>(
              /*map_to_high_word=*/true),
          /*recycle_buffers=*/false);

  view->SetRenderableManager(
      std::make_unique<imp::RenderableManagerWrapper>(*view));

  auto split_engine_serializer =
      std::make_unique<imp::split_engine::SplitEngineSerializerImpl>(
          *view, std::move(bridge_sender), std::move(bridge_one_shot_sender),
          bridge_buffer_size_bytes);

  view->SetSplitEngineSerializer(std::move(split_engine_serializer));

  imp::split_engine::SplitEngineAndroidBridge* bridge_ptr = bridge.get();
  view->GetRegistry().Register<imp::split_engine::SplitEngineAndroidBridge>(
      std::move(bridge));

  THROW_IF_ERROR(env, view_host->Setup(filament::Engine::Backend::NOOP));
  THROW_IF_ERROR(env, view_host->CreateSwapChain(nullptr));

  imp::Executor* foreground_executor = imp::Executor::ForegroundExecutor();
  if (!foreground_executor) {
    IMP_LOG(imp::FATAL) << "Failed to get the foreground executor!";
  }
  bridge_ptr->GetSplitEngineSharedMemoryBridgeClient().Initialize(
      [foreground_executor](std::function<void()> work_item) {
        if (!work_item) return;
        foreground_executor->Schedule(work_item);
      });
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

// LINT.IfChange(provider)
// This method is called by the phone version of the split engine renderer, and
// is used to create a SplitEngineBridge outside of XROS, primarily as a
// development tool.
JNI_METHOD_PROVIDER(jobject, nCreateBridge)
(JNIEnv* env, jclass /*clazz*/, jobject serviceBinder) {
#if __ANDROID_API__ >= 34
  ndk::SpAIBinder native_binder =
      ndk::SpAIBinder(AIBinder_fromJavaBinder(env, serviceBinder));
  auto client = std::make_unique<
      imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk>(native_binder,
                                                                 env);

  // This pointer is passed up to the Java layer, and will be passed back to
  // Impress to take ownership of.
  auto bridge =
      std::make_unique<imp::split_engine::SplitEngineAndroidSharedMemoryBridge>(
          std::move(client));
  SplitEngineBridge bridge_wrapper(env, std::move(bridge));
  return bridge_wrapper.Release();
#else
  THROW_IF_ERROR(
      env, absl::FailedPreconditionError("Android API must be 34 or greater."));
  return nullptr;
#endif
}
// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/view/splitengine/SplitEngineBridgeServiceProvider.java:provider)

}  // extern "C"
