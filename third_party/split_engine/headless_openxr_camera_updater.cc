// Copyright 2026 Google LLC
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

#include "split_engine/headless_openxr_camera_updater.h"

#include <jni.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "core/camera/camera_component.h"
#include "core/camera/camera_manager.h"
#include "core/common/context.h"
#include "core/math/transform.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/update_system.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

#define XR_USE_PLATFORM_ANDROID 1
#define XR_EXTENSION_PROTOTYPES 1
#define XR_OS_ANDROID 1
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_helpers.h"

namespace imp {
namespace split_engine {

struct HeadlessOpenXrCameraUpdater::OpenXrContext {
  XrInstance instance = XR_NULL_HANDLE;
  XrSystemId system_id = XR_NULL_SYSTEM_ID;
  XrSession session = XR_NULL_HANDLE;
  XrSpace space = XR_NULL_HANDLE;
  XrSpace view_space = XR_NULL_HANDLE;
  bool session_running = false;
};

absl::StatusOr<std::unique_ptr<HeadlessOpenXrCameraUpdater>>
HeadlessOpenXrCameraUpdater::Create(imp::BaseView& view) {
  auto updater = absl::WrapUnique(new HeadlessOpenXrCameraUpdater(view));
  absl::Status status = updater->Setup();
  if (!status.ok()) {
    return status;
  }
  return updater;
}

HeadlessOpenXrCameraUpdater::HeadlessOpenXrCameraUpdater(imp::BaseView& view)
    : imp::UpdateSystem::Updater<HeadlessOpenXrCameraUpdater>(view),
      ctx_(std::make_unique<OpenXrContext>()),
      view_(view) {}

HeadlessOpenXrCameraUpdater::~HeadlessOpenXrCameraUpdater() {
  if (ctx_) {
    if (ctx_->view_space != XR_NULL_HANDLE) {
      xrDestroySpace(ctx_->view_space);
    }
    if (ctx_->space != XR_NULL_HANDLE) {
      xrDestroySpace(ctx_->space);
    }
    if (ctx_->session != XR_NULL_HANDLE) {
      xrDestroySession(ctx_->session);
    }
    if (ctx_->instance != XR_NULL_HANDLE) {
      xrDestroyInstance(ctx_->instance);
    }
    ctx_.reset();
  }
}

absl::Status HeadlessOpenXrCameraUpdater::Setup() {
  const imp::Context& app_context = view_.GetContext();
  JNIEnv* env = app_context.GetJniEnv();
  jobject android_context = app_context.GetActivityContext();

  XrLoaderInitInfoAndroidKHR loader_init = {
      .type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR,
      .next = nullptr,
      .applicationVM = nullptr,
      .applicationContext = android_context,
  };
  if (env != nullptr) {
    env->GetJavaVM(reinterpret_cast<JavaVM**>(&loader_init.applicationVM));
  }

  // OpenXR Loader initialization mapping
  PFN_xrInitializeLoaderKHR initializeLoader = nullptr;
  xrGetInstanceProcAddr(
      XR_NULL_HANDLE, "xrInitializeLoaderKHR",
      reinterpret_cast<PFN_xrVoidFunction*>(&initializeLoader));

  if (initializeLoader) {
    if (initializeLoader(reinterpret_cast<const XrLoaderInitInfoBaseHeaderKHR*>(
            &loader_init)) != XR_SUCCESS) {
      return absl::InternalError("Failed to initialize OpenXR loader");
    }
  }

  const char* extensions[] = {XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME,
                              XR_MND_HEADLESS_EXTENSION_NAME};

  XrInstanceCreateInfoAndroidKHR instance_create_info_android = {
      .type = XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR,
      .applicationVM = loader_init.applicationVM,
      .applicationActivity = android_context,
  };

  XrInstanceCreateInfo instance_create_info = {
      .type = XR_TYPE_INSTANCE_CREATE_INFO,
      .next = &instance_create_info_android,
      .applicationInfo =
          {
              .applicationVersion = 1,
              .engineName = "Impress",
              .engineVersion = 1,
              .apiVersion = XR_MAKE_VERSION(1, 0, 34),
          },
      .enabledExtensionCount = 2,
      .enabledExtensionNames = extensions,
  };
  absl::SNPrintF(instance_create_info.applicationInfo.applicationName,
                 sizeof(instance_create_info.applicationInfo.applicationName),
                 "%s", view_.GetTitle());

  if (xrCreateInstance(&instance_create_info, &ctx_->instance) != XR_SUCCESS) {
    return absl::InternalError("Failed to create OpenXR instance");
  }

  XrSystemGetInfo system_get_info = {
      .type = XR_TYPE_SYSTEM_GET_INFO,
      .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY,
  };
  if (xrGetSystem(ctx_->instance, &system_get_info, &ctx_->system_id) !=
      XR_SUCCESS) {
    return absl::InternalError("Failed to get OpenXR system");
  }

  XrSessionCreateInfo session_create_info = {
      .type = XR_TYPE_SESSION_CREATE_INFO,
      .systemId = ctx_->system_id,
  };
  if (xrCreateSession(ctx_->instance, &session_create_info, &ctx_->session) !=
      XR_SUCCESS) {
    return absl::InternalError("Failed to create OpenXR session");
  }

  XrReferenceSpaceCreateInfo space_create_info = {
      .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
      .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL,
      .poseInReferenceSpace =
          {
              .orientation = {0, 0, 0, 1},
              .position = {0, 0, 0},
          },
  };
  if (xrCreateReferenceSpace(ctx_->session, &space_create_info, &ctx_->space) !=
      XR_SUCCESS) {
    return absl::InternalError("Failed to create OpenXR reference space");
  }

  XrReferenceSpaceCreateInfo view_space_create_info = {
      .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
      .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW,
      .poseInReferenceSpace =
          {
              .orientation = {0, 0, 0, 1},
              .position = {0, 0, 0},
          },
  };
  if (xrCreateReferenceSpace(ctx_->session, &view_space_create_info,
                             &ctx_->view_space) != XR_SUCCESS) {
    return absl::InternalError("Failed to create OpenXR view space");
  }

  return absl::OkStatus();
}

void HeadlessOpenXrCameraUpdater::Update(const imp::FrameTime& frame_time) {
  // Poll OpenXR events to manage the session lifecycle.
  XrEventDataBuffer event = {XR_TYPE_EVENT_DATA_BUFFER};
  while (xrPollEvent(ctx_->instance, &event) == XR_SUCCESS) {
    if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
      auto* state_changed =
          reinterpret_cast<XrEventDataSessionStateChanged*>(&event);
      if (state_changed->state == XR_SESSION_STATE_READY) {
        XrSessionBeginInfo begin_info = {
            .type = XR_TYPE_SESSION_BEGIN_INFO,
            .next = nullptr,
            .primaryViewConfigurationType =
                XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
        };
        XrResult result = xrBeginSession(ctx_->session, &begin_info);
        if (result == XR_SUCCESS) {
          ctx_->session_running = true;
        } else {
          IMP_LOG(imp::ERROR) << "Failed to begin OpenXR session: " << result;
        }
      } else if (state_changed->state == XR_SESSION_STATE_STOPPING) {
        xrEndSession(ctx_->session);
        ctx_->session_running = false;
      }
    }
    event = {XR_TYPE_EVENT_DATA_BUFFER};
  }

  if (!ctx_->session_running) return;

  XrFrameWaitInfo wait_frame_info = {XR_TYPE_FRAME_WAIT_INFO};
  XrFrameState frame_state = {XR_TYPE_FRAME_STATE};
  xrWaitFrame(ctx_->session, &wait_frame_info, &frame_state);

  XrFrameBeginInfo begin_frame_info = {XR_TYPE_FRAME_BEGIN_INFO};
  xrBeginFrame(ctx_->session, &begin_frame_info);

  XrViewLocateInfo view_locate_info = {
      .type = XR_TYPE_VIEW_LOCATE_INFO,
      .viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
      .displayTime = frame_state.predictedDisplayTime,
      .space = ctx_->space,
  };

  XrViewState view_state = {XR_TYPE_VIEW_STATE};
  uint32_t view_count_output = 0;
  XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};

  xrLocateViews(ctx_->session, &view_locate_info, &view_state, 2,
                &view_count_output, views);

  XrFrameEndInfo end_frame_info = {
      .type = XR_TYPE_FRAME_END_INFO,
      .displayTime = frame_state.predictedDisplayTime,
      .environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE,
      .layerCount = 0,
      .layers = nullptr,
  };
  xrEndFrame(ctx_->session, &end_frame_info);

  imp::Transform<float> eye_center_transform;
  bool head_tracked = false;

  XrSpaceLocation head_location = {XR_TYPE_SPACE_LOCATION};
  XrResult locate_res =
      xrLocateSpace(ctx_->view_space, ctx_->space,
                    frame_state.predictedDisplayTime, &head_location);
  if (locate_res == XR_SUCCESS &&
      (head_location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
      (head_location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
    eye_center_transform = imp::ToTransform(head_location.pose);
    head_tracked = true;
  }

  if (!head_tracked) {
    if (view_count_output >= 2) {
      eye_center_transform = imp::GetEyeCenterTransform(views[0], views[1]);
      head_tracked = true;
    } else {
      return;
    }
  }

  // Pipe directly to the Camera Component
  ComponentHandle<CameraComponent> camera =
      view_.GetCameraManager().GetCamera();
  if (camera.IsValid()) {
    camera->GetNode()->SetLocalTrs(eye_center_transform.AsMat4());
    if (view_count_output >= 2) {
      std::vector<XrView> xr_views = {views[0], views[1]};
      imp::SetCustomEyeProjectionOnCamera(
          &view_.GetHost()->GetView()->getCamera(), xr_views);
    }
  }
}

}  // namespace split_engine
}  // namespace imp
