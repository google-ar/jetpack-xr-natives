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

#include "core/view/platforms/xr_android/xr_session_host.h"

#if IMP_MATERIAL_API(OPENGL) && IMP_PLATFORM(ANDROID)
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>  // NOLINT
#include <GLES3/gl31.h>
#endif  // IMP_MATERIAL_API(OPENGL) && IMP_PLATFORM(ANDROID)

#include <jni.h>
#include <stdlib.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/ColorGrading.h"
#include "filament/filament/include/filament/ColorSpace.h"
#include "filament/filament/include/filament/Fence.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "filament/filament/include/filament/View.h"
#include "filament/filament/include/filament/Viewport.h"
#include "filament/libs/utils/include/utils/Systrace.h"
#include "core/async/executor.h"
#include "core/common/enum_flags.h"
#include "core/common/invocable.h"
#include "core/common/platform_helpers.h"
#include "core/common/robin_set.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/monitor/duration_measurement.h"
#include "core/monitor/scoped_duration_measurement.h"
#include "core/monitor/value_measurement.h"
#include "core/ncsb/component_handle.h"
#include "core/render/content_security_level.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_helpers.h"

#if IMP_MATERIAL_API(VULKAN) && IMP_PLATFORM(ANDROID)
#include "core/view/platforms/xr_android/xr_vulkan_platform.h"
#define XR_KHR_GRAPHICS_ENABLE_EXTENSION_NAME \
  XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME
#define XR_GRAPHICS_BINDING_TYPE XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN2_KHR
#define XR_GET_GRAPHICS_REQUIREMENTS_NAME "xrGetVulkanGraphicsRequirements2KHR"
#else
#include "core/view/platforms/xr_android/xr_opengl_platform.h"
#define XR_KHR_GRAPHICS_ENABLE_EXTENSION_NAME \
  XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME
#define XR_GRAPHICS_BINDING_TYPE XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR
#define XR_GET_GRAPHICS_REQUIREMENTS_NAME "xrGetOpenGLESGraphicsRequirementsKHR"
#endif

#if IMP_RUNTIME(DEV)
#include "core/view/platforms/xr_android/xr_editor_plugin.h"
#endif
#include "core/monitor/profiling_clock.h"
#include "core/view/platforms/xr_android/generic_stereo_materials.h"
#include "core/view/platforms/xr_android/xr_events.proto.imp.h"
#include "core/view/view_host.h"
#include "core/window/filament_host.h"
#include "core/xr/openxr_events.h"
#include "mediapipe/framework/port/status_macros.h"
#include "mediapipe/framework/deps/clock.h"

#define XR_ANDROID_unbounded_reference_space 1
#define XR_ANDROID_unbounded_reference_space_SPEC_VERSION 1
#define XR_ANDROID_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME \
  "XR_ANDROID_unbounded_reference_space"

namespace imp {

namespace {

std::array<const char*, 17> kOpenXRExtensionsCore = {
    XR_KHR_ANDROID_THREAD_SETTINGS_EXTENSION_NAME,        //
    XR_KHR_GRAPHICS_ENABLE_EXTENSION_NAME,                //
    XR_EXT_HAND_TRACKING_EXTENSION_NAME,                  //
    XR_ANDROID_DEVICE_ANCHOR_PERSISTENCE_EXTENSION_NAME,  //
    XR_ANDROID_TRACKABLES_EXTENSION_NAME,                 //
    XR_ANDROID_TRACKABLES_OBJECT_EXTENSION_NAME,          //
    XR_EXT_UUID_EXTENSION_NAME,                           //
    XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME,        //
    XR_FB_COMPOSITION_LAYER_DEPTH_TEST_EXTENSION_NAME,    //
    XR_FB_SWAPCHAIN_UPDATE_STATE_EXTENSION_NAME,          //
    XR_ANDROID_HAND_MESH_EXTENSION_NAME,                  //
    XR_EXT_HAND_INTERACTION_EXTENSION_NAME,               //
    XR_FB_HAND_TRACKING_AIM_EXTENSION_NAME,               //
    XR_ANDROID_MOUSE_INTERACTION_EXTENSION_NAME,          //
    XR_ANDROID_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME,  //
    XR_EXT_LOCAL_FLOOR_EXTENSION_NAME,                    //
    XR_FB_HAND_TRACKING_MESH_EXTENSION_NAME,              //
};

std::array<const char*, 2> kOpenXRExtensionsFbFoveation = {
    XR_FB_FOVEATION_EXTENSION_NAME,                //
    XR_FB_FOVEATION_CONFIGURATION_EXTENSION_NAME,  //
};

std::array<const char*, 1> kOpenXRExtensionsVarjoQuadViews = {
    XR_VARJO_QUAD_VIEWS_EXTENSION_NAME,
};

std::array<const char*, 1> kOpenXRExtensionsVarjoFoveatedRendering = {
    XR_VARJO_FOVEATED_RENDERING_EXTENSION_NAME,
};

std::array<const char*, 1> kOpenXRExtensionAndroidXOccupancyGrid = {
    XR_ANDROIDX_OCCUPANCY_GRID_EXTENSION_NAME,
};

std::array<const char*, 1> kOpenXRExtensionCreateInstanceExtension = {
    XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME,
};

std::array<const char*, 1> kOpenXRExtensionsEyeGazeInteraction = {
    XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME,
};

std::array<const char*, 1> kOpenXRExtensionsAndroidDepthTexture = {
    XR_ANDROID_DEPTH_TEXTURE_EXTENSION_NAME,
};

std::array<const char*, 1> kOpenXRExtensionsFbColorSpace = {
    XR_FB_COLOR_SPACE_EXTENSION_NAME,
};

std::array<const char*, 2> kOpenXRExtensionsAndroidSys = {
    XR_ANDROIDSYS_ANCHOR_SHARING_IMPORT_EXTENSION_NAME,
    XR_ANDROIDSYS_INPUT_TRACING_EXTENSION_NAME,
};

absl::Time GetFilamentTimeNow() {
  // A monotonic/profiling clock for internal use only when on filament thread.
  static mediapipe::Clock* filament_clock = new ProfilingClock();
  return filament_clock->TimeNow();
}

absl::string_view GetApplicationName() {
#if IMP_PLATFORM(ANDROID)
  return getprogname();
#else
  return "ImpressApplication";
#endif
}

}  // namespace

XrSessionHost::XrSessionHost(std::unique_ptr<BaseView> view,
                             XrSessionHostOptions options)
    : ViewHost(std::move(view)),
      reference_space_type_(options.reference_space_type),
      is_composition_layer_depth_enabled_(options.use_composition_layer_depth),
      is_enhanced_stereoscopic_rendering_enabled_(
          options.use_enhanced_stereoscopic_rendering),
      use_max_swapchain_size_(options.use_max_swapchain_size),
      swapchain_size_multiplier_(options.swapchain_size_multiplier),
      current_foveation_level_(options.foveation_level),
      view_configuration_type_(
          options.use_quad_views ? XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO
                                 : XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO),
      is_varjo_foveated_rendering_enabled_(
          options.use_quad_views && options.use_varjo_foveated_rendering),
      msaa_sample_count_(options.msaa_sample_count),
      eye_tracking_enabled_(options.use_eye_gaze_interaction),
      is_android_depth_texture_enabled_(options.use_android_depth_texture),
      display_enabled_duration_(GetView()->GetMonitor(),
                                kXrDisplayEnabledStatistics),
      xr_timing_summary_(*GetView()),
      is_fb_color_space_enabled_(options.use_fb_color_space),
      is_android_system_extensions_enabled_(
          options.enable_android_system_extensions) {
  SetupXrTimingSummary(xr_timing_summary_.GetSummary(),
                       xr_performance_state_.metrics);
  if (is_varjo_foveated_rendering_enabled_) {
    eye_tracking_enabled_ = true;
  }
}

XrSessionHost::~XrSessionHost() {
  if (instance_ == XR_NULL_HANDLE) {
    return;
  }
  imp::output::Xr("Calling xrDestroyInstance");
  absl::Status status = ToStatus(xrDestroyInstance(instance_));
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Error calling xrDestroyInstance `" << status << "`";
  }
}

absl::Status XrSessionHost::PreBeginRender() {
  // Set current swap chain according to the ContentSecurityLevel setting.
  switch (content_security_level_) {
    case ContentSecurityLevel::kProtected: {
      if (!swap_chain_protected_) {
        // Lazily create the protected swap chain if needed since the emulator
        // does not support protected buffers.
        MP_ASSIGN_OR_RETURN(
            swap_chain_protected_,
            AddSwapChain(this,
                         filament::SwapChain::CONFIG_TRANSPARENT |
                             filament::SwapChain::CONFIG_PROTECTED_CONTENT));
      }
      MP_RETURN_IF_ERROR(SetActiveSwapChain(swap_chain_protected_));
    } break;
    default:
      MP_RETURN_IF_ERROR(SetActiveSwapChain(swap_chain_standard_));
  }

  return absl::OkStatus();
}

absl::Status XrSessionHost::Setup(JNIEnv* env, JavaVM* vm, jobject context) {
  IMP_TRACE();

  if (is_enhanced_stereoscopic_rendering_enabled_) {
#if !defined(IMP_INCLUDE_STEREO_VARIANT_BY_DEFAULT)
    return absl::InvalidArgumentError(
        "Instanced Rendering requires stereo variant materials");
#endif
    GetView()->GetAssetManager().SetDefaultLoadOptions(
        {.materials_url_override = std::string(
             materials::kCompiledImpDefaultStereoGltfMaterialsZip.GetUrl())});
  }

  imp::output::Xr("Composition layer depth enabled: %d",
                  is_composition_layer_depth_enabled_);

  // This method is called on the Impress thread.
  //
  // Performs the bulk of the work for initializing OpenXr by creating the
  // XrInstance & XrSession, then creating the XrSwapChain, and resizing the
  // Impress View.
  //
  // OpenXr has specific requirements for how initialization is done, including
  // what order all these methods must be called in.

  imp::output::Xr("Creating XrInstance.");
  MP_ASSIGN_OR_RETURN(instance_, CreateInstance(env, vm, context));

  imp::output::Xr("Obtaining XrSystemId.");
  MP_ASSIGN_OR_RETURN(system_id_, ObtainSystemId());

  // Required by OpenXr to check graphics requirements before creating the
  // XrSession.
  imp::output::Xr("Checking Xr Graphics Requirements.");
  MP_RETURN_IF_ERROR(CheckGraphicsRequirements());

  imp::output::Xr("Creating XrPlatform.");
  platform_ = std::make_unique<PlatformType>();

#if IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(VULKAN)
  // Pass the XrInstance and SystemId to the platform if we are using a vulkan
  // backend.
  platform_->setXrInstance(instance_);
  platform_->setXrSystemId(system_id_);

  // Create the Vulkan instance, physical device, and logical device.
  VkInstance vk_instance = platform_->createVulkanInstance();
  platform_->bindVulkanInstance(vk_instance);

  VkPhysicalDevice vk_physical_device =
      platform_->getVulkanPhysicalDevice(vk_instance);

  // Identify the graphics queue family index.
  uint32_t graphics_queue_family_index =
      platform_->identifyVulkanGraphicsQueueFamilyIndex(vk_physical_device);

  uint32_t protected_graphics_queue_family_index =
      platform_->identifyVulkanProtectedGraphicsQueueFamilyIndex(
          vk_physical_device);

  VkDevice vk_device = platform_->createVulkanLogicalDevice(
      vk_physical_device, vk_instance, graphics_queue_family_index,
      protected_graphics_queue_family_index, IsMultiviewStereo());

  // Create the Vulkan shared context. This is used to pass the Vulkan instance,
  // physical device, logical device, graphics queue family index, and graphics
  // queue index to the Filament backend.
  filament::backend::VulkanPlatform::VulkanSharedContext shared_context =
      platform_->getVulkanSharedContext();
  shared_context.instance = vk_instance;
  shared_context.logicalDevice = vk_device;
  shared_context.physicalDevice = vk_physical_device;
  shared_context.graphicsQueueFamilyIndex = graphics_queue_family_index;
  shared_context.graphicsQueueIndex = 0;

  platform_->setVulkanSharedContext(shared_context);
  imp::output::Xr("Calling Impress Setup.");
  MP_RETURN_IF_ERROR(ViewHost::Setup(platform_.get(), &shared_context));

#else
  imp::output::Xr("Calling Impress Setup.");
  MP_RETURN_IF_ERROR(ViewHost::Setup(platform_.get(), nullptr));

#endif

  // Sets settings on the Filament view that are required for OpenXR.
  // TODO: Provide helper method GetRecommendedColorSpace on
  // FilamentHost to provide easy access to this when specific experiences in Xr
  // are customizing color grading.
  filament::View* filament_view = FilamentHost::GetView();
  filament::color::ColorSpace color_space =
      filament::color::Rec709 - filament::color::Linear - filament::color::D65;
  auto color_grading =
      filament::ColorGrading::Builder()
          .toneMapping(filament::ColorGrading::ToneMapping::FILMIC)
          .outputColorSpace(color_space)
          .build(*GetEngine());
  filament_view->setColorGrading(color_grading);

  // Disable post-processing, this is incompatible with instanced stereo.
  // TODO: Remove this once multiview is supported.
  filament_view->setPostProcessingEnabled(false);

  xr_between_frame_timing_ =
      DurationMeasurement(monitor_.get(), kXrBetweenFrameTiming);
  DurationMeasurement::AddHistogram(*monitor_, kXrBetweenFrameTiming,
                                    absl::ZeroDuration(), absl::Milliseconds(1),
                                    60);
  xr_scheduled_timing_ =
      DurationMeasurement(monitor_.get(), kXrScheduledFrameTiming);
  filament_thread_timing_ =
      DurationMeasurement(monitor_.get(), kXrBeginFrameToEndFrame);
  display_state_statistics_ =
      std::make_unique<ValueMeasurement>(*monitor_, kXrDisplayEnabledStatistics,
                                         static_cast<int64_t>(display_state_));
  imp::output::Xr("Completed calling Impress Setup.");

  return absl::OkStatus();
}

absl::Status XrSessionHost::onWindowAttached() {
  IMP_TRACE();
  

  imp::output::Xr("Creating XrSession.");
  MP_ASSIGN_OR_RETURN(session_, CreateSession());

  imp::output::Xr("Obtaining Xr reference space.");
  MP_ASSIGN_OR_RETURN(reference_space_, ObtainXrSpace(reference_space_type_));

  // TODO: Consider changing/renaming/moving as part of
  // designing system for Xr input.
  GetView()->GetDispatcher().Send(OpenXrReferenceSpaceCreatedEvent{});

  imp::output::Xr("Obtaining Xr view space.");
  MP_ASSIGN_OR_RETURN(view_space_, ObtainXrSpace(XR_REFERENCE_SPACE_TYPE_VIEW));

  if (is_varjo_foveated_rendering_enabled_) {
    imp::output::Xr("Obtaining Xr render gaze space.");
    MP_ASSIGN_OR_RETURN(render_gaze_space_,
                     ObtainXrSpace(XR_REFERENCE_SPACE_TYPE_COMBINED_EYE_VARJO));

    imp::output::Xr("Obtaining Xr foveated view configs.");
    MP_ASSIGN_OR_RETURN(varjo_foveation_view_configs_,
                     ObtainFoveatedViewConfigs());
    varjo_foveation_display_size_ =
        CalculateDisplaySize(varjo_foveation_view_configs_);
  }

  imp::output::Xr("Obtaining Xr view configs.");
  MP_ASSIGN_OR_RETURN(view_configs_, ObtainViewConfigs());
  display_size_ = CalculateDisplaySize(view_configs_);

  // This will ultimately trigger XrPlatform::createSwapChain to be called.
  imp::output::Xr("Creating SwapChain.");
  MP_ASSIGN_OR_RETURN(swap_chain_standard_,
                   AddSwapChain(this, filament::SwapChain::CONFIG_TRANSPARENT));

  MP_RETURN_IF_ERROR(SetActiveSwapChain(swap_chain_standard_));

  // Resize Impress view.
  imp::output::Xr("Resizing Impress view.");
  ResizeImpressView();

  // Ensure that the main app thread is prioritized to avoid jank. The render
  // thread is already set to XR_ANDROID_THREAD_TYPE_RENDER_KHR.
  MP_RETURN_IF_ERROR(SetThreadType(XR_ANDROID_THREAD_TYPE_APPLICATION_MAIN_KHR));

  shown_state_ = ShownState::kShown;

  return absl::OkStatus();
}

absl::Status XrSessionHost::AdvanceFrame() {
  IMP_TRACE();
  

  if (session_ == XR_NULL_HANDLE) {
    return absl::InternalError("AdvanceFrame called when there is no session");
  }

  MP_RETURN_IF_ERROR(PollEvents());

  if (state_ != RunningState::kRunning) {
    return absl::OkStatus();
  }

  absl::optional<XrPreAdvanceFrameEvent::XrFrameSkippedReason>
      frame_skipped_reason = absl::nullopt;

  // Creates a percentage metric by adding 100 or 0 each frame.
  display_enabled_duration_.AddSample(
      display_state_ == XrHelpers::DisplayState::kDisplayEnabled
          ? absl::Milliseconds(100)
          : absl::ZeroDuration());

  imp::output::Xr("Calling Xr wait frame.");
  MP_ASSIGN_OR_RETURN(XrFrameState frame_state, WaitFrame());
  imp::output::Xr(
      "Finished calling Xr wait frame. predictedDisplayPeriod=%i, "
      "predictedDisplayTime=%i, shouldRender=%s",
      frame_state.predictedDisplayPeriod, frame_state.predictedDisplayTime,
      frame_state.shouldRender ? "true" : "false");

  if (frame_state.predictedDisplayPeriod != XR_INFINITE_DURATION) {
    // Report if predictedDisplayPeriod is exceeded significantly.
    auto maximum_display_period =
        absl::Nanoseconds(frame_state.predictedDisplayPeriod * 1.25);
    if (xr_timing_summary_.GetDisplayPeriod() != maximum_display_period) {
      xr_timing_summary_.SetDisplayPeriod(maximum_display_period,
                                          kXrBetweenFrameTiming);
    }
  }

  if (shown_state_ == ShownState::kHidden) {
    imp::output::Xr("show_state_ is kHidden, discarding frame.");
    // previous_view_state_ must be considered changed when next shown.
    previous_view_state_.reset();
    frame_skipped_reason = XrPreAdvanceFrameEvent::XrFrameSkippedReason::HIDDEN;
  }

  if (!frame_state.shouldRender) {
    imp::output::Xr("XrFrameState::shouldRender is false, discarding frame.");

    // When shouldRender is false, the OpenXr spec requires us to begin & end an
    // empty frame. shouldRender is guaranteed to be false on the first call to
    // the frame loop, but going through this flow is required by OpenXr to
    // reach the VISIBLE or FOCUSED states where shouldRender will start to
    // return true.
    frame_skipped_reason =
        XrPreAdvanceFrameEvent::XrFrameSkippedReason::SHOULD_RENDER_IS_FALSE;
  }

  LocateViewsResult locate_views_result;
  LocateSpaceResult view_location_result;

  if (!frame_skipped_reason.has_value()) {
    MP_ASSIGN_OR_RETURN(locate_views_result,
                     LocateViews(frame_state.predictedDisplayTime));
    if (should_resize_view_) {
      ResizeImpressView();
    }

    MP_ASSIGN_OR_RETURN(view_location_result,
                     LocateViewInReference(frame_state.predictedDisplayTime));
    if (locate_views_result.status == LocateViewsStatus::kUnableToObtainPose ||
        view_location_result.status == LocateSpaceStatus::kUnableToObtainPose) {
      imp::output::Xr("Unable to obtain Xr view pose, discarding frame.");

      // Similar to shouldRender returning false above, OpenXr spec requires us
      // to begin & end an empty frame when we are unable to obtain a pose. When
      // tracking is lost, OpenXr should still give us the last known valid
      // pose, allowing us to render. This case is mainly hit before tracking
      // starts for the first time.
      frame_skipped_reason =
          XrPreAdvanceFrameEvent::XrFrameSkippedReason::UNABLE_TO_OBTAIN_POSE;
    }
  }

  // Send event that the XR frame is being advanced (with optional skip reason).
  XrPreAdvanceFrameEvent pre_advance_frame_event;
  pre_advance_frame_event.frame_skipped_reason = frame_skipped_reason;
  GetView()->GetDispatcher().Send(pre_advance_frame_event);

  // The OpenXr spec requires us to begin & end an empty frame to skip a frame.
  if (frame_skipped_reason.has_value()) {
    return BeginAndDiscardFrame(frame_state.predictedDisplayTime);
  }

  xr_scheduled_timing_->AddSample(
      absl::Nanoseconds(frame_state.predictedDisplayPeriod));

  latest_predicted_display_time_ = frame_state.predictedDisplayTime;

  // TODO: Consider changing/renaming/moving as part of
  // designing system for Xr input.
  if (session_state_ == XR_SESSION_STATE_FOCUSED) {
    GetView()->GetDispatcher().Send(OpenXrFocusedWaitFrameEvent{});
  }

  // Enqueue the views and frame time so that the render thread can pop the
  // information from the queue to submit the frame to OpenXr.
  if (display_state_ == XrHelpers::DisplayState::kDisplayEnabled) {
    absl::MutexLock lock(&frame_queue_mutex_);
    imp::output::Xr("Adding time %i to queue",
                    frame_state.predictedDisplayTime);
    frame_queue_.push(
        {.display_time = frame_state.predictedDisplayTime,
         .views = locate_views_result.views,
         .should_render_varjo_foveation = use_varjo_foveation_this_frame_});
    if (prev_filament_thread_duration_) {
      filament_thread_timing_->AddSample(*prev_filament_thread_duration_);
      prev_filament_thread_duration_.reset();
    }
  }

  // Store the latest views. This is only needed so that
  // XrSessionHost::PerformRender can access the views to render the left &
  // right eyes. PerformRender is guaranteed to be called on the Impress thread
  // during the below call to RenderNextFrame, so this field is used almost
  // immediately.
  latest_views_ = locate_views_result.views;

  // Update camera to match the eye center so that any UX logic that is done by
  // the app during the frame that uses the camera position is correct. For
  // example, the app might have a component that billboards a panel so that it
  // faces the camera.
  UpdateCameraFromXrPose(view_location_result.location.pose);

  // Ensure that the result of filament::Renderer::beginFrame is ignored in
  // OpenXR because the OpenXR library is used to control frame pacing instead.
  // This is a supported usage of filament::Renderer::beginFrame.
  EnsureNextRenderCompletes();

  imp::output::Xr("Calling RenderNextFrame.");
  // TODO: Investigate using xrConvertTimeToTimespecTimeKHR here.
  absl::Duration frame_time =
      absl::Nanoseconds(frame_state.predictedDisplayTime);

  absl::Status return_value;
  if (display_state_ == XrHelpers::DisplayState::kDisplayDisabled) {
    // Execute a frame loop without the render step.
    imp::Flags<imp::window::FilamentHost::RenderResultFlags> flags;
    return_value = IsolatedPreRender(last_frame_time_, frame_time, &flags);
    return_value.Update(IsolatedPostRender(&flags));
    return_value.Update(BeginAndDiscardFrame(frame_state.predictedDisplayTime));
  } else {
    MP_ASSIGN_OR_RETURN(RenderResult render_result,
                     RenderNextFrame(last_frame_time_, frame_time));
    // unused.
    (void)render_result;
    did_last_advance_frame_succeed_ = true;
    xr_between_frame_timing_->EndSample();
    xr_between_frame_timing_->BeginSample();
  }
  last_frame_time_ = frame_time;

  return return_value;
}

bool XrSessionHost::IsXrFbFoveationEnabled() const {
  return current_foveation_level_ !=
         XrFoveationLevelFB::XR_FOVEATION_LEVEL_NONE_FB;
}

bool XrSessionHost::IsXrVarjoQuadViewsEnabled() const {
  return view_configuration_type_ ==
         XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO;
}

bool XrSessionHost::IsXrVarjoFoveatedRenderingEnabled() const {
  return is_varjo_foveated_rendering_enabled_;
}

bool XrSessionHost::IsXrFbColorSpaceEnabled() const {
  return is_fb_color_space_enabled_;
}

bool XrSessionHost::ShouldRenderVarjoFoveationThisFrame() {
  {
    absl::MutexLock lock(&frame_queue_mutex_);
    if (frame_queue_.empty()) {
      return false;
    }
    return frame_queue_.front().should_render_varjo_foveation;
  }
}

bool XrSessionHost::IsXrAndroidXOccupancyGridEnabled() const {
  std::vector<const char*> extensions;
  extensions.insert(extensions.end(),
                    kOpenXRExtensionAndroidXOccupancyGrid.begin(),
                    kOpenXRExtensionAndroidXOccupancyGrid.end());
  absl::Status is_supported = EnsureSupportedExtensions(extensions);
  return is_supported.ok();
}

bool XrSessionHost::IsXrAndroidDepthTextureEnabled() const {
  return is_android_depth_texture_enabled_;
}

bool XrSessionHost::IsXrEyeGazeInteractionEnabled() const {
  return eye_tracking_enabled_;
}

bool XrSessionHost::IsXrAndroidSystemExtensionsEnabled() const {
  return is_android_system_extensions_enabled_;
}

absl::Status XrSessionHost::EnsureSupportedExtensions(
    const std::vector<const char*>& required_extensions) const {
  // Check to ensure our extensions are all provided.
  std::vector<XrExtensionProperties> v;
  uint32_t propertyCount;

  MP_RETURN_IF_ERROR(ToStatus(xrEnumerateInstanceExtensionProperties(
      nullptr, 0, &propertyCount, nullptr)));
  v.resize(propertyCount,
           XrExtensionProperties{XR_TYPE_EXTENSION_PROPERTIES, nullptr});

  MP_RETURN_IF_ERROR(ToStatus(xrEnumerateInstanceExtensionProperties(
      nullptr, propertyCount, &propertyCount, v.data())));

  RobinSet<std::string> provided_extensions;
  for (uint32_t i = 0; i < propertyCount; ++i) {
    provided_extensions.insert(v[i].extensionName);
  }

  for (const char* extension : required_extensions) {
    if (!provided_extensions.contains(std::string(extension))) {
      return absl::UnavailableError(
          absl::StrCat("Extension ", extension, " is not supported."));
    }
  }
  return absl::OkStatus();
}

std::vector<const char*> XrSessionHost::FilterUnsupportedExtensions(
    const absl::Span<const char* const>& extensions) {
  std::vector<XrExtensionProperties> extension_properties;
  uint32_t propertyCount;

  bool ok = xrEnumerateInstanceExtensionProperties(nullptr, 0, &propertyCount,
                                                   nullptr) == XR_SUCCESS;
  if (!ok) {
    return {};
  }

  extension_properties.resize(
      propertyCount,
      XrExtensionProperties{XR_TYPE_EXTENSION_PROPERTIES, nullptr});

  ok = xrEnumerateInstanceExtensionProperties(
           nullptr, propertyCount, &propertyCount,
           extension_properties.data()) == XR_SUCCESS;

  if (!ok) {
    return {};
  }

  RobinSet<std::string> provided_extensions;
  for (uint32_t i = 0; i < propertyCount; ++i) {
    provided_extensions.insert(extension_properties[i].extensionName);
  }

  std::vector<const char*> filtered_extensions;

  for (const char* extension : extensions) {
    if (provided_extensions.contains(std::string(extension))) {
      filtered_extensions.push_back(extension);
    }
  }

  return filtered_extensions;
}

absl::StatusOr<XrInstance> XrSessionHost::CreateInstance(JNIEnv* env,
                                                         JavaVM* vm,
                                                         jobject context) {
  IMP_TRACE();

  XrInstance instance;

  // Build the list of desired extensions. Start with core extensions, meaning
  // there are no conditions on including them.
  std::vector<const char*> extensions;

  const auto& extensions_to_load = GetExtensionsToLoad();
  extensions.insert(extensions.end(), extensions_to_load.begin(),
                    extensions_to_load.end());

  // This is only used if the context is an activity.
  // It's created outside of the is_context_an_activity if-statement below so
  // that it doesn't fall out of scope when it gets used.
  const XrInstanceCreateInfoAndroidKHR create_info_android = {
      .type = XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR,
      .next = nullptr,
      .applicationVM = vm,
      .applicationActivity = context,
  };

  jclass activity_class = env->FindClass("android/app/Activity");
  jboolean is_context_an_activity = env->IsInstanceOf(context, activity_class);

  // If the context is an activity, then we must pass the create_info_android
  // structure as the next field. Otherwise, it should be nullptr.
  const void* next = nullptr;
  if (is_context_an_activity) {
    next = &create_info_android;
    // The kOpenXRExtensionCreateInstanceExtension is required for passing the
    // XrInstanceCreateInfoAndroidKHR structure to xrCreateInstance.
    extensions.insert(extensions.end(),
                      kOpenXRExtensionCreateInstanceExtension.begin(),
                      kOpenXRExtensionCreateInstanceExtension.end());
  }
  if (IsXrFbFoveationEnabled()) {
    extensions.insert(extensions.end(), kOpenXRExtensionsFbFoveation.begin(),
                      kOpenXRExtensionsFbFoveation.end());
  }
  if (IsXrVarjoQuadViewsEnabled()) {
    extensions.insert(extensions.end(), kOpenXRExtensionsVarjoQuadViews.begin(),
                      kOpenXRExtensionsVarjoQuadViews.end());
    if (is_varjo_foveated_rendering_enabled_) {
      extensions.insert(extensions.end(),
                        kOpenXRExtensionsVarjoFoveatedRendering.begin(),
                        kOpenXRExtensionsVarjoFoveatedRendering.end());
    }
  } else {
    if (is_varjo_foveated_rendering_enabled_) {
      output::Warning(
          "IMP: varjo foveated rendering requires the OpenXR VarjoQuadViews "
          "extension to also be enabled.");
    }
  }
  if (IsXrAndroidXOccupancyGridEnabled()) {
    extensions.insert(extensions.end(),
                      kOpenXRExtensionAndroidXOccupancyGrid.begin(),
                      kOpenXRExtensionAndroidXOccupancyGrid.end());
  }
  if (IsXrEyeGazeInteractionEnabled()) {
    extensions.insert(extensions.end(),
                      kOpenXRExtensionsEyeGazeInteraction.begin(),
                      kOpenXRExtensionsEyeGazeInteraction.end());
  }
  if (IsXrAndroidDepthTextureEnabled()) {
    extensions.insert(extensions.end(),
                      kOpenXRExtensionsAndroidDepthTexture.begin(),
                      kOpenXRExtensionsAndroidDepthTexture.end());
  }
  if (IsXrFbColorSpaceEnabled()) {
    extensions.insert(extensions.end(), kOpenXRExtensionsFbColorSpace.begin(),
                      kOpenXRExtensionsFbColorSpace.end());
  }
  if (IsXrAndroidSystemExtensionsEnabled()) {
    extensions.insert(extensions.end(), kOpenXRExtensionsAndroidSys.begin(),
                      kOpenXRExtensionsAndroidSys.end());
  }

  if (absl::Status all_supported = EnsureSupportedExtensions(extensions);
      !all_supported.ok()) {
    // Crash to make it extremely clear that a required extension is lacking.
    IMP_LOG(imp::FATAL) << all_supported;
  }

  const auto& optional_extensions_to_load =
      FilterUnsupportedExtensions(GetOptionalExtensionsToLoad());

  extensions.insert(extensions.end(), optional_extensions_to_load.begin(),
                    optional_extensions_to_load.end());

  XrInstanceCreateInfo create_info = {
      .type = XR_TYPE_INSTANCE_CREATE_INFO,
      .next = next,
      .applicationInfo =
          {
              .engineName = "Impress",
              // TODO: Revert back to XR_CURRENT_API_VERSION after
              // the OpenXR loader version mismatch issue is resolved
              // (broken link).
              .apiVersion = XR_MAKE_VERSION(1, 0, 34),
          },
      .enabledApiLayerCount = 0,
      .enabledApiLayerNames = nullptr,
      .enabledExtensionCount = static_cast<uint32_t>(extensions.size()),
      .enabledExtensionNames = extensions.data(),
  };

  absl::SNPrintF(create_info.applicationInfo.applicationName,
                 XR_MAX_APPLICATION_NAME_SIZE, "%s", GetApplicationName());

  MP_RETURN_IF_ERROR(ToStatus(xrCreateInstance(&create_info, &instance)));

  if (instance == XR_NULL_SYSTEM_ID) {
    return absl::InternalError("XrInstance is NULL.");
  }

  enabled_extensions_.insert(extensions.begin(), extensions.end());

  return instance;
}

absl::StatusOr<XrSystemId> XrSessionHost::ObtainSystemId() const {
  if (instance_ == XR_NULL_HANDLE) {
    return absl::InternalError("instance_ is NULL.");
  }

  XrSystemId system_id = XR_NULL_SYSTEM_ID;

  const XrSystemGetInfo system_info = {
      .type = XR_TYPE_SYSTEM_GET_INFO,
      .next = nullptr,
      .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY,
  };
  MP_RETURN_IF_ERROR(ToStatus(xrGetSystem(instance_, &system_info, &system_id)));
  if (system_id == XR_NULL_SYSTEM_ID) {
    return absl::InternalError("Failed to get system id.");
  }

  return system_id;
}

absl::Status XrSessionHost::CheckGraphicsRequirements() const {
  

  if (instance_ == XR_NULL_HANDLE) {
    return absl::InternalError("instance_ is NULL.");
  }

  if (system_id_ == XR_NULL_SYSTEM_ID) {
    return absl::InternalError("system_id_ is NULL.");
  }

  XrGetGraphicsRequirements xrGetOpenGLESGraphicsRequirementsKHR = nullptr;
  MP_RETURN_IF_ERROR(ToStatus(
      xrGetInstanceProcAddr(instance_, XR_GET_GRAPHICS_REQUIREMENTS_NAME,
                            reinterpret_cast<PFN_xrVoidFunction*>(
                                &xrGetOpenGLESGraphicsRequirementsKHR))));
  XrSessionHost::XrGraphicsRequirements graphics_requirements{
      .type = XR_GRAPHICS_BINDING_TYPE, .next = nullptr};
  return ToStatus(xrGetOpenGLESGraphicsRequirementsKHR(instance_, system_id_,
                                                       &graphics_requirements));
}

absl::Status XrSessionHost::SetThreadType(XrAndroidThreadTypeKHR threadType) {
// TODO: remove once this code is not built for linux in presubmit.
#if IMP_PLATFORM(LINUX)
#define gettid() syscall(SYS_gettid)
#endif  // IMP_PLATFORM(LINUX)

  if (instance_ == XR_NULL_HANDLE) {
    return absl::InternalError("instance_ is NULL.");
  }
  PFN_xrSetAndroidApplicationThreadKHR xrSetAndroidApplicationThreadKHR =
      nullptr;
  MP_RETURN_IF_ERROR(ToStatus(
      xrGetInstanceProcAddr(instance_, "xrSetAndroidApplicationThreadKHR",
                            reinterpret_cast<PFN_xrVoidFunction*>(
                                &xrSetAndroidApplicationThreadKHR))));
  return ToStatus(
      xrSetAndroidApplicationThreadKHR(GetXrSession(), threadType, gettid()));
}

absl::StatusOr<XrSession> XrSessionHost::CreateSession() const {
  IMP_TRACE();
  

  if (instance_ == XR_NULL_HANDLE) {
    return absl::InternalError("instance_ is NULL.");
  }

  XrSession session = XR_NULL_HANDLE;

  XrGraphicsBinding graphics_binding = platform_->GetGraphicsBinding();

  XrSessionCreateInfo session_create_info = {
      .type = XR_TYPE_SESSION_CREATE_INFO,
      .next = reinterpret_cast<const XrBaseInStructure*>(&graphics_binding),
      .systemId = system_id_,
  };
  MP_RETURN_IF_ERROR(
      ToStatus(xrCreateSession(instance_, &session_create_info, &session)));

  return session;
}

absl::StatusOr<XrSpace> XrSessionHost::ObtainXrSpace(
    XrReferenceSpaceType space_type) const {
  

  if (session_ == XR_NULL_HANDLE) {
    return absl::InternalError("session_ is NULL.");
  }

  const XrReferenceSpaceCreateInfo create_info = {
      .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
      .next = nullptr,
      .referenceSpaceType = space_type,
      .poseInReferenceSpace = {{0.f, 0.f, 0.f, 1.f}, {0.f}},
  };

  XrSpace result;
  MP_RETURN_IF_ERROR(
      ToStatus(xrCreateReferenceSpace(session_, &create_info, &result)));

  return result;
}

absl::StatusOr<std::vector<XrViewConfigurationView>>
XrSessionHost::ObtainFoveatedViewConfigs() const {
  IMP_TRACE();

  if (session_ == XR_NULL_HANDLE) {
    return absl::InternalError("session_ is NULL.");
  }

  if (system_id_ == XR_NULL_SYSTEM_ID) {
    return absl::InternalError("system_id_ is NULL.");
  }

  

  XrSystemFoveatedRenderingPropertiesVARJO foveatedRenderingProperties{
      XR_TYPE_SYSTEM_FOVEATED_RENDERING_PROPERTIES_VARJO};

  XrSystemProperties system_properties{XR_TYPE_SYSTEM_PROPERTIES,
                                       &foveatedRenderingProperties};
  if (xrGetSystemProperties(GetXrInstance(), GetSystemId(),
                            &system_properties) != XR_SUCCESS) {
    return absl::InternalError("Failed to get system properties.");
  }

  if (!foveatedRenderingProperties.supportsFoveatedRendering) {
    IMP_LOG(imp::WARNING) << "IMP: Foveated rendering is not supported";
  }

  std::vector<XrViewConfigurationView> view_configs;

  uint32_t view_count;
  MP_RETURN_IF_ERROR(ToStatus(xrEnumerateViewConfigurationViews(
      instance_, system_id_, view_configuration_type_, 0, &view_count,
      nullptr)));

  // Request foveated view configs
  std::vector<XrFoveatedViewConfigurationViewVARJO> request_foveated_configs(
      view_count);
  view_configs.resize(view_count);
  for (int i = 0; i < view_count; i++) {
    auto& view_config = view_configs[i];
    auto& request_foveated_config = request_foveated_configs[i];
    request_foveated_config.type =
        XR_TYPE_FOVEATED_VIEW_CONFIGURATION_VIEW_VARJO;
    request_foveated_config.foveatedRenderingActive = XR_TRUE;
    view_config.type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
    view_config.next = &request_foveated_config;
  }

  uint32_t written_view_count;
  MP_RETURN_IF_ERROR(ToStatus(xrEnumerateViewConfigurationViews(
      instance_, system_id_, view_configuration_type_, view_count,
      &written_view_count, view_configs.data())));
  if (written_view_count != view_count) {
    output::Warning(
        "IMP: Written view count is not equal to requested view count");
  }

  return view_configs;
}

absl::StatusOr<std::vector<XrViewConfigurationView>>
XrSessionHost::ObtainViewConfigs() const {
  IMP_TRACE();

  if (session_ == XR_NULL_HANDLE) {
    return absl::InternalError("session_ is NULL.");
  }

  if (system_id_ == XR_NULL_SYSTEM_ID) {
    return absl::InternalError("system_id_ is NULL.");
  }

  

  std::vector<XrViewConfigurationView> view_configs;

  uint32_t view_count;
  MP_RETURN_IF_ERROR(ToStatus(xrEnumerateViewConfigurationViews(
      instance_, system_id_, view_configuration_type_, 0, &view_count,
      nullptr)));
  view_configs.resize(view_count, XrViewConfigurationView{
                                      .type = XR_TYPE_VIEW_CONFIGURATION_VIEW,
                                      .next = nullptr});
  MP_RETURN_IF_ERROR(ToStatus(xrEnumerateViewConfigurationViews(
      instance_, system_id_, view_configuration_type_, view_count, &view_count,
      view_configs.data())));

  return view_configs;
}

absl::Status XrSessionHost::PollEvents() {
  

  while (std::optional<const XrEventDataBaseHeader*> event = GetNextEvent()) {
    switch ((*event)->type) {
      case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
        const auto& instance_loss_pending =
            *reinterpret_cast<const XrEventDataInstanceLossPending*>(*event);
        imp::output::Xr("XR instance loss in %i",
                        instance_loss_pending.lossTime);
        break;
      }
      case XR_TYPE_EVENT_DATA_EVENTS_LOST: {
        imp::output::Xr("Xr events lost.");
        break;
      }
      case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
        imp::output::Xr("Xr session state changed.");
        MP_RETURN_IF_ERROR(HandleSessionStateChanged(
            *reinterpret_cast<const XrEventDataSessionStateChanged*>(*event)));
        break;
      }
      case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED:
        imp::output::Xr("Xr interaction profile changed.");
        // TODO: Consider changing/renaming/moving as part of
        // designing system for Xr input.
        GetView()->GetDispatcher().Send(OpenXrInteractionProfileChangedEvent{});
        break;
      case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
        imp::output::Xr("Xr space change pending.");
        GetView()->GetDispatcher().Send(OpenXrSpaceChangePendingEvent{});
        break;
      default: {
        imp::output::Xr("Ignoring event type %i", (*event)->type);
        break;
      }
    }
  }

  return absl::OkStatus();
}

std::optional<const XrEventDataBaseHeader*> XrSessionHost::GetNextEvent() {
  

  XrEventDataBaseHeader* base_header =
      reinterpret_cast<XrEventDataBaseHeader*>(&event_data_buffer_);
  *base_header = {.type = XR_TYPE_EVENT_DATA_BUFFER, .next = nullptr};
  XrResult result = xrPollEvent(instance_, &event_data_buffer_);
  if (result == XR_SUCCESS) {
    return base_header;
  }

  return std::nullopt;
}

absl::Status XrSessionHost::HandleSessionStateChanged(
    const XrEventDataSessionStateChanged& event) {
  

  session_state_ = event.state;

  switch (event.state) {
    case XR_SESSION_STATE_IDLE: {
      imp::output::Xr("XrSessionState is idle.");
      break;
    }
    case XR_SESSION_STATE_READY: {
      imp::output::Xr("XrSessionState is ready.");
      MP_RETURN_IF_ERROR(BeginSession());
      break;
    }
    case XR_SESSION_STATE_STOPPING: {
      imp::output::Xr("XrSessionState is stopping.");
      MP_RETURN_IF_ERROR(EndSession());
      break;
    }
    case XR_SESSION_STATE_EXITING: {
      imp::output::Xr("XrSessionState is exiting.");
      break;
    }
    case XR_SESSION_STATE_LOSS_PENDING: {
      imp::output::Xr("XrSessionState is loss pending.");
      break;
    }
    default:
      imp::output::Xr("Ignored XrSessionState change: %i", event.state);
      break;
  }
  return absl::OkStatus();
}

absl::Status XrSessionHost::BeginSession() {
  IMP_TRACE();
  

  if (session_ == XR_NULL_HANDLE) {
    return absl::InternalError("session_ is NULL.");
  }

  const XrSessionBeginInfo session_begin_info = {
      .type = XR_TYPE_SESSION_BEGIN_INFO,
      .next = nullptr,
      .primaryViewConfigurationType = view_configuration_type_,
  };

  imp::output::Xr("Beginning Xr Session.");
  MP_RETURN_IF_ERROR(ToStatus(xrBeginSession(session_, &session_begin_info)));

  state_ = RunningState::kRunning;

  // TODO: Consider changing/renaming/moving as part of
  // designing system for Xr input.
  GetView()->GetDispatcher().Send(OpenXrSessionBeginEvent{});

  return absl::OkStatus();
}

absl::Status XrSessionHost::EndSession() {
  MP_RETURN_IF_ERROR(ToStatus(xrEndSession(session_)));
  state_ = RunningState::kNotRunning;
  return absl::OkStatus();
}

void XrSessionHost::SetEyeTrackingEnabled(bool enabled) {
  eye_tracking_enabled_ = enabled;
}

void XrSessionHost::SetEnvironmentBlendMode(
    XrEnvironmentBlendMode xr_environment_blend_mode) {
  environment_blend_mode_ = xr_environment_blend_mode;
}

absl::Status XrSessionHost::SetShownState(ShownState new_state) {
  if (new_state == shown_state_) {
    return absl::OkStatus();
  }

  // Discard the current sample
  xr_between_frame_timing_->CancelSample();
  shown_state_ = new_state;
  if (session_ != XR_NULL_HANDLE) {
    // Discard the current frame if needed.
    MP_RETURN_IF_ERROR(AdvanceFrame());
  }

  return absl::OkStatus();
}

absl::StatusOr<std::vector<XrColorSpaceFB>>
XrSessionHost::EnumerateColorSpaces() {
  if (!is_fb_color_space_enabled_) {
    return absl::UnimplementedError("Color space extension is not supported.");
  }

  if (session_ == XR_NULL_HANDLE) {
    return absl::InternalError("session_ is NULL.");
  }

  PFN_xrEnumerateColorSpacesFB xrEnumerateColorSpacesFB_ptr = nullptr;
  uint32_t color_space_count;
  MP_RETURN_IF_ERROR(ToStatus(xrGetInstanceProcAddr(
      instance_, "xrEnumerateColorSpacesFB",
      reinterpret_cast<PFN_xrVoidFunction*>(&xrEnumerateColorSpacesFB_ptr))));
  xrEnumerateColorSpacesFB_ptr(session_, 0, &color_space_count, nullptr);
  std::vector<XrColorSpaceFB> color_spaces(color_space_count);
  xrEnumerateColorSpacesFB_ptr(session_, color_space_count, &color_space_count,
                               color_spaces.data());

  return color_spaces;
}

absl::Status XrSessionHost::SetColorSpace(XrColorSpaceFB color_space) {
  if (!is_fb_color_space_enabled_) {
    return absl::UnimplementedError("Color space extension is not supported.");
  }

  if (session_ == XR_NULL_HANDLE) {
    return absl::InternalError("session_ is NULL.");
  }

  PFN_xrSetColorSpaceFB xrSetColorSpaceFB_ptr = nullptr;
  MP_RETURN_IF_ERROR(ToStatus(xrGetInstanceProcAddr(
      instance_, "xrSetColorSpaceFB",
      reinterpret_cast<PFN_xrVoidFunction*>(&xrSetColorSpaceFB_ptr))));

  xrSetColorSpaceFB_ptr(session_, color_space);

  return absl::OkStatus();
}

uint32_t imp::XrSessionHost::GetFenceFd() const {
  uint32_t fenceFd = -1;
  // TODO: Implement this for Vulkan.
#if IMP_MATERIAL_API(OPENGL) && IMP_PLATFORM(ANDROID)
  EGLDisplay display = eglGetCurrentDisplay();
  if (display == EGL_NO_DISPLAY) {
    IMP_LOG(imp::ERROR) << "eglGetCurrentDisplay failed: %d" << eglGetError();
    return fenceFd;
  }
  glFlush();
  EGLSyncKHR sync = EGL_NO_SYNC_KHR;
  PFNEGLCREATESYNCKHRPROC eglCreateSyncKHR_ext =
      (PFNEGLCREATESYNCKHRPROC)eglGetProcAddress("eglCreateSyncKHR");
  if (eglCreateSyncKHR_ext) {
    sync =
        eglCreateSyncKHR_ext(display, EGL_SYNC_NATIVE_FENCE_ANDROID, nullptr);
  } else {
    IMP_LOG(imp::ERROR) << "eglCreateSyncKHR not available on this platform.";
    return fenceFd;
  }
  if (sync == EGL_NO_SYNC_KHR) {
    IMP_LOG(imp::ERROR) << "eglCreateSyncKHR failed: %d" << eglGetError();
    return fenceFd;
  }
  PFNEGLDUPNATIVEFENCEFDANDROIDPROC eglDupNativeFenceFDANDROID_ext =
      (PFNEGLDUPNATIVEFENCEFDANDROIDPROC)eglGetProcAddress(
          "eglDupNativeFenceFDANDROID");
  if (eglDupNativeFenceFDANDROID_ext) {
    fenceFd = eglDupNativeFenceFDANDROID_ext(display, sync);
  } else {
    IMP_LOG(imp::ERROR) << "eglDupNativeFenceFDANDROID not available.";
    return fenceFd;
  }

  PFNEGLDESTROYSYNCKHRPROC eglDestroySyncKHR_ext =
      (PFNEGLDESTROYSYNCKHRPROC)eglGetProcAddress("eglDestroySyncKHR");
  if (eglDestroySyncKHR_ext) {
    eglDestroySyncKHR_ext(display, sync);
  } else {
    IMP_LOG(imp::ERROR) << "eglDestroySyncKHR not available.";
    return fenceFd;
  }

  if (fenceFd == EGL_NO_NATIVE_FENCE_FD_ANDROID) {
    IMP_LOG(imp::ERROR) << "eglDupNativeFenceFDANDROID failed: %d" << eglGetError();
    return fenceFd;
  }

  return fenceFd;
#endif  // IMP_MATERIAL_API(OPENGL) && IMP_PLATFORM(ANDROID)
  return fenceFd;
}

absl::Status XrSessionHost::SetDisplayState(XrHelpers::DisplayState new_state) {
  imp::output::Xr("XrSessionHost::SetDisplayState: %i", new_state);
  if (new_state == display_state_) {
    return absl::OkStatus();
  }

  display_state_ = new_state;

  display_state_statistics_->SetValue(static_cast<int64_t>(display_state_));

  // BUG((broken link)): workaround GltfRenderer crash when display is disabled.
  GltfRenderer::SkinningSystemOverride new_skinning_mode =
      (display_state_ == XrHelpers::DisplayState::kDisplayEnabled)
          ? GltfRenderer::SkinningSystemOverride::kSkinningSystemEnabled
          : GltfRenderer::SkinningSystemOverride::kSkinningSystemDisabled;

  GetView()
      ->GetComponentManager()
      .GetComponentSystem<GltfRenderer>()
      .SetSkinningSystemOverride(new_skinning_mode);

  return absl::OkStatus();
}

absl::StatusOr<XrFrameState> XrSessionHost::WaitFrame() {
  IMP_TRACE();
  

  if (session_ == XR_NULL_HANDLE) {
    return absl::InternalError("session_ is NULL.");
  }

  XrFrameState result = XrFrameState{
      .type = XR_TYPE_FRAME_STATE,
      .next = nullptr,
      .predictedDisplayTime = 0,
      .predictedDisplayPeriod = 0,
      .shouldRender = false,
  };

  XrFrameWaitInfo frame_wait_info = {
      .type = XR_TYPE_FRAME_WAIT_INFO,
      .next = nullptr,
  };

  ScopedDurationMeasurement wait_frame_timing(monitor_.get(),
                                              kXrWaitFrameTiming);
  MP_RETURN_IF_ERROR(ToStatus(xrWaitFrame(session_, &frame_wait_info, &result)));
  {
    SYSTRACE_CONTEXT();
    SYSTRACE_ASYNC_BEGIN("Impress Frame", result.predictedDisplayTime);
  }
  CaptureVsyncTime();
  return result;
}

absl::StatusOr<XrSessionHost::LocateViewsResult> XrSessionHost::LocateViews(
    XrTime predictedDisplayTime) {
  IMP_TRACE();
  

  if (session_ == XR_NULL_HANDLE) {
    return absl::InternalError("session_ is NULL.");
  }

  imp::output::Xr("Calling Xr locate views.");

  bool used_varjo_foveation_previous_frame = use_varjo_foveation_this_frame_;
  if (is_varjo_foveated_rendering_enabled_ && eye_tracking_enabled_) {
    XrSpaceLocation render_gaze_location{.type = XR_TYPE_SPACE_LOCATION,
                                         .next = nullptr};
    xrLocateSpace(render_gaze_space_, view_space_, predictedDisplayTime,
                  &render_gaze_location);
    use_varjo_foveation_this_frame_ =
        (render_gaze_location.locationFlags &
         XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT) != 0;
  }
  should_resize_view_ =
      (used_varjo_foveation_previous_frame != use_varjo_foveation_this_frame_);

  XrViewLocateFoveatedRenderingVARJO view_locate_foveated_rendering{
      .type = XR_TYPE_VIEW_LOCATE_FOVEATED_RENDERING_VARJO, .next = nullptr};
  view_locate_foveated_rendering.foveatedRenderingActive =
      use_varjo_foveation_this_frame_;
  XrViewLocateInfo view_locate_info = {
      .type = XR_TYPE_VIEW_LOCATE_INFO,
      .next = use_varjo_foveation_this_frame_ ? &view_locate_foveated_rendering
                                              : nullptr,
      .viewConfigurationType = view_configuration_type_,
      .displayTime = predictedDisplayTime,
      .space = reference_space_};

  std::vector<XrView> views;
  views.resize(GetActiveViewConfigs()->size(),
               XrView{.type = XR_TYPE_VIEW, .next = nullptr});

  XrViewState view_state{.type = XR_TYPE_VIEW_STATE, .next = nullptr};

  uint32_t view_count;
  MP_RETURN_IF_ERROR(
      ToStatus(xrLocateViews(session_, &view_locate_info, &view_state,
                             views.size(), &view_count, views.data())));

  

  bool is_position_and_orientation_valid =
      ((view_state.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) != 0 &&
       (view_state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) != 0);

  XrViewStateChangedEvent::ViewState next_view_state;
  next_view_state.is_position_tracked =
      (view_state.viewStateFlags & XR_VIEW_STATE_POSITION_TRACKED_BIT);
  next_view_state.is_orientation_tracked =
      (view_state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_TRACKED_BIT);
  next_view_state.is_position_valid =
      (view_state.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT);
  next_view_state.is_orientation_valid =
      (view_state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT);

  // Emit an event every time this status changes.
  // Note that the default message does not have a view_state set, so there will
  // always be a message on the first call.
  if (!previous_view_state_.has_value() ||
      !(previous_view_state_->is_position_tracked ==
            next_view_state.is_position_tracked &&
        previous_view_state_->is_orientation_tracked ==
            next_view_state.is_orientation_tracked &&
        previous_view_state_->is_position_valid ==
            next_view_state.is_position_valid &&
        previous_view_state_->is_orientation_valid ==
            next_view_state.is_orientation_valid)) {
    previous_view_state_ = next_view_state;
    XrViewStateChangedEvent event;
    event.view_state = next_view_state;
    GetView()->GetDispatcher().Send(event);
  }

  LocateViewsStatus status = is_position_and_orientation_valid
                                 ? LocateViewsStatus::kObtainedPose
                                 : LocateViewsStatus::kUnableToObtainPose;

  return LocateViewsResult{.views = std::move(views), .status = status};
}

absl::StatusOr<XrSessionHost::LocateSpaceResult>
XrSessionHost::LocateViewInReference(XrTime predicted_display_time) {
  IMP_TRACE();
  

  imp::output::Xr("Calling Xr locate view in reference space.");

  XrSpaceLocation view_in_reference = {.type = XR_TYPE_SPACE_LOCATION,
                                       .next = nullptr,
                                       .locationFlags = 0,
                                       .pose = {{0, 0, 0, 1}, {0, 0, 0}}};
  MP_RETURN_IF_ERROR(
      ToStatus(xrLocateSpace(view_space_, reference_space_,
                             predicted_display_time, &view_in_reference)));

  bool is_position_and_orientation_valid =
      ((view_in_reference.locationFlags &
        XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
       (view_in_reference.locationFlags &
        XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0);

  const LocateSpaceStatus status = is_position_and_orientation_valid
                                       ? LocateSpaceStatus::kObtainedPose
                                       : LocateSpaceStatus::kUnableToObtainPose;

  return LocateSpaceResult{.location = view_in_reference, .status = status};
}

absl::StatusOr<XrSessionHost::ViewInfo> XrSessionHost::GetLatestViews() {
  if (latest_views_.empty()) {
    return absl::FailedPreconditionError("No latest views available.");
  }
  return XrSessionHost::ViewInfo{
      .views = latest_views_,
      .view_configuration_type =
          use_varjo_foveation_this_frame_
              ? XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO
              : XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
}

void XrSessionHost::ResizeImpressView() {
  const XrViewConfigurationView& left_eye_view_config =
      GetActiveViewConfigs()->front();
  uint2 view_size{GetViewWidth(left_eye_view_config),
                  GetViewHeight(left_eye_view_config)};
  imp::output::Xr("Resizing Impress View with size %s",
                  imp::ToString(view_size));
  Resize(view_size, float2{1.0f, 1.0f});

  imp::output::Xr("Completed calling Impress resize.");
}

absl::Status XrSessionHost::BeginFrame() {
  // WARNING: This method can be called from both the Impress Thread and
  // Filament's Render Thread. As such, it is generally not safe to call either
  // Impress View APIs or Filament APIs because they cannot be called from
  // filament's render thread.
  //
  // When a frame is being discarded, this will be called from the Impress
  // thread. When a frame is being rendered, it will be called from filament's
  // render thread through XrPlatform.

  IMP_TRACE();
  imp::output::Xr("Calling Xr begin frame.");
  filament_begin_timestamp_ = GetFilamentTimeNow();
  XrFrameBeginInfo frame_begin_info{.type = XR_TYPE_FRAME_BEGIN_INFO,
                                    .next = nullptr};
  XrResult result = xrBeginFrame(session_, &frame_begin_info);
  if (result > XR_SUCCESS) {
    // All non-negative values other than XR_SUCCESS are considered success. But
    // we still want to log the value just in case.
    imp::output::Xr(
        "Successful non-XR_SUCCESS return code for xrBeginFrame: %i", result);
  }
  return ToStatus(result);
}

absl::Status XrSessionHost::EndFrame(XrSwapchain swapchain,
                                     XrSwapchain depth_swapchain) {
  // WARNING: This method can be called from both the Impress Thread and
  // Filament's Render Thread. As such, it is generally not safe to call either
  // Impress View APIs or Filament APIs because they cannot be called from
  // filament's render thread.
  //
  // When a frame is being discarded, this will be called from the Impress
  // thread. When a frame is being rendered, it will be called from filament's
  // render thread through XrPlatform.

  IMP_TRACE();

  QueuedFrameInfo frame_info;
  {
    absl::MutexLock lock(&frame_queue_mutex_);
    frame_info = std::move(frame_queue_.front());
    imp::output::Xr("Calling Xr end frame. displayTime=%i, queueSize=%i",
                    frame_info.display_time, frame_queue_.size());
    frame_queue_.pop();
    prev_filament_thread_duration_ = filament_thread_duration_;
    filament_thread_duration_.reset();
    if (frame_info.before_end_frame_callback) {
      frame_info.before_end_frame_callback();
    }
  }

  std::vector<XrCompositionLayerProjectionView> layer_views;
  layer_depth_infos_.clear();
  layer_depth_infos_.reserve(frame_info.views.size());
  int32_t x_offset = 0;
  int32_t y_offset = 0;
  for (int view_index = 0; view_index < frame_info.views.size(); ++view_index) {
    XrViewConfigurationView& view_config =
        frame_info.should_render_varjo_foveation
            ? varjo_foveation_view_configs_[view_index]
            : view_configs_[view_index];
    int32_t width = static_cast<int32_t>(GetViewWidth(view_config));
    int32_t height = static_cast<int32_t>(GetViewHeight(view_config));

    XrSwapchainSubImage color_sub_image = {
        .swapchain = swapchain,
        .imageRect =
            {
                .offset = {x_offset, y_offset},
                .extent = {width, height},
            },
    };
    if (IsMultiviewStereo()) {
      color_sub_image.imageArrayIndex = static_cast<uint32_t>(view_index);
    } else {
      // Move the start offset of X-axis for the next view.
      x_offset += width;
    }

    XrCompositionLayerDepthInfoKHR* next_depth_info = nullptr;
    if (depth_swapchain != XR_NULL_HANDLE &&
        is_composition_layer_depth_enabled_) {
      XrSwapchainSubImage depth_sub_image = color_sub_image;
      depth_sub_image.swapchain = depth_swapchain;
      layer_depth_infos_.push_back({
          .type = XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR,
          .next = nullptr,
          .subImage = depth_sub_image,
          .minDepth = 0.f,
          .maxDepth = 1.f,
          .nearZ = GetView()->GetCameraManager().GetCamera()->GetFarClip(),
          .farZ = GetView()->GetCameraManager().GetCamera()->GetNearClip(),
      });
      next_depth_info = &layer_depth_infos_[view_index];
    }
    imp::output::Xr("Ending frame of depth swapchain: %d; %d",
                    depth_swapchain != XR_NULL_HANDLE,
                    next_depth_info != nullptr);
    layer_views.push_back({
        .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW,
        .next = next_depth_info,
        .pose = frame_info.views[view_index].pose,
        .fov = frame_info.views[view_index].fov,
        .subImage = color_sub_image,
    });
  }

  XrCompositionLayerDepthTestFB depth_test;
  void* next = nullptr;
  if (depth_swapchain != XR_NULL_HANDLE &&
      is_composition_layer_depth_enabled_) {
    depth_test.type = XR_TYPE_COMPOSITION_LAYER_DEPTH_TEST_FB;
    depth_test.next = nullptr;
    depth_test.depthMask = XR_TRUE;
    depth_test.compareOp = XrCompareOpFB::XR_COMPARE_OP_LESS_OR_EQUAL_FB;
    next = &depth_test;
  }

  std::vector<XrCompositionLayerBaseHeader*> layers;
  XrCompositionLayerProjection layer = {
      .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION,
      .next = next,
      .layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT,
      .space = reference_space_,
      .viewCount = static_cast<uint32_t>(layer_views.size()),
      .views = layer_views.data(),
  };
  layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer));

  // Add non-projection composition layers.
  for (auto& [layer, weight] : composition_layers_) {
    layers.push_back(layer);
  }

  // Sort layers based on weight
  // Layers towards the end of the array are drawn on top of layers towards the
  // beginning of the array.
  std::sort(
      layers.begin(), layers.end(),
      [&layer, this](XrCompositionLayerBaseHeader* a,
                     XrCompositionLayerBaseHeader* b) {
        int a_weight;
        int b_weight;
        // If the layer is our projection layer, treat it's weight as 0;
        if (a == reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer)) {
          a_weight = 0;
        } else {
          a_weight = composition_layers_[a];
        }
        if (b == reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer)) {
          b_weight = 0;
        } else {
          b_weight = composition_layers_[b];
        }
        // Using > so that smaller weights are drawn later
        return a_weight > b_weight;
      });

  XrFrameEndInfo frame_end_info{
      .type = XR_TYPE_FRAME_END_INFO,
      .next = nullptr,
      .displayTime = frame_info.display_time,
      .environmentBlendMode = environment_blend_mode_,
      .layerCount = static_cast<uint32_t>(layers.size()),
      .layers = layers.data(),
  };
  absl::Status return_value = ToStatus(xrEndFrame(session_, &frame_end_info));
  {
    SYSTRACE_CONTEXT();
    SYSTRACE_ASYNC_END("Impress Frame", frame_info.display_time);
  }
  filament_thread_duration_ = GetFilamentTimeNow() - filament_begin_timestamp_;

  if (frame_info.after_end_frame_callback) {
    // TODO: Pass fence returned by xrEndFrame.
    frame_info.after_end_frame_callback(-1);
  }

  return return_value;
}

absl::Status XrSessionHost::BeginAndDiscardFrame(XrTime predictedDisplayTime) {
  IMP_TRACE();
  

  // We use a fence to guarantee that we don't create
  // sequencing issues with any other frames in the queue on the render
  // thread. That can only occur if shouldRender returns false because we've
  // changed states after rendering has begun.
  //
  // This also only needs to be done if the previous frame succeeded, since
  // otherwise we know that the previous frame was also discarded and there is
  // no possibility of a race condition with the render thread.
  if (did_last_advance_frame_succeed_) {
    filament::Fence* fence = GetEngine()->createFence();
    fence->wait();
    GetEngine()->destroy(fence);
  }

  // When the XrSession is initializing we need to call xrEndFrame to ensure
  // that the session can be synchronized. (See
  // https://registry.khronos.org/OpenXR/specs/1.1-khr/html/xrspec.html#session-lifecycle).
  //
  // However, once we are in the visible or focused states, we no longer need to
  // call xrEndFrame if we are not rendering any layers once we've called
  // xrEndFrame at least once with no layers to inform the XR runtime we have
  // nothing to render.
  bool skip_end_frame = session_state_ == XR_SESSION_STATE_VISIBLE ||
                        session_state_ == XR_SESSION_STATE_FOCUSED;

  // Since the last frame succeeded we have just entered a non-visible state, so
  // don't skip calling xrEndFrame.
  if (did_last_advance_frame_succeed_) {
    skip_end_frame = false;
  }

  did_last_advance_frame_succeed_ = false;

  // Even when discarding the frame, still ensure that the foreground executor
  // is advanced. This makes it possible to continue to load assets while Xr
  // is still booting up, allowing for better parallelization of work and
  // faster startup times.
  GetView()->AdvanceForegroundExecutor();

  // After advancing the foreground executor, flush the engine.
  //
  // This is important because it will allow filament to flush any
  // assets that were loaded or other work that was done in the foreground
  // executor or from the regular simulation loop.
  renderer_->skipFrame();

  MP_RETURN_IF_ERROR(BeginFrame());

  absl::Status result = absl::OkStatus();
  if (skip_end_frame) {
    imp::output::Xr("Discard xrEndFrame skipped.");
  } else {
    XrFrameEndInfo frame_end_info{
        .type = XR_TYPE_FRAME_END_INFO,
        .next = nullptr,
        .displayTime = predictedDisplayTime,
        .environmentBlendMode = environment_blend_mode_,
        .layerCount = 0,
        .layers = nullptr,
    };

    imp::output::Xr("Discard xrEndFrame called.");
    result = ToStatus(xrEndFrame(session_, &frame_end_info));
  }

  {
    SYSTRACE_CONTEXT();
    SYSTRACE_ASYNC_END("Impress Frame", predictedDisplayTime);
  }
  return result;
}

void XrSessionHost::PerformRender(filament::View* view) {
  IMP_TRACE();
  if (is_enhanced_stereoscopic_rendering_enabled_) {
    PerformEnhancedStereoscopicRender(view);
  } else {
    PerformNaiveStereoscopicRender(view);
  }
}

void XrSessionHost::PerformNaiveStereoscopicRender(filament::View* view) {
  // In the future we should pass through a viewport config instead of
  // directly using the view_configs.
  int viewport_offset_x = 0;
  for (int i = 0; i < latest_views_.size(); ++i) {
    UpdateCameraFromXrView(latest_views_[i]);
    uint32_t view_width = GetViewWidth(GetActiveViewConfigs()->at(i));
    view->setViewport(
        filament::Viewport{viewport_offset_x, 0, view_width,
                           GetViewHeight(GetActiveViewConfigs()->at(i))});
    viewport_offset_x += view_width;
    renderer_->render(view);
  };
}

void XrSessionHost::PerformEnhancedStereoscopicRender(filament::View* view) {
  if (view->isPostProcessingEnabled()) {
    IMP_LOG(imp::FATAL) << "Post Processing is not supported with instanced rendering.";
  }
  filament::Camera* camera = &view->getCamera();
  SetCustomEyeProjectionOnCamera(camera, latest_views_);

  if (!is_enhanced_stereoscopic_rendering_initialized_) {
    view->setStereoscopicOptions({.enabled = true});
    // Each view is rendered side-by-side in one texture for instanced-stereo,
    // and multiple layers of a texture for multiview-stereo.
    uint2 display_size = use_varjo_foveation_this_frame_
                             ? varjo_foveation_display_size_
                             : display_size_;
    view->setViewport(filament::Viewport{0, 0, display_size.x, display_size.y});

    imp::output::Xr("Instanced rendering is now initialized.");
    is_enhanced_stereoscopic_rendering_initialized_ = true;
  }
  SetEyeModelMatrixOnCamera(GetEngine(), camera, latest_views_);
  renderer_->render(view);
}

filament::Engine::StereoscopicType XrSessionHost::GetStereoscopicType() const {
  if (!is_enhanced_stereoscopic_rendering_enabled_) {
    return filament::Engine::StereoscopicType::NONE;
  }
#if IMP_ENABLE_STEREO_TYPE_MULTIVIEW
  return filament::Engine::StereoscopicType::MULTIVIEW;
#else
  return filament::Engine::StereoscopicType::INSTANCED;
#endif
}

bool XrSessionHost::IsMultiviewStereo() const {
  return GetStereoscopicType() == filament::Engine::StereoscopicType::MULTIVIEW;
}

uint32_t XrSessionHost::GetLogicalEyeCount() const {
  return view_configuration_type_ ==
                 XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO
             ? 4
             : 2;
}

void XrSessionHost::UpdateCameraFromXrView(const XrView& view) {
  // Get the camera.
  ComponentHandle<CameraComponent> camera =
      GetView()->GetCameraManager().GetCamera();

  // Create and assign the projection matrix.
  mat4 projection_matrix = GetProjectionMatrix(view.fov, camera->GetNearClip(),
                                               camera->GetFarClip());
  camera->SetProjectionMatrix(projection_matrix);

  // Create and assign the transform.
  imp::Transform<float> transform = ToTransform(view.pose);
  camera->GetNode()->SetLocalTrs(transform.AsMat4());
}

void XrSessionHost::UpdateCameraFromXrPose(const XrPosef& pose) {
  // Get the camera.
  ComponentHandle<CameraComponent> camera =
      GetView()->GetCameraManager().GetCamera();

  // Create and assign the transform.
  imp::Transform<float> transform = ToTransform(pose);
  camera->GetNode()->SetLocalTrs(transform.AsMat4());
}

const std::vector<XrViewConfigurationView>*
XrSessionHost::GetActiveViewConfigs() const {
  return use_varjo_foveation_this_frame_ ? &varjo_foveation_view_configs_
                                         : &view_configs_;
}

uint32_t XrSessionHost::GetViewWidth(
    const XrViewConfigurationView& view_config) {
  return use_max_swapchain_size_ ? view_config.maxImageRectWidth
                                 : view_config.recommendedImageRectWidth *
                                       swapchain_size_multiplier_;
}

uint32_t XrSessionHost::GetViewHeight(
    const XrViewConfigurationView& view_config) {
  return use_max_swapchain_size_ ? view_config.maxImageRectHeight
                                 : view_config.recommendedImageRectHeight *
                                       swapchain_size_multiplier_;
}

uint2 XrSessionHost::CalculateDisplaySize(
    const std::vector<XrViewConfigurationView>& view_configs) {
  uint32_t width = 0;
  uint32_t height = 0;
  for (const XrViewConfigurationView& view_config : view_configs) {
    view_sample_count_ = std::max(view_sample_count_,
                                  view_config.recommendedSwapchainSampleCount);
    if (IsMultiviewStereo()) {
      width = std::max(width, GetViewWidth(view_config));
    } else {
      width += GetViewWidth(view_config);
    }
    height = std::max(height, GetViewHeight(view_config));
  }
  return {width, height};
}

uint2 XrSessionHost::GetDisplaySize() const { return display_size_; }

uint2 XrSessionHost::GetVarjoFoveationDisplaySize() const {
  return varjo_foveation_display_size_;
}

uint32_t XrSessionHost::GetViewSampleCount() const {
  return view_sample_count_;
}

XrInstance XrSessionHost::GetXrInstance() const { return instance_; }

XrSession XrSessionHost::GetXrSession() const { return session_; }

XrSystemId XrSessionHost::GetSystemId() const { return system_id_; }

XrSpace XrSessionHost::GetXrSpace() const { return reference_space_; }

XrTime XrSessionHost::GetPredictedDisplayTime() const {
  

  return latest_predicted_display_time_;
}

absl::Status XrSessionHost::ToStatus(XrResult result) const {
  return ::imp::ToStatus(instance_, result);
}

#if IMP_RUNTIME(DEV)
std::unique_ptr<imp::editor::EditorPlugin> XrSessionHost::CreateEditorPlugin() {
  return std::make_unique<imp::editor::XrEditorPlugin>(GetView());
}
#endif

bool XrSessionHost::IsCompositionLayerDepthEnabled() {
  return is_composition_layer_depth_enabled_;
}

void XrSessionHost::SetCompositionLayerDepthEnabled(
    bool composition_layer_depth_enabled) {
  is_composition_layer_depth_enabled_ = composition_layer_depth_enabled;
}

ContentSecurityLevel XrSessionHost::GetContentSecurityLevel() {
  return content_security_level_;
}

void XrSessionHost::SetContentSecurityLevel(ContentSecurityLevel security) {
  content_security_level_ = security;
}

void XrSessionHost::SetFoveationLevel(
    XrFoveationLevelFB xr_foveation_level_fb) {
  current_foveation_level_ = xr_foveation_level_fb;
}

XrFoveationLevelFB XrSessionHost::GetCurrentFoveationLevel() {
  return current_foveation_level_;
}

int XrSessionHost::GetMsaaSampleCount() const { return msaa_sample_count_; }

filament::Engine::Config XrSessionHost::GetEngineConfig() {
  filament::Engine::Config engine_config =
      window::FilamentHost::GetEngineConfig();
  engine_config.stereoscopicEyeCount = GetLogicalEyeCount();
  engine_config.stereoscopicType = GetStereoscopicType();
  return engine_config;
};

void XrSessionHost::SetBeforeEndFrameCallback(Invocable<void()> callback) {
  absl::MutexLock lock(&frame_queue_mutex_);
  if (frame_queue_.empty()) {
    return;
  }
  frame_queue_.back().before_end_frame_callback = std::move(callback);
}

void XrSessionHost::SetAfterEndFrameCallback(Invocable<void(int)> callback) {
  absl::MutexLock lock(&frame_queue_mutex_);
  if (frame_queue_.empty()) {
    return;
  }
  frame_queue_.back().after_end_frame_callback = std::move(callback);
}

absl::Span<const char* const>& XrSessionHost::GetExtensionsToLoad() {
  static absl::Span<const char* const> extensions_to_load(
      kOpenXRExtensionsCore);
  return extensions_to_load;
}

absl::Span<const char* const>& XrSessionHost::GetOptionalExtensionsToLoad() {
  static absl::Span<const char* const> extensions_to_load;
  return extensions_to_load;
}
std::optional<XrSystemProperties> XrSessionHost::GetSystemProperties() const {
  XrSystemProperties system_properties{.type = XR_TYPE_SYSTEM_PROPERTIES,
                                       .next = XR_NULL_HANDLE};
  if (xrGetSystemProperties(GetXrInstance(), GetSystemId(),
                            &system_properties) != XR_SUCCESS) {
    return std::nullopt;
  }
  return system_properties;
}

RobinSet<std::string> XrSessionHost::GetEnabledExtensions() const {
  return enabled_extensions_;
}

void XrSessionHost::AddCompositionLayer(XrCompositionLayerBaseHeader* layer,
                                        int weight) {
  composition_layers_[layer] = weight;
}

void XrSessionHost::RemoveCompositionLayer(
    XrCompositionLayerBaseHeader* layer) {
  auto it = composition_layers_.find(layer);
  if (it == composition_layers_.end()) {
    return;
  }
  composition_layers_.erase(it);
}

}  // namespace imp
