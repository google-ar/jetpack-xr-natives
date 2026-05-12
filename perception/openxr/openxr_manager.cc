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

#include "openxr/openxr_manager.h"

#include <jni.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_reflection.h>
#include <openxr/public/all_extensions.h>
#include <openxr/public/xr_androidx2_geospatial_anchor.h>
#include <openxr/public/xr_androidx2_geospatial_streetscape.h>
#include <stdbool.h>
#include <sys/types.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "openxr/openxr.h"
#include "absl/base/no_destructor.h"
#include "absl/log/log.h"
#include "absl/numeric/int128.h"
#include "absl/synchronization/mutex.h"
#include "openxr/openxr_manager_clock.h"
#include "openxr/openxr_manager_utils.h"

namespace androidx::xr::openxr {
namespace {
// TODO: (broken link) - Inject the application name for the OpenXR session.
constexpr char kApplicationName[] = "JetpackXrCore";

struct OpenXrExtension {
  const char* name;
  std::vector<const char*> dependencies;
};

// TODO: (broken link) - Change this from a global list to something more
// flexible. Also split up between "required" and "optional" extensions, and
// check against xrEnumerateInstanceExtensionProperties()
const std::array<std::string, 13> kRequiredExtensions = {
    // (broken link) start
    XR_ANDROID_ANCHOR_SHARING_EXPORT_EXTENSION_NAME,
    XR_ANDROID_DEPTH_TEXTURE_EXTENSION_NAME,
    XR_ANDROID_DEVICE_ANCHOR_PERSISTENCE_EXTENSION_NAME,
    XR_ANDROID_EYE_TRACKING_EXTENSION_NAME,
    XR_ANDROID_FACE_TRACKING_EXTENSION_NAME,
    XR_ANDROID_RAYCAST_EXTENSION_NAME,
    XR_ANDROID_TRACKABLES_EXTENSION_NAME,
    XR_ANDROID_TRACKABLES_OBJECT_EXTENSION_NAME,
    XR_ANDROID_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME,
    XR_EXT_FUTURE_EXTENSION_NAME,
    XR_EXT_HAND_TRACKING_EXTENSION_NAME,
    XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME,
    XR_MND_HEADLESS_EXTENSION_NAME,
    // (broken link) end
};

// Extensions must be listed after their dependencies.
const std::array<OpenXrExtension, 7> kOptionalExtensions = {{
    {XR_ANDROID_GEOSPATIAL_EXTENSION_NAME, {XR_EXT_FUTURE_EXTENSION_NAME}},
    {XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME, {XR_EXT_FUTURE_EXTENSION_NAME}},
    {XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME,
     {XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME}},
    {XR_ANDROIDX2_GEOSPATIAL_ANCHOR_EXTENSION_NAME,
     {XR_ANDROID_GEOSPATIAL_EXTENSION_NAME, XR_EXT_FUTURE_EXTENSION_NAME,
      XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME,
      XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME}},
    {XR_ANDROIDX2_GEOSPATIAL_STREETSCAPE_EXTENSION_NAME,
     {XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME}},
    {XR_ANDROID_SPATIAL_ANCHOR_SPACE_EXTENSION_NAME,
     {XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME}},
    {XR_ANDROID_GOOGLE_CLOUD_AUTH_EXTENSION_NAME,
     {XR_EXT_FUTURE_EXTENSION_NAME}},
}};

const std::array<std::string, 7> kGeospatialExtensions = {
    XR_ANDROID_GEOSPATIAL_EXTENSION_NAME,
    XR_ANDROIDX2_GEOSPATIAL_ANCHOR_EXTENSION_NAME,
    XR_ANDROIDX2_GEOSPATIAL_STREETSCAPE_EXTENSION_NAME,
    XR_ANDROID_SPATIAL_ANCHOR_SPACE_EXTENSION_NAME,
    XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME,
    XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME,
    XR_ANDROID_GOOGLE_CLOUD_AUTH_EXTENSION_NAME,
};

constexpr XrPosef kIdentityPose = {
    .orientation = {.x = 0, .y = 0, .z = 0, .w = 1},
    .position = {.x = 0, .y = 0, .z = 0},
};

constexpr XrSpaceLocationFlags kPoseValidFlags =
    XR_SPACE_LOCATION_POSITION_VALID_BIT |
    XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;

constexpr XrViewStateFlags kViewStateValidFlags =
    XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT;

constexpr XrDepthSwapchainCreateFlagsANDROID kDepthSwapchainRawOnlyFlags =
    XR_DEPTH_SWAPCHAIN_CREATE_RAW_DEPTH_IMAGE_BIT_ANDROID |
    XR_DEPTH_SWAPCHAIN_CREATE_RAW_CONFIDENCE_IMAGE_BIT_ANDROID;

constexpr XrDepthSwapchainCreateFlagsANDROID kDepthSwapchainSmoothOnlyFlags =
    XR_DEPTH_SWAPCHAIN_CREATE_SMOOTH_DEPTH_IMAGE_BIT_ANDROID |
    XR_DEPTH_SWAPCHAIN_CREATE_SMOOTH_CONFIDENCE_IMAGE_BIT_ANDROID;

// Maps the XrViewConfigurationType to the number of views for that view type.
const int kViewTypeStereoViewCount = 2;

#define XR_ENUM_CASE_STR(name, val) \
  case name:                        \
    return #name;

// Returns a string of the enum represented by this XrResult value.
constexpr const char* XrEnumStr(XrResult e) {
  switch (e) {
    XR_LIST_ENUM_XrResult(XR_ENUM_CASE_STR) default : return "Unknown";
  }
}

// Returns a string of the enum represented by this XrSessionState value.
constexpr const char* XrSessionStateEnumStr(XrSessionState e) {
  switch (e) {
    XR_LIST_ENUM_XrSessionState(XR_ENUM_CASE_STR) default : return "Unknown";
  }
}

constexpr const char* XrPlaneTypeEnumStr(XrPlaneTypeANDROID e) {
  switch (e) {
    XR_LIST_ENUM_XrPlaneTypeANDROID(XR_ENUM_CASE_STR) default
        : return "Unknown";
  }
}

constexpr const char* XrPlaneLabelEnumStr(XrPlaneLabelANDROID e) {
  switch (e) {
    XR_LIST_ENUM_XrPlaneLabelANDROID(XR_ENUM_CASE_STR) default
        : return "Unknown";
  }
}

constexpr const char* XrDepthCameraResolutionEnumStr(
    XrDepthCameraResolutionANDROID e) {
  switch (e) {
    XR_LIST_ENUM_XrDepthCameraResolutionANDROID(XR_ENUM_CASE_STR) default
        : return "Unknown";
  }
}

// Returns false and logs an error if the result is not XR_SUCCESS.
#define XR_RETURN_IF_FAILED(expr)                                     \
  do {                                                                \
    const XrResult xr_result = (expr);                                \
    if (XR_FAILED(xr_result)) {                                       \
      LOG(ERROR) << #expr << " failed with " << XrEnumStr(xr_result); \
      return false;                                                   \
    } else {                                                          \
      VLOG(3) << #expr << " succeeded!";                              \
    }                                                                 \
  } while (false)

// Returns XrResult and logs an error if the result is not XR_SUCCESS.
#define XR_RETURN_RESULT_IF_FAILED(expr)                              \
  do {                                                                \
    const XrResult xr_result = (expr);                                \
    if (XR_FAILED(xr_result)) {                                       \
      LOG(ERROR) << #expr << " failed with " << XrEnumStr(xr_result); \
      return xr_result;                                               \
    } else {                                                          \
      VLOG(3) << #expr << " succeeded!";                              \
    }                                                                 \
  } while (false)

// Gets the depth image width and height for a given resolution. Returns false
// if the resolution is not supported.
bool GetDepthCameraImageWidthAndHeight(
    const XrDepthCameraResolutionANDROID& resolution, int* width, int* height) {
  if (width == nullptr || height == nullptr) {
    LOG(ERROR) << "width and height must not be null";
    return false;
  }

  switch (resolution) {
    case XR_DEPTH_CAMERA_RESOLUTION_80x80_ANDROID:
      *width = 80;
      *height = 80;
      return true;
    case XR_DEPTH_CAMERA_RESOLUTION_160x160_ANDROID:
      *width = 160;
      *height = 160;
      return true;
    case XR_DEPTH_CAMERA_RESOLUTION_320x320_ANDROID:
      *width = 320;
      *height = 320;
      return true;
    case XR_DEPTH_CAMERA_RESOLUTION_MAX_ENUM_ANDROID:
    default:
      return false;
  }
}

}  // namespace

OpenXrManager::CreateAnchorResult OpenXrManager::MapAnchorCreateResult(
    XrResult xr_result) {
  switch (static_cast<int>(xr_result)) {
    case XR_SUCCESS:
      return OpenXrManager::CreateAnchorResult::kSuccess;
    case XR_ERROR_LIMIT_REACHED:
      return OpenXrManager::CreateAnchorResult::kErrorLimitReached;
    case XR_ERROR_RUNTIME_FAILURE:
      return OpenXrManager::CreateAnchorResult::kErrorRuntimeFailure;
    case XR_ERROR_GEOSPATIAL_CLOUD_AUTH_FAILED_ANDROID:
      return OpenXrManager::CreateAnchorResult::kErrorCloudAuthFailed;
    case XR_ERROR_GEOSPATIAL_TRACKER_NOT_RUNNING_ANDROID:
      return OpenXrManager::CreateAnchorResult::
          kErrorGeospatialTrackerNotRunning;
    case XR_ERROR_GEOSPATIAL_COORDINATES_INVALID_ANDROID:
      return OpenXrManager::CreateAnchorResult::
          kErrorGeospatialCoordinatesInvalid;
    case XR_ERROR_SURFACE_ANCHOR_LOCATION_UNSUPPORTED_ANDROIDX2:
      return OpenXrManager::CreateAnchorResult::
          kErrorSurfaceAnchorLocationUnsupported;
    default:
      return OpenXrManager::CreateAnchorResult::kErrorRuntimeFailure;
  }
}

OpenXrManagerClockInterface* OpenXrManager::GetOpenXrManagerClock() {
  // The clock should be a singleton. So we are creating it here without any
  // destruction. It will not be destroyed which is ok because it is a singleton
  // that needs to be alive for the entire duration of the process.
  static OpenXrManagerClock* kOpenXrManagerClock = new OpenXrManagerClock();
  return kOpenXrManagerClock;
}

OpenXrManager& OpenXrManager::GetOpenXrManager() {
  return GetOpenXrManager(GetOpenXrManagerClock());
}

OpenXrManager& OpenXrManager::GetOpenXrManager(
    OpenXrManagerClockInterface* clock) {
  static absl::NoDestructor<OpenXrManager> kOpenXrManager(clock);
  return *kOpenXrManager;
}

bool OpenXrManager::InitExtensionFunctions() {
  absl::MutexLock lock(mutex_);
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrConvertTimespecTimeToTimeKHR",
                            (PFN_xrVoidFunction*)(&convert_time_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrCreateTrackableTrackerANDROID",
                            (PFN_xrVoidFunction*)(&create_trackable_tracker_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrGetAllTrackablesANDROID",
                            (PFN_xrVoidFunction*)(&get_all_trackables_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrGetTrackablePlaneANDROID",
                            (PFN_xrVoidFunction*)(&get_trackable_plane_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrGetTrackableObjectANDROID",
                            (PFN_xrVoidFunction*)(&get_trackable_object_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrDestroyTrackableTrackerANDROID",
      (PFN_xrVoidFunction*)(&destroy_trackable_tracker_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrCreateAnchorSpaceANDROID",
                            (PFN_xrVoidFunction*)(&create_anchor_space_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrShareAnchorANDROID",
                            (PFN_xrVoidFunction*)(&share_anchor_)));
  // Set up persistence functions.
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrCreateDeviceAnchorPersistenceANDROID",
                            reinterpret_cast<PFN_xrVoidFunction*>(
                                &create_device_anchor_persistence_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrDestroyDeviceAnchorPersistenceANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(
          &destroy_device_anchor_persistence_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrPersistAnchorANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&persist_anchor_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrEnumeratePersistedAnchorsANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&enumerate_persisted_anchors_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrGetAnchorPersistStateANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&get_anchor_persist_state_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrUnpersistAnchorANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&unpersist_anchor_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrCreatePersistedAnchorSpaceANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&create_persisted_anchor_space_)));
  // Hit test functions.
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrRaycastANDROID",
                            reinterpret_cast<PFN_xrVoidFunction*>(&raycast_)));
  // Depth functions.
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrCreateDepthSwapchainANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&create_depth_swapchain_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrDestroyDepthSwapchainANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&destroy_depth_swapchain_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrEnumerateDepthSwapchainImagesANDROID",
                            reinterpret_cast<PFN_xrVoidFunction*>(
                                &enumerate_depth_swapchain_images_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrEnumerateDepthResolutionsANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&enumerate_depth_resolutions_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrAcquireDepthSwapchainImagesANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&acquire_depth_swapchain_images_)));
  // Hand functions.
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrCreateHandTrackerEXT",
      reinterpret_cast<PFN_xrVoidFunction*>(&create_hand_tracker_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrDestroyHandTrackerEXT",
      reinterpret_cast<PFN_xrVoidFunction*>(&destroy_hand_tracker_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrLocateHandJointsEXT",
      reinterpret_cast<PFN_xrVoidFunction*>(&locate_hand_joints_)));

  // Face functions.
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrCreateFaceTrackerANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&create_face_tracker_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrDestroyFaceTrackerANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&destroy_face_tracker_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrGetFaceCalibrationStateANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&get_face_calibration_state_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrGetFaceStateANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&get_face_state_)));

  // Eye functions.
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrCreateEyeTrackerANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&create_eye_tracker_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrDestroyEyeTrackerANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&destroy_eye_tracker_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrGetFineTrackingEyesInfoANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&get_fine_tracking_eyes_info_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrGetCoarseTrackingEyesInfoANDROID",
      reinterpret_cast<PFN_xrVoidFunction*>(&get_coarse_tracking_eyes_info_)));

  // Future functions.
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrCancelFutureEXT",
      reinterpret_cast<PFN_xrVoidFunction*>(&cancel_future_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrPollFutureEXT",
      reinterpret_cast<PFN_xrVoidFunction*>(&poll_future_)));

  std::vector<std::string> enabled_exts;
  GetEnabledExtensions(enabled_exts);

  auto all_geospatial_present =
      std::all_of(kGeospatialExtensions.begin(), kGeospatialExtensions.end(),
                  [&](const std::string& ext) {
                    return std::find(enabled_exts.begin(), enabled_exts.end(),
                                     ext) != enabled_exts.end();
                  });

  if (std::find(enabled_exts.begin(), enabled_exts.end(),
                XR_ANDROID_GOOGLE_CLOUD_AUTH_EXTENSION_NAME) !=
      enabled_exts.end()) {
    cloud_auth_exts_loaded_ = true;
    // Google Cloud Authentication functions.
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrSetGoogleCloudAuthAsyncANDROID",
        reinterpret_cast<PFN_xrVoidFunction*>(&set_google_cloud_auth_async_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrSetGoogleCloudAuthCompleteANDROID",
        reinterpret_cast<PFN_xrVoidFunction*>(
            &set_google_cloud_auth_complete_)));
  }

  if (all_geospatial_present) {
    // Spatial Entities functions.
    XR_RETURN_IF_FAILED(
        xrGetInstanceProcAddr(instance_, "xrEnumerateSpatialCapabilitiesEXT",
                              reinterpret_cast<PFN_xrVoidFunction*>(
                                  &enumerate_spatial_capabilities_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrEnumerateSpatialCapabilityComponentTypesEXT",
        reinterpret_cast<PFN_xrVoidFunction*>(
            &enumerate_spatial_capability_component_types_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrEnumerateSpatialCapabilityFeaturesEXT",
        reinterpret_cast<PFN_xrVoidFunction*>(
            &enumerate_spatial_capability_features_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCreateSpatialContextAsyncEXT",
        reinterpret_cast<PFN_xrVoidFunction*>(&create_spatial_context_async_)));
    XR_RETURN_IF_FAILED(
        xrGetInstanceProcAddr(instance_, "xrCreateSpatialContextCompleteEXT",
                              reinterpret_cast<PFN_xrVoidFunction*>(
                                  &create_spatial_context_complete_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrDestroySpatialContextEXT",
        reinterpret_cast<PFN_xrVoidFunction*>(&destroy_spatial_context_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCreateSpatialDiscoverySnapshotAsyncEXT",
        reinterpret_cast<PFN_xrVoidFunction*>(
            &create_spatial_discovery_snapshot_async_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCreateSpatialDiscoverySnapshotCompleteEXT",
        reinterpret_cast<PFN_xrVoidFunction*>(
            &create_spatial_discovery_snapshot_complete_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrQuerySpatialComponentDataEXT",
        reinterpret_cast<PFN_xrVoidFunction*>(&query_spatial_component_data_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrDestroySpatialSnapshotEXT",
        reinterpret_cast<PFN_xrVoidFunction*>(&destroy_spatial_snapshot_)));
    XR_RETURN_IF_FAILED(
        xrGetInstanceProcAddr(instance_, "xrCreateSpatialEntityFromIdEXT",
                              reinterpret_cast<PFN_xrVoidFunction*>(
                                  &create_spatial_entity_from_id_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrDestroySpatialEntityEXT",
        reinterpret_cast<PFN_xrVoidFunction*>(&destroy_spatial_entity_)));
    XR_RETURN_IF_FAILED(
        xrGetInstanceProcAddr(instance_, "xrCreateSpatialUpdateSnapshotEXT",
                              reinterpret_cast<PFN_xrVoidFunction*>(
                                  &create_spatial_update_snapshot_)));

    // Spatial Anchors functions.
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCreateSpatialAnchorEXT",
        reinterpret_cast<PFN_xrVoidFunction*>(&create_spatial_anchor_)));

    // Spatial Anchor Space functions.
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCreateSpatialAnchorSpaceFromIdANDROID",
        reinterpret_cast<PFN_xrVoidFunction*>(
            &create_spatial_anchor_space_from_id_)));

    // Geospatial functions.
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCreateGeospatialTrackerANDROID",
        reinterpret_cast<PFN_xrVoidFunction*>(&create_geospatial_tracker_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrDestroyGeospatialTrackerANDROID",
        reinterpret_cast<PFN_xrVoidFunction*>(&destroy_geospatial_tracker_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrLocateGeospatialPoseFromPoseANDROID",
        reinterpret_cast<PFN_xrVoidFunction*>(
            &locate_geospatial_pose_from_pose_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrLocateGeospatialPoseANDROID",
        reinterpret_cast<PFN_xrVoidFunction*>(&locate_geospatial_pose_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCreateGeospatialAnchorANDROIDX2",
        reinterpret_cast<PFN_xrVoidFunction*>(&create_geospatial_anchor_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCreateSurfaceAnchorAsyncANDROIDX2",
        reinterpret_cast<PFN_xrVoidFunction*>(&create_surface_anchor_async_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCreateSurfaceAnchorCompleteANDROIDX2",
        reinterpret_cast<PFN_xrVoidFunction*>(
            &create_surface_anchor_complete_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCheckVpsAvailabilityAsyncANDROID",
        reinterpret_cast<PFN_xrVoidFunction*>(&check_vps_availability_async_)));
    XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
        instance_, "xrCheckVpsAvailabilityCompleteANDROID",
        reinterpret_cast<PFN_xrVoidFunction*>(
            &check_vps_availability_complete_)));

    geospatial_exts_loaded_ = true;
  }

  return true;
}

bool OpenXrManager::CreateStageReferenceSpace() {
  XrReferenceSpaceCreateInfo createInfo = {
      .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
      .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE,
      .poseInReferenceSpace = kIdentityPose,
  };

  {
    absl::MutexLock lock(mutex_);
    XR_RETURN_IF_FAILED(
        xrCreateReferenceSpace(session_, &createInfo, &stage_space_));
  }
  return true;
}

bool OpenXrManager::CreateUnboundedReferenceSpace() {
  XrReferenceSpaceCreateInfo createInfo = {
      .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
      .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID,
      .poseInReferenceSpace = kIdentityPose,
  };

  {
    absl::MutexLock lock(mutex_);
    XR_RETURN_IF_FAILED(
        xrCreateReferenceSpace(session_, &createInfo, &unbounded_space_));
  }
  return true;
}

bool OpenXrManager::CreateViewReferenceSpace() {
  XrReferenceSpaceCreateInfo createInfo = {
      .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
      .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW,
      .poseInReferenceSpace = kIdentityPose,
  };
  absl::MutexLock lock(mutex_);
  XR_RETURN_IF_FAILED(
      xrCreateReferenceSpace(session_, &createInfo, &view_space_));
  return true;
}

bool OpenXrManager::LocateHandJoints(bool is_left_hand, XrTime time,
                                     XrHandJointLocationsEXT* hand_joints) {
  {
    absl::MutexLock lock(mutex_);
    XrHandTrackerEXT hand_tracker =
        is_left_hand ? left_hand_tracker_ : right_hand_tracker_;

    XrHandJointsLocateInfoEXT locate_info{
        .type = XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT,
        .baseSpace = GetSpaceInDefaultReferenceSpace(),
        .time = time,
    };

    hand_joints->type = XR_TYPE_HAND_JOINT_LOCATIONS_EXT;
    hand_joints->next = nullptr;
    hand_joints->jointCount = XR_HAND_JOINT_COUNT_EXT;
    hand_joints->jointLocations =
        is_left_hand ? left_hand_joint_locations_ : right_hand_joint_locations_;

    XR_RETURN_IF_FAILED(
        locate_hand_joints_(hand_tracker, &locate_info, hand_joints));
  }
  return true;
}

std::byte* OpenXrManager::PrepareHandDataBuffer(bool is_left_hand) {
  absl::MutexLock lock(mutex_);
  // clean up the previous buffer.
  if (is_left_hand) {
    if (left_hand_joint_poses_buffer_[left_hand_joint_buffer_index_] !=
        nullptr) {
      free(left_hand_joint_poses_buffer_[left_hand_joint_buffer_index_]);
    }
    left_hand_joint_poses_buffer_[left_hand_joint_buffer_index_] = nullptr;
  } else {
    if (right_hand_joint_poses_buffer_[right_hand_joint_buffer_index_] !=
        nullptr) {
      free(right_hand_joint_poses_buffer_[right_hand_joint_buffer_index_]);
    }
    right_hand_joint_poses_buffer_[right_hand_joint_buffer_index_] = nullptr;
  }

  std::byte* new_buffer = (std::byte*)malloc(kHandJointsBufferSize);
  // if the malloc fails, return nullptr.
  if (!new_buffer) {
    return nullptr;
  }

  // Update the buffer.
  if (is_left_hand) {
    left_hand_joint_poses_buffer_[left_hand_joint_buffer_index_] = new_buffer;
    left_hand_joint_buffer_index_ =
        (left_hand_joint_buffer_index_ + 1) % CACHE_SIZE;
  } else {
    right_hand_joint_poses_buffer_[right_hand_joint_buffer_index_] = new_buffer;
    right_hand_joint_buffer_index_ =
        (right_hand_joint_buffer_index_ + 1) % CACHE_SIZE;
  }

  return new_buffer;
}

void OpenXrManager::FillInHandDataBuffer(std::byte* buffer,
                                         XrHandJointLocationsEXT hand_joints) {
  // Set up data.
  *((int*)buffer) = hand_joints.isActive;
  if (!hand_joints.isActive) {
    // if the hand is not active, we don't need to set the joint data.
    return;
  }

  float* floatBuffer = (float*)(buffer + sizeof(int));
  for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i) {
    const XrHandJointLocationEXT& joint = hand_joints.jointLocations[i];
    FillQuaternionIntoFloatBuffer(&floatBuffer[i * kFloatPerPose],
                                  joint.pose.orientation);
    FillVector3IntoFloatBuffer(
        &floatBuffer[i * kFloatPerPose + kFloatPerQuaternion],
        joint.pose.position);
  }
}

void OpenXrManager::FillVector3IntoFloatBuffer(float* floatBuffer,
                                               XrVector3f vector) {
  floatBuffer[0] = vector.x;
  floatBuffer[1] = vector.y;
  floatBuffer[2] = vector.z;
}

void OpenXrManager::FillQuaternionIntoFloatBuffer(float* floatBuffer,
                                                  XrQuaternionf quaternion) {
  floatBuffer[0] = quaternion.x;
  floatBuffer[1] = quaternion.y;
  floatBuffer[2] = quaternion.z;
  floatBuffer[3] = quaternion.w;
}

// TODO: (broken link) - Update this to dynamically create a space for a reference
// space type.
XrSpace OpenXrManager::GetSpaceInReferenceSpace(
    XrReferenceSpaceType space_type) {
  switch (space_type) {
    case XR_REFERENCE_SPACE_TYPE_STAGE:
      return stage_space_;
    case XR_REFERENCE_SPACE_TYPE_VIEW:
      return view_space_;
    case XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID:
      return unbounded_space_;
    default:
      LOG(ERROR) << "Unsupported reference space type: " << space_type;
      return XR_NULL_HANDLE;
  }
}

XrSpace OpenXrManager::GetSpaceInDefaultReferenceSpace() {
  return GetSpaceInReferenceSpace(default_reference_space_);
}

XrTime OpenXrManager::GetXrTimeNow() const {
  return GetXrTimeFromTimespec(clock_->TimeNow());
}

XrTime OpenXrManager::GetXrTimeFromNanoseconds(int64_t time_ns) const {
  if (time_ns < 0) {
    return GetXrTimeNow();
  }
  timespec timespec_time;
  timespec_time.tv_sec = time_ns / kNanosPerSecond;
  timespec_time.tv_nsec = time_ns % kNanosPerSecond;
  return GetXrTimeFromTimespec(timespec_time);
}

XrTime OpenXrManager::GetXrTimeFromTimespec(
    const timespec& timespec_time) const {
  XrTime xr_time;
  XrResult result;
  {
    absl::MutexLock lock(mutex_);
    result = reinterpret_cast<PFN_xrConvertTimespecTimeToTimeKHR>(
        convert_time_)(instance_, &timespec_time, &xr_time);
  }
  if (XR_FAILED(result)) {
    LOG(ERROR) << "Failed to convert time with error: " << XrEnumStr(result);
    return -1;
  }
  return xr_time;
}

bool OpenXrManager::PersistAnchor(XrSpace anchor_space,
                                  XrUuidEXT* out_anchor_uuid) {
  XrPersistedAnchorSpaceInfoANDROID create_info = {
      .type = XR_TYPE_PERSISTED_ANCHOR_SPACE_INFO_ANDROID,
      .next = nullptr,
      .anchor = anchor_space};
  {
    absl::MutexLock lock(mutex_);
    if (XR_FAILED(CreatePersistenceHandleIfNecessary())) {
      return false;
    }
    XR_RETURN_IF_FAILED(
        persist_anchor_(persistence_handle_, &create_info, out_anchor_uuid));
  }
  return true;
}

bool OpenXrManager::GetAnchorPersistState(
    const XrUuidEXT& anchor_uuid,
    XrAnchorPersistStateANDROID* out_persist_state) {
  {
    absl::MutexLock lock(mutex_);
    if (XR_FAILED(CreatePersistenceHandleIfNecessary())) {
      return false;
    }
    XR_RETURN_IF_FAILED(get_anchor_persist_state_(
        persistence_handle_, &anchor_uuid, out_persist_state));
  }
  return true;
}

std::vector<XrUuidEXT> OpenXrManager::GetPersistedAnchorUuids() {
  uint32_t uuid_count_output = 0;
  std::vector<XrUuidEXT> uuids;

  // Query the number of anchors available.
  XrResult result;
  {
    absl::MutexLock lock(mutex_);
    if (XR_FAILED(CreatePersistenceHandleIfNecessary())) {
      return {};
    }
    result = enumerate_persisted_anchors_(persistence_handle_, 0,
                                          &uuid_count_output, nullptr);
  }

  if (result != XR_SUCCESS) {
    LOG(ERROR) << "Unable to enumerate persisted anchors with error: "
               << XrEnumStr(result);
    return {};
  }

  LOG(INFO) << "Enumerating persisted anchors count: " << uuid_count_output;
  if (uuid_count_output == 0) {
    return {};
  }
  uuids.resize(uuid_count_output);

  // Fetch the actual uuids in the appropriately resized array.
  {
    absl::MutexLock lock(mutex_);
    result =
        enumerate_persisted_anchors_(persistence_handle_, uuid_count_output,
                                     &uuid_count_output, uuids.data());
  }

  if (result != XR_SUCCESS) {
    LOG(ERROR) << "Unable to load anchors with error: " << XrEnumStr(result);
    return {};
  }
  return uuids;
}

bool OpenXrManager::UnpersistAnchor(const XrUuidEXT& anchor_uuid) {
  {
    absl::MutexLock lock(mutex_);
    if (XR_FAILED(CreatePersistenceHandleIfNecessary())) {
      return false;
    }
    XR_RETURN_IF_FAILED(unpersist_anchor_(persistence_handle_, &anchor_uuid));
  }
  return true;
}

OpenXrManager::CreateAnchorResult OpenXrManager::LocatePersistedAnchorSpace(
    const XrUuidEXT& anchor_uuid, XrSpace* out_anchor_space) {
  XrSpace anchor_space = XR_NULL_HANDLE;
  XrResult xr_result;
  {
    absl::MutexLock lock(mutex_);
    if (XR_FAILED(CreatePersistenceHandleIfNecessary())) {
      return CreateAnchorResult::kErrorRuntimeFailure;
    }
    XrPersistedAnchorSpaceCreateInfoANDROID create_info = {
        .type = XR_TYPE_PERSISTED_ANCHOR_SPACE_CREATE_INFO_ANDROID,
        .next = nullptr,
        .anchorId = anchor_uuid,
    };
    XrResult xr_result = create_persisted_anchor_space_(
        persistence_handle_, &create_info, &anchor_space);
    if (XR_FAILED(xr_result)) {
      LOG(ERROR) << "Failed to create persisted anchor space with: "
                 << XrEnumStr(xr_result);
      return MapAnchorCreateResult(xr_result);
    }
  }

  XrSpaceLocation location = {
      .type = XR_TYPE_SPACE_LOCATION,
  };
  xr_result = xrLocateSpace(anchor_space, GetSpaceInDefaultReferenceSpace(),
                            GetXrTimeNow(), &location);
  if (XR_FAILED(xr_result)) {
    LOG(ERROR) << "Failed to locate persisted anchor with: "
               << XrEnumStr(xr_result);
    return MapAnchorCreateResult(xr_result);
  }
  *out_anchor_space = anchor_space;
  return MapAnchorCreateResult(xr_result);
}

bool OpenXrManager::HitTest(XrRaycastInfoANDROID* raycast_info,
                            XrRaycastHitResultsANDROID* out_hit_results) {
  {
    absl::MutexLock lock(mutex_);
    raycast_info->space = GetSpaceInDefaultReferenceSpace();
    raycast_info->trackerCount = 1;
    raycast_info->trackers = &planes_trackable_tracker_;
    XR_RETURN_IF_FAILED(raycast_(session_, raycast_info, out_hit_results));
  }
  return true;
}

std::byte* OpenXrManager::GetHandDataBuffer(bool is_left_hand, XrTime time) {
  XrHandJointLocationsEXT hand_joints = {};
  if (!LocateHandJoints(is_left_hand, time, &hand_joints)) {
    return nullptr;
  }

  std::byte* buffer = PrepareHandDataBuffer(is_left_hand);
  if (buffer == nullptr) {
    return nullptr;
  }

  FillInHandDataBuffer(buffer, hand_joints);
  return buffer;
}

bool OpenXrManager::IsFaceTrackerCalibrated() {
  absl::MutexLock lock(mutex_);
  return face_tracker_calibration_state_ ==
         FaceTrackingCalibrationState::kCalibrated;
}

bool OpenXrManager::GetDepthImage(XrTime time,
                                  const float** out_smooth_depth_image,
                                  int* out_image_width, int* out_image_height) {
  absl::MutexLock lock(mutex_);
  if (XR_FAILED(
          CreateDepthSwapchainIfNecessary(DepthEstimationMode::kSmoothOnly))) {
    LOG(ERROR) << "Failed to create depth swapchain.";
    return false;
  }

  XrDepthAcquireInfoANDROID acquire_info = {
      .type = XR_TYPE_DEPTH_ACQUIRE_INFO_ANDROID,
      .space = GetSpaceInDefaultReferenceSpace(),
      .displayTime = time,
  };
  XrDepthAcquireResultANDROID acquire_result = {
      .type = XR_TYPE_DEPTH_ACQUIRE_RESULT_ANDROID,
  };
  XR_RETURN_IF_FAILED(acquire_depth_swapchain_images_(
      depth_swapchain_handle_, &acquire_info, &acquire_result));

  if (acquire_result.acquiredIndex >= depth_images_.size()) {
    LOG(ERROR) << "acquiredIndex: " << acquire_result.acquiredIndex
               << " is greater than swapchain_size_: " << depth_images_.size();
    return false;
  }
  const XrDepthSwapchainImageANDROID& acquired_depth_image =
      depth_images_[acquire_result.acquiredIndex];

  *out_smooth_depth_image = acquired_depth_image.smoothDepthImage;
  if (!GetDepthCameraImageWidthAndHeight(supported_depth_resolution_,
                                         out_image_width, out_image_height)) {
    LOG(ERROR) << "Failed to get depth image width and height.";
    return false;
  }

  LOG(INFO) << "Successfully get depth image with resolution: width = "
            << *out_image_width << ", height = " << *out_image_height << ".";
  return true;
}

bool OpenXrManager::GetAllDepthImages(
    XrTime time, std::vector<DepthImageBuffer>& out_image_buffers) {
  absl::MutexLock lock(mutex_);

  if (depth_image_buffer_count_ == 0) {
    LOG(ERROR) << "Depth image buffer count is 0.";
    return false;
  }

  out_image_buffers.resize(depth_image_buffer_count_);

  XrDepthAcquireInfoANDROID acquire_info = {
      .type = XR_TYPE_DEPTH_ACQUIRE_INFO_ANDROID,
      .space = GetSpaceInDefaultReferenceSpace(),
      .displayTime = time,
  };
  XrDepthAcquireResultANDROID acquire_result = {
      .type = XR_TYPE_DEPTH_ACQUIRE_RESULT_ANDROID,
  };

  XR_RETURN_IF_FAILED(acquire_depth_swapchain_images_(
      depth_swapchain_handle_, &acquire_info, &acquire_result));

  if (acquire_result.acquiredIndex >= depth_images_.size()) {
    LOG(ERROR) << "acquiredIndex: " << acquire_result.acquiredIndex
               << " is greater than swapchain_size_: " << depth_images_.size();
    return false;
  }

  const XrDepthSwapchainImageANDROID& acquired_depth_image =
      depth_images_[acquire_result.acquiredIndex];

  if (config_settings_.depth_estimation_mode == DepthEstimationMode::kRawOnly) {
    PopulateDepthImageBuffer(out_image_buffers,
                             acquired_depth_image.rawDepthImage,
                             acquired_depth_image.rawDepthConfidenceImage);
  } else if (config_settings_.depth_estimation_mode ==
             DepthEstimationMode::kSmoothOnly) {
    PopulateDepthImageBuffer(out_image_buffers,
                             acquired_depth_image.smoothDepthImage,
                             acquired_depth_image.smoothDepthConfidenceImage);
  } else {
    LOG(ERROR) << "Unsupported depth estimation mode.";
    return false;
  }
  return true;
}

void OpenXrManager::PopulateDepthImageBuffer(
    std::vector<DepthImageBuffer>& out_image_buffers,
    const float* image_buffers, const uint8_t* confidence_image_buffers) {
  auto& left_eye_image = out_image_buffers[static_cast<int>(
      OpenXrManager::DepthImageBufferOrder::kLeftEyeImage)];
  auto& right_eye_image = out_image_buffers[static_cast<int>(
      OpenXrManager::DepthImageBufferOrder::kRightEyeImage)];
  auto& left_eye_confidence_image = out_image_buffers[static_cast<int>(
      OpenXrManager::DepthImageBufferOrder::kLeftEyeConfidenceImage)];
  auto& right_eye_confidence_image = out_image_buffers[static_cast<int>(
      OpenXrManager::DepthImageBufferOrder::kRightEyeConfidenceImage)];

  left_eye_image.buffer = image_buffers;
  left_eye_image.buffer_size = depth_data_image_buffer_size_;

  right_eye_image.buffer = image_buffers + depth_data_image_num_elements_;
  right_eye_image.buffer_size = depth_data_image_buffer_size_;

  left_eye_confidence_image.buffer = confidence_image_buffers;
  left_eye_confidence_image.buffer_size =
      depth_data_confidence_image_buffer_size_;

  right_eye_confidence_image.buffer =
      confidence_image_buffers + depth_data_image_num_elements_;
  right_eye_confidence_image.buffer_size =
      depth_data_confidence_image_buffer_size_;
}

int OpenXrManager::GetDepthImageWidth() {
  absl::MutexLock lock(mutex_);
  return depth_image_width_;
}

int OpenXrManager::GetDepthImageHeight() {
  absl::MutexLock lock(mutex_);
  return depth_image_height_;
}

bool OpenXrManager::IsGeospatialSupported() {
  absl::MutexLock lock(mutex_);
  if (!geospatial_exts_loaded_) {
    return false;
  }

  XrSystemGeospatialPropertiesANDROID geospatialSystemProperties{
      XR_TYPE_SYSTEM_GEOSPATIAL_PROPERTIES_ANDROID};
  XrSystemProperties systemProperties{.type = XR_TYPE_SYSTEM_PROPERTIES,
                                      .next = &geospatialSystemProperties};
  XR_RETURN_IF_FAILED(
      xrGetSystemProperties(instance_, system_id_, &systemProperties));

  return geospatialSystemProperties.supportsGeospatial;
}

OpenXrManager::GeospatialState OpenXrManager::GetGeospatialState() {
  absl::MutexLock lock(mutex_);
  // Geospatial has not been initialized.
  if (geospatial_tracker_ == XR_NULL_HANDLE ||
      !last_geospatial_tracker_state_update_.has_value() ||
      last_geospatial_tracker_state_update_->state ==
          XR_GEOSPATIAL_TRACKER_STATE_STOPPED_ANDROID) {
    return GeospatialState::kStopped;
  }

  if (last_geospatial_tracker_state_update_->state ==
      XR_GEOSPATIAL_TRACKER_STATE_RUNNING_ANDROID) {
    // Ensure the spatial context is ready before returning RUNNING to Jetpack.
    if (geospatial_anchors_spatial_context_ == XR_NULL_HANDLE) {
      return GeospatialState::kStopped;
    }

    return GeospatialState::kRunning;
  }

  // Geospatial state is initialization failed.
  switch (last_geospatial_tracker_state_update_->initializationResult) {
    case XR_ERROR_PERMISSION_INSUFFICIENT:
      return GeospatialState::kErrorNotAuthorized;
    case XR_ERROR_LIMIT_REACHED:
      return GeospatialState::kErrorResourcesExhausted;
    default:
      return GeospatialState::kErrorInternal;
  }

  return GeospatialState::kErrorInternal;
}

OpenXrManager::GeospatialPoseResult OpenXrManager::LocateGeospatialPoseFromPose(
    XrTime time, const XrPosef& pose,
    XrGeospatialPoseResultANDROID* out_geospatial_pose_result) {
  if (GetGeospatialState() != GeospatialState::kRunning) {
    return GeospatialPoseResult::kErrorIllegalState;
  }

  absl::MutexLock lock(mutex_);
  XrGeospatialPoseFromPoseLocateInfoANDROID locate_info = {
      .type = XR_TYPE_GEOSPATIAL_POSE_FROM_POSE_LOCATE_INFO_ANDROID,
      .next = nullptr,
      .space = GetSpaceInDefaultReferenceSpace(),
      .time = time,
      .pose = pose,
  };

  *out_geospatial_pose_result = {
      .type = XR_TYPE_GEOSPATIAL_POSE_RESULT_ANDROID,
      .next = nullptr,
  };

  XrResult result = locate_geospatial_pose_from_pose_(
      geospatial_tracker_, &locate_info, out_geospatial_pose_result);

  switch (result) {
    case XR_SUCCESS:
      if (!(out_geospatial_pose_result->poseFlags &
                XR_GEOSPATIAL_POSE_ORIENTATION_VALID_BIT_ANDROID &&
            out_geospatial_pose_result->poseFlags &
                XR_GEOSPATIAL_POSE_POSITION_VALID_BIT_ANDROID)) {
        return GeospatialPoseResult::kErrorNotTracking;
      }

      return GeospatialPoseResult::kSuccess;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wswitch"
    case XR_ERROR_GEOSPATIAL_COORDINATES_INVALID_ANDROID:
      return GeospatialPoseResult::kErrorInvalidArgument;
#pragma clang diagnostic pop
    default:
      return GeospatialPoseResult::kErrorIllegalState;
  }
}

OpenXrManager::GeospatialPoseResult OpenXrManager::LocatePoseFromGeospatialPose(
    XrTime time, const XrGeospatialPoseANDROID& geospatial_pose,
    XrSpaceLocation* out_location) {
  if (GetGeospatialState() != GeospatialState::kRunning) {
    return GeospatialPoseResult::kErrorIllegalState;
  }

  absl::MutexLock lock(mutex_);
  XrGeospatialPoseLocateInfoANDROID locate_info = {
      .type = XR_TYPE_GEOSPATIAL_POSE_LOCATE_INFO_ANDROID,
      .next = nullptr,
      .space = GetSpaceInDefaultReferenceSpace(),
      .time = time,
      .geospatialPose = geospatial_pose,
  };

  *out_location = {
      .type = XR_TYPE_SPACE_LOCATION,
      .next = nullptr,
  };

  XrResult result =
      locate_geospatial_pose_(geospatial_tracker_, &locate_info, out_location);

  switch (result) {
    case XR_SUCCESS:
      if (!((out_location->locationFlags &
             XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) &&
            (out_location->locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT))) {
        return GeospatialPoseResult::kErrorNotTracking;
      }
      return GeospatialPoseResult::kSuccess;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wswitch"
    case XR_ERROR_GEOSPATIAL_COORDINATES_INVALID_ANDROID:
      return GeospatialPoseResult::kErrorInvalidArgument;
#pragma clang diagnostic pop
    default:
      return GeospatialPoseResult::kErrorIllegalState;
  }
}

XrResult OpenXrManager::CheckVpsAvailabilityAsync(
    double latitude, double longitude,
    std::function<void(const XrVPSAvailabilityCheckCompletionANDROID&)>
        on_complete,
    std::function<void()> on_cancel) {
  absl::MutexLock lock(mutex_);
  XrFutureEXT future;
  XR_RETURN_RESULT_IF_FAILED(
      check_vps_availability_async_(session_, latitude, longitude, &future));

  pending_futures_.push_back(
      {future,
       {[this, on_complete](XrFutureEXT future) {
          XrVPSAvailabilityCheckCompletionANDROID completion = {
              .type = XR_TYPE_VPS_AVAILABILITY_CHECK_COMPLETION_ANDROID,
              .next = nullptr,
          };
          XrResult result;
          {
            absl::MutexLock lock(mutex_);
            result =
                check_vps_availability_complete_(session_, future, &completion);
          }

          if (XR_FAILED(result)) {
            LOG(ERROR) << "Failed to complete VPS availability check future "
                          "with error: "
                       << XrEnumStr(result);
            completion.futureResult = result;
          }

          on_complete(completion);
        },
        on_cancel}});

  return XR_SUCCESS;
}

OpenXrManager::CreateAnchorResult OpenXrManager::CreateAnchorSpaceFromEntityId(
    XrSpatialEntityIdEXT entity_id, XrSpace* out_anchor_space) {
  absl::MutexLock lock(mutex_);
  XrSpatialAnchorSpaceFromIdCreateInfoANDROID space_create_info = {
      .type = XR_TYPE_SPATIAL_ANCHOR_SPACE_FROM_ID_CREATE_INFO_ANDROID,
      .next = nullptr,
      .anchorEntityId = entity_id,
  };

  XrResult result = create_spatial_anchor_space_from_id_(
      session_, geospatial_anchors_spatial_context_, &space_create_info,
      out_anchor_space);

  if (XR_FAILED(result)) {
    LOG(ERROR) << "Failed to create spatial anchor space from id with: "
               << XrEnumStr(result);
    return MapAnchorCreateResult(result);
  }
  return CreateAnchorResult::kSuccess;
}

XrResult OpenXrManager::SetGoogleCloudAuthAsync(
    const XrGoogleCloudAuthInfoBaseHeaderANDROID* auth_info,
    std::function<void(const XrFutureCompletionEXT&)> on_complete,
    std::function<void()> on_cancel) {
  absl::MutexLock lock(mutex_);
  if (!cloud_auth_exts_loaded_) {
    return XR_ERROR_FUNCTION_UNSUPPORTED;
  }

  XrFutureEXT future;
  XR_RETURN_RESULT_IF_FAILED(
      set_google_cloud_auth_async_(session_, auth_info, &future));

  pending_futures_.push_back(
      {future,
       {[this, on_complete](XrFutureEXT future) {
          XrFutureCompletionEXT completion = {
              .type = XR_TYPE_FUTURE_COMPLETION_EXT,
              .next = nullptr,
          };
          XrResult result;
          {
            absl::MutexLock lock(mutex_);
            result =
                set_google_cloud_auth_complete_(session_, future, &completion);
          }

          if (XR_FAILED(result)) {
            LOG(ERROR) << "Failed to complete google cloud auth future "
                          "with error: "
                       << XrEnumStr(result);
            completion.futureResult = result;
          }

          on_complete(completion);
        },
        on_cancel}});

  return XR_SUCCESS;
}

bool OpenXrManager::Init(JNIEnv* env, jobject context,
                         XrReferenceSpaceType default_reference_space,
                         bool start_polling_thread) {
  java_env_ = env;
  java_env_->GetJavaVM(&app_vm_);
  absl::MutexLock state_lock(initialization_mutex_);
  {
    absl::MutexLock lock(mutex_);

    if (open_xr_state_ == OpenXrState::kResumed) {
      LOG(INFO) << "Returning existing OpenXR session.";
      return true;
    } else if (open_xr_state_ == OpenXrState::kPaused) {
      LOG(INFO)
          << "Returning an existing OpenXR session, and resuming if requested.";
      if (start_polling_thread) {
        StartPollingThread();
      }
      return true;
    }
    open_xr_state_ = OpenXrState::kInitializing;
    default_reference_space_ = default_reference_space;
  }

  // Load OpenXR.
  if (!LoadOpenXr(context)) {
    DeInitWithLockHeld();
    return false;
  }

  // Create an OpenXR instance.
  if (!CreateInstance()) {
    DeInitWithLockHeld();
    return false;
  }

  // Create an OpenXR session.
  if (!CreateSession()) {
    DeInitWithLockHeld();
    return false;
  }

  // Loads in the OpenXR extension functions that will need to be called by the
  // openXR manager.
  if (!InitExtensionFunctions()) {
    DeInitWithLockHeld();
    return false;
  }

  // Creates a reference space that will be used for retrieving the trackables.
  if (!CreateStageReferenceSpace()) {
    DeInitWithLockHeld();
    return false;
  }

  // Creates a unbounded space that will be used for retrieving trackables in
  // the unbounded space.
  if (!CreateUnboundedReferenceSpace()) {
    DeInitWithLockHeld();
    return false;
  }

  if (!CreateViewReferenceSpace()) {
    DeInitWithLockHeld();
    return false;
  }

  // TODO: Temporarily enabling some features by default until
  // session configuration is fully implemented.
  if (XR_FAILED(ConfigureSession(ConfigSettings{
          .anchor_persistence_mode = AnchorPersistenceMode::kLocal,
      }))) {
    DeInitWithLockHeld();
    return false;
  }

  {
    absl::MutexLock lock(mutex_);
    if (start_polling_thread) {
      StartPollingThread();
    } else {
      open_xr_state_ = OpenXrState::kPaused;
    }
  }
  return true;
}

void OpenXrManager::DeInit(bool stop_polling_thread) {
  absl::ReaderMutexLock state_lock(initialization_mutex_);
  DeInitWithLockHeld(stop_polling_thread);
}

void OpenXrManager::DeInitWithLockHeld(bool stop_polling_thread) {
  {
    absl::MutexLock lock(mutex_);
    if (open_xr_state_ == OpenXrState::kUninitialized ||
        open_xr_state_ == OpenXrState::kUninitializing) {
      return;
    }
    open_xr_state_ = OpenXrState::kUninitializing;

    XrResult session_result = xrDestroySession(session_);
    if (XR_FAILED(session_result)) {
      LOG(ERROR) << "Failed to destroy session with error: "
                 << XrEnumStr(session_result);
    }

    XrResult instance_result = xrDestroyInstance(instance_);
    if (XR_FAILED(instance_result)) {
      LOG(ERROR) << "Failed to destroy instance with error: "
                 << XrEnumStr(instance_result);
    }
    // Destroy planes tracker.
    if (planes_trackable_tracker_ != XR_NULL_HANDLE) {
      XrResult result = destroy_trackable_tracker_(planes_trackable_tracker_);
      planes_trackable_tracker_ = XR_NULL_HANDLE;
      if (XR_FAILED(result)) {
        LOG(ERROR) << "Failed to destroy planes tracker with error: "
                   << XrEnumStr(result);
      }
    }

    // Destroy object tracker.
    if (object_trackable_tracker_ != XR_NULL_HANDLE) {
      XrResult result = destroy_trackable_tracker_(object_trackable_tracker_);
      object_trackable_tracker_ = XR_NULL_HANDLE;
      if (XR_FAILED(result)) {
        LOG(ERROR) << "Failed to destroy object tracker with error: "
                   << XrEnumStr(result);
      }
    }

    // Destroy hand trackers.
    if (left_hand_tracker_ != XR_NULL_HANDLE) {
      XrResult result = destroy_hand_tracker_(left_hand_tracker_);
      left_hand_tracker_ = XR_NULL_HANDLE;
      if (XR_FAILED(result)) {
        LOG(ERROR) << "Failed to destroy left hand tracker with error: "
                   << XrEnumStr(result);
      }
    }
    if (right_hand_tracker_ != XR_NULL_HANDLE) {
      XrResult result = destroy_hand_tracker_(right_hand_tracker_);
      right_hand_tracker_ = XR_NULL_HANDLE;
      if (XR_FAILED(result)) {
        LOG(ERROR) << "Failed to destroy right hand tracker with error: "
                   << XrEnumStr(result);
      }
    }

    // Destroy face tracker.
    if (face_tracker_ != XR_NULL_HANDLE) {
      XrResult result = destroy_face_tracker_(face_tracker_);
      face_tracker_ = XR_NULL_HANDLE;
      if (XR_FAILED(result)) {
        LOG(ERROR) << "Failed to destroy face tracker with error: "
                   << XrEnumStr(result);
      }
    }

    // Destroy eye tracker.
    if (eye_tracker_ != XR_NULL_HANDLE) {
      XrResult result = destroy_eye_tracker_(eye_tracker_);
      eye_tracker_ = XR_NULL_HANDLE;
      if (XR_FAILED(result)) {
        LOG(ERROR) << "Failed to destroy eye tracker with error: "
                   << XrEnumStr(result);
      }
    }

    // Destroy the persistence handle.
    if (persistence_handle_ != XR_NULL_HANDLE) {
      XrResult result = destroy_device_anchor_persistence_(persistence_handle_);
      persistence_handle_ = XR_NULL_HANDLE;
      if (XR_FAILED(result)) {
        LOG(ERROR) << "Failed to destroy device anchor persistence with error: "
                   << XrEnumStr(result);
      }
    }

    // Destroy the depth.
    if (depth_swapchain_handle_ != XR_NULL_HANDLE) {
      XrResult destroy_depth_result =
          destroy_depth_swapchain_(depth_swapchain_handle_);
      depth_swapchain_handle_ = XR_NULL_HANDLE;
      if (XR_FAILED(destroy_depth_result)) {
        LOG(ERROR) << "Failed to destroy depth swapchain with error: "
                   << XrEnumStr(destroy_depth_result);
      }
    }

    CleanupGeospatial();

    // Destroy view space.
    if (view_space_ != XR_NULL_HANDLE) {
      XrResult destroy_view_space_result = xrDestroySpace(view_space_);
      view_space_ = XR_NULL_HANDLE;
      if (XR_FAILED(destroy_view_space_result)) {
        LOG(ERROR) << "Failed to destroy view space with error: "
                   << XrEnumStr(destroy_view_space_result);
      }
    }

    // Cancel any pending futures.
    CancelPendingFutures();

    instance_ = XR_NULL_HANDLE;
    system_id_ = XR_NULL_SYSTEM_ID;
    session_ = XR_NULL_HANDLE;
    stop_polling_ = true;
    planes_trackable_tracker_ = XR_NULL_HANDLE;
    face_tracker_calibration_state_ = FaceTrackingCalibrationState::kUnknown;
    geospatial_exts_loaded_ = false;
    cloud_auth_exts_loaded_ = false;
  }
  if (stop_polling_thread) {
    JoinPollingThread();
  }
  {
    absl::MutexLock lock(mutex_);
    open_xr_state_ = OpenXrState::kUninitialized;
  }
}

bool OpenXrManager::PauseSession() {
  {
    absl::MutexLock lock(mutex_);
    if (open_xr_state_ != OpenXrState::kResumed) {
      LOG(ERROR) << "Attempted to pause the Open XR session when session is "
                    "not in a resumed state. ";
      return false;
    }
    stop_polling_ = true;
  }
  JoinPollingThread();
  {
    absl::MutexLock lock(mutex_);
    open_xr_state_ = OpenXrState::kPaused;
  }
  return true;
}

bool OpenXrManager::LoadOpenXr(jobject context) {
  PFN_xrInitializeLoaderKHR initialize_loader = nullptr;

  // Gets a function pointer to the OpenXR loader.
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                            (PFN_xrVoidFunction*)(&initialize_loader)));
  if (initialize_loader == nullptr) {
    LOG(ERROR) << "Failure loading OpenXR. Loader is null.";
    return false;
  }
  XrLoaderInitInfoAndroidKHR loader_init_info_android;
  {
    absl::MutexLock lock(mutex_);
    loader_init_info_android = {
        .type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR,
        .applicationVM = app_vm_,
        .applicationContext = context,
    };
  }

  // Call the loader function obtained above to load OpenXR.
  XR_RETURN_IF_FAILED(
      initialize_loader(reinterpret_cast<const XrLoaderInitInfoBaseHeaderKHR*>(
          &loader_init_info_android)));
  LOG(INFO) << "OpenXR is loaded.";
  return true;
}

bool OpenXrManager::GetEnabledExtensions(
    std::vector<std::string>& enabled_exts) {
  std::vector<XrExtensionProperties> available_ext_props;
  uint32_t property_count;
  XR_RETURN_IF_FAILED(xrEnumerateInstanceExtensionProperties(
      /*layerName=*/nullptr, /*propertyCapacityInput=*/0, &property_count,
      /*properties=*/nullptr));
  available_ext_props.resize(property_count);
  for (auto& prop : available_ext_props) {
    prop.type = XR_TYPE_EXTENSION_PROPERTIES;
  }
  XR_RETURN_IF_FAILED(xrEnumerateInstanceExtensionProperties(
      /*layerName=*/nullptr, property_count, &property_count,
      available_ext_props.data()));

  std::unordered_set<std::string_view> available_exts;
  for (const auto& prop : available_ext_props) {
    available_exts.insert(prop.extensionName);
  }

  enabled_exts.clear();
  for (const auto& required_ext : kRequiredExtensions) {
    enabled_exts.push_back(required_ext);
  }

  for (const auto& optional_ext : kOptionalExtensions) {
    if (available_exts.count(optional_ext.name)) {
      bool dependencies_met = true;
      for (const auto& dependency : optional_ext.dependencies) {
        if (std::find(enabled_exts.begin(), enabled_exts.end(), dependency) ==
            enabled_exts.end()) {
          dependencies_met = false;
          break;
        }
      }
      if (dependencies_met) {
        enabled_exts.push_back(optional_ext.name);
      }
    }
  }
  return true;
}

bool OpenXrManager::CreateInstance() {
  std::vector<std::string> enabled_exts_str;
  if (!GetEnabledExtensions(enabled_exts_str)) {
    return false;
  }

  std::vector<const char*> enabled_exts;
  enabled_exts.reserve(enabled_exts_str.size());
  for (const auto& ext : enabled_exts_str) {
    enabled_exts.push_back(ext.c_str());
  }

  XrInstanceCreateInfo create_info = {
      .type = XR_TYPE_INSTANCE_CREATE_INFO,
      .applicationInfo =
          {
              .apiVersion = XR_API_VERSION_1_0,
          },
      .enabledApiLayerCount = 0,
      .enabledApiLayerNames = nullptr,
      .enabledExtensionCount = static_cast<uint32_t>(enabled_exts.size()),
      .enabledExtensionNames = enabled_exts.data(),
  };
  strncpy(create_info.applicationInfo.applicationName, kApplicationName,
          XR_MAX_APPLICATION_NAME_SIZE);

  {
    absl::MutexLock lock(mutex_);
    // Create an OpenXR instance.
    XR_RETURN_IF_FAILED(xrCreateInstance(&create_info, &instance_));
  }

  LOG(INFO) << "XrInstance created.";
  return true;
}

bool OpenXrManager::GetXrSystem() {
  // TODO: (broken link) - Update this to dynamically evaluate the form factor
  // once we support multiple form factors.
  XrSystemGetInfo system_info = {
      .type = XR_TYPE_SYSTEM_GET_INFO,
      .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY,
  };

  {
    absl::MutexLock lock(mutex_);
    XR_RETURN_IF_FAILED(xrGetSystem(instance_, &system_info, &system_id_));
    if (system_id_ == XR_NULL_SYSTEM_ID) {
      LOG(ERROR) << "XrSystemId is null, this should not be possible.";
      return false;
    }
  }
  return true;
}

bool OpenXrManager::CreateSession() {
  if (!GetXrSystem()) {
    LOG(ERROR) << "failure retrieving the XrSystem";
    return false;
  }

  {
    absl::MutexLock lock(mutex_);
    XrSessionCreateInfo session_create_info = {
        .type = XR_TYPE_SESSION_CREATE_INFO,
        .systemId = system_id_,
    };
    XR_RETURN_IF_FAILED(
        xrCreateSession(instance_, &session_create_info, &session_));
  }
  LOG(INFO) << "Successfully created XrSession.";
  return true;
}

XrResult OpenXrManager::ConfigureSession(
    const ConfigSettings& new_config_settings) {
  {
    absl::MutexLock lock(mutex_);
    XrResult result;
    result = ConfigureFeatures(new_config_settings);
    if (XR_FAILED(result)) {
      AbortConfigureSession();
    } else {
      config_settings_ = new_config_settings;
    }
    return result;
  }
}

XrResult OpenXrManager::ConfigureFeatures(
    const ConfigSettings& new_config_settings) {
  XR_RETURN_RESULT_IF_FAILED(
      ConfigurePlaneTracking(new_config_settings.plane_tracking_mode));
  XR_RETURN_RESULT_IF_FAILED(
      ConfigureHandTracking(new_config_settings.hand_tracking_mode));
  XR_RETURN_RESULT_IF_FAILED(
      ConfigureHeadTracking(new_config_settings.head_tracking_mode));
  XR_RETURN_RESULT_IF_FAILED(
      ConfigureDepthEstimation(new_config_settings.depth_estimation_mode));
  XR_RETURN_RESULT_IF_FAILED(
      ConfigureAnchorPersistence(new_config_settings.anchor_persistence_mode));
  XR_RETURN_RESULT_IF_FAILED(
      ConfigureFaceTracking(new_config_settings.face_tracking_mode));
  XR_RETURN_RESULT_IF_FAILED(
      ConfigureObjectTracking(new_config_settings.object_tracking_mode,
                              new_config_settings.object_tracking_labels));
  XR_RETURN_RESULT_IF_FAILED(
      ConfigureGeospatialTracking(new_config_settings.geospatial_mode));
  XR_RETURN_RESULT_IF_FAILED(
      ConfigureEyeTracking(new_config_settings.eye_tracking_mode));
  return XR_SUCCESS;
}

void OpenXrManager::AbortConfigureSession() {
  ConfigurePlaneTracking(config_settings_.plane_tracking_mode);
  ConfigureHandTracking(config_settings_.hand_tracking_mode);
  ConfigureHeadTracking(config_settings_.head_tracking_mode);
  ConfigureDepthEstimation(config_settings_.depth_estimation_mode);
  ConfigureAnchorPersistence(config_settings_.anchor_persistence_mode);
  ConfigureFaceTracking(config_settings_.face_tracking_mode);
  ConfigureObjectTracking(config_settings_.object_tracking_mode,
                          config_settings_.object_tracking_labels);
  ConfigureGeospatialTracking(config_settings_.geospatial_mode);
}

XrResult OpenXrManager::ConfigurePlaneTracking(PlaneTrackingMode mode) {
  switch (mode) {
    case PlaneTrackingMode::kDisabled: {
      if (planes_trackable_tracker_ != XR_NULL_HANDLE) {
        XR_RETURN_RESULT_IF_FAILED(
            destroy_trackable_tracker_(planes_trackable_tracker_));
        planes_trackable_tracker_ = XR_NULL_HANDLE;
      }
      return XR_SUCCESS;
      break;
    }
    case PlaneTrackingMode::kHorizontalAndVertical: {
      return MaybeCreatePlanesTracker();
      break;
    }
  }
}

XrResult OpenXrManager::ConfigureObjectTracking(
    const ObjectTrackingMode& object_tracking_mode,
    const std::vector<XrObjectLabelANDROID>& object_tracking_labels) {
  if (object_trackable_tracker_ != XR_NULL_HANDLE) {
    XR_RETURN_RESULT_IF_FAILED(
        destroy_trackable_tracker_(object_trackable_tracker_));
    object_trackable_tracker_ = XR_NULL_HANDLE;
  }

  switch (object_tracking_mode) {
    case ObjectTrackingMode::kDisabled: {
      object_tracking_config_ = {
          .type = XR_TYPE_TRACKABLE_OBJECT_CONFIGURATION_ANDROID,
          .next = nullptr,
          .labelCount = 0,
          .activeLabels = nullptr,
      };
      return XR_SUCCESS;
    }
    case ObjectTrackingMode::kEnabled: {
      object_tracking_config_ = {
          .type = XR_TYPE_TRACKABLE_OBJECT_CONFIGURATION_ANDROID,
          .next = nullptr,
          .labelCount = (uint32_t)object_tracking_labels.size(),
          .activeLabels = object_tracking_labels.data(),
      };
      return MaybeCreateObjectTracker();
    }
  }
}

XrResult OpenXrManager::ConfigureHandTracking(HandTrackingMode mode) {
  switch (mode) {
    case HandTrackingMode::kDisabled: {
      if (left_hand_tracker_ != XR_NULL_HANDLE) {
        XR_RETURN_RESULT_IF_FAILED(destroy_hand_tracker_(left_hand_tracker_));
        left_hand_tracker_ = XR_NULL_HANDLE;
      }
      if (right_hand_tracker_ != XR_NULL_HANDLE) {
        XR_RETURN_RESULT_IF_FAILED(destroy_hand_tracker_(right_hand_tracker_));
        right_hand_tracker_ = XR_NULL_HANDLE;
      }
      return XR_SUCCESS;
      break;
    }
    case HandTrackingMode::kBoth: {
      return MaybeCreateHandTrackers();
      break;
    }
  }
}

// TODO: (broken link) - Implement head tracking configuration.
XrResult OpenXrManager::ConfigureHeadTracking(HeadTrackingMode mode) {
  return XR_SUCCESS;
}

XrResult OpenXrManager::ConfigureDepthEstimation(DepthEstimationMode mode) {
  switch (mode) {
    case DepthEstimationMode::kDisabled: {
      if (depth_swapchain_handle_ != XR_NULL_HANDLE) {
        XR_RETURN_RESULT_IF_FAILED(
            destroy_depth_swapchain_(depth_swapchain_handle_));
        depth_swapchain_handle_ = XR_NULL_HANDLE;
      }
      return XR_SUCCESS;
      break;
    };
    case DepthEstimationMode::kRawOnly: {
      depth_image_buffer_count_ = 4;
      return CreateDepthSwapchainIfNecessary(mode);
      break;
    };
    case DepthEstimationMode::kSmoothOnly: {
      depth_image_buffer_count_ = 4;
      return CreateDepthSwapchainIfNecessary(mode);
      break;
    };
    case DepthEstimationMode::kSmoothAndRaw: {
      LOG(ERROR) << "OpenXR does not support both raw and smooth"
                 << "depth images at the same time.";
      return XR_ERROR_RUNTIME_FAILURE;
      break;
    };
  }
}

XrResult OpenXrManager::ConfigureAnchorPersistence(AnchorPersistenceMode mode) {
  switch (mode) {
    case AnchorPersistenceMode::kDisabled: {
      if (persistence_handle_ != XR_NULL_HANDLE) {
        XR_RETURN_RESULT_IF_FAILED(
            destroy_device_anchor_persistence_(persistence_handle_));
        persistence_handle_ = XR_NULL_HANDLE;
      }
      return XR_SUCCESS;
      break;
    };
    case AnchorPersistenceMode::kLocal: {
      return CreatePersistenceHandleIfNecessary();
      break;
    };
  }
}

XrResult OpenXrManager::ConfigureFaceTracking(FaceTrackingMode mode) {
  switch (mode) {
    case FaceTrackingMode::kDisabled: {
      if (face_tracker_ != XR_NULL_HANDLE) {
        XR_RETURN_RESULT_IF_FAILED(destroy_face_tracker_(face_tracker_));
        face_tracker_ = XR_NULL_HANDLE;
      }
      return XR_SUCCESS;
      break;
    };
    case FaceTrackingMode::kUser: {
      return MaybeCreateFaceTracker();
      break;
    };
  }
}

XrResult OpenXrManager::ConfigureGeospatialTracking(GeospatialMode mode) {
  switch (mode) {
    case GeospatialMode::kDisabled: {
      CleanupGeospatial();
      break;
    };
    case GeospatialMode::kEnabled: {
      XR_RETURN_RESULT_IF_FAILED(MaybeCreateGeospatialTracker());

      if (geospatial_anchors_spatial_context_ != XR_NULL_HANDLE) {
        return XR_SUCCESS;
      }

      // Create the spatial context for geospatial anchors.
      std::vector<XrSpatialComponentTypeEXT> enabledComponents = {
          XR_SPATIAL_COMPONENT_TYPE_ANCHOR_EXT,
      };

      XrSpatialCapabilityConfigurationAnchorEXT anchorConfig{
          XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_ANCHOR_EXT};
      anchorConfig.capability = XR_SPATIAL_CAPABILITY_ANCHOR_EXT;
      anchorConfig.enabledComponentCount = enabledComponents.size();
      anchorConfig.enabledComponents = enabledComponents.data();

      std::array<XrSpatialCapabilityConfigurationBaseHeaderEXT*, 1>
          capabilityConfigs = {
              reinterpret_cast<XrSpatialCapabilityConfigurationBaseHeaderEXT*>(
                  &anchorConfig),
          };

      XrSpatialContextCreateInfoEXT spatialContextCreateInfo{
          XR_TYPE_SPATIAL_CONTEXT_CREATE_INFO_EXT};
      spatialContextCreateInfo.capabilityConfigCount = capabilityConfigs.size();
      spatialContextCreateInfo.capabilityConfigs = capabilityConfigs.data();

      auto callback = [this](const XrCreateSpatialContextCompletionEXT&
                                 completion) {
        absl::MutexLock lock(mutex_);
        if (completion.futureResult == XR_SUCCESS) {
          geospatial_anchors_spatial_context_ = completion.spatialContext;
        } else {
          // Simulate an internal error for this, since the configure
          // call already returned success.
          last_geospatial_tracker_state_update_ = {
              .type =
                  XR_TYPE_EVENT_DATA_GEOSPATIAL_TRACKER_STATE_CHANGED_ANDROID,
              .state =
                  XR_GEOSPATIAL_TRACKER_STATE_INITIALIZATION_FAILED_ANDROID,
              .initializationResult = XR_ERROR_RUNTIME_FAILURE,
          };

          LOG(ERROR) << "Async spatial context creation failed with: "
                     << XrEnumStr(completion.futureResult);
        }
      };

      XrResult result =
          MaybeCreateSpatialContextAsync(spatialContextCreateInfo, callback);
      if (XR_FAILED(result)) {
        // Delete the geospatial tracker if the spatial context creation
        // failed.
        CleanupGeospatial();
        return result;
      }
      break;
    };
  }

  return XR_SUCCESS;
}

XrResult OpenXrManager::ConfigureEyeTracking(EyeTrackingMode mode) {
  if (mode == EyeTrackingMode::kDisabled) {
    if (eye_tracker_ != XR_NULL_HANDLE) {
      XR_RETURN_RESULT_IF_FAILED(destroy_eye_tracker_(eye_tracker_));
      eye_tracker_ = XR_NULL_HANDLE;
    }
    return XR_SUCCESS;
  }
  return MaybeCreateEyeTracker();
}

XrResult OpenXrManager::MaybeCreatePlanesTracker() {
  if (planes_trackable_tracker_ != XR_NULL_HANDLE) {
    return XR_SUCCESS;
  }
  XrTrackableTrackerCreateInfoANDROID createInfo = {
      .type = XR_TYPE_TRACKABLE_TRACKER_CREATE_INFO_ANDROID,
      .trackableType = XR_TRACKABLE_TYPE_PLANE_ANDROID};
  XR_RETURN_RESULT_IF_FAILED(create_trackable_tracker_(
      session_, &createInfo, &planes_trackable_tracker_));
  return XR_SUCCESS;
}

XrResult OpenXrManager::MaybeCreateObjectTracker() {
  if (object_trackable_tracker_ != XR_NULL_HANDLE) {
    return XR_SUCCESS;
  }
  XrTrackableTrackerCreateInfoANDROID createInfo = {
      .type = XR_TYPE_TRACKABLE_TRACKER_CREATE_INFO_ANDROID,
      .trackableType = XR_TRACKABLE_TYPE_OBJECT_ANDROID};

  if (object_tracking_config_.labelCount != UINT32_MAX) {
    createInfo.next = &object_tracking_config_;
  }
  return create_trackable_tracker_(session_, &createInfo,
                                   &object_trackable_tracker_);
}

XrResult OpenXrManager::MaybeCreateHandTrackers() {
  if (left_hand_tracker_ == XR_NULL_HANDLE) {
    XrHandTrackerCreateInfoEXT left_hand_tracker_create_info = {
        .type = XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT,
        .next = nullptr,
        .hand = XR_HAND_LEFT_EXT,
        .handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT,
    };

    XR_RETURN_RESULT_IF_FAILED(create_hand_tracker_(
        session_, &left_hand_tracker_create_info, &left_hand_tracker_));
  }

  if (right_hand_tracker_ == XR_NULL_HANDLE) {
    XrHandTrackerCreateInfoEXT right_hand_tracker_create_info = {
        .type = XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT,
        .next = nullptr,
        .hand = XR_HAND_RIGHT_EXT,
        .handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT,
    };
    XR_RETURN_RESULT_IF_FAILED(create_hand_tracker_(
        session_, &right_hand_tracker_create_info, &right_hand_tracker_));
  }
  return XR_SUCCESS;
}

XrResult OpenXrManager::MaybeCreateFaceTracker() {
  if (face_tracker_ == XR_NULL_HANDLE) {
    XrFaceTrackerCreateInfoANDROID face_tracker_create_info = {
        .type = XR_TYPE_FACE_TRACKER_CREATE_INFO_ANDROID,
        .next = nullptr,
    };
    XR_RETURN_RESULT_IF_FAILED(create_face_tracker_(
        session_, &face_tracker_create_info, &face_tracker_));
  }

  XrBool32 isCalibrated = face_tracker_calibration_state_ >
                          FaceTrackingCalibrationState::kServiceNotReady;
  XrResult result = get_face_calibration_state_(face_tracker_, &isCalibrated);
  int attempts = 0;
  while (XR_FAILED(result) && attempts < kFaceTrackerStartupCheckMaxAttempts) {
    face_tracker_calibration_state_ =
        FaceTrackingCalibrationState::kServiceNotReady;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kFaceTrackerStartupWaitMs));
    result = get_face_calibration_state_(face_tracker_, &isCalibrated);
    attempts++;
  }
  face_tracker_calibration_state_ =
      isCalibrated ? FaceTrackingCalibrationState::kCalibrated
                   : FaceTrackingCalibrationState::kNotCalibrated;
  return XR_SUCCESS;
}

XrResult OpenXrManager::MaybeCreateEyeTracker() {
  if (eye_tracker_ == XR_NULL_HANDLE) {
    XrEyeTrackerCreateInfoANDROID eye_tracker_create_info = {
        .type = XR_TYPE_EYE_TRACKER_CREATE_INFO_ANDROID,
        .next = nullptr,
    };
    XR_RETURN_RESULT_IF_FAILED(
        create_eye_tracker_(session_, &eye_tracker_create_info, &eye_tracker_));
  }
  return XR_SUCCESS;
}

XrResult OpenXrManager::GetFaceState(
    XrTime time, XrFaceStateANDROID* outFaceState,
    std::vector<float>& out_blend_shape_values,
    std::vector<float>& out_confidence_values) {
  absl::MutexLock lock(mutex_);
  XrFaceStateGetInfoANDROID getInfo = {
      .type = XR_TYPE_FACE_STATE_GET_INFO_ANDROID,
      .next = nullptr,
      .time = time};

  out_blend_shape_values.resize(XR_FACE_PARAMETER_COUNT_ANDROID);
  out_confidence_values.resize(XR_FACE_REGION_CONFIDENCE_COUNT_ANDROID);

  *outFaceState = {
      .type = XR_TYPE_FACE_STATE_ANDROID,
      .next = nullptr,
      .parametersCapacityInput = XR_FACE_PARAMETER_COUNT_ANDROID,
      .parametersCountOutput = XR_FACE_PARAMETER_COUNT_ANDROID,
      .parameters = out_blend_shape_values.data(),
      .regionConfidencesCapacityInput = XR_FACE_REGION_CONFIDENCE_COUNT_ANDROID,
      .regionConfidencesCountOutput = XR_FACE_REGION_CONFIDENCE_COUNT_ANDROID,
      .regionConfidences = out_confidence_values.data(),
  };

  XrResult result = get_face_state_(face_tracker_, &getInfo, outFaceState);
  if (XR_FAILED(result)) {
    LOG(ERROR) << "Failed to get face state with error: " << XrEnumStr(result);
    return result;
  }
  return XR_SUCCESS;
}

XrResult OpenXrManager::MaybeCreateGeospatialTracker() {
  if (geospatial_tracker_ != XR_NULL_HANDLE) {
    return XR_SUCCESS;
  }
  XrGeospatialTrackerAnchorTrackingInfoANDROIDX2 anchor_tracking_info = {
      .type = XR_TYPE_GEOSPATIAL_TRACKER_ANCHOR_TRACKING_INFO_ANDROIDX2,
      .next = nullptr,
      .shouldTrackPlanes = XR_FALSE,
  };
  XrGeospatialTrackerCreateInfoANDROID create_info = {
      .type = XR_TYPE_GEOSPATIAL_TRACKER_CREATE_INFO_ANDROID,
      .next = &anchor_tracking_info,
  };
  XR_RETURN_RESULT_IF_FAILED(
      create_geospatial_tracker_(session_, &create_info, &geospatial_tracker_));
  return XR_SUCCESS;
}

XrResult OpenXrManager::MaybeCreateSpatialContextAsync(
    XrSpatialContextCreateInfoEXT create_info,
    std::function<void(const XrCreateSpatialContextCompletionEXT&)> callback) {
  XrFutureEXT future;
  XR_RETURN_RESULT_IF_FAILED(
      create_spatial_context_async_(session_, &create_info, &future));

  pending_futures_.push_back(
      {future,
       {[this, callback](XrFutureEXT future) {
          XrCreateSpatialContextCompletionEXT completion = {
              .type = XR_TYPE_CREATE_SPATIAL_CONTEXT_COMPLETION_EXT,
              .next = nullptr,
          };
          XrResult result;
          {
            absl::MutexLock lock(mutex_);
            result =
                create_spatial_context_complete_(session_, future, &completion);
          }

          if (XR_FAILED(result)) {
            LOG(ERROR)
                << "Failed to complete spatial context future with error: "
                << XrEnumStr(result);
            completion.futureResult = result;
            completion.spatialContext = XR_NULL_HANDLE;
          }

          callback(completion);
        },
        /*on_cancel=*/nullptr}});

  return XR_SUCCESS;
}

void OpenXrManager::CleanupGeospatial() {
  // Destroy the geospatial tracker.
  if (geospatial_tracker_ != XR_NULL_HANDLE) {
    XrResult destroy_geospatial_result =
        destroy_geospatial_tracker_(geospatial_tracker_);
    geospatial_tracker_ = XR_NULL_HANDLE;
    last_geospatial_tracker_state_update_.reset();
    geospatial_anchor_space_to_entity_.clear();
    if (XR_FAILED(destroy_geospatial_result)) {
      LOG(ERROR) << "Failed to destroy geospatial tracker with error: "
                 << XrEnumStr(destroy_geospatial_result);
    }
  }

  // Destroy geospatial anchors spatial context.
  if (geospatial_anchors_spatial_context_ != XR_NULL_HANDLE) {
    XrResult destroy_spatial_context_result =
        destroy_spatial_context_(geospatial_anchors_spatial_context_);
    geospatial_anchors_spatial_context_ = XR_NULL_HANDLE;
    if (XR_FAILED(destroy_spatial_context_result)) {
      LOG(ERROR) << "Failed to destroy spatial context with error: "
                 << XrEnumStr(destroy_spatial_context_result);
    }
  }
}

XrResult OpenXrManager::GetEyesInfo(XrTime time, XrEyesANDROID* out_eyes) {
  absl::MutexLock lock(mutex_);
  XrEyesGetInfoANDROID getInfo = {
      .type = XR_TYPE_EYES_GET_INFO_ANDROID,
      .next = nullptr,
      .time = time,
      .baseSpace = GetSpaceInReferenceSpace(XR_REFERENCE_SPACE_TYPE_VIEW)};
  *out_eyes = {.type = XR_TYPE_EYES_ANDROID, .next = nullptr};
  bool is_fine_tracking_mode =
      config_settings_.eye_tracking_mode == EyeTrackingMode::kFine;
  XrResult result =
      is_fine_tracking_mode
          ? get_fine_tracking_eyes_info_(eye_tracker_, &getInfo, out_eyes)
          : get_coarse_tracking_eyes_info_(eye_tracker_, &getInfo, out_eyes);
  if (XR_FAILED(result)) {
    LOG(ERROR) << "Failed to get coarse eyes info with error: "
               << XrEnumStr(result);
    return result;
  }
  return XR_SUCCESS;
}

XrResult OpenXrManager::CreatePersistenceHandleIfNecessary() {
  if (persistence_handle_ != XR_NULL_HANDLE) {
    return XR_SUCCESS;
  }
  XrDeviceAnchorPersistenceCreateInfoANDROID create_info = {
      .type = XR_TYPE_DEVICE_ANCHOR_PERSISTENCE_CREATE_INFO_ANDROID,
      .next = nullptr,
  };
  XR_RETURN_RESULT_IF_FAILED(create_device_anchor_persistence_(
      session_, &create_info, &persistence_handle_));
  return XR_SUCCESS;
}

XrResult OpenXrManager::CreateDepthSwapchainIfNecessary(
    DepthEstimationMode mode) {
  if (config_settings_.depth_estimation_mode != mode &&
      depth_swapchain_handle_ != XR_NULL_HANDLE) {
    XR_RETURN_RESULT_IF_FAILED(
        destroy_depth_swapchain_(depth_swapchain_handle_));
    depth_swapchain_handle_ = XR_NULL_HANDLE;
  } else if (depth_swapchain_handle_ != XR_NULL_HANDLE) {
    return XR_SUCCESS;
  }
  XrDepthSwapchainCreateFlagsANDROID depthSwapchainFlags =
      mode == DepthEstimationMode::kRawOnly ? kDepthSwapchainRawOnlyFlags
                                            : kDepthSwapchainSmoothOnlyFlags;
  std::vector<XrDepthCameraResolutionANDROID> supported_depth_resolutions_;
  uint32_t supported_resolution_count = 0;
  XR_RETURN_RESULT_IF_FAILED(enumerate_depth_resolutions_(
      /*session=*/session_, /*resolutionCapacityInput=*/0,
      /*resolutionCountOutput=*/&supported_resolution_count,
      /*resolutions=*/nullptr));

  if (supported_resolution_count == 0) {
    LOG(ERROR) << "No supported depth resolutions found.";
    return XR_ERROR_RUNTIME_FAILURE;
  }

  supported_depth_resolutions_.resize(supported_resolution_count);

  XR_RETURN_RESULT_IF_FAILED(enumerate_depth_resolutions_(
      /*session=*/session_,
      /*resolutionCapacityInput=*/supported_resolution_count,
      /*resolutionCountOutput=*/&supported_resolution_count,
      /*resolutions=*/supported_depth_resolutions_.data()));

  supported_depth_resolution_ = supported_depth_resolutions_[0];

  LOG(INFO) << "Supported depth resolution: "
            << XrDepthCameraResolutionEnumStr(supported_depth_resolution_);

  XrDepthSwapchainCreateInfoANDROID create_info = {
      .type = XR_TYPE_DEPTH_SWAPCHAIN_CREATE_INFO_ANDROID,
      .next = nullptr,
      .resolution = supported_depth_resolution_,
      .createFlags = depthSwapchainFlags};
  XR_RETURN_RESULT_IF_FAILED(
      create_depth_swapchain_(/*session=*/session_,
                              /*createInfo=*/&create_info,
                              /*swapchain=*/&depth_swapchain_handle_));

  uint32_t swapchain_size = 0;
  XR_RETURN_RESULT_IF_FAILED(enumerate_depth_swapchain_images_(
      /*depthSwapchain=*/depth_swapchain_handle_,
      /*depthImageCapacityInput=*/0, /*depthImageCountOutput=*/&swapchain_size,
      /*depthImages=*/nullptr));
  LOG(INFO) << "Depth texture swapchain has size = " << swapchain_size;

  depth_images_.resize(swapchain_size);
  for (uint32_t i = 0; i < swapchain_size; ++i) {
    depth_images_[i] = {.type = XR_TYPE_DEPTH_SWAPCHAIN_IMAGE_ANDROID};
  }
  XR_RETURN_RESULT_IF_FAILED(enumerate_depth_swapchain_images_(
      /*depthSwapchain=*/depth_swapchain_handle_,
      /*depthImageCapacityInput*/ swapchain_size,
      /*depthImageCountOutput*/ &swapchain_size,
      /*depthImages*/ depth_images_.data()));

  LOG(INFO) << "Depth swapchain created successfully.";

  if (!GetDepthCameraImageWidthAndHeight(supported_depth_resolution_,
                                         &depth_image_width_,
                                         &depth_image_height_)) {
    LOG(ERROR) << "Failed to get depth image width and height.";
    return XR_ERROR_RUNTIME_FAILURE;
  }

  depth_data_image_num_elements_ = depth_image_width_ * depth_image_height_;
  depth_data_image_buffer_size_ =
      sizeof(float) * depth_image_width_ * depth_image_height_;
  depth_data_confidence_image_buffer_size_ =
      sizeof(uint8_t) * depth_image_width_ * depth_image_height_;

  return XR_SUCCESS;
}

std::vector<XrTrackableANDROID> OpenXrManager::GetTrackableObjects(
    XrTime time) {
  uint32_t trackableCountOutput = 0;

  // Query the number of trackables
  XrResult result;
  {
    absl::MutexLock lock(mutex_);
    if (object_trackable_tracker_ == XR_NULL_HANDLE) {
      LOG(ERROR) << "Object trackable tracker is null";
      return {};
    }
    result = get_all_trackables_(object_trackable_tracker_, 0,
                                 &trackableCountOutput, nullptr);
  }

  if (result != XR_SUCCESS) {
    LOG(ERROR) << "Unable to query trackables with error: "
               << XrEnumStr(result);
    return {};
  }

  if (trackableCountOutput == 0) {
    return {};
  }

  std::vector<XrTrackableANDROID> all_objects;
  all_objects.resize(trackableCountOutput);

  // Fetch the actual trackable handles in the appropriately resized array.
  {
    absl::MutexLock lock(mutex_);
    result =
        get_all_trackables_(object_trackable_tracker_, trackableCountOutput,
                            &trackableCountOutput, all_objects.data());
  }

  if (result != XR_SUCCESS) {
    LOG(ERROR) << "Unable to get trackables with error: " << XrEnumStr(result);
    return {};
  }

  return all_objects;
}

bool OpenXrManager::GetTrackableObjectState(
    XrTrackableANDROID object_id, XrReferenceSpaceType reference_space_type,
    XrTime time, XrTrackableObjectANDROID& out_object) {
  XrSpace base_space = GetSpaceInReferenceSpace(reference_space_type);
  if (base_space == XR_NULL_HANDLE) {
    return false;
  }
  XrTrackableGetInfoANDROID object_get_info = {
      .type = XR_TYPE_TRACKABLE_GET_INFO_ANDROID,
      .trackable = object_id,
      .baseSpace = base_space,
      .time = time,
  };

  out_object.type = XR_TYPE_TRACKABLE_OBJECT_ANDROID;
  out_object.next = nullptr;

  XrResult result;
  {
    absl::ReaderMutexLock lock(mutex_);
    result = get_trackable_object_(object_trackable_tracker_, &object_get_info,
                                   &out_object);
  }
  if (XR_FAILED(result)) {
    LOG(ERROR) << "Failed to get trackable object state with error: "
               << XrEnumStr(result);
    return false;
  }
  return true;
}

std::vector<XrTrackableANDROID> OpenXrManager::GetPlanes() {
  uint32_t trackableCountOutput = 0;

  // Query the number of trackables
  XrResult result;
  {
    absl::MutexLock lock(mutex_);
    result = get_all_trackables_(planes_trackable_tracker_, 0,
                                 &trackableCountOutput, nullptr);
  }

  if (result != XR_SUCCESS) {
    LOG(ERROR) << "Unable to query trackables with error: "
               << XrEnumStr(result);
    return {};
  }

  if (trackableCountOutput == 0) {
    return {};
  }
  all_plane_trackables_.resize(trackableCountOutput);

  // Fetch the actual trackable handles in the appropriately resized array.
  {
    absl::MutexLock lock(mutex_);
    result = get_all_trackables_(planes_trackable_tracker_,
                                 trackableCountOutput, &trackableCountOutput,
                                 all_plane_trackables_.data());
  }

  if (result != XR_SUCCESS) {
    LOG(ERROR) << "Unable to get trackables with error: " << XrEnumStr(result);
    return {};
  }
  return all_plane_trackables_;
}

bool OpenXrManager::GetPlaneState(XrTrackableANDROID plane_id,
                                  XrReferenceSpaceType reference_space_type,
                                  XrTime time,
                                  XrTrackablePlaneANDROID& out_plane,
                                  std::vector<XrVector2f>& out_vertices) {
  XrSpace base_space = GetSpaceInReferenceSpace(reference_space_type);
  if (base_space == XR_NULL_HANDLE) {
    return false;
  }
  XrTrackableGetInfoANDROID plane_get_info = {
      .type = XR_TYPE_TRACKABLE_GET_INFO_ANDROID,
      .trackable = plane_id,
      .baseSpace = base_space,
      .time = time,
  };

  XrResult result;
  {
    absl::ReaderMutexLock lock(mutex_);
    result = get_trackable_plane_(planes_trackable_tracker_, &plane_get_info,
                                  &out_plane);
  }
  if (XR_FAILED(result)) {
    LOG(ERROR) << "Failed to get plane vertex count with error: "
               << XrEnumStr(result);
    return false;
  }
  out_plane.vertexCapacityInput = *out_plane.vertexCountOutput;
  out_vertices.resize(*out_plane.vertexCountOutput);
  out_plane.vertices = out_vertices.data();
  {
    absl::ReaderMutexLock lock(mutex_);
    result = get_trackable_plane_(planes_trackable_tracker_, &plane_get_info,
                                  &out_plane);
  }
  if (XR_FAILED(result)) {
    LOG(ERROR) << "Failed to get plane with error: " << XrEnumStr(result);
    return false;
  }

  return true;
}

bool OpenXrManager::ChoosePlane(const PlaneConstraints& plane_constraints,
                                XrTrackableANDROID* out_trackable,
                                XrTrackablePlaneANDROID* out_plane) {
  auto all_plane_trackables = GetPlanes();
  if (all_plane_trackables.empty()) {
    return false;
  }
  XrTime time = GetXrTimeNow();
  // Iterate through the trackables to find the first one that fits the
  // constraint.
  for (XrTrackableANDROID trackable : all_plane_trackables) {
    XrTrackableGetInfoANDROID plane_get_info = {
        .type = XR_TYPE_TRACKABLE_GET_INFO_ANDROID,
        .trackable = trackable,
        .baseSpace = GetSpaceInDefaultReferenceSpace(),
        .time = time,
    };

    // We are ignoring the vertex count for now but creating it here as it
    // cannot be null in the XrTrackablePlaneANDROID.
    uint32_t vertex_count = 0;
    XrTrackablePlaneANDROID plane = {
        .type = XR_TYPE_TRACKABLE_PLANE_ANDROID,
        .vertexCapacityInput = 0,
        .vertexCountOutput = &vertex_count,
        .vertices = nullptr,
    };

    XrResult result;
    {
      absl::ReaderMutexLock lock(mutex_);
      result = get_trackable_plane_(planes_trackable_tracker_, &plane_get_info,
                                    &plane);
    }
    if (XR_FAILED(result)) {
      LOG(ERROR) << "Failed to get plane with error: " << XrEnumStr(result);
      continue;
    }
    if ((plane.planeType == plane_constraints.type ||
         plane_constraints.type == XR_PLANE_TYPE_ARBITRARY_ANDROID) &&
        (plane.planeLabel == plane_constraints.label ||
         plane_constraints.label == XR_PLANE_LABEL_UNKNOWN_ANDROID) &&
        plane.extents.height >= plane_constraints.min_height &&
        plane.extents.width >= plane_constraints.min_width) {
      LOG(INFO) << "Successfully found plane; type: "
                << XrPlaneTypeEnumStr(plane.planeType)
                << ", plane label: " << XrPlaneLabelEnumStr(plane.planeLabel)
                << ", plane height: " << plane.extents.height
                << ", plane width: " << plane.extents.width;
      *out_trackable = trackable;
      *out_plane = plane;
      return true;
    }
  }
  LOG(WARNING) << "Failed to locate a suitable plane.";
  return false;
}

OpenXrManager::CreateAnchorResult OpenXrManager::CreateAnchor(
    XrTime time, const XrPosef& pose, XrSpace* out_anchor_space) {
  XrAnchorSpaceCreateInfoANDROID trackable_anchor_create_info = {
      .type = XR_TYPE_ANCHOR_SPACE_CREATE_INFO_ANDROID,
      .space = GetSpaceInDefaultReferenceSpace(),
      .time = time,
      .pose = pose,
      .trackable = XR_NULL_TRACKABLE_ANDROID,
  };

  XrResult xr_result;
  {
    absl::ReaderMutexLock lock(mutex_);
    xr_result = create_anchor_space_(session_, &trackable_anchor_create_info,
                                     out_anchor_space);
    if (XR_FAILED(xr_result)) {
      LOG(ERROR) << "Failed to create anchor with: " << XrEnumStr(xr_result);
    }
  }
  return MapAnchorCreateResult(xr_result);
}

OpenXrManager::CreateAnchorResult OpenXrManager::CreateAnchorForPlane(
    XrTrackableANDROID trackable, XrTrackablePlaneANDROID* plane, XrTime time,
    const XrPosef& relative_pose, XrSpace* out_anchor_space) {
  if (trackable == XR_NULL_TRACKABLE_ANDROID) {
    LOG(ERROR) << "Cannot create anchor for null trackable.";
    return CreateAnchorResult::kErrorRuntimeFailure;
  }

  // If the provided plane is null, we will retrieve the plane from the
  // trackable.
  uint32_t vertex_count = 0;
  std::vector<XrVector2f> vertices;  // needs same lifetime as retrieved_plane
  XrTrackablePlaneANDROID retrieved_plane = {
      .type = XR_TYPE_TRACKABLE_PLANE_ANDROID,
      .vertexCapacityInput = 0,
      .vertexCountOutput = &vertex_count,
      .vertices = nullptr,
  };
  if (plane == nullptr && !GetPlaneState(trackable, default_reference_space_,
                                         time, retrieved_plane, vertices)) {
    LOG(ERROR) << "Failed to get plane for trackable during anchor creation.";
    return CreateAnchorResult::kErrorRuntimeFailure;
  }

  // We are creating the anchor relative to the center pose of the plane as we
  // currently see it. If further tracking on the plane shows the center
  // pose in a different place, the anchor position will NOT update to the
  // new center pose.
  XrAnchorSpaceCreateInfoANDROID trackableAnchorCreateInfo = {
      .type = XR_TYPE_ANCHOR_SPACE_CREATE_INFO_ANDROID,
      .space = GetSpaceInDefaultReferenceSpace(),
      .time = time,
      .pose = MultiplyPoses(
          plane == nullptr ? retrieved_plane.centerPose : plane->centerPose,
          relative_pose),
      .trackable = trackable,
  };

  XrResult xr_result;
  {
    absl::ReaderMutexLock lock(mutex_);
    xr_result = create_anchor_space_(session_, &trackableAnchorCreateInfo,
                                     out_anchor_space);
    if (XR_FAILED(xr_result)) {
      LOG(ERROR) << "Failed to create anchor for plane with: "
                 << XrEnumStr(xr_result);
    }
  }
  return MapAnchorCreateResult(xr_result);
}

OpenXrManager::CreateAnchorResult OpenXrManager::CreateAnchorForObject(
    XrTrackableANDROID trackable, XrTrackableObjectANDROID* object, XrTime time,
    const XrPosef& relative_pose, XrSpace* out_anchor_space) {
  if (trackable == XR_NULL_TRACKABLE_ANDROID) {
    LOG(ERROR) << "Cannot create anchor for null trackable.";
    return CreateAnchorResult::kErrorRuntimeFailure;
  }

  // If the provided object is null, we will retrieve the object from the
  // trackable.
  XrTrackableObjectANDROID retrieved_object = {
      .type = XR_TYPE_TRACKABLE_OBJECT_ANDROID,
  };
  if (object == nullptr &&
      !GetTrackableObjectState(trackable, default_reference_space_, time,
                               retrieved_object)) {
    LOG(ERROR) << "Failed to get object for trackable during anchor creation.";
    return CreateAnchorResult::kErrorRuntimeFailure;
  }

  // We are creating the anchor relative to the center pose of the object as we
  // currently see it. If further tracking on the object shows the center
  // pose in a different place, the anchor position will NOT update to the
  // new center pose.
  XrAnchorSpaceCreateInfoANDROID trackableAnchorCreateInfo = {
      .type = XR_TYPE_ANCHOR_SPACE_CREATE_INFO_ANDROID,
      .space = GetSpaceInDefaultReferenceSpace(),
      .time = time,
      .pose = MultiplyPoses(
          object == nullptr ? retrieved_object.centerPose : object->centerPose,
          relative_pose),
      .trackable = trackable,
  };

  XrResult xr_result;
  {
    absl::ReaderMutexLock lock(mutex_);
    xr_result = create_anchor_space_(session_, &trackableAnchorCreateInfo,
                                     out_anchor_space);
    if (XR_FAILED(xr_result)) {
      LOG(ERROR) << "Failed to create anchor for objectwith: "
                 << XrEnumStr(xr_result);
    }
  }
  return MapAnchorCreateResult(xr_result);
}

OpenXrManager::CreateAnchorResult OpenXrManager::CreateGeospatialAnchor(
    XrTime time, double latitude, double longitude, double altitude,
    const XrQuaternionf& east_up_south_quaternion, XrSpace* out_anchor_space) {
  if (GetGeospatialState() != GeospatialState::kRunning) {
    return CreateAnchorResult::kErrorGeospatialTrackerNotRunning;
  }

    XrResult result;
    XrSpatialEntityIdEXT anchor_entity_id;

    {
      absl::MutexLock lock(mutex_);
      XrGeospatialAnchorCreateInfoANDROIDX2 create_info = {
          .type = XR_TYPE_GEOSPATIAL_ANCHOR_CREATE_INFO_ANDROIDX2,
          .next = nullptr,
          .geospatialTracker = geospatial_tracker_,

          .geospatialPose =
              {
                  .eastUpSouthOrientation = east_up_south_quaternion,
                  .latitude = latitude,
                  .longitude = longitude,
                  .altitude = altitude,
              },
      };

      result = create_geospatial_anchor_(geospatial_anchors_spatial_context_,
                                         &create_info, &anchor_entity_id);
    }

    if (XR_FAILED(result)) {
      LOG(ERROR) << "Failed to create geospatial anchor with: "
                 << XrEnumStr(result);
      // TODO: Handle more specific error translation.
      return MapAnchorCreateResult(result);
    }

    return CreateAnchorSpaceFromEntityId(anchor_entity_id, out_anchor_space);
}

OpenXrManager::CreateAnchorResult OpenXrManager::CreateSurfaceAnchorAsync(
    XrSurfaceAnchorTypeANDROIDX2 anchor_type, double latitude, double longitude,
    double altitude_relative_to_surface,
    const XrQuaternionf& east_up_south_quaternion,
    std::function<void(const XrSurfaceAnchorCreateCompletionANDROIDX2&)>
        on_complete,
    std::function<void()> on_cancel) {
  if (GetGeospatialState() != GeospatialState::kRunning) {
    return CreateAnchorResult::kErrorGeospatialTrackerNotRunning;
  }

  {
    absl::MutexLock lock(mutex_);

    XrSurfaceAnchorCreateInfoANDROIDX2 create_info = {
        .type = XR_TYPE_SURFACE_ANCHOR_CREATE_INFO_ANDROIDX2,
        .next = nullptr,
        .geospatialTracker = geospatial_tracker_,
        .surfaceAnchorType = anchor_type,
        .eastUpSouthOrientation = east_up_south_quaternion,
        .latitude = latitude,
        .longitude = longitude,
        .altitudeRelativeToSurface = altitude_relative_to_surface,
    };

    XrFutureEXT future;
    XrResult result = create_surface_anchor_async_(
        geospatial_anchors_spatial_context_, &create_info, &future);
    if (XR_FAILED(result)) {
      LOG(ERROR) << "create_surface_anchor_async_ failed with "
                 << XrEnumStr(result);
      return MapAnchorCreateResult(result);
    }

    pending_futures_.push_back(
        {future,
         {[this, on_complete](XrFutureEXT future) {
            XrSurfaceAnchorCreateCompletionANDROIDX2 completion = {
                .type = XR_TYPE_SURFACE_ANCHOR_CREATE_COMPLETION_ANDROIDX2,
                .next = nullptr,
            };
            XrResult result;
            {
              absl::MutexLock lock(mutex_);
              result = create_surface_anchor_complete_(
                  geospatial_anchors_spatial_context_, future, &completion);
            }

            if (XR_FAILED(result)) {
              LOG(ERROR) << "Failed to complete surface anchor future "
                            "with error: "
                         << XrEnumStr(result);
              completion.futureResult = result;
            }

            on_complete(completion);
          },
          on_cancel}});
  }

  return CreateAnchorResult::kSuccess;
}

bool OpenXrManager::GetAnchorLocationData(
    XrSpace anchor_space, XrTime time, XrSpaceLocation* out_anchor_location) {
  XR_RETURN_IF_FAILED(xrLocateSpace(anchor_space,
                                    GetSpaceInDefaultReferenceSpace(), time,
                                    out_anchor_location));
  return true;
}

bool OpenXrManager::ExportAnchor(XrSpace anchor_space,
                                 AIBinder** out_anchor_token) {
  XrAnchorSharingInfoANDROID sharingInfo = {
      .type = XR_TYPE_ANCHOR_SHARING_INFO_ANDROID,
      .anchor = anchor_space,
  };

  XrAnchorSharingTokenANDROID token = {
      .type = XR_TYPE_ANCHOR_SHARING_TOKEN_ANDROID,
  };

  {
    absl::ReaderMutexLock lock(mutex_);
    XR_RETURN_IF_FAILED(share_anchor_(session_, &sharingInfo, &token));
  }
  *out_anchor_token = token.token;
  return true;
}

bool OpenXrManager::CreateSemanticAnchor(
    const PlaneConstraints& plane_constraints, AIBinder** out_anchor_token,
    XrSpace* out_anchor_space) {
  XrTrackableANDROID selected_trackable;
  XrTrackablePlaneANDROID selected_plane;
  if (!ChoosePlane(plane_constraints, &selected_trackable, &selected_plane)) {
    return false;
  }
  if (CreateAnchorForPlane(selected_trackable, &selected_plane, GetXrTimeNow(),
                           kIdentityPose,
                           out_anchor_space) != CreateAnchorResult::kSuccess) {
    return false;
  }
  if (!ExportAnchor(*out_anchor_space, out_anchor_token)) {
    xrDestroySpace(*out_anchor_space);
    return false;
  }
  return true;
}

bool OpenXrManager::DestroyAnchor(XrSpace anchor_space) {
  {
    absl::MutexLock lock(mutex_);
    auto it = geospatial_anchor_space_to_entity_.find(anchor_space);
    if (it != geospatial_anchor_space_to_entity_.end()) {
      destroy_spatial_entity_(it->second);
      geospatial_anchor_space_to_entity_.erase(it);
    }
  }

  XR_RETURN_IF_FAILED(xrDestroySpace(anchor_space));
  return true;
}

bool OpenXrManager::GetHeadPose(XrTime time, XrPosef* out_pose) {
  XrSpaceLocation space_location = {.type = XR_TYPE_SPACE_LOCATION,
                                    .next = nullptr,
                                    .locationFlags = 0,
                                    .pose = kIdentityPose};
  XR_RETURN_IF_FAILED(xrLocateSpace(
      view_space_, GetSpaceInDefaultReferenceSpace(), time, &space_location));

  bool is_valid_pose =
      (space_location.locationFlags & kPoseValidFlags) == kPoseValidFlags;

  if (!is_valid_pose) {
    LOG(WARNING) << "xrLocateSpace returned an invalid pose.";
    return false;
  }

  *out_pose = space_location.pose;
  return true;
}

bool OpenXrManager::GetStereoViews(XrTime time,
                                   std::vector<XrView>* out_views) {
  return GetStereoViews(time, /*is_head_tracking_enabled=*/true, out_views);
}

bool OpenXrManager::GetStereoViews(XrTime time, bool is_head_tracking_enabled,
                                   std::vector<XrView>* out_views) {
  if (out_views == nullptr || out_views->size() != kViewTypeStereoViewCount) {
    LOG(ERROR) << "GetStereoViews expected out_views to be of size "
               << kViewTypeStereoViewCount << " but got " << out_views->size();
    return false;
  }

  XrSpace space = is_head_tracking_enabled
                      ? GetSpaceInDefaultReferenceSpace()
                      : GetSpaceInReferenceSpace(XR_REFERENCE_SPACE_TYPE_VIEW);

  XrViewState view_state{.type = XR_TYPE_VIEW_STATE, .next = nullptr};
  const XrViewLocateInfo view_locate_info = {
      .type = XR_TYPE_VIEW_LOCATE_INFO,
      .next = nullptr,
      .viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
      .displayTime = time,
      .space = space};
  uint32_t view_count;
  {
    absl::MutexLock lock(mutex_);
    XR_RETURN_IF_FAILED(xrLocateViews(session_, &view_locate_info, &view_state,
                                      kViewTypeStereoViewCount, &view_count,
                                      out_views->data()));
  }

  if (view_count != kViewTypeStereoViewCount) {
    LOG(WARNING) << "xrLocateViews returned incorrect number of views.";
    return false;
  }

  if ((view_state.viewStateFlags & kViewStateValidFlags) !=
      kViewStateValidFlags) {
    LOG(WARNING) << "xrLocateViews returned invalid view state.";
    return false;
  }

  return true;
}

bool OpenXrManager::GetEnvironmentBlendModes(
    std::vector<XrEnvironmentBlendMode>* out_modes) {
  if (out_modes == nullptr) {
    LOG(ERROR) << "out_modes vector is null.";
    return false;
  }
  absl::MutexLock lock(mutex_);
  uint32_t count = 0;
  XR_RETURN_IF_FAILED(xrEnumerateEnvironmentBlendModes(
      instance_, system_id_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0,
      &count, nullptr));

  if (count == 0) {
    out_modes->clear();
    return true;
  }
  out_modes->resize(count);
  XR_RETURN_IF_FAILED(xrEnumerateEnvironmentBlendModes(
      instance_, system_id_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, count,
      &count, out_modes->data()));
  return true;
}

void OpenXrManager::HandleSessionChangedEvent(
    const XrEventDataSessionStateChanged& changed_event) {
  LOG(INFO) << "Received session changed event: "
            << XrSessionStateEnumStr(changed_event.state);

  switch (changed_event.state) {
    case XR_SESSION_STATE_READY: {
      XrSessionBeginInfo beginInfo = {
          .type = XR_TYPE_SESSION_BEGIN_INFO,
          .primaryViewConfigurationType =
              XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};

      {
        absl::MutexLock lock(mutex_);
        xrBeginSession(session_, &beginInfo);
      }
      break;
    }
    case XR_SESSION_STATE_STOPPING: {
      {
        absl::MutexLock lock(mutex_);
        xrEndSession(session_);
      }
      break;
    }
    case XR_SESSION_STATE_EXITING: {
      DeInit(/*stop_polling_thread=*/false);
      break;
    }
    case XR_SESSION_STATE_FOCUSED: {
      break;
    }
    case XR_SESSION_STATE_IDLE:
    case XR_SESSION_STATE_SYNCHRONIZED:
    case XR_SESSION_STATE_VISIBLE:
    case XR_SESSION_STATE_LOSS_PENDING: {
      // We expect the above state but do not act on them.
      break;
    }
    default: {
      LOG(WARNING) << "Received unexpected session changed event with state: "
                   << changed_event.state;
      break;
    }
  }
}

void OpenXrManager::StartPollingThread() {
  if (polling_thread_ != nullptr) {
    if (stop_polling_ == true) {
      JoinPollingThread();
    } else {
      LOG(ERROR)
          << "Attempted to start a new polling thread while one still exists.";
      return;
    }
  }
  polling_thread_ = std::make_unique<std::thread>([this]() { PollingLoop(); });
  stop_polling_ = false;
  open_xr_state_ = OpenXrState::kResumed;
}

bool OpenXrManager::ShouldPoll() const {
  absl::MutexLock lock(mutex_);
  return !stop_polling_;
}

void OpenXrManager::PollingLoop() {
  timespec poll_time = clock_->TimeNow();
  while (ShouldPoll()) {
    PollOpenXR();
    PollFutures();
    poll_time = AddTimespecs(poll_time, kPollingInterval);
    {
      absl::MutexLock lock(mutex_);
      clock_->AwaitWithDeadline(&mutex_, &stop_polling_, poll_time);
    }
  }
}

void OpenXrManager::PollOpenXR() {
  while (true) {
    XrEventDataBuffer event = {XR_TYPE_EVENT_DATA_BUFFER};
    XrResult result;
    {
      absl::MutexLock lock(mutex_);
      if (instance_ == XR_NULL_HANDLE) {
        return;
      }
      result = xrPollEvent(instance_, &event);
    }
    if (result == XR_EVENT_UNAVAILABLE) {
      return;
    } else if (result != XR_SUCCESS) {
      LOG(ERROR) << "Failed to poll event: " << XrEnumStr(result);
      return;
    }
    switch (event.type) {
      case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
        const XrEventDataInstanceLossPending instance_loss_event =
            *reinterpret_cast<const XrEventDataInstanceLossPending*>(&event);
        LOG(INFO) << "Received data instance loss event, session will end at: "
                  << instance_loss_event.lossTime;
        break;
      }
      case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
        const XrEventDataSessionStateChanged& session_state_changed_event =
            *reinterpret_cast<XrEventDataSessionStateChanged*>(&event);
        HandleSessionChangedEvent(session_state_changed_event);
        break;
      }
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wswitch"
      case XR_TYPE_EVENT_DATA_GEOSPATIAL_TRACKER_STATE_CHANGED_ANDROID: {
        LOG(INFO) << "Received geospatial tracker state changed event";
        const auto& geospatial_event = *reinterpret_cast<
            const XrEventDataGeospatialTrackerStateChangedANDROID*>(&event);
        absl::MutexLock lock(mutex_);
        last_geospatial_tracker_state_update_ = geospatial_event;
        break;
      }
#pragma clang diagnostic pop
      case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING: {
        LOG(INFO) << "Received reference space change pending event";
        break;
      }
      case XR_TYPE_EVENT_DATA_EVENTS_LOST: {
        LOG(INFO) << "Received events lost event";
        break;
      }
      default: {
        LOG(INFO) << "Received unexpected event of type: " << event.type;
        break;
      }
    }
  }
}

void OpenXrManager::PollFutures() {
  // Poll pending futures.
  std::vector<std::pair<XrFutureEXT, FutureCallbackInfo>> ready_futures;
  {
    absl::MutexLock lock(mutex_);
    auto it = pending_futures_.begin();
    while (it != pending_futures_.end()) {
      auto& [future, callback_info] = *it;

      XrFuturePollInfoEXT poll_info = {.type = XR_TYPE_FUTURE_POLL_INFO_EXT,
                                       .future = future};
      XrFuturePollResultEXT poll_result = {.type =
                                               XR_TYPE_FUTURE_POLL_RESULT_EXT};

      XrResult result = poll_future_(instance_, &poll_info, &poll_result);
      if (XR_FAILED(result)) {
        LOG(ERROR) << "Failed to poll future with error: " << XrEnumStr(result);
        it = pending_futures_.erase(it);
        continue;
      }

      if (poll_result.state == XR_FUTURE_STATE_READY_EXT) {
        ready_futures.push_back(*it);
        it = pending_futures_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Run the callbacks without holding the lock to prevent deadlock.
  for (const auto& [future, callback_info] : ready_futures) {
    callback_info.on_complete(future);
  }
}

void OpenXrManager::CancelPendingFutures() {
  for (const auto& [future, callback_info] : pending_futures_) {
    XrFutureCancelInfoEXT cancel_info = {
        .type = XR_TYPE_FUTURE_CANCEL_INFO_EXT,
        .future = future,
    };
    XrResult cancel_result = cancel_future_(instance_, &cancel_info);
    if (XR_FAILED(cancel_result)) {
      LOG(ERROR) << "Failed to cancel future with error: "
                 << XrEnumStr(cancel_result);
    }
    if (callback_info.on_cancel) {
      callback_info.on_cancel();
    }
  }
  pending_futures_.clear();
}

void OpenXrManager::JoinPollingThread() {
  if (polling_thread_ != nullptr && polling_thread_->joinable()) {
    polling_thread_->join();
    polling_thread_.reset();
  }
}

XrSession OpenXrManager::GetXrSession() {
  absl::MutexLock lock(mutex_);

  return session_;
}

XrInstance OpenXrManager::GetXrInstance() {
  absl::MutexLock lock(mutex_);

  return instance_;
}

}  // namespace androidx::xr::openxr
