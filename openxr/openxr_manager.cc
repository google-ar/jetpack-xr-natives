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
#include <sys/types.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <memory>
#include <thread>
#include <vector>

#include "absl/base/no_destructor.h"
#include "absl/log/log.h"
#include "absl/numeric/int128.h"
#include "absl/synchronization/mutex.h"
#include "openxr/openxr_manager_clock.h"
#include "openxr/openxr_manager_utils.h"

namespace androidx::xr::openxr {
namespace {
// TODO: b/327486673 - Inject the application name for the OpenXR session.
constexpr char kApplicationName[] = "JetpackXrCore";

// TODO: b/327487822 - Change this from a global list to something more
// flexible. Also split up between "required" and "optional" extensions, and
// check against xrEnumerateInstanceExtensionProperties()
std::array<const char *, 9> kExtensions = {
    XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME,
    XR_MND_HEADLESS_EXTENSION_NAME,
    XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME,
    XR_ANDROID_TRACKABLES_EXTENSION_NAME,
    XR_ANDROID_ANCHOR_SHARING_EXPORT_EXTENSION_NAME,
    XR_ANDROID_DEVICE_ANCHOR_PERSISTENCE_EXTENSION_NAME,
    XR_ANDROID_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME,
    XR_ANDROID_RAYCAST_EXTENSION_NAME,
    XR_ANDROID_DEPTH_TEXTURE_EXTENSION_NAME,
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

// Maps the XrViewConfigurationType to the number of views for that view type.
const int kViewTypeStereoViewCount = 2;

constexpr int64_t kNanosPerSecond = 1000000000;

#define XR_ENUM_CASE_STR(name, val) \
  case name:                        \
    return #name;

// Returns a string of the enum represented by this XrResult value.
constexpr const char *XrEnumStr(XrResult e) {
  switch (e) {
    XR_LIST_ENUM_XrResult(XR_ENUM_CASE_STR) default : return "Unknown";
  }
}

// Returns a string of the enum represented by this XrSessionState value.
constexpr const char *XrSessionStateEnumStr(XrSessionState e) {
  switch (e) {
    XR_LIST_ENUM_XrSessionState(XR_ENUM_CASE_STR) default : return "Unknown";
  }
}

constexpr const char *XrPlaneTypeEnumStr(XrPlaneTypeANDROID e) {
  switch (e) {
    XR_LIST_ENUM_XrPlaneTypeANDROID(XR_ENUM_CASE_STR) default
        : return "Unknown";
  }
}

constexpr const char *XrPlaneLabelEnumStr(XrPlaneLabelANDROID e) {
  switch (e) {
    XR_LIST_ENUM_XrPlaneLabelANDROID(XR_ENUM_CASE_STR) default
        : return "Unknown";
  }
}

constexpr const char *XrDepthCameraResolutionEnumStr(
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

// Gets the depth image width and height for a given resolution. Returns false
// if the resolution is not supported.
bool GetDepthCameraImageWidthAndHeight(
    const XrDepthCameraResolutionANDROID &resolution, int *width, int *height) {
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

OpenXrManagerClockInterface *OpenXrManager::GetOpenXrManagerClock() {
  // The clock should be a singleton. So we are creating it here without any
  // destruction. It will not be destroyed which is ok because it is a singleton
  // that needs to be alive for the entire duration of the process.
  static OpenXrManagerClock *kOpenXrManagerClock = new OpenXrManagerClock();
  return kOpenXrManagerClock;
}

OpenXrManager &OpenXrManager::GetOpenXrManager() {
  return GetOpenXrManager(GetOpenXrManagerClock());
}

OpenXrManager &OpenXrManager::GetOpenXrManager(
    OpenXrManagerClockInterface *clock) {
  static absl::NoDestructor<OpenXrManager> kOpenXrManager(clock);
  return *kOpenXrManager;
}

bool OpenXrManager::InitExtensionFunctions() {
  absl::MutexLock lock(&mutex_);
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrConvertTimespecTimeToTimeKHR",
                            (PFN_xrVoidFunction *)(&convert_time_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrCreateTrackableTrackerANDROID",
      (PFN_xrVoidFunction *)(&create_trackable_tracker_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrGetAllTrackablesANDROID",
                            (PFN_xrVoidFunction *)(&get_all_trackables_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrGetTrackablePlaneANDROID",
                            (PFN_xrVoidFunction *)(&get_trackable_plane_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrDestroyTrackableTrackerANDROID",
      (PFN_xrVoidFunction *)(&destroy_trackable_tracker_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrCreateAnchorSpaceANDROID",
                            (PFN_xrVoidFunction *)(&create_anchor_space_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrShareAnchorANDROID",
                            (PFN_xrVoidFunction *)(&share_anchor_)));
  // Set up persistence functions.
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrCreateDeviceAnchorPersistenceANDROID",
                            reinterpret_cast<PFN_xrVoidFunction *>(
                                &create_device_anchor_persistence_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrDestroyDeviceAnchorPersistenceANDROID",
      reinterpret_cast<PFN_xrVoidFunction *>(
          &destroy_device_anchor_persistence_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrPersistAnchorANDROID",
      reinterpret_cast<PFN_xrVoidFunction *>(&persist_anchor_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrEnumeratePersistedAnchorsANDROID",
      reinterpret_cast<PFN_xrVoidFunction *>(&enumerate_persisted_anchors_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrGetAnchorPersistStateANDROID",
      reinterpret_cast<PFN_xrVoidFunction *>(&get_anchor_persist_state_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrUnpersistAnchorANDROID",
      reinterpret_cast<PFN_xrVoidFunction *>(&unpersist_anchor_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrCreatePersistedAnchorSpaceANDROID",
      reinterpret_cast<PFN_xrVoidFunction *>(&create_persisted_anchor_space_)));
  // Hit test functions.
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrRaycastANDROID",
                            reinterpret_cast<PFN_xrVoidFunction *>(&raycast_)));
  // Depth functions.
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrCreateDepthSwapchainANDROID",
      reinterpret_cast<PFN_xrVoidFunction *>(&create_depth_swapchain_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrDestroyDepthSwapchainANDROID",
      reinterpret_cast<PFN_xrVoidFunction *>(&destroy_depth_swapchain_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrEnumerateDepthSwapchainImagesANDROID",
                            reinterpret_cast<PFN_xrVoidFunction *>(
                                &enumerate_depth_swapchain_images_)));
  XR_RETURN_IF_FAILED(xrGetInstanceProcAddr(
      instance_, "xrEnumerateDepthResolutionsANDROID",
      reinterpret_cast<PFN_xrVoidFunction *>(&enumerate_depth_resolutions_)));
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(instance_, "xrAcquireDepthSwapchainImagesANDROID",
                            reinterpret_cast<PFN_xrVoidFunction *>(
                                &acquire_depth_swapchain_images_)));
  return true;
}

bool OpenXrManager::CreateStageReferenceSpace() {
  XrReferenceSpaceCreateInfo createInfo = {
      .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
      .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE,
      .poseInReferenceSpace = kIdentityPose,
  };

  {
    absl::MutexLock lock(&mutex_);
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
    absl::MutexLock lock(&mutex_);
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

  {
    absl::MutexLock lock(&mutex_);
    XR_RETURN_IF_FAILED(
        xrCreateReferenceSpace(session_, &createInfo, &view_space_));
  }
  return true;
}

// TODO: b/344678481 - Update this to dynamically create a space for a reference
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
    const timespec &timespec_time) const {
  XrTime xr_time;
  XrResult result;
  {
    absl::MutexLock lock(&mutex_);
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
                                  XrUuidEXT *out_anchor_uuid) {
  XrPersistedAnchorSpaceInfoANDROID create_info = {
      .type = XR_TYPE_PERSISTED_ANCHOR_SPACE_INFO_ANDROID,
      .next = nullptr,
      .anchor = anchor_space};
  {
    absl::MutexLock lock(&mutex_);
    if (!CreatePersistenceHandleIfNecessary()) {
      return false;
    }
    XR_RETURN_IF_FAILED(
        persist_anchor_(persistence_handle_, &create_info, out_anchor_uuid));
  }
  return true;
}

bool OpenXrManager::GetAnchorPersistState(
    const XrUuidEXT &anchor_uuid,
    XrAnchorPersistStateANDROID *out_persist_state) {
  {
    absl::MutexLock lock(&mutex_);
    if (!CreatePersistenceHandleIfNecessary()) {
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
    absl::MutexLock lock(&mutex_);
    if (!CreatePersistenceHandleIfNecessary()) {
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
    absl::MutexLock lock(&mutex_);
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

bool OpenXrManager::UnpersistAnchor(const XrUuidEXT &anchor_uuid) {
  {
    absl::MutexLock lock(&mutex_);
    if (!CreatePersistenceHandleIfNecessary()) {
      return false;
    }
    XR_RETURN_IF_FAILED(unpersist_anchor_(persistence_handle_, &anchor_uuid));
  }
  return true;
}

bool OpenXrManager::LocatePersistedAnchorSpace(const XrUuidEXT &anchor_uuid,
                                               XrSpace *out_anchor_space) {
  absl::uint128 anchor_uuid_uint128 = UuidToUint128(anchor_uuid);
  XrSpace anchor_space = XR_NULL_HANDLE;
  {
    absl::MutexLock lock(&mutex_);
    auto iter = persist_anchor_uuid_to_space_map_.find(anchor_uuid_uint128);
    if (iter != persist_anchor_uuid_to_space_map_.end()) {
      anchor_space = iter->second;
    } else {
      if (!CreatePersistenceHandleIfNecessary()) {
        return false;
      }
      XrPersistedAnchorSpaceCreateInfoANDROID create_info = {
          .type = XR_TYPE_PERSISTED_ANCHOR_SPACE_CREATE_INFO_ANDROID,
          .next = nullptr,
          .anchorId = anchor_uuid,
      };
      XR_RETURN_IF_FAILED(create_persisted_anchor_space_(
          persistence_handle_, &create_info, &anchor_space));
      persist_anchor_uuid_to_space_map_[anchor_uuid_uint128] = anchor_space;
    }
  }

  XrSpaceLocation location = {
      .type = XR_TYPE_SPACE_LOCATION,
  };
  XR_RETURN_IF_FAILED(xrLocateSpace(anchor_space,
                                    GetSpaceInDefaultReferenceSpace(),
                                    GetXrTimeNow(), &location));
  *out_anchor_space = anchor_space;
  return true;
}

bool OpenXrManager::HitTest(XrRaycastInfoANDROID *raycast_info,
                            XrRaycastHitResultsANDROID *out_hit_results) {
  {
    absl::MutexLock lock(&mutex_);
    raycast_info->space = GetSpaceInDefaultReferenceSpace();
    raycast_info->trackerCount = 1;
    raycast_info->trackers = &planes_trackable_tracker_;
    XR_RETURN_IF_FAILED(raycast_(session_, raycast_info, out_hit_results));
  }
  return true;
}

bool OpenXrManager::GetDepthImage(XrTime time,
                                  const float **out_smooth_depth_image,
                                  int *out_image_width, int *out_image_height) {
  absl::MutexLock lock(&mutex_);
  if (!CreateDepthSwapchainIfNecessary()) {
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
  const XrDepthSwapchainImageANDROID &acquired_depth_image =
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

bool OpenXrManager::Init(JNIEnv *env, jobject activity,
                         XrReferenceSpaceType default_reference_space) {
  java_env_ = env;
  java_env_->GetJavaVM(&app_vm_);
  absl::MutexLock state_lock(&initialization_mutex_);
  {
    absl::MutexLock lock(&mutex_);

    if (open_xr_state_ == OpenXrState::kResumed) {
      LOG(INFO) << "Returning existing OpenXR session.";
      return true;
    } else if (open_xr_state_ == OpenXrState::kPaused) {
      LOG(INFO) << "Resuming an existing OpenXR session.";
      StartPollingThread();
      return true;
    }
    open_xr_state_ = OpenXrState::kInitializing;
    activity_ = java_env_->NewGlobalRef(activity);
    default_reference_space_ = default_reference_space;
  }

  // Load OpenXR.
  if (!LoadOpenXr()) {
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

  // Creates a view space that will be used for retrieving the head poses.
  if (!CreateViewReferenceSpace()) {
    DeInitWithLockHeld();
    return false;
  }

  {
    absl::MutexLock lock(&mutex_);
    StartPollingThread();
  }
  return true;
}

void OpenXrManager::DeInit(bool stop_polling_thread) {
  absl::ReaderMutexLock state_lock(&initialization_mutex_);
  DeInitWithLockHeld(stop_polling_thread);
}

void OpenXrManager::DeInitWithLockHeld(bool stop_polling_thread) {
  {
    absl::MutexLock lock(&mutex_);
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
    // Destroy trackable tracker.
    if (planes_trackable_tracker_ != XR_NULL_HANDLE) {
      XrResult result = destroy_trackable_tracker_(planes_trackable_tracker_);
      planes_trackable_tracker_ = XR_NULL_HANDLE;
      if (XR_FAILED(result)) {
        LOG(ERROR) << "Failed to destroy trackable tracker with error: "
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

    instance_ = XR_NULL_HANDLE;
    system_id_ = XR_NULL_SYSTEM_ID;
    session_ = XR_NULL_HANDLE;
    stop_polling_ = true;
    planes_trackable_tracker_ = XR_NULL_HANDLE;
    persist_anchor_uuid_to_space_map_.clear();
    java_env_->DeleteGlobalRef(activity_);
  }
  if (stop_polling_thread) {
    JoinPollingThread();
  }
  {
    absl::MutexLock lock(&mutex_);
    open_xr_state_ = OpenXrState::kUninitialized;
  }
}

bool OpenXrManager::PauseSession() {
  {
    absl::MutexLock lock(&mutex_);
    if (open_xr_state_ != OpenXrState::kResumed) {
      LOG(ERROR) << "Attempted to pause the Open XR session when session is "
                    "not in a resumed state. ";
      return false;
    }
    stop_polling_ = true;
  }
  JoinPollingThread();
  {
    absl::MutexLock lock(&mutex_);
    open_xr_state_ = OpenXrState::kPaused;
  }
  return true;
}

bool OpenXrManager::LoadOpenXr() {
  PFN_xrInitializeLoaderKHR initialize_loader = nullptr;

  // Gets a function pointer to the OpenXR loader.
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                            (PFN_xrVoidFunction *)(&initialize_loader)));
  if (initialize_loader == nullptr) {
    LOG(ERROR) << "Failure loading OpenXR. Loader is null.";
    return false;
  }
  XrLoaderInitInfoAndroidKHR loader_init_info_android;
  {
    absl::MutexLock lock(&mutex_);
    loader_init_info_android = {
        .type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR,
        .applicationVM = app_vm_,
        .applicationContext = activity_,
    };
  }

  // Call the loader function obtained above to load OpenXR.
  XR_RETURN_IF_FAILED(
      initialize_loader(reinterpret_cast<const XrLoaderInitInfoBaseHeaderKHR *>(
          &loader_init_info_android)));
  LOG(INFO) << "OpenXR is loaded.";
  return true;
}

bool OpenXrManager::CreateInstance() {
  XrInstanceCreateInfoAndroidKHR create_info_android;
  {
    absl::MutexLock lock(&mutex_);
    create_info_android = {
        .type = XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR,
        .applicationVM = app_vm_,
        .applicationActivity = activity_,
    };
  }

  XrInstanceCreateInfo create_info = {
      .type = XR_TYPE_INSTANCE_CREATE_INFO,
      .next = &create_info_android,
      .applicationInfo =
          {
              // TODO(b/341623322): Revert back to XR_CURRENT_API_VERSION after
              // the OpenXR loader version mismatch issue is resolved.
              .apiVersion =
                  XR_MAKE_VERSION(1, 0, 34),  // XR_CURRENT_API_VERSION,
          },
      .enabledApiLayerCount = 0,
      .enabledApiLayerNames = nullptr,
      .enabledExtensionCount = kExtensions.size(),
      .enabledExtensionNames = kExtensions.data(),
  };
  strncpy(create_info.applicationInfo.applicationName, kApplicationName,
          XR_MAX_APPLICATION_NAME_SIZE);

  {
    absl::MutexLock lock(&mutex_);
    // Create an OpenXR instance.
    XR_RETURN_IF_FAILED(xrCreateInstance(&create_info, &instance_));
  }

  LOG(INFO) << "XrInstance created.";
  return true;
}

bool OpenXrManager::GetXrSystem() {
  // TODO: b/328066169 - Update this to dynamically evaluate the form factor
  // once we support multiple form factors.
  XrSystemGetInfo system_info = {
      .type = XR_TYPE_SYSTEM_GET_INFO,
      .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY,
  };

  {
    absl::MutexLock lock(&mutex_);
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
    absl::MutexLock lock(&mutex_);
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

bool OpenXrManager::MaybeCreateTrackableTracker() {
  if (planes_trackable_tracker_ != XR_NULL_HANDLE) {
    return true;
  }
  XrTrackableTrackerCreateInfoANDROID createInfo = {
      .type = XR_TYPE_TRACKABLE_TRACKER_CREATE_INFO_ANDROID,
      .trackableType = XR_TRACKABLE_TYPE_PLANE_ANDROID};
  XR_RETURN_IF_FAILED(create_trackable_tracker_(session_, &createInfo,
                                                &planes_trackable_tracker_));
  return true;
}

std::vector<XrTrackableANDROID> OpenXrManager::GetPlanes() {
  uint32_t trackableCountOutput = 0;

  // Query the number of trackables available.
  XrResult result;
  {
    absl::MutexLock lock(&mutex_);
    if (!MaybeCreateTrackableTracker()) {
      return {};
    }
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
    absl::MutexLock lock(&mutex_);
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
                                  XrTrackablePlaneANDROID &out_plane) {
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
    absl::ReaderMutexLock lock(&mutex_);
    result = get_trackable_plane_(planes_trackable_tracker_, &plane_get_info,
                                  &out_plane);
  }
  if (XR_FAILED(result)) {
    LOG(ERROR) << "Failed to get plane vertex count with error: "
               << XrEnumStr(result);
    return false;
  }

  out_plane.vertexCapacityInput = *out_plane.vertexCountOutput;
  std::vector<XrVector2f> vertices;
  vertices.resize(*out_plane.vertexCountOutput);
  out_plane.vertices = vertices.data();
  {
    absl::ReaderMutexLock lock(&mutex_);
    result = get_trackable_plane_(planes_trackable_tracker_, &plane_get_info,
                                  &out_plane);
  }
  if (XR_FAILED(result)) {
    LOG(ERROR) << "Failed to get plane with error: " << XrEnumStr(result);
    return false;
  }

  return true;
}

bool OpenXrManager::ChoosePlane(const PlaneConstraints &plane_constraints,
                                XrTrackableANDROID *out_trackable,
                                XrTrackablePlaneANDROID *out_plane) {
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
      absl::ReaderMutexLock lock(&mutex_);
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

bool OpenXrManager::CreateAnchor(XrTime time, const XrPosef &pose,
                                 XrSpace *out_anchor_space) {
  XrAnchorSpaceCreateInfoANDROID trackable_anchor_create_info = {
      .type = XR_TYPE_ANCHOR_SPACE_CREATE_INFO_ANDROID,
      .space = GetSpaceInDefaultReferenceSpace(),
      .time = time,
      .pose = pose,
      .trackable = XR_NULL_TRACKABLE_ANDROID,
  };

  {
    absl::ReaderMutexLock lock(&mutex_);
    XR_RETURN_IF_FAILED(create_anchor_space_(
        session_, &trackable_anchor_create_info, out_anchor_space));
  }
  return true;
}

bool OpenXrManager::CreateAnchorForPlane(XrTrackableANDROID trackable,
                                         XrTrackablePlaneANDROID *plane,
                                         XrTime time,
                                         const XrPosef &relative_pose,
                                         XrSpace *out_anchor_space) {
  if (trackable == XR_NULL_TRACKABLE_ANDROID) {
    LOG(ERROR) << "Cannot create anchor for null trackable.";
    return false;
  }

  // If the provided plane is null, we will retrieve the plane from the
  // trackable.
  uint32_t vertex_count = 0;
  XrTrackablePlaneANDROID retrieved_plane = {
      .type = XR_TYPE_TRACKABLE_PLANE_ANDROID,
      .vertexCapacityInput = 0,
      .vertexCountOutput = &vertex_count,
      .vertices = nullptr,
  };
  if (plane == nullptr && !GetPlaneState(trackable, default_reference_space_,
                                         time, retrieved_plane)) {
    LOG(ERROR) << "Failed to get plane for trackable during anchor creation.";
    return false;
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

  {
    absl::ReaderMutexLock lock(&mutex_);
    XR_RETURN_IF_FAILED(create_anchor_space_(
        session_, &trackableAnchorCreateInfo, out_anchor_space));
  }
  return true;
}

bool OpenXrManager::GetAnchorLocationData(
    XrSpace anchor_space, XrTime time, XrSpaceLocation *out_anchor_location) {
  XR_RETURN_IF_FAILED(xrLocateSpace(anchor_space,
                                    GetSpaceInDefaultReferenceSpace(), time,
                                    out_anchor_location));
  return true;
}

bool OpenXrManager::ExportAnchor(XrSpace anchor_space,
                                 AIBinder **out_anchor_token) {
  XrAnchorSharingInfoANDROID sharingInfo = {
      .type = XR_TYPE_ANCHOR_SHARING_INFO_ANDROID,
      .anchor = anchor_space,
  };

  XrAnchorSharingTokenANDROID token = {
      .type = XR_TYPE_ANCHOR_SHARING_TOKEN_ANDROID,
  };

  {
    absl::ReaderMutexLock lock(&mutex_);
    XR_RETURN_IF_FAILED(share_anchor_(session_, &sharingInfo, &token));
  }
  *out_anchor_token = token.token;
  return true;
}

bool OpenXrManager::CreateSemanticAnchor(
    const PlaneConstraints &plane_constraints, AIBinder **out_anchor_token,
    XrSpace *out_anchor_space) {
  XrTrackableANDROID selected_trackable;
  XrTrackablePlaneANDROID selected_plane;
  if (!ChoosePlane(plane_constraints, &selected_trackable, &selected_plane)) {
    return false;
  }
  if (!CreateAnchorForPlane(selected_trackable, &selected_plane, GetXrTimeNow(),
                            kIdentityPose, out_anchor_space)) {
    return false;
  }
  if (!ExportAnchor(*out_anchor_space, out_anchor_token)) {
    xrDestroySpace(*out_anchor_space);
    return false;
  }
  return true;
}

bool OpenXrManager::DestroyAnchor(XrSpace anchor_space) {
  XR_RETURN_IF_FAILED(xrDestroySpace(anchor_space));
  return true;
}

bool OpenXrManager::GetHeadPose(XrTime time, XrPosef *out_pose) {
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
                                   std::vector<XrView> *out_views) {
  if (out_views == nullptr || out_views->size() != kViewTypeStereoViewCount) {
    LOG(ERROR) << "GetStereoViews expected out_views to be of size "
               << kViewTypeStereoViewCount << " but got " << out_views->size();
    return false;
  }

  XrViewState view_state{.type = XR_TYPE_VIEW_STATE, .next = nullptr};
  const XrViewLocateInfo view_locate_info = {
      .type = XR_TYPE_VIEW_LOCATE_INFO,
      .next = nullptr,
      .viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
      .displayTime = time,
      .space = GetSpaceInDefaultReferenceSpace()};
  uint32_t view_count;
  {
    absl::MutexLock lock(&mutex_);
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

void OpenXrManager::HandleSessionChangedEvent(
    const XrEventDataSessionStateChanged &changed_event) {
  LOG(INFO) << "Received session changed event: "
            << XrSessionStateEnumStr(changed_event.state);

  switch (changed_event.state) {
    case XR_SESSION_STATE_READY: {
      XrSessionBeginInfo beginInfo = {
          .type = XR_TYPE_SESSION_BEGIN_INFO,
          .primaryViewConfigurationType =
              XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};

      {
        absl::MutexLock lock(&mutex_);
        xrBeginSession(session_, &beginInfo);
      }
      break;
    }
    case XR_SESSION_STATE_STOPPING: {
      {
        absl::MutexLock lock(&mutex_);
        xrEndSession(session_);
      }
      break;
    }
    case XR_SESSION_STATE_EXITING: {
      DeInit(/*stop_polling_thread=*/false);
      break;
    }
    case XR_SESSION_STATE_FOCUSED: {
      // When we are in focused state create the trackable tracker if it is
      // not already created.
      {
        absl::MutexLock lock(&mutex_);
        MaybeCreateTrackableTracker();
      }
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
  absl::MutexLock lock(&mutex_);
  return !stop_polling_;
}

void OpenXrManager::PollingLoop() {
  timespec poll_time = clock_->TimeNow();
  while (ShouldPoll()) {
    PollOpenXR();
    poll_time = AddTimespecs(poll_time, kPollingInterval);
    {
      absl::MutexLock lock(&mutex_);
      clock_->AwaitWithDeadline(&mutex_, &stop_polling_, poll_time);
    }
  }
}

void OpenXrManager::PollOpenXR() {
  while (true) {
    XrEventDataBuffer event = {XR_TYPE_EVENT_DATA_BUFFER};
    XrResult result;
    {
      absl::MutexLock lock(&mutex_);
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
            *reinterpret_cast<const XrEventDataInstanceLossPending *>(&event);
        LOG(INFO) << "Received data instance loss event, session will end at: "
                  << instance_loss_event.lossTime;
        break;
      }
      case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
        const XrEventDataSessionStateChanged &session_state_changed_event =
            *reinterpret_cast<XrEventDataSessionStateChanged *>(&event);
        HandleSessionChangedEvent(session_state_changed_event);
        break;
      }
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

bool OpenXrManager::CreatePersistenceHandleIfNecessary() {
  if (persistence_handle_ != XR_NULL_HANDLE) {
    return true;
  }
  XrDeviceAnchorPersistenceCreateInfoANDROID create_info = {
      .type = XR_TYPE_DEVICE_ANCHOR_PERSISTENCE_CREATE_INFO_ANDROID,
      .next = nullptr,
  };
  XR_RETURN_IF_FAILED(create_device_anchor_persistence_(session_, &create_info,
                                                        &persistence_handle_));
  return true;
}

void OpenXrManager::JoinPollingThread() {
  if (polling_thread_ != nullptr && polling_thread_->joinable()) {
    polling_thread_->join();
    polling_thread_.reset();
  }
}

XrSession OpenXrManager::GetXrSession() {
  absl::MutexLock lock(&mutex_);

  return session_;
}

XrInstance OpenXrManager::GetXrInstance() {
  absl::MutexLock lock(&mutex_);

  return instance_;
}

bool OpenXrManager::CreateDepthSwapchainIfNecessary() {
  if (depth_swapchain_handle_ != XR_NULL_HANDLE) {
    return true;
  }

  uint32_t supported_resolution_count = 0;
  XR_RETURN_IF_FAILED(enumerate_depth_resolutions_(
      session_, 1, &supported_resolution_count, &supported_depth_resolution_));
  if (supported_resolution_count == 0) {
    LOG(ERROR) << "No supported depth resolutions found.";
    return false;
  }

  LOG(INFO) << "Supported depth resolution: "
            << XrDepthCameraResolutionEnumStr(supported_depth_resolution_);

  XrDepthSwapchainCreateInfoANDROID create_info = {
      .type = XR_TYPE_DEPTH_SWAPCHAIN_CREATE_INFO_ANDROID,
      .next = nullptr,
      .resolution = supported_depth_resolution_,
      .createFlags =
          XR_DEPTH_SWAPCHAIN_CREATE_SMOOTH_DEPTH_IMAGE_BIT_ANDROID |
          XR_DEPTH_SWAPCHAIN_CREATE_SMOOTH_CONFIDENCE_IMAGE_BIT_ANDROID |
          XR_DEPTH_SWAPCHAIN_CREATE_RAW_DEPTH_IMAGE_BIT_ANDROID |
          XR_DEPTH_SWAPCHAIN_CREATE_RAW_CONFIDENCE_IMAGE_BIT_ANDROID,
  };
  XR_RETURN_IF_FAILED(create_depth_swapchain_(session_, &create_info,
                                              &depth_swapchain_handle_));

  uint32_t swapchain_size = 0;
  XR_RETURN_IF_FAILED(enumerate_depth_swapchain_images_(
      depth_swapchain_handle_, 0, &swapchain_size, nullptr));
  LOG(INFO) << "Depth texture swapchain has size = " << swapchain_size;

  depth_images_.resize(swapchain_size);
  for (uint32_t i = 0; i < swapchain_size; ++i) {
    depth_images_[i] = {.type = XR_TYPE_DEPTH_SWAPCHAIN_IMAGE_ANDROID};
  }
  XR_RETURN_IF_FAILED(
      enumerate_depth_swapchain_images_(depth_swapchain_handle_, swapchain_size,
                                        &swapchain_size, depth_images_.data()));

  LOG(INFO) << "Depth swapchain created successfully.";
  return true;
}

}  // namespace androidx::xr::openxr
