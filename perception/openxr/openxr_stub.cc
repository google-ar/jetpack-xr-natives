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

#include <jni.h>  // IWYU pragma: keep
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_platform_defines.h>

#include <cstdint>
#include <cstring>
#include <ctime>

#include "openxr/openxr.h"
#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"

namespace {
const XrPosef kPose = XrPosef{
    .orientation = XrQuaternionf{0.0f, 1.0f, 0.0f, 1.0f},
    .position = XrVector3f{0.0f, 0.0f, 2.0f},
};

const XrExtent2Df kExtent2D = XrExtent2Df{1.0f, 2.0f};

uint32_t kVertexCapacityInput = 4;
uint32_t kVertexCountOutput = 4;
XrVector2f* kVertices = new XrVector2f[kVertexCapacityInput]{
    {2.0f, 0.0f}, {2.0f, 2.0f}, {0.0f, 0.0f}, {0.0f, 2.0f}};

const XrSpaceLocationFlags kLocationFlags =
    XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
    XR_SPACE_LOCATION_POSITION_VALID_BIT |
    XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT |
    XR_SPACE_LOCATION_POSITION_TRACKED_BIT;

const XrInstance kInstance = XrInstance(1111);
const XrSystemId kSystemId = XrSystemId(2222);
const XrSession kSession = XrSession(3333);
const XrSpace kSpace = XrSpace(4444);
const XrTime kTime = 1000;
const XrUuidEXT kUuid = {
    .data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}};
const XrTrackableANDROID kTrackable = XrTrackableANDROID(1);
const XrTrackableANDROID kPlaneSubsumedId = XrTrackableANDROID(67890L);
XrRaycastHitResultANDROID kRaycastHitResult = {
    .type = XR_TRACKABLE_TYPE_PLANE_ANDROID,
    .trackable = kTrackable,
    .pose = kPose,
};
const XrRaycastHitResultsANDROID kRaycastHitResults = {
    .type = XR_TYPE_RAYCAST_HIT_RESULTS_ANDROID,
    .next = nullptr,
    .resultsCapacityInput = 1,
    .resultsCountOutput = 1,
    .results = &kRaycastHitResult,
};
const XrSpaceLocationFlags kValidLocationFlags =
    XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
    XR_SPACE_LOCATION_POSITION_VALID_BIT;
const XrHandJointLocationEXT kHandJoint = {
    .locationFlags = kValidLocationFlags,
    .pose = {.orientation = {.x = 1, .y = 2, .z = 3, .w = 4},
             .position = {.x = 5, .y = 6, .z = 7}},
    .radius = 1,
};
const XrTrackableTrackerANDROID kTrackableTracker =
    XrTrackableTrackerANDROID(1);
const XrHandTrackerEXT kHandTracker = XrHandTrackerEXT(1);
const XrDeviceAnchorPersistenceANDROID kAnchorPersistence =
    XrDeviceAnchorPersistenceANDROID(1);

const int kAnchorResourceLimit = 5;
const XrUuid kZeroUuid = {0};

int convert_to_khr_time_call_counter = 0;
int create_anchor_call_counter = 0;

bool anchor_persistence_handle_created = false;

}  // namespace

extern "C" {
XRAPI_ATTR XrResult XRAPI_CALL Internal_xrInitializeLoaderKHR(
    const XrLoaderInitInfoBaseHeaderKHR* loaderInitInfo) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrConvertTimespecTimeToTimeKHR(
    XrInstance instance, const struct timespec* timespecTime, XrTime* time) {
  *time = kTime * ++convert_to_khr_time_call_counter;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateTrackableTrackerANDROID(
    XrSession session, const XrTrackableTrackerCreateInfoANDROID* createInfo,
    XrTrackableTrackerANDROID* trackableTracker) {
  *trackableTracker = kTrackableTracker;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrDestroyTrackableTrackerANDROID(
    XrTrackableTrackerANDROID trackableTracker) {
  if (!trackableTracker) {
    return XR_ERROR_HANDLE_INVALID;
  }
  trackableTracker = XR_NULL_HANDLE;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrGetAllTrackablesANDROID(
    XrTrackableTrackerANDROID trackableTracker, uint32_t trackableCapacityInput,
    uint32_t* trackableCountOutput, XrTrackableANDROID* trackables) {
  if (!trackableTracker) {
    return XR_ERROR_HANDLE_INVALID;
  }
  *trackableCountOutput = 1;
  if (trackables) {
    *trackables = kTrackable;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrGetTrackablePlaneANDROID(XrTrackableTrackerANDROID trackableTracker,
                                    const XrTrackableGetInfoANDROID* getInfo,
                                    XrTrackablePlaneANDROID* planeOutput) {
  if (!trackableTracker) {
    return XR_ERROR_HANDLE_INVALID;
  }
  planeOutput->trackingState =
      XrTrackingStateANDROID(XR_TRACKING_STATE_TRACKING_ANDROID);
  planeOutput->centerPose = kPose;
  planeOutput->extents = kExtent2D;
  planeOutput->vertexCapacityInput = kVertexCapacityInput;
  planeOutput->vertexCountOutput = &kVertexCountOutput;
  planeOutput->vertices = kVertices;
  if (getInfo->trackable == kPlaneSubsumedId) {
    planeOutput->subsumedByPlane = kTrackable;
  } else {
    planeOutput->subsumedByPlane = XR_NULL_TRACKABLE_ANDROID;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateAnchorSpaceANDROID(
    XrSession session, const XrAnchorSpaceCreateInfoANDROID* createInfo,
    XrSpace* anchorOutput) {
  ++create_anchor_call_counter;
  if (create_anchor_call_counter > kAnchorResourceLimit) {
    return XR_ERROR_LIMIT_REACHED;
  }
  *anchorOutput = kSpace;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrShareAnchorANDROID(
    XrSession session, const XrAnchorSharingInfoANDROID* sharingInfo,
    XrAnchorSharingTokenANDROID* anchorToken) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateDeviceAnchorPersistenceANDROID(
    XrSession session,
    const XrDeviceAnchorPersistenceCreateInfoANDROID* createInfo,
    XrDeviceAnchorPersistenceANDROID* outHandle) {
  *outHandle = kAnchorPersistence;
  anchor_persistence_handle_created = true;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrDestroyDeviceAnchorPersistenceANDROID(
    XrDeviceAnchorPersistenceANDROID handle) {
  anchor_persistence_handle_created = false;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrPersistAnchorANDROID(
    XrDeviceAnchorPersistenceANDROID handle,
    const XrPersistedAnchorSpaceInfoANDROID* persistedInfo,
    XrUuidEXT* anchorIdOutput) {
  if (!anchor_persistence_handle_created) {
    return XR_ERROR_HANDLE_INVALID;
  }
  if (anchorIdOutput) {
    *anchorIdOutput = kUuid;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrGetAnchorPersistStateANDROID(
    XrDeviceAnchorPersistenceANDROID handle, const XrUuidEXT* anchorId,
    XrAnchorPersistStateANDROID* persistState) {
  if (!anchor_persistence_handle_created) {
    return XR_ERROR_HANDLE_INVALID;
  }
  if (persistState) {
    *persistState =
        XrAnchorPersistStateANDROID(XR_ANCHOR_PERSIST_STATE_PERSISTED_ANDROID);
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreatePersistedAnchorSpaceANDROID(
    XrDeviceAnchorPersistenceANDROID handle,
    const XrPersistedAnchorSpaceCreateInfoANDROID* createInfo,
    XrSpace* anchorOutput) {
  if (!anchor_persistence_handle_created) {
    return XR_ERROR_HANDLE_INVALID;
  }
  if (createInfo != nullptr &&
      memcmp(createInfo->anchorId.data, kZeroUuid.data, XR_UUID_SIZE) == 0) {
    return XR_ERROR_PERSIST_UUID_NOT_FOUND_EXT;
  }
  ++create_anchor_call_counter;
  if (create_anchor_call_counter > kAnchorResourceLimit) {
    return XR_ERROR_LIMIT_REACHED;
  }
  if (anchorOutput) {
    *anchorOutput = kSpace;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrEnumeratePersistedAnchorsANDROID(
    XrDeviceAnchorPersistenceANDROID handle, uint32_t anchorIdsCapacityInput,
    uint32_t* anchorIdsCountOutput, XrUuidEXT* anchorIds) {
  if (!anchor_persistence_handle_created) {
    return XR_ERROR_HANDLE_INVALID;
  }
  if (anchorIdsCountOutput) {
    *anchorIdsCountOutput = 1;
  }
  if (anchorIds) {
    *anchorIds = kUuid;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrUnpersistAnchorANDROID(
    XrDeviceAnchorPersistenceANDROID handle, const XrUuidEXT* anchorId) {
    if (anchorId != nullptr &&
        memcmp(anchorId->data, kZeroUuid.data, XR_UUID_SIZE) == 0) {
      return XR_ERROR_PERSIST_UUID_NOT_FOUND_EXT;
    }
  if (!anchor_persistence_handle_created) {
    return XR_ERROR_HANDLE_INVALID;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrRaycastANDROID(
    XrSession session, const XrRaycastInfoANDROID* rayInfo,
    XrRaycastHitResultsANDROID* outHitResults) {
  *outHitResults = kRaycastHitResults;

  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateHandTrackerEXT(
    XrSession session, const XrHandTrackerCreateInfoEXT* createInfo,
    XrHandTrackerEXT* handTracker) {
  *handTracker = kHandTracker;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrDestroyHandTrackerEXT(XrHandTrackerEXT handTracker) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrLocateHandJointsEXT(
    XrHandTrackerEXT handTracker, const XrHandJointsLocateInfoEXT* locateInfo,
    XrHandJointLocationsEXT* locations) {
  if (!handTracker) {
    return XR_ERROR_HANDLE_INVALID;
  }
  locations->type = XR_TYPE_HAND_JOINT_LOCATIONS_EXT;
  locations->isActive = true;
  locations->jointCount = XR_HAND_JOINT_COUNT_EXT;
  for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i) {
    locations->jointLocations[i].locationFlags = kValidLocationFlags;
    locations->jointLocations[i].pose.orientation.x = i + 0.1f;
    locations->jointLocations[i].pose.orientation.y = i + 0.2f;
    locations->jointLocations[i].pose.orientation.z = i + 0.3f;
    locations->jointLocations[i].pose.orientation.w = i + 0.4f;
    locations->jointLocations[i].pose.position.x = i + 0.5f;
    locations->jointLocations[i].pose.position.y = i + 0.6f;
    locations->jointLocations[i].pose.position.z = i + 0.7f;
    locations->jointLocations[i].radius = i + 0.8f;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrEnumerateDepthResolutionsANDROID(
    XrSession session, uint32_t resolutionCapacityInput,
    uint32_t* resolutionCountOutput,
    XrDepthCameraResolutionANDROID* resolutions) {
  // Returning this temporarily to test different configuration results while
  // Depth APIs are unused.
  return XR_ERROR_PERMISSION_INSUFFICIENT;
}

}  // extern "C"

namespace {
template <typename TypedFunction>
PFN_xrVoidFunction ToXrVoidFunction(TypedFunction* function) {
  return reinterpret_cast<PFN_xrVoidFunction>(function);
}

const auto kXrFunctions = new absl::flat_hash_map<absl::string_view,
                                                  PFN_xrVoidFunction>({
    {"xrInitializeLoaderKHR", ToXrVoidFunction(Internal_xrInitializeLoaderKHR)},
    {"xrConvertTimespecTimeToTimeKHR",
     ToXrVoidFunction(Internal_xrConvertTimespecTimeToTimeKHR)},
    {"xrCreateTrackableTrackerANDROID",
     ToXrVoidFunction(Internal_xrCreateTrackableTrackerANDROID)},
    {"xrGetAllTrackablesANDROID",
     ToXrVoidFunction(Internal_xrGetAllTrackablesANDROID)},
    {"xrGetTrackablePlaneANDROID",
     ToXrVoidFunction(Internal_xrGetTrackablePlaneANDROID)},
    {"xrDestroyTrackableTrackerANDROID",
     ToXrVoidFunction(Internal_xrDestroyTrackableTrackerANDROID)},
    {"xrCreateAnchorSpaceANDROID",
     ToXrVoidFunction(Internal_xrCreateAnchorSpaceANDROID)},
    {"xrShareAnchorANDROID", ToXrVoidFunction(Internal_xrShareAnchorANDROID)},
    {"xrCreateDeviceAnchorPersistenceANDROID",
     ToXrVoidFunction(Internal_xrCreateDeviceAnchorPersistenceANDROID)},
    {"xrDestroyDeviceAnchorPersistenceANDROID",
     ToXrVoidFunction(Internal_xrDestroyDeviceAnchorPersistenceANDROID)},
    {"xrPersistAnchorANDROID",
     ToXrVoidFunction(Internal_xrPersistAnchorANDROID)},
    {"xrGetAnchorPersistStateANDROID",
     ToXrVoidFunction(Internal_xrGetAnchorPersistStateANDROID)},
    {"xrCreatePersistedAnchorSpaceANDROID",
     ToXrVoidFunction(Internal_xrCreatePersistedAnchorSpaceANDROID)},
    {"xrEnumeratePersistedAnchorsANDROID",
     ToXrVoidFunction(Internal_xrEnumeratePersistedAnchorsANDROID)},
    {"xrUnpersistAnchorANDROID",
     ToXrVoidFunction(Internal_xrUnpersistAnchorANDROID)},
    {"xrRaycastANDROID", ToXrVoidFunction(Internal_xrRaycastANDROID)},
    {"xrCreateHandTrackerEXT",
     ToXrVoidFunction(Internal_xrCreateHandTrackerEXT)},
    {"xrDestroyHandTrackerEXT",
     ToXrVoidFunction(Internal_xrDestroyHandTrackerEXT)},
    {"xrLocateHandJointsEXT", ToXrVoidFunction(Internal_xrLocateHandJointsEXT)},
    {"xrEnumerateDepthResolutionsANDROID",
     ToXrVoidFunction(Internal_xrEnumerateDepthResolutionsANDROID)},
});

}  // namespace

extern "C" {
XRAPI_ATTR XrResult XRAPI_CALL xrGetInstanceProcAddr(
    XrInstance instance, const char* name, PFN_xrVoidFunction* function) {
  auto it = kXrFunctions->find(name);
  if (it != kXrFunctions->end()) {
    *function = it->second;
  } else {
    *function = nullptr;
  }
  // We still return XR_SUCCESS even if the function if we do not have a test
  // implementation for them since the OpenXR manager requires that all
  // function lookups return success.
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
xrCreateInstance(const XrInstanceCreateInfo* createInfo, XrInstance* instance) {
  *instance = kInstance;
  create_anchor_call_counter = 0;
  // TODO: Temporarily enabling some features by default until
  // session configuration is fully implemented.
  anchor_persistence_handle_created = true;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetSystem(XrInstance instance,
                                           const XrSystemGetInfo* getInfo,
                                           XrSystemId* systemId) {
  *systemId = kSystemId;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
xrCreateSession(XrInstance instance, const XrSessionCreateInfo* createInfo,
                XrSession* session) {
  *session = kSession;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
xrBeginSession(XrSession session, const XrSessionBeginInfo* beginInfo) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrEndSession(XrSession session) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroySession(XrSession session) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyInstance(XrInstance instance) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroySpace(XrSpace space) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrLocateSpace(XrSpace space, XrSpace baseSpace,
                                             XrTime time,
                                             XrSpaceLocation* location) {
  location->type = XR_TYPE_SPACE_LOCATION;
  location->next = nullptr;
  location->locationFlags = kLocationFlags;
  location->pose = kPose;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
xrLocateViews(XrSession session, const XrViewLocateInfo* viewLocateInfo,
              XrViewState* viewState, uint32_t viewCapacityInput,
              uint32_t* viewCountOutput, XrView* views) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrPollEvent(XrInstance instance,
                                           XrEventDataBuffer* eventData) {
  return XR_EVENT_UNAVAILABLE;
}

XRAPI_ATTR XrResult XRAPI_CALL xrCreateReferenceSpace(
    XrSession session, const XrReferenceSpaceCreateInfo* createInfo,
    XrSpace* space) {
  *space = kSpace;
  return XR_SUCCESS;
}

}  // extern "C"
