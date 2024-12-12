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
#include <ctime>

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
int convert_to_khr_time_call_counter = 0;

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
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrDestroyTrackableTrackerANDROID(
    XrTrackableTrackerANDROID trackableTracker) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrGetAllTrackablesANDROID(
    XrTrackableTrackerANDROID trackableTracker, uint32_t trackableCapacityInput,
    uint32_t* trackableCountOutput, XrTrackableANDROID* trackables) {
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
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrDestroyDeviceAnchorPersistenceANDROID(
    XrDeviceAnchorPersistenceANDROID handle) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrPersistAnchorANDROID(
    XrDeviceAnchorPersistenceANDROID handle,
    const XrPersistedAnchorSpaceInfoANDROID* persistedInfo,
    XrUuidEXT* anchorIdOutput) {
  if (anchorIdOutput) {
    *anchorIdOutput = kUuid;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrGetAnchorPersistStateANDROID(
    XrDeviceAnchorPersistenceANDROID handle, const XrUuidEXT* anchorId,
    XrAnchorPersistStateANDROID* persistState) {
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
  if (anchorOutput) {
    *anchorOutput = kSpace;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrEnumeratePersistedAnchorsANDROID(
    XrDeviceAnchorPersistenceANDROID handle, uint32_t anchorIdsCapacityInput,
    uint32_t* anchorIdsCountOutput, XrUuidEXT* anchorIds) {
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
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrRaycastANDROID(
    XrSession session, const XrRaycastInfoANDROID* rayInfo,
    XrRaycastHitResultsANDROID* outHitResults) {
  *outHitResults = kRaycastHitResults;

  return XR_SUCCESS;
}

}  // extern "C"

namespace {
template <typename TypedFunction>
PFN_xrVoidFunction ToXrVoidFunction(TypedFunction* function) {
  return reinterpret_cast<PFN_xrVoidFunction>(function);
}

const auto kXrFunctions =
    new absl::flat_hash_map<absl::string_view, PFN_xrVoidFunction>({
        {"xrInitializeLoaderKHR",
         ToXrVoidFunction(Internal_xrInitializeLoaderKHR)},
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
        {"xrShareAnchorANDROID",
         ToXrVoidFunction(Internal_xrShareAnchorANDROID)},
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
