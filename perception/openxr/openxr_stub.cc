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
#include <openxr/public/all_extensions.h>

#include <cstdint>
#include <cstring>
#include <ctime>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"

namespace {
const XrPosef kPose = XrPosef{
    .orientation = XrQuaternionf{0.0f, 1.0f, 0.0f, 1.0f},
    .position = XrVector3f{0.0f, 0.0f, 2.0f},
};
const XrPosef kLeftViewPose = XrPosef{
    .orientation = XrQuaternionf{0.0f, 1.0f, 0.0f, 1.0f},
    .position = XrVector3f{2.0f, 0.0f, 0.0f},
};
const XrPosef kRightViewPose = XrPosef{
    .orientation = XrQuaternionf{0.0f, 1.0f, 0.0f, 1.0f},
    .position = XrVector3f{0.0f, 2.0f, 0.0f},
};
const XrFovf kLeftViewFov = XrFovf{1.0f, 2.0f, 3.0f, 4.0f};
const XrFovf kRightViewFov = XrFovf{2.0f, 1.0f, 3.0f, 4.0f};

const XrExtent2Df kExtent2D = XrExtent2Df{1.0f, 2.0f};
// Depth Map Test Resolution is 80x80
const int kDepthBufferSize = (80 * 80) * 2;

const std::vector<float> kTestRawDepthData(kDepthBufferSize, 8.0f);
const std::vector<uint8_t> kTestRawDepthConfidenceData(kDepthBufferSize, 100);
const std::vector<float> kTestSmoothDepthData(kDepthBufferSize, 10.0f);
const std::vector<uint8_t> kTestSmoothDepthConfidenceData(kDepthBufferSize,
                                                          200);
const XrExtent3Df kExtent3D = XrExtent3Df{1.0f, 2.0f, 3.0f};

const XrObjectLabelANDROID kObjectLabel = XR_OBJECT_LABEL_KEYBOARD_ANDROID;

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
const XrSpatialEntityIdEXT kEntityId = XrSpatialEntityIdEXT(5555);
const XrSpatialEntityEXT kSpatialEntity = XrSpatialEntityEXT(6666);
const XrSpatialContextEXT kSpatialContext = XrSpatialContextEXT(7777);
const XrDepthSwapchainANDROID kDepthSwapchain = XrDepthSwapchainANDROID(5555);
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
constexpr XrViewStateFlags kValidViewStateFlags =
    XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT;
const XrSpaceLocationFlags kValidLocationFlags =
    XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
    XR_SPACE_LOCATION_POSITION_VALID_BIT;
const XrHandJointLocationEXT kHandJoint = {
    .locationFlags = kValidLocationFlags,
    .pose = {.orientation = {.x = 1, .y = 2, .z = 3, .w = 4},
             .position = {.x = 5, .y = 6, .z = 7}},
    .radius = 1,
};
const std::vector<XrEnvironmentBlendMode> kBlendModes = {
    XR_ENVIRONMENT_BLEND_MODE_OPAQUE, XR_ENVIRONMENT_BLEND_MODE_ADDITIVE};

const XrTrackableTrackerANDROID kTrackableTracker =
    XrTrackableTrackerANDROID(1);
const XrHandTrackerEXT kHandTracker = XrHandTrackerEXT(1);
const XrFaceTrackerANDROID kFaceTracker = XrFaceTrackerANDROID(1);
const XrGeospatialTrackerANDROIDX2 kGeospatialTracker =
    XrGeospatialTrackerANDROIDX2(1);
const XrEyeTrackerANDROID kEyeTracker = XrEyeTrackerANDROID(1);
const XrDeviceAnchorPersistenceANDROID kAnchorPersistence =
    XrDeviceAnchorPersistenceANDROID(1);

const int kAnchorResourceLimit = 5;
const XrUuid kZeroUuid = {0};

int convert_to_khr_time_call_counter = 0;
int create_anchor_call_counter = 0;
uint64_t next_future_id = 1;

bool anchor_persistence_handle_created = false;
bool geospatial_tracker_handle_created = false;
bool geospatial_tracker_running = false;

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
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
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

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrGetTrackableObjectANDROID(XrTrackableTrackerANDROID trackableTracker,
                                     const XrTrackableGetInfoANDROID* getInfo,
                                     XrTrackableObjectANDROID* objectOutput) {
  if (!trackableTracker) {
    return XR_ERROR_HANDLE_INVALID;
  }
  objectOutput->trackingState =
      XrTrackingStateANDROID(XR_TRACKING_STATE_TRACKING_ANDROID);
  objectOutput->centerPose = kPose;
  objectOutput->extents = kExtent3D;
  objectOutput->objectLabel = kObjectLabel;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateAnchorSpaceANDROID(
    XrSession session, const XrAnchorSpaceCreateInfoANDROID* createInfo,
    XrSpace* anchorOutput) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
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
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateDeviceAnchorPersistenceANDROID(
    XrSession session,
    const XrDeviceAnchorPersistenceCreateInfoANDROID* createInfo,
    XrDeviceAnchorPersistenceANDROID* outHandle) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
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
    return XR_ERROR_ANCHOR_ID_NOT_FOUND_ANDROID;
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
      return XR_ERROR_ANCHOR_ID_NOT_FOUND_ANDROID;
    }
  if (!anchor_persistence_handle_created) {
    return XR_ERROR_HANDLE_INVALID;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrRaycastANDROID(
    XrSession session, const XrRaycastInfoANDROID* rayInfo,
    XrRaycastHitResultsANDROID* outHitResults) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  *outHitResults = kRaycastHitResults;

  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateHandTrackerEXT(
    XrSession session, const XrHandTrackerCreateInfoEXT* createInfo,
    XrHandTrackerEXT* handTracker) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
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

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateFaceTrackerANDROID(
    XrSession session, const XrFaceTrackerCreateInfoANDROID* createInfo,
    XrFaceTrackerANDROID* faceTracker) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  *faceTracker = kFaceTracker;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrDestroyFaceTrackerANDROID(
    XrFaceTrackerANDROID faceTracker) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrGetFaceStateANDROID(
    XrFaceTrackerANDROID faceTracker, const XrFaceStateGetInfoANDROID *getInfo,
    XrFaceStateANDROID* faceState) {
  if (!faceTracker) {
    return XR_ERROR_HANDLE_INVALID;
  }

  faceState->type = XR_TYPE_FACE_STATE_ANDROID;
  faceState->next = nullptr;
  faceState->faceTrackingState = XR_FACE_TRACKING_STATE_TRACKING_ANDROID;
  faceState->isValid = true;
  faceState->parametersCapacityInput = XR_FACE_PARAMETER_COUNT_ANDROID;
  faceState->parametersCountOutput = XR_FACE_PARAMETER_COUNT_ANDROID;
  faceState->regionConfidencesCapacityInput =
      XR_FACE_REGION_CONFIDENCE_COUNT_ANDROID;
  faceState->regionConfidencesCountOutput =
      XR_FACE_REGION_CONFIDENCE_COUNT_ANDROID;
  for (int i = 0; i < XR_FACE_PARAMETER_COUNT_ANDROID; ++i) {
    faceState->parameters[i] = (float)i / XR_FACE_PARAMETER_COUNT_ANDROID;
  }
  for (int i = 0; i < XR_FACE_REGION_CONFIDENCE_COUNT_ANDROID; ++i) {
    faceState->regionConfidences[i] = (float)i /
      XR_FACE_REGION_CONFIDENCE_COUNT_ANDROID;
  }

  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrGetFaceCalibrationStateANDROID(
    XrFaceTrackerANDROID faceTracker, XrBool32* outIsCalibrated) {
  if (!faceTracker) {
    return XR_ERROR_HANDLE_INVALID;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateGeospatialTrackerANDROIDX2(
    XrSession session, const XrGeospatialTrackerCreateInfoANDROIDX2* createInfo,
    XrGeospatialTrackerANDROIDX2* geospatialTracker) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  *geospatialTracker = kGeospatialTracker;
  geospatial_tracker_handle_created = true;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrDestroyGeospatialTrackerANDROIDX2(
    XrGeospatialTrackerANDROIDX2 geospatialTracker) {
  if (geospatialTracker == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  geospatial_tracker_handle_created = false;
  geospatial_tracker_running = false;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCheckVpsAvailabilityAsyncANDROIDX2(
    XrSession session, double latitude, double longitude, XrFutureEXT* future) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCheckVpsAvailabilityCompleteANDROIDX2(
    XrSession session, XrFutureEXT future,
    XrVPSAvailabilityCheckCompletionANDROIDX2* completion) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrLocateGeospatialPoseANDROIDX2(
    XrGeospatialTrackerANDROIDX2 geospatialTracker,
    const XrGeospatialPoseLocateInfoANDROIDX2* locateInfo,
    XrSpaceLocation* location) {
  if (geospatialTracker == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  location->type = XR_TYPE_SPACE_LOCATION;
  location->next = nullptr;
  location->locationFlags = kLocationFlags;
  location->pose = kPose;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrLocateGeospatialPoseFromPoseANDROIDX2(
    XrGeospatialTrackerANDROIDX2 geospatialTracker,
    const XrGeospatialPoseFromPoseLocateInfoANDROIDX2* locateInfo,
    XrGeospatialPoseResultANDROIDX2* geospatialPose) {
  if (geospatialTracker == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  geospatialPose->type = XR_TYPE_GEOSPATIAL_POSE_RESULT_ANDROIDX2;
  geospatialPose->poseFlags =
      XR_GEOSPATIAL_POSE_POSITION_VALID_BIT_ANDROIDX2 |
      XR_GEOSPATIAL_POSE_ORIENTATION_VALID_BIT_ANDROIDX2;
  geospatialPose->geospatialPose.latitude = 37.422;
  geospatialPose->geospatialPose.longitude = -122.084;
  geospatialPose->geospatialPose.altitude = 10.0;
  geospatialPose->geospatialPose.eastUpSouthOrientation = {0, 0, 0, 1};
  geospatialPose->horizontalAccuracy = 1.0;
  geospatialPose->verticalAccuracy = 2.0;
  geospatialPose->orientationYawAccuracy = 3.0;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateGeospatialAnchorANDROIDX2(
    XrSpatialContextEXT spatialContext,
    const XrGeospatialAnchorCreateInfoANDROIDX2* createInfo,
    XrSpatialEntityIdEXT* anchorEntityId) {
  ++create_anchor_call_counter;
  if (create_anchor_call_counter > kAnchorResourceLimit) {
    return XR_ERROR_LIMIT_REACHED;
  }
  *anchorEntityId = kEntityId;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateEyeTrackerANDROID(
    XrSession session, const XrEyeTrackerCreateInfoANDROID* createInfo,
    XrEyeTrackerANDROID* eyeTracker) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  *eyeTracker = kEyeTracker;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrDestroyEyeTrackerANDROID(XrEyeTrackerANDROID eyeTracker) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrGetCoarseTrackingEyesInfoANDROID(
    XrEyeTrackerANDROID eyeTracker, const XrEyesGetInfoANDROID* getInfo,
    XrEyesANDROID* outEyes) {
  if (!eyeTracker) {
    return XR_ERROR_HANDLE_INVALID;
  }
  outEyes->type = XR_TYPE_EYES_ANDROID;
  outEyes->next = nullptr;
  outEyes->eyes[XR_EYE_INDEX_LEFT_ANDROID] = {
      .eyeState = XR_EYE_STATE_GAZING_ANDROID,
      .eyePose =
          {
              .orientation = {0.1f, 0.1f, 0.1f, 0.1f},
              .position = {0.2f, 0.2f, 0.2f},
          },
  };
  outEyes->eyes[XR_EYE_INDEX_RIGHT_ANDROID] = {
      .eyeState = XR_EYE_STATE_GAZING_ANDROID,
      .eyePose =
          {
              .orientation = {0.3f, 0.3f, 0.3f, 0.3f},
              .position = {0.4f, 0.4f, 0.4f},
          },
  };
  outEyes->mode = XR_EYE_TRACKING_MODE_BOTH_ANDROID;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrGetFineTrackingEyesInfoANDROID(
    XrEyeTrackerANDROID eyeTracker, const XrEyesGetInfoANDROID* getInfo,
    XrEyesANDROID* outEyes) {
  if (!eyeTracker) {
    return XR_ERROR_HANDLE_INVALID;
  }
  outEyes->type = XR_TYPE_EYES_ANDROID;
  outEyes->next = nullptr;
  outEyes->eyes[XR_EYE_INDEX_LEFT_ANDROID] = {
      .eyeState = XR_EYE_STATE_GAZING_ANDROID,
      .eyePose =
          {
              .orientation = {0.11111f, 0.11111f, 0.11111f, 0.11111f},
              .position = {0.22222f, 0.22222f, 0.22222f},
          },
  };
  outEyes->eyes[XR_EYE_INDEX_RIGHT_ANDROID] = {
      .eyeState = XR_EYE_STATE_GAZING_ANDROID,
      .eyePose =
          {
              .orientation = {0.33333f, 0.33333f, 0.33333f, 0.33333f},
              .position = {0.44444f, 0.44444f, 0.44444f},
          },
  };
  outEyes->mode = XR_EYE_TRACKING_MODE_BOTH_ANDROID;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrEnumerateDepthResolutionsANDROID(
    XrSession session, uint32_t resolutionCapacityInput,
    uint32_t* resolutionCountOutput,
    XrDepthCameraResolutionANDROID* resolutions) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  if (resolutionCapacityInput == 0 && resolutions == nullptr) {
    *resolutionCountOutput = 1;
    return XR_SUCCESS;
  }

  *resolutionCountOutput = 1;
  resolutions[0] = XR_DEPTH_CAMERA_RESOLUTION_80x80_ANDROID;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateDepthSwapchainANDROID(
    XrSession session, const XrDepthSwapchainCreateInfoANDROID* createInfo,
    XrDepthSwapchainANDROID* swapchain) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  *swapchain = kDepthSwapchain;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrDestroyDepthSwapchainANDROID(XrDepthSwapchainANDROID swapchain) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrEnumerateDepthSwapchainImagesANDROID(
    XrDepthSwapchainANDROID swapchain, uint32_t imageCapacityInput,
    uint32_t* imageCountOutput, XrDepthSwapchainImageANDROID* images) {
  if (imageCapacityInput == 0 && images == nullptr) {
    *imageCountOutput = 1;
    return XR_SUCCESS;
  }
  *imageCountOutput = 1;

  images[0].type = XR_TYPE_DEPTH_SWAPCHAIN_IMAGE_ANDROID;
  images[0].rawDepthImage = kTestRawDepthData.data();
  images[0].rawDepthConfidenceImage = kTestRawDepthConfidenceData.data();
  images[0].smoothDepthImage = kTestSmoothDepthData.data();
  images[0].smoothDepthConfidenceImage = kTestSmoothDepthConfidenceData.data();

  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrAcquireDepthSwapchainImagesANDROID(
    XrDepthSwapchainANDROID swapchain,
    const XrDepthAcquireInfoANDROID* acquireInfo,
    XrDepthAcquireResultANDROID* acquireResult) {
  acquireResult->type = XR_TYPE_DEPTH_ACQUIRE_RESULT_ANDROID;
  acquireResult->acquiredIndex = 0;
  acquireResult->exposureTimestamp = kTime;
  acquireResult->views[0].type = XR_TYPE_DEPTH_VIEW_ANDROID;
  acquireResult->views[0].fov = kLeftViewFov;
  acquireResult->views[0].pose = kLeftViewPose;
  acquireResult->views[1].type = XR_TYPE_DEPTH_VIEW_ANDROID;
  acquireResult->views[1].fov = kRightViewFov;
  acquireResult->views[1].pose = kRightViewPose;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCancelFutureEXT(
    XrInstance instance, const XrFutureCancelInfoEXT* cancelInfo) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrPollFutureEXT(
    XrInstance instance, const XrFuturePollInfoEXT* pollInfo,
    XrFuturePollResultEXT* pollResult) {
  pollResult->state = XR_FUTURE_STATE_READY_EXT;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrEnumerateSpatialCapabilitiesEXT(
    XrInstance instance, XrSystemId systemId, uint32_t capabilityCapacityInput,
    uint32_t* capabilityCountOutput, XrSpatialCapabilityEXT* capabilities) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrEnumerateSpatialCapabilityComponentTypesEXT(
    XrInstance instance, XrSystemId systemId, XrSpatialCapabilityEXT capability,
    XrSpatialCapabilityComponentTypesEXT* capabilityComponents) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrEnumerateSpatialCapabilityFeaturesEXT(
    XrInstance instance, XrSystemId systemId, XrSpatialCapabilityEXT capability,
    uint32_t capabilityFeatureCapacityInput,
    uint32_t* capabilityFeatureCountOutput,
    XrSpatialCapabilityFeatureEXT* capabilityFeatures) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateSpatialContextAsyncEXT(
    XrSession session, const XrSpatialContextCreateInfoEXT* createInfo,
    XrFutureEXT* future) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  *future = (XrFutureEXT)next_future_id++;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateSpatialContextCompleteEXT(
    XrSession session, XrFutureEXT future,
    XrCreateSpatialContextCompletionEXT* completion) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  if (completion == nullptr) {
    return XR_ERROR_VALIDATION_FAILURE;
  }
  completion->type = XR_TYPE_CREATE_SPATIAL_CONTEXT_COMPLETION_EXT;
  completion->futureResult = XR_SUCCESS;
  completion->spatialContext = kSpatialContext;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrDestroySpatialContextEXT(XrSpatialContextEXT spatialContext) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrCreateSpatialDiscoverySnapshotAsyncEXT(
    XrSpatialContextEXT spatialContext,
    const XrSpatialDiscoverySnapshotCreateInfoEXT* createInfo,
    XrFutureEXT* future) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrCreateSpatialDiscoverySnapshotCompleteEXT(
    XrSpatialContextEXT spatialContext,
    const XrCreateSpatialDiscoverySnapshotCompletionInfoEXT*
        createSnapshotCompletionInfo,
    XrCreateSpatialDiscoverySnapshotCompletionEXT* completion) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrQuerySpatialComponentDataEXT(
    XrSpatialSnapshotEXT snapshot,
    const XrSpatialComponentDataQueryConditionEXT* queryCondition,
    XrSpatialComponentDataQueryResultEXT* queryResult) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrDestroySpatialSnapshotEXT(XrSpatialSnapshotEXT snapshot) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateSpatialEntityFromIdEXT(
    XrSpatialContextEXT spatialContext,
    const XrSpatialEntityFromIdCreateInfoEXT* createInfo,
    XrSpatialEntityEXT* spatialEntity) {
  *spatialEntity = kSpatialEntity;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrDestroySpatialEntityEXT(XrSpatialEntityEXT spatialEntity) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateSpatialUpdateSnapshotEXT(
    XrSpatialContextEXT spatialContext,
    const XrSpatialUpdateSnapshotCreateInfoEXT* createInfo,
    XrSpatialSnapshotEXT* snapshot) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrCreateSpatialAnchorEXT(
    XrSpatialContextEXT spatialContext,
    const XrSpatialAnchorCreateInfoEXT* createInfo,
    XrSpatialEntityIdEXT* anchorEntityId, XrSpatialEntityEXT* anchorEntity) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
Internal_xrCreateSpatialAnchorSpaceFromIdANDROID(
    XrSession session, XrSpatialContextEXT spatialContext,
    const XrSpatialAnchorSpaceFromIdCreateInfoANDROID* createInfo,
    XrSpace* anchorSpace) {
  if (session == XR_NULL_HANDLE || spatialContext == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  *anchorSpace = kSpace;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrSetGoogleCloudAuthAsyncANDROIDX2(
    XrSession session, const XrGoogleCloudAuthInfoBaseHeaderANDROIDX2* authInfo,
    XrFutureEXT* future) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL Internal_xrSetGoogleCloudAuthCompleteANDROIDX2(
    XrSession session, XrFutureEXT future, XrFutureCompletionEXT* completion) {
  return XR_SUCCESS;
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
    {"xrGetTrackableObjectANDROID",
     ToXrVoidFunction(Internal_xrGetTrackableObjectANDROID)},
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
    {"xrCreateFaceTrackerANDROID",
     ToXrVoidFunction(Internal_xrCreateFaceTrackerANDROID)},
    {"xrDestroyFaceTrackerANDROID",
     ToXrVoidFunction(Internal_xrDestroyFaceTrackerANDROID)},
    {"xrGetFaceStateANDROID", ToXrVoidFunction(Internal_xrGetFaceStateANDROID)},
    {"xrGetFaceCalibrationStateANDROID",
     ToXrVoidFunction(Internal_xrGetFaceCalibrationStateANDROID)},
    {"xrCreateGeospatialTrackerANDROIDX2",
     ToXrVoidFunction(Internal_xrCreateGeospatialTrackerANDROIDX2)},
    {"xrDestroyGeospatialTrackerANDROIDX2",
     ToXrVoidFunction(Internal_xrDestroyGeospatialTrackerANDROIDX2)},
    {"xrCheckVpsAvailabilityAsyncANDROIDX2",
     ToXrVoidFunction(Internal_xrCheckVpsAvailabilityAsyncANDROIDX2)},
    {"xrCheckVpsAvailabilityCompleteANDROIDX2",
     ToXrVoidFunction(Internal_xrCheckVpsAvailabilityCompleteANDROIDX2)},
    {"xrLocateGeospatialPoseFromPoseANDROIDX2",
     ToXrVoidFunction(Internal_xrLocateGeospatialPoseFromPoseANDROIDX2)},
    {"xrLocateGeospatialPoseANDROIDX2",
     ToXrVoidFunction(Internal_xrLocateGeospatialPoseANDROIDX2)},
    {"xrCreateGeospatialAnchorANDROIDX2",
     ToXrVoidFunction(Internal_xrCreateGeospatialAnchorANDROIDX2)},
    {"xrCreateDepthSwapchainANDROID",
     ToXrVoidFunction(Internal_xrCreateDepthSwapchainANDROID)},
    {"xrDestroyDepthSwapchainANDROID",
     ToXrVoidFunction(Internal_xrDestroyDepthSwapchainANDROID)},
    {"xrEnumerateDepthSwapchainImagesANDROID",
     ToXrVoidFunction(Internal_xrEnumerateDepthSwapchainImagesANDROID)},
    {"xrAcquireDepthSwapchainImagesANDROID",
     ToXrVoidFunction(Internal_xrAcquireDepthSwapchainImagesANDROID)},
    {"xrCreateEyeTrackerANDROID",
     ToXrVoidFunction(Internal_xrCreateEyeTrackerANDROID)},
    {"xrDestroyEyeTrackerANDROID",
     ToXrVoidFunction(Internal_xrDestroyEyeTrackerANDROID)},
    {"xrGetCoarseTrackingEyesInfoANDROID",
     ToXrVoidFunction(Internal_xrGetCoarseTrackingEyesInfoANDROID)},
    {"xrGetFineTrackingEyesInfoANDROID",
     ToXrVoidFunction(Internal_xrGetFineTrackingEyesInfoANDROID)},
    {"xrCancelFutureEXT", ToXrVoidFunction(Internal_xrCancelFutureEXT)},
    {"xrPollFutureEXT", ToXrVoidFunction(Internal_xrPollFutureEXT)},
    {"xrEnumerateSpatialCapabilitiesEXT",
     ToXrVoidFunction(Internal_xrEnumerateSpatialCapabilitiesEXT)},
    {"xrEnumerateSpatialCapabilityComponentTypesEXT",
     ToXrVoidFunction(Internal_xrEnumerateSpatialCapabilityComponentTypesEXT)},
    {"xrEnumerateSpatialCapabilityFeaturesEXT",
     ToXrVoidFunction(Internal_xrEnumerateSpatialCapabilityFeaturesEXT)},
    {"xrCreateSpatialContextAsyncEXT",
     ToXrVoidFunction(Internal_xrCreateSpatialContextAsyncEXT)},
    {"xrCreateSpatialContextCompleteEXT",
     ToXrVoidFunction(Internal_xrCreateSpatialContextCompleteEXT)},
    {"xrDestroySpatialContextEXT",
     ToXrVoidFunction(Internal_xrDestroySpatialContextEXT)},
    {"xrCreateSpatialDiscoverySnapshotAsyncEXT",
     ToXrVoidFunction(Internal_xrCreateSpatialDiscoverySnapshotAsyncEXT)},
    {"xrCreateSpatialDiscoverySnapshotCompleteEXT",
     ToXrVoidFunction(Internal_xrCreateSpatialDiscoverySnapshotCompleteEXT)},
    {"xrQuerySpatialComponentDataEXT",
     ToXrVoidFunction(Internal_xrQuerySpatialComponentDataEXT)},
    {"xrDestroySpatialSnapshotEXT",
     ToXrVoidFunction(Internal_xrDestroySpatialSnapshotEXT)},
    {"xrCreateSpatialEntityFromIdEXT",
     ToXrVoidFunction(Internal_xrCreateSpatialEntityFromIdEXT)},
    {"xrDestroySpatialEntityEXT",
     ToXrVoidFunction(Internal_xrDestroySpatialEntityEXT)},
    {"xrCreateSpatialUpdateSnapshotEXT",
     ToXrVoidFunction(Internal_xrCreateSpatialUpdateSnapshotEXT)},
    {"xrCreateSpatialAnchorEXT",
     ToXrVoidFunction(Internal_xrCreateSpatialAnchorEXT)},
    {"xrCreateSpatialAnchorSpaceFromIdANDROID",
     ToXrVoidFunction(Internal_xrCreateSpatialAnchorSpaceFromIdANDROID)},
    {"xrSetGoogleCloudAuthAsyncANDROIDX2",
     ToXrVoidFunction(Internal_xrSetGoogleCloudAuthAsyncANDROIDX2)},
    {"xrSetGoogleCloudAuthCompleteANDROIDX2",
     ToXrVoidFunction(Internal_xrSetGoogleCloudAuthCompleteANDROIDX2)},
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

const std::vector<XrExtensionProperties> kExtensions = {
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_ANCHOR_SHARING_EXPORT_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_DEPTH_TEXTURE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_DEVICE_ANCHOR_PERSISTENCE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_EYE_TRACKING_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_FACE_TRACKING_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr, XR_ANDROID_RAYCAST_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_TRACKABLES_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_TRACKABLES_OBJECT_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr, XR_EXT_FUTURE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_EXT_HAND_TRACKING_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr, XR_MND_HEADLESS_EXTENSION_NAME},
    // Geospatial extensions
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROIDX2_GEOSPATIAL_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROIDX2_GEOSPATIAL_ANCHOR_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROIDX2_GEOSPATIAL_STREETSCAPE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_SPATIAL_ANCHOR_SPACE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROIDX2_GOOGLE_CLOUD_AUTH_EXTENSION_NAME},
};

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateInstanceExtensionProperties(
    const char* layerName, uint32_t propertyCapacityInput,
    uint32_t* propertyCountOutput, XrExtensionProperties* properties) {
  *propertyCountOutput = kExtensions.size();
  if (propertyCapacityInput == 0) {
    return XR_SUCCESS;
  }
  if (propertyCapacityInput < kExtensions.size()) {
    return XR_ERROR_SIZE_INSUFFICIENT;
  }

  for (int i = 0; i < kExtensions.size(); ++i) {
    properties[i] = kExtensions[i];
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
xrCreateInstance(const XrInstanceCreateInfo* createInfo, XrInstance* instance) {
  *instance = kInstance;
  create_anchor_call_counter = 0;
  // TODO: Temporarily enabling some features by default until
  // session configuration is fully implemented.
  anchor_persistence_handle_created = true;
  geospatial_tracker_handle_created = false;
  geospatial_tracker_running = false;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetSystem(XrInstance instance,
                                           const XrSystemGetInfo* getInfo,
                                           XrSystemId* systemId) {
  *systemId = kSystemId;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetSystemProperties(
    XrInstance instance, XrSystemId systemId, XrSystemProperties* properties) {
  if (properties == nullptr) {
    return XR_ERROR_HANDLE_INVALID;
  }
  if (properties->next != nullptr) {
    reinterpret_cast<XrSystemGeospatialPropertiesANDROIDX2*>(properties->next)
        ->supportsGeospatial = XR_TRUE;
  }
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
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrEndSession(XrSession session) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroySession(XrSession session) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
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
  if (session == XR_NULL_HANDLE || views == nullptr) {
    return XR_ERROR_HANDLE_INVALID;
  }
  views[0].pose = kLeftViewPose;
  views[1].pose = kRightViewPose;
  views[0].fov = kLeftViewFov;
  views[1].fov = kRightViewFov;
  *viewCountOutput = 2;
  viewState->viewStateFlags = kValidViewStateFlags;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateEnvironmentBlendModes(
    XrInstance instance, XrSystemId systemId,
    XrViewConfigurationType viewConfigurationType,
    uint32_t environmentBlendModeCapacityInput,
    uint32_t* environmentBlendModeCountOutput,
    XrEnvironmentBlendMode* environmentBlendModes) {
  if (environmentBlendModeCapacityInput == 0) {
    *environmentBlendModeCountOutput = kBlendModes.size();
    return XR_SUCCESS;
  }
  if (environmentBlendModeCapacityInput < kBlendModes.size()) {
    return XR_ERROR_SIZE_INSUFFICIENT;
  }
  *environmentBlendModeCountOutput = kBlendModes.size();
  for (int i = 0; i < kBlendModes.size(); ++i) {
    environmentBlendModes[i] = kBlendModes[i];
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrPollEvent(XrInstance instance,
                                           XrEventDataBuffer* eventData) {
  if (geospatial_tracker_handle_created && !geospatial_tracker_running) {
    geospatial_tracker_running = true;

    XrEventDataGeospatialTrackerStateChangedANDROIDX2* geospatial_event =
        reinterpret_cast<XrEventDataGeospatialTrackerStateChangedANDROIDX2*>(
            eventData);
    geospatial_event->type =
        XR_TYPE_EVENT_DATA_GEOSPATIAL_TRACKER_STATE_CHANGED_ANDROIDX2;
    geospatial_event->next = nullptr;
    geospatial_event->state = XR_GEOSPATIAL_TRACKER_STATE_RUNNING_ANDROIDX2;
    geospatial_event->initializationResult = XR_SUCCESS;
    return XR_SUCCESS;
  }

  return XR_EVENT_UNAVAILABLE;
}

XRAPI_ATTR XrResult XRAPI_CALL xrCreateReferenceSpace(
    XrSession session, const XrReferenceSpaceCreateInfo* createInfo,
    XrSpace* space) {
  if (session == XR_NULL_HANDLE) {
    return XR_ERROR_HANDLE_INVALID;
  }
  *space = kSpace;
  return XR_SUCCESS;
}

}  // extern "C"
