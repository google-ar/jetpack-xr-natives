#ifndef XR_ANDROID_GEOSPATIAL_H_
#define XR_ANDROID_GEOSPATIAL_H_ 1

/*
** Copyright 2017-2026 The Khronos Group Inc.
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

/*
** This header is generated from the Khronos OpenXR XML API Registry.
**
*/


#ifdef __cplusplus
extern "C" {
#endif


#ifndef XR_ANDROID_geospatial

// XR_ANDROID_geospatial is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_geospatial 1
#define XR_TYPE_SYSTEM_GEOSPATIAL_PROPERTIES_ANDROID ((XrStructureType) 1000789000U)
#define XR_TYPE_GEOSPATIAL_TRACKER_CREATE_INFO_ANDROID ((XrStructureType) 1000789001U)
#define XR_TYPE_EVENT_DATA_GEOSPATIAL_TRACKER_STATE_CHANGED_ANDROID ((XrStructureType) 1000789002U)
#define XR_TYPE_GEOSPATIAL_POSE_FROM_POSE_LOCATE_INFO_ANDROID ((XrStructureType) 1000789003U)
#define XR_TYPE_GEOSPATIAL_POSE_RESULT_ANDROID ((XrStructureType) 1000789004U)
#define XR_TYPE_GEOSPATIAL_POSE_LOCATE_INFO_ANDROID ((XrStructureType) 1000789005U)
#define XR_TYPE_VPS_AVAILABILITY_CHECK_COMPLETION_ANDROID ((XrStructureType) 1000789006U)
#define XR_OBJECT_TYPE_GEOSPATIAL_TRACKER_ANDROID ((XrObjectType) 1000789000U)
#define XR_ERROR_GEOSPATIAL_TRACKER_NOT_RUNNING_ANDROID ((XrResult) -1000789000U)
#define XR_ERROR_GEOSPATIAL_COORDINATES_INVALID_ANDROID ((XrResult) -1000789001U)
#define XR_ERROR_GEOSPATIAL_CLOUD_AUTH_FAILED_ANDROID ((XrResult) -1000789002U)

XR_DEFINE_HANDLE(XrGeospatialTrackerANDROID)
#define XR_ANDROID_geospatial_SPEC_VERSION 1
#define XR_ANDROID_GEOSPATIAL_EXTENSION_NAME "XR_ANDROID_geospatial"

typedef enum XrGeospatialTrackerStateANDROID {
    // The Geospatial Tracker is not running.
    XR_GEOSPATIAL_TRACKER_STATE_STOPPED_ANDROID = 0,
    // The Geospatial tracker is running and usable.
    XR_GEOSPATIAL_TRACKER_STATE_RUNNING_ANDROID = 1,
    // The Geospatial Tracker failed to initialize, and will never be usable.
    XR_GEOSPATIAL_TRACKER_STATE_INITIALIZATION_FAILED_ANDROID = 2,
    XR_GEOSPATIAL_TRACKER_STATE_MAX_ENUM_ANDROID = 0x7FFFFFFF
} XrGeospatialTrackerStateANDROID;

typedef enum XrVPSAvailabilityANDROID {
    // VPS is not available near the given location.
    XR_VPS_AVAILABILITY_UNAVAILABLE_ANDROID = 1,
    // VPS is available near the given location.
    XR_VPS_AVAILABILITY_AVAILABLE_ANDROID = 2,
    XR_VPSAVAILABILITY_MAX_ENUM_ANDROID = 0x7FFFFFFF
} XrVPSAvailabilityANDROID;
typedef XrFlags64 XrGeospatialPoseFlagsANDROID;

// Flag bits for XrGeospatialPoseFlagsANDROID
// Indicates that the orientation member contains valid data
static const XrGeospatialPoseFlagsANDROID XR_GEOSPATIAL_POSE_ORIENTATION_VALID_BIT_ANDROID = 0x00000001;
// Indicates that the position member contains valid data
static const XrGeospatialPoseFlagsANDROID XR_GEOSPATIAL_POSE_POSITION_VALID_BIT_ANDROID = 0x00000002;

typedef struct XrGeospatialPoseANDROID {
    XrQuaternionf    eastUpSouthOrientation;
    double           latitude;
    double           longitude;
    double           altitude;
} XrGeospatialPoseANDROID;

typedef struct XrSystemGeospatialPropertiesANDROID {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    XrBool32              supportsGeospatial;
} XrSystemGeospatialPropertiesANDROID;

typedef struct XrGeospatialTrackerCreateInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrGeospatialTrackerCreateInfoANDROID;

typedef struct XrEventDataGeospatialTrackerStateChangedANDROID {
    XrStructureType                    type;
    const void* XR_MAY_ALIAS           next;
    XrGeospatialTrackerANDROID         geospatialTracker;
    XrGeospatialTrackerStateANDROID    state;
    XrResult                           initializationResult;
    XrTime                             time;
} XrEventDataGeospatialTrackerStateChangedANDROID;

typedef struct XrGeospatialPoseFromPoseLocateInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrSpace                     space;
    XrTime                      time;
    XrPosef                     pose;
} XrGeospatialPoseFromPoseLocateInfoANDROID;

typedef struct XrGeospatialPoseResultANDROID {
    XrStructureType                 type;
    void* XR_MAY_ALIAS              next;
    XrGeospatialPoseFlagsANDROID    poseFlags;
    XrGeospatialPoseANDROID         geospatialPose;
    double                          horizontalAccuracy;
    double                          verticalAccuracy;
    double                          orientationYawAccuracy;
} XrGeospatialPoseResultANDROID;

typedef struct XrGeospatialPoseLocateInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrSpace                     space;
    XrTime                      time;
    XrGeospatialPoseANDROID     geospatialPose;
} XrGeospatialPoseLocateInfoANDROID;

typedef struct XrVPSAvailabilityCheckCompletionANDROID {
    XrStructureType             type;
    void* XR_MAY_ALIAS          next;
    XrResult                    futureResult;
    XrVPSAvailabilityANDROID    availability;
} XrVPSAvailabilityCheckCompletionANDROID;

typedef XrResult (XRAPI_PTR *PFN_xrCreateGeospatialTrackerANDROID)(XrSession session, const XrGeospatialTrackerCreateInfoANDROID* createInfo, XrGeospatialTrackerANDROID* geospatialTrackerOutput);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyGeospatialTrackerANDROID)(XrGeospatialTrackerANDROID geospatialTracker);
typedef XrResult (XRAPI_PTR *PFN_xrLocateGeospatialPoseFromPoseANDROID)(XrGeospatialTrackerANDROID geospatialTracker, const XrGeospatialPoseFromPoseLocateInfoANDROID* locateInfo, XrGeospatialPoseResultANDROID* geospatialPoseResult);
typedef XrResult (XRAPI_PTR *PFN_xrLocateGeospatialPoseANDROID)(XrGeospatialTrackerANDROID geospatialTracker, const XrGeospatialPoseLocateInfoANDROID* locateInfo, XrSpaceLocation* location);
typedef XrResult (XRAPI_PTR *PFN_xrCheckVpsAvailabilityAsyncANDROID)(XrSession session, double latitude, double longitude, XrFutureEXT* future);
typedef XrResult (XRAPI_PTR *PFN_xrCheckVpsAvailabilityCompleteANDROID)(XrSession session, XrFutureEXT future, XrVPSAvailabilityCheckCompletionANDROID* completion);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateGeospatialTrackerANDROID(
    XrSession                                   session,
    const XrGeospatialTrackerCreateInfoANDROID* createInfo,
    XrGeospatialTrackerANDROID*                 geospatialTrackerOutput);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyGeospatialTrackerANDROID(
    XrGeospatialTrackerANDROID                  geospatialTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrLocateGeospatialPoseFromPoseANDROID(
    XrGeospatialTrackerANDROID                  geospatialTracker,
    const XrGeospatialPoseFromPoseLocateInfoANDROID* locateInfo,
    XrGeospatialPoseResultANDROID*              geospatialPoseResult);

XRAPI_ATTR XrResult XRAPI_CALL xrLocateGeospatialPoseANDROID(
    XrGeospatialTrackerANDROID                  geospatialTracker,
    const XrGeospatialPoseLocateInfoANDROID*    locateInfo,
    XrSpaceLocation*                            location);

XRAPI_ATTR XrResult XRAPI_CALL xrCheckVpsAvailabilityAsyncANDROID(
    XrSession                                   session,
    double                                      latitude,
    double                                      longitude,
    XrFutureEXT*                                future);

XRAPI_ATTR XrResult XRAPI_CALL xrCheckVpsAvailabilityCompleteANDROID(
    XrSession                                   session,
    XrFutureEXT                                 future,
    XrVPSAvailabilityCheckCompletionANDROID*    completion);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_ENUM_XrGeospatialTrackerStateANDROID(_) \
    _(XR_GEOSPATIAL_TRACKER_STATE_STOPPED_ANDROID, 0) \
    _(XR_GEOSPATIAL_TRACKER_STATE_RUNNING_ANDROID, 1) \
    _(XR_GEOSPATIAL_TRACKER_STATE_INITIALIZATION_FAILED_ANDROID, 2) \
    _(XR_GEOSPATIAL_TRACKER_STATE_MAX_ENUM_ANDROID, 0x7FFFFFFF)

#define XR_LIST_ENUM_XrVPSAvailabilityANDROID(_) \
    _(XR_VPS_AVAILABILITY_UNAVAILABLE_ANDROID, 1) \
    _(XR_VPS_AVAILABILITY_AVAILABLE_ANDROID, 2) \
    _(XR_VPSAVAILABILITY_MAX_ENUM_ANDROID, 0x7FFFFFFF)

#define XR_LIST_BITS_XrGeospatialPoseFlagsANDROID(_) \
    _(XR_GEOSPATIAL_POSE_ORIENTATION_VALID_BIT_ANDROID, 0x00000001) \
    _(XR_GEOSPATIAL_POSE_POSITION_VALID_BIT_ANDROID, 0x00000002)

#define XR_LIST_STRUCT_XrGeospatialPoseANDROID(_) \
    _(eastUpSouthOrientation) \
    _(latitude) \
    _(longitude) \
    _(altitude)

#define XR_LIST_STRUCT_XrSystemGeospatialPropertiesANDROID(_) \
    _(type) \
    _(next) \
    _(supportsGeospatial)

#define XR_LIST_STRUCT_XrGeospatialTrackerCreateInfoANDROID(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrEventDataGeospatialTrackerStateChangedANDROID(_) \
    _(type) \
    _(next) \
    _(geospatialTracker) \
    _(state) \
    _(initializationResult) \
    _(time)

#define XR_LIST_STRUCT_XrGeospatialPoseFromPoseLocateInfoANDROID(_) \
    _(type) \
    _(next) \
    _(space) \
    _(time) \
    _(pose)

#define XR_LIST_STRUCT_XrGeospatialPoseResultANDROID(_) \
    _(type) \
    _(next) \
    _(poseFlags) \
    _(geospatialPose) \
    _(horizontalAccuracy) \
    _(verticalAccuracy) \
    _(orientationYawAccuracy)

#define XR_LIST_STRUCT_XrGeospatialPoseLocateInfoANDROID(_) \
    _(type) \
    _(next) \
    _(space) \
    _(time) \
    _(geospatialPose)

#define XR_LIST_STRUCT_XrVPSAvailabilityCheckCompletionANDROID(_) \
    _(type) \
    _(next) \
    _(futureResult) \
    _(availability)

#define XR_LIST_FUNCTIONS_XR_ANDROID_geospatial(_) \
    _(CreateGeospatialTrackerANDROID, ANDROID_geospatial) \
    _(DestroyGeospatialTrackerANDROID, ANDROID_geospatial) \
    _(LocateGeospatialPoseFromPoseANDROID, ANDROID_geospatial) \
    _(LocateGeospatialPoseANDROID, ANDROID_geospatial) \
    _(CheckVpsAvailabilityAsyncANDROID, ANDROID_geospatial) \
    _(CheckVpsAvailabilityCompleteANDROID, ANDROID_geospatial)

#endif /* XR_ANDROID_geospatial */

#ifdef __cplusplus
}
#endif

#endif
