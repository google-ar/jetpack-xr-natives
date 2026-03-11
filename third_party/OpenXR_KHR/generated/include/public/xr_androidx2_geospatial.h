#ifndef XR_ANDROIDX2_GEOSPATIAL_H_
#define XR_ANDROIDX2_GEOSPATIAL_H_ 1

/*
** Copyright 2017-2025 The Khronos Group Inc.
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


#ifndef XR_ANDROIDX2_geospatial

// XR_ANDROIDX2_geospatial is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX2_geospatial 1
XR_DEFINE_HANDLE(XrGeospatialTrackerANDROIDX2)
#define XR_ANDROIDX2_geospatial_SPEC_VERSION 1
#define XR_ANDROIDX2_GEOSPATIAL_EXTENSION_NAME "XR_ANDROIDX2_geospatial"
#define XR_TYPE_SYSTEM_GEOSPATIAL_PROPERTIES_ANDROIDX2 ((XrStructureType) 1000789000U)
#define XR_TYPE_GEOSPATIAL_TRACKER_CREATE_INFO_ANDROIDX2 ((XrStructureType) 1000789001U)
#define XR_TYPE_EVENT_DATA_GEOSPATIAL_TRACKER_STATE_CHANGED_ANDROIDX2 ((XrStructureType) 1000789002U)
#define XR_TYPE_GEOSPATIAL_POSE_FROM_POSE_LOCATE_INFO_ANDROIDX2 ((XrStructureType) 1000789003U)
#define XR_TYPE_GEOSPATIAL_POSE_RESULT_ANDROIDX2 ((XrStructureType) 1000789004U)
#define XR_TYPE_GEOSPATIAL_POSE_LOCATE_INFO_ANDROIDX2 ((XrStructureType) 1000789005U)
#define XR_TYPE_VPS_AVAILABILITY_CHECK_COMPLETION_ANDROIDX2 ((XrStructureType) 1000789006U)
#define XR_OBJECT_TYPE_GEOSPATIAL_TRACKER_ANDROIDX2 ((XrObjectType) 1000789000U)
#define XR_ERROR_GEOSPATIAL_TRACKER_NOT_RUNNING_ANDROIDX2 ((XrResult) -1000789000U)
#define XR_ERROR_GEOSPATIAL_COORDINATES_INVALID_ANDROIDX2 ((XrResult) -1000789001U)
#define XR_ERROR_GEOSPATIAL_CLOUD_AUTH_FAILED_ANDROIDX2 ((XrResult) -1000789002U)

typedef enum XrGeospatialTrackerStateANDROIDX2 {
    // The Geospatial Tracker is not running.
    XR_GEOSPATIAL_TRACKER_STATE_STOPPED_ANDROIDX2 = 0,
    // The Geospatial tracker is running and usable.
    XR_GEOSPATIAL_TRACKER_STATE_RUNNING_ANDROIDX2 = 1,
    // The Geospatial Tracker failed to initialize, and will never be usable.
    XR_GEOSPATIAL_TRACKER_STATE_INITIALIZATION_FAILED_ANDROIDX2 = 2,
    XR_GEOSPATIAL_TRACKER_STATE_ANDROIDX2_MAX_ENUM = 0x7FFFFFFF
} XrGeospatialTrackerStateANDROIDX2;

typedef enum XrVPSAvailabilityANDROIDX2 {
    // VPS is not available near the given location.
    XR_VPS_AVAILABILITY_UNAVAILABLE_ANDROIDX2 = 1,
    // VPS is available near the given location.
    XR_VPS_AVAILABILITY_AVAILABLE_ANDROIDX2 = 2,
    XR_VPSAVAILABILITY_ANDROIDX2_MAX_ENUM = 0x7FFFFFFF
} XrVPSAvailabilityANDROIDX2;
typedef XrFlags64 XrGeospatialPoseFlagsANDROIDX2;

// Flag bits for XrGeospatialPoseFlagsANDROIDX2
// Indicates that the orientation member contains valid data
static const XrGeospatialPoseFlagsANDROIDX2 XR_GEOSPATIAL_POSE_ORIENTATION_VALID_BIT_ANDROIDX2 = 0x00000001;
// Indicates that the position member contains valid data
static const XrGeospatialPoseFlagsANDROIDX2 XR_GEOSPATIAL_POSE_POSITION_VALID_BIT_ANDROIDX2 = 0x00000002;

typedef struct XrGeospatialPoseANDROIDX2 {
    XrQuaternionf    eastUpSouthOrientation;
    double           latitude;
    double           longitude;
    double           altitude;
} XrGeospatialPoseANDROIDX2;

typedef struct XrSystemGeospatialPropertiesANDROIDX2 {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    XrBool32              supportsGeospatial;
} XrSystemGeospatialPropertiesANDROIDX2;

typedef struct XrGeospatialTrackerCreateInfoANDROIDX2 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrGeospatialTrackerCreateInfoANDROIDX2;

typedef struct XrEventDataGeospatialTrackerStateChangedANDROIDX2 {
    XrStructureType                      type;
    const void* XR_MAY_ALIAS             next;
    XrGeospatialTrackerANDROIDX2         geospatialTracker;
    XrGeospatialTrackerStateANDROIDX2    state;
    XrResult                             initializationResult;
    XrTime                               time;
} XrEventDataGeospatialTrackerStateChangedANDROIDX2;

typedef struct XrGeospatialPoseFromPoseLocateInfoANDROIDX2 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrSpace                     space;
    XrTime                      time;
    XrPosef                     pose;
} XrGeospatialPoseFromPoseLocateInfoANDROIDX2;

typedef struct XrGeospatialPoseResultANDROIDX2 {
    XrStructureType                   type;
    void* XR_MAY_ALIAS                next;
    XrGeospatialPoseFlagsANDROIDX2    poseFlags;
    XrGeospatialPoseANDROIDX2         geospatialPose;
    double                            horizontalAccuracy;
    double                            verticalAccuracy;
    double                            orientationYawAccuracy;
} XrGeospatialPoseResultANDROIDX2;

typedef struct XrGeospatialPoseLocateInfoANDROIDX2 {
    XrStructureType              type;
    const void* XR_MAY_ALIAS     next;
    XrSpace                      space;
    XrTime                       time;
    XrGeospatialPoseANDROIDX2    geospatialPose;
} XrGeospatialPoseLocateInfoANDROIDX2;

typedef struct XrVPSAvailabilityCheckCompletionANDROIDX2 {
    XrStructureType               type;
    void* XR_MAY_ALIAS            next;
    XrResult                      futureResult;
    XrVPSAvailabilityANDROIDX2    availability;
} XrVPSAvailabilityCheckCompletionANDROIDX2;

typedef XrResult (XRAPI_PTR *PFN_xrCreateGeospatialTrackerANDROIDX2)(XrSession session, const XrGeospatialTrackerCreateInfoANDROIDX2* createInfo, XrGeospatialTrackerANDROIDX2* geospatialTrackerOutput);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyGeospatialTrackerANDROIDX2)(XrGeospatialTrackerANDROIDX2 geospatialTracker);
typedef XrResult (XRAPI_PTR *PFN_xrLocateGeospatialPoseFromPoseANDROIDX2)(XrGeospatialTrackerANDROIDX2 geospatialTracker, const XrGeospatialPoseFromPoseLocateInfoANDROIDX2* locateInfo, XrGeospatialPoseResultANDROIDX2* geospatialPoseResult);
typedef XrResult (XRAPI_PTR *PFN_xrLocateGeospatialPoseANDROIDX2)(XrGeospatialTrackerANDROIDX2 geospatialTracker, const XrGeospatialPoseLocateInfoANDROIDX2* locateInfo, XrSpaceLocation* location);
typedef XrResult (XRAPI_PTR *PFN_xrCheckVpsAvailabilityAsyncANDROIDX2)(XrSession session, double latitude, double longitude, XrFutureEXT* future);
typedef XrResult (XRAPI_PTR *PFN_xrCheckVpsAvailabilityCompleteANDROIDX2)(XrSession session, XrFutureEXT future, XrVPSAvailabilityCheckCompletionANDROIDX2* completion);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateGeospatialTrackerANDROIDX2(
    XrSession                                   session,
    const XrGeospatialTrackerCreateInfoANDROIDX2* createInfo,
    XrGeospatialTrackerANDROIDX2*               geospatialTrackerOutput);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyGeospatialTrackerANDROIDX2(
    XrGeospatialTrackerANDROIDX2                geospatialTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrLocateGeospatialPoseFromPoseANDROIDX2(
    XrGeospatialTrackerANDROIDX2                geospatialTracker,
    const XrGeospatialPoseFromPoseLocateInfoANDROIDX2* locateInfo,
    XrGeospatialPoseResultANDROIDX2*            geospatialPoseResult);

XRAPI_ATTR XrResult XRAPI_CALL xrLocateGeospatialPoseANDROIDX2(
    XrGeospatialTrackerANDROIDX2                geospatialTracker,
    const XrGeospatialPoseLocateInfoANDROIDX2*  locateInfo,
    XrSpaceLocation*                            location);

XRAPI_ATTR XrResult XRAPI_CALL xrCheckVpsAvailabilityAsyncANDROIDX2(
    XrSession                                   session,
    double                                      latitude,
    double                                      longitude,
    XrFutureEXT*                                future);

XRAPI_ATTR XrResult XRAPI_CALL xrCheckVpsAvailabilityCompleteANDROIDX2(
    XrSession                                   session,
    XrFutureEXT                                 future,
    XrVPSAvailabilityCheckCompletionANDROIDX2*  completion);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROIDX2_geospatial */

#ifdef __cplusplus
}
#endif

#endif
