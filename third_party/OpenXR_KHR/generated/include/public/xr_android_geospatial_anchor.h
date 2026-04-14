#ifndef XR_ANDROID_GEOSPATIAL_ANCHOR_H_
#define XR_ANDROID_GEOSPATIAL_ANCHOR_H_ 1

// Standalone dependencies:
#include <openxr/public/xr_android_geospatial.h>

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


#ifndef XR_ANDROID_geospatial_anchor

// XR_ANDROID_geospatial_anchor is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_geospatial_anchor 1
#define XR_ANDROID_geospatial_anchor_SPEC_VERSION 1
#define XR_ANDROID_GEOSPATIAL_ANCHOR_EXTENSION_NAME "XR_ANDROID_geospatial_anchor"
#define XR_TYPE_GEOSPATIAL_ANCHOR_CREATE_INFO_ANDROID ((XrStructureType) 1000797000U)
#define XR_TYPE_SURFACE_ANCHOR_CREATE_INFO_ANDROID ((XrStructureType) 1000797001U)
#define XR_TYPE_SURFACE_ANCHOR_CREATE_COMPLETION_ANDROID ((XrStructureType) 1000797002U)
#define XR_TYPE_SYSTEM_GEOSPATIAL_ANCHOR_PROPERTIES_ANDROID ((XrStructureType) 1000797003U)
#define XR_TYPE_GEOSPATIAL_TRACKER_ANCHOR_TRACKING_INFO_ANDROID ((XrStructureType) 1000797004U)
#define XR_ERROR_SURFACE_ANCHOR_LOCATION_UNSUPPORTED_ANDROID ((XrResult) -1000797000U)

typedef enum XrSurfaceAnchorTypeANDROID {
    // Type of an anchor placed relative to the ground.
    XR_SURFACE_ANCHOR_TYPE_TERRAIN_ANDROID = 1,
    // Type of an anchor placed relative to the rooftop, or ground where there is no building.
    XR_SURFACE_ANCHOR_TYPE_ROOFTOP_ANDROID = 2,
    XR_SURFACE_ANCHOR_TYPE_MAX_ENUM_ANDROID = 0x7FFFFFFF
} XrSurfaceAnchorTypeANDROID;
typedef struct XrSystemGeospatialAnchorPropertiesANDROID {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    uint32_t              maxSurfaceAnchorCount;
} XrSystemGeospatialAnchorPropertiesANDROID;

// XrGeospatialTrackerAnchorTrackingInfoANDROID extends XrGeospatialTrackerCreateInfoANDROID
typedef struct XrGeospatialTrackerAnchorTrackingInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrBool32                    shouldTrackPlanes;
} XrGeospatialTrackerAnchorTrackingInfoANDROID;

typedef struct XrGeospatialAnchorCreateInfoANDROID {
    XrStructureType               type;
    const void* XR_MAY_ALIAS      next;
    XrGeospatialTrackerANDROID    geospatialTracker;
    XrGeospatialPoseANDROID       geospatialPose;
} XrGeospatialAnchorCreateInfoANDROID;

typedef struct XrSurfaceAnchorCreateInfoANDROID {
    XrStructureType               type;
    const void* XR_MAY_ALIAS      next;
    XrGeospatialTrackerANDROID    geospatialTracker;
    XrSurfaceAnchorTypeANDROID    surfaceAnchorType;
    XrQuaternionf                 eastUpSouthOrientation;
    double                        latitude;
    double                        longitude;
    double                        altitudeRelativeToSurface;
} XrSurfaceAnchorCreateInfoANDROID;

typedef struct XrSurfaceAnchorCreateCompletionANDROID {
    XrStructureType         type;
    void* XR_MAY_ALIAS      next;
    XrResult                futureResult;
    XrSpatialEntityIdEXT    anchorEntityId;
} XrSurfaceAnchorCreateCompletionANDROID;

typedef XrResult (XRAPI_PTR *PFN_xrCreateGeospatialAnchorANDROID)(XrSpatialContextEXT spatialContext, const XrGeospatialAnchorCreateInfoANDROID* createInfo, XrSpatialEntityIdEXT* anchorEntityId);
typedef XrResult (XRAPI_PTR *PFN_xrCreateSurfaceAnchorAsyncANDROID)(XrSpatialContextEXT spatialContext, const XrSurfaceAnchorCreateInfoANDROID* createInfo, XrFutureEXT* future);
typedef XrResult (XRAPI_PTR *PFN_xrCreateSurfaceAnchorCompleteANDROID)(XrSpatialContextEXT spatialContext, XrFutureEXT future, XrSurfaceAnchorCreateCompletionANDROID* completion);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateGeospatialAnchorANDROID(
    XrSpatialContextEXT                         spatialContext,
    const XrGeospatialAnchorCreateInfoANDROID*  createInfo,
    XrSpatialEntityIdEXT*                       anchorEntityId);

XRAPI_ATTR XrResult XRAPI_CALL xrCreateSurfaceAnchorAsyncANDROID(
    XrSpatialContextEXT                         spatialContext,
    const XrSurfaceAnchorCreateInfoANDROID*     createInfo,
    XrFutureEXT*                                future);

XRAPI_ATTR XrResult XRAPI_CALL xrCreateSurfaceAnchorCompleteANDROID(
    XrSpatialContextEXT                         spatialContext,
    XrFutureEXT                                 future,
    XrSurfaceAnchorCreateCompletionANDROID*     completion);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROID_geospatial_anchor */

#ifdef __cplusplus
}
#endif

#endif
