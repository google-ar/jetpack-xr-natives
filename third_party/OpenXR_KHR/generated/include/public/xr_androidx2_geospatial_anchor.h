#ifndef XR_ANDROIDX2_GEOSPATIAL_ANCHOR_H_
#define XR_ANDROIDX2_GEOSPATIAL_ANCHOR_H_ 1

// Standalone dependencies:
#include <openxr/public/xr_android_geospatial.h>

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


#ifndef XR_ANDROIDX2_geospatial_anchor

// XR_ANDROIDX2_geospatial_anchor is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX2_geospatial_anchor 1
#define XR_ANDROIDX2_geospatial_anchor_SPEC_VERSION 1
#define XR_ANDROIDX2_GEOSPATIAL_ANCHOR_EXTENSION_NAME "XR_ANDROIDX2_geospatial_anchor"
#define XR_TYPE_GEOSPATIAL_ANCHOR_CREATE_INFO_ANDROIDX2 ((XrStructureType) 1000797000U)
#define XR_TYPE_SURFACE_ANCHOR_CREATE_INFO_ANDROIDX2 ((XrStructureType) 1000797001U)
#define XR_TYPE_SURFACE_ANCHOR_CREATE_COMPLETION_ANDROIDX2 ((XrStructureType) 1000797002U)
#define XR_TYPE_SYSTEM_GEOSPATIAL_ANCHOR_PROPERTIES_ANDROIDX2 ((XrStructureType) 1000797003U)
#define XR_TYPE_GEOSPATIAL_TRACKER_ANCHOR_TRACKING_INFO_ANDROIDX2 ((XrStructureType) 1000797004U)
#define XR_ERROR_SURFACE_ANCHOR_LOCATION_UNSUPPORTED_ANDROIDX2 ((XrResult) -1000797000U)

typedef enum XrSurfaceAnchorTypeANDROIDX2 {
    // Type of an anchor placed relative to the ground.
    XR_SURFACE_ANCHOR_TYPE_TERRAIN_ANDROIDX2 = 1,
    // Type of an anchor placed relative to the rooftop, or ground where there is no building.
    XR_SURFACE_ANCHOR_TYPE_ROOFTOP_ANDROIDX2 = 2,
    XR_SURFACE_ANCHOR_TYPE_ANDROIDX2_MAX_ENUM = 0x7FFFFFFF
} XrSurfaceAnchorTypeANDROIDX2;
typedef struct XrSystemGeospatialAnchorPropertiesANDROIDX2 {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    uint32_t              maxSurfaceAnchorCount;
} XrSystemGeospatialAnchorPropertiesANDROIDX2;

// XrGeospatialTrackerAnchorTrackingInfoANDROIDX2 extends XrGeospatialTrackerCreateInfoANDROID
typedef struct XrGeospatialTrackerAnchorTrackingInfoANDROIDX2 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrBool32                    shouldTrackPlanes;
} XrGeospatialTrackerAnchorTrackingInfoANDROIDX2;

typedef struct XrGeospatialAnchorCreateInfoANDROIDX2 {
    XrStructureType               type;
    const void* XR_MAY_ALIAS      next;
    XrGeospatialTrackerANDROID    geospatialTracker;
    XrGeospatialPoseANDROID       geospatialPose;
} XrGeospatialAnchorCreateInfoANDROIDX2;

typedef struct XrSurfaceAnchorCreateInfoANDROIDX2 {
    XrStructureType                 type;
    const void* XR_MAY_ALIAS        next;
    XrGeospatialTrackerANDROID      geospatialTracker;
    XrSurfaceAnchorTypeANDROIDX2    surfaceAnchorType;
    XrQuaternionf                   eastUpSouthOrientation;
    double                          latitude;
    double                          longitude;
    double                          altitudeRelativeToSurface;
} XrSurfaceAnchorCreateInfoANDROIDX2;

typedef struct XrSurfaceAnchorCreateCompletionANDROIDX2 {
    XrStructureType         type;
    void* XR_MAY_ALIAS      next;
    XrResult                futureResult;
    XrSpatialEntityIdEXT    anchorEntityId;
} XrSurfaceAnchorCreateCompletionANDROIDX2;

typedef XrResult (XRAPI_PTR *PFN_xrCreateGeospatialAnchorANDROIDX2)(XrSpatialContextEXT spatialContext, const XrGeospatialAnchorCreateInfoANDROIDX2* createInfo, XrSpatialEntityIdEXT* anchorEntityId);
typedef XrResult (XRAPI_PTR *PFN_xrCreateSurfaceAnchorAsyncANDROIDX2)(XrSpatialContextEXT spatialContext, const XrSurfaceAnchorCreateInfoANDROIDX2* createInfo, XrFutureEXT* future);
typedef XrResult (XRAPI_PTR *PFN_xrCreateSurfaceAnchorCompleteANDROIDX2)(XrSpatialContextEXT spatialContext, XrFutureEXT future, XrSurfaceAnchorCreateCompletionANDROIDX2* completion);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateGeospatialAnchorANDROIDX2(
    XrSpatialContextEXT                         spatialContext,
    const XrGeospatialAnchorCreateInfoANDROIDX2* createInfo,
    XrSpatialEntityIdEXT*                       anchorEntityId);

XRAPI_ATTR XrResult XRAPI_CALL xrCreateSurfaceAnchorAsyncANDROIDX2(
    XrSpatialContextEXT                         spatialContext,
    const XrSurfaceAnchorCreateInfoANDROIDX2*   createInfo,
    XrFutureEXT*                                future);

XRAPI_ATTR XrResult XRAPI_CALL xrCreateSurfaceAnchorCompleteANDROIDX2(
    XrSpatialContextEXT                         spatialContext,
    XrFutureEXT                                 future,
    XrSurfaceAnchorCreateCompletionANDROIDX2*   completion);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROIDX2_geospatial_anchor */

#ifdef __cplusplus
}
#endif

#endif
