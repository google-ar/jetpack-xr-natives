#ifndef XR_ANDROIDSYS_TRACKABLES_SHOEBOX_H_
#define XR_ANDROIDSYS_TRACKABLES_SHOEBOX_H_ 1

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


#ifndef XR_ANDROIDSYS_trackables_shoebox

// XR_ANDROIDSYS_trackables_shoebox is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_trackables_shoebox 1
#define XR_TYPE_TRACKABLE_SHOEBOX_ANDROIDSYS ((XrStructureType) 1000728000U)
// Indicates that the trackable is the shoebox.
#define XR_TRACKABLE_TYPE_SHOEBOX_ANDROIDSYS ((XrTrackableTypeANDROID) 1000728000U)

#define XR_MAX_SHOEBOX_MATERIAL_LABEL_SIZE_ANDROIDSYS 24
#define XR_SHOEBOX_SURFACE_SIZE_ANDROIDSYS 6
#define XR_ANDROIDSYS_trackables_shoebox_SPEC_VERSION 1
#define XR_ANDROIDSYS_TRACKABLES_SHOEBOX_EXTENSION_NAME "XR_ANDROIDSYS_trackables_shoebox"
typedef struct XrMaterialANDROIDSYS {
    uint32_t    label;
    float       areaRatio;
    float       confidence;
} XrMaterialANDROIDSYS;

typedef struct XrMaterialListANDROIDSYS {
    uint32_t                numMaterials;
    XrMaterialANDROIDSYS    materials[XR_MAX_SHOEBOX_MATERIAL_LABEL_SIZE_ANDROIDSYS];
} XrMaterialListANDROIDSYS;

typedef struct XrRoomTypeANDROIDSYS {
    uint32_t    label;
    float       confidence;
} XrRoomTypeANDROIDSYS;

typedef struct XrTrackableShoeboxANDROIDSYS {
    XrStructureType             type;
    void* XR_MAY_ALIAS          next;
    XrTrackingStateANDROID      trackingState;
    XrPosef                     centerPose;
    XrExtent3DfEXT              dimensions;
    XrExtent3DfEXT              confidenceDimensions;
    float                       emptiness;
    float                       confidenceScore;
    XrPlaneLabelANDROID         semanticLabels[XR_SHOEBOX_SURFACE_SIZE_ANDROIDSYS];
    XrMaterialListANDROIDSYS    materialList[XR_SHOEBOX_SURFACE_SIZE_ANDROIDSYS];
    XrRoomTypeANDROIDSYS        roomType;
    XrTime                      lastUpdatedTime;
} XrTrackableShoeboxANDROIDSYS;

typedef XrResult (XRAPI_PTR *PFN_xrGetTrackableShoeboxANDROIDSYS)(XrTrackableTrackerANDROID trackableTracker, const XrTrackableGetInfoANDROID* getInfo, XrTrackableShoeboxANDROIDSYS* shoeboxOutput);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrGetTrackableShoeboxANDROIDSYS(
    XrTrackableTrackerANDROID                   trackableTracker,
    const XrTrackableGetInfoANDROID*            getInfo,
    XrTrackableShoeboxANDROIDSYS*               shoeboxOutput);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_STRUCT_XrMaterialANDROIDSYS(_) \
    _(label) \
    _(areaRatio) \
    _(confidence)

#define XR_LIST_STRUCT_XrMaterialListANDROIDSYS(_) \
    _(numMaterials) \
    _(materials)

#define XR_LIST_STRUCT_XrRoomTypeANDROIDSYS(_) \
    _(label) \
    _(confidence)

#define XR_LIST_STRUCT_XrTrackableShoeboxANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(trackingState) \
    _(centerPose) \
    _(dimensions) \
    _(confidenceDimensions) \
    _(emptiness) \
    _(confidenceScore) \
    _(semanticLabels) \
    _(materialList) \
    _(roomType) \
    _(lastUpdatedTime)

#define XR_LIST_FUNCTIONS_XR_ANDROIDSYS_trackables_shoebox(_) \
    _(GetTrackableShoeboxANDROIDSYS, ANDROIDSYS_trackables_shoebox)

#endif /* XR_ANDROIDSYS_trackables_shoebox */

#ifdef __cplusplus
}
#endif

#endif
