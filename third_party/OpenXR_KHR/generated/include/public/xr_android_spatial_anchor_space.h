#ifndef XR_ANDROID_SPATIAL_ANCHOR_SPACE_H_
#define XR_ANDROID_SPATIAL_ANCHOR_SPACE_H_ 1

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


#ifndef XR_ANDROID_spatial_anchor_space

// XR_ANDROID_spatial_anchor_space is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_spatial_anchor_space 1
// The specified spatial entity ID is invalid for an anchor.
#define XR_ERROR_SPATIAL_ANCHOR_ENTITY_ID_INVALID_ANDROID ((XrResult) -1000795001U)
#define XR_TYPE_SPATIAL_ANCHOR_SPACE_FROM_ID_CREATE_INFO_ANDROID ((XrStructureType) 1000795000U)

#define XR_ANDROID_spatial_anchor_space_SPEC_VERSION 1
#define XR_ANDROID_SPATIAL_ANCHOR_SPACE_EXTENSION_NAME "XR_ANDROID_spatial_anchor_space"
typedef struct XrSpatialAnchorSpaceFromIdCreateInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrSpatialEntityIdEXT        anchorEntityId;
} XrSpatialAnchorSpaceFromIdCreateInfoANDROID;

typedef XrResult (XRAPI_PTR *PFN_xrCreateSpatialAnchorSpaceANDROID)(XrSession session, XrSpatialContextEXT spatialContext, const XrSpatialAnchorCreateInfoEXT* createInfo, XrSpatialEntityIdEXT* anchorEntityId, XrSpace* anchorSpace);
typedef XrResult (XRAPI_PTR *PFN_xrCreateSpatialAnchorSpaceFromIdANDROID)(XrSession session, XrSpatialContextEXT spatialContext, const XrSpatialAnchorSpaceFromIdCreateInfoANDROID* createInfo, XrSpace* anchorSpace);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateSpatialAnchorSpaceANDROID(
    XrSession                                   session,
    XrSpatialContextEXT                         spatialContext,
    const XrSpatialAnchorCreateInfoEXT*         createInfo,
    XrSpatialEntityIdEXT*                       anchorEntityId,
    XrSpace*                                    anchorSpace);

XRAPI_ATTR XrResult XRAPI_CALL xrCreateSpatialAnchorSpaceFromIdANDROID(
    XrSession                                   session,
    XrSpatialContextEXT                         spatialContext,
    const XrSpatialAnchorSpaceFromIdCreateInfoANDROID* createInfo,
    XrSpace*                                    anchorSpace);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_STRUCT_XrSpatialAnchorSpaceFromIdCreateInfoANDROID(_) \
    _(type) \
    _(next) \
    _(anchorEntityId)

#define XR_LIST_FUNCTIONS_XR_ANDROID_spatial_anchor_space(_) \
    _(CreateSpatialAnchorSpaceANDROID, ANDROID_spatial_anchor_space) \
    _(CreateSpatialAnchorSpaceFromIdANDROID, ANDROID_spatial_anchor_space)

#endif /* XR_ANDROID_spatial_anchor_space */

#ifdef __cplusplus
}
#endif

#endif
