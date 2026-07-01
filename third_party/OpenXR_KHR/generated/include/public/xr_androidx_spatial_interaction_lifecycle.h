#ifndef XR_ANDROIDX_SPATIAL_INTERACTION_LIFECYCLE_H_
#define XR_ANDROIDX_SPATIAL_INTERACTION_LIFECYCLE_H_ 1

// Standalone dependencies:
#include <openxr/public/xr_androidx_spatial_interaction.h>

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


#ifndef XR_ANDROIDX_spatial_interaction_lifecycle

// XR_ANDROIDX_spatial_interaction_lifecycle is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX_spatial_interaction_lifecycle 1

#define XR_ANDROIDX_spatial_interaction_lifecycle_SPEC_VERSION 1
#define XR_ANDROIDX_SPATIAL_INTERACTION_LIFECYCLE_EXTENSION_NAME "XR_ANDROIDX_spatial_interaction_lifecycle"
typedef XrResult (XRAPI_PTR *PFN_xrEnableSpatialInteractionANDROIDX)(XrSession session);
typedef XrResult (XRAPI_PTR *PFN_xrDisableSpatialInteractionANDROIDX)(XrSession session);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrEnableSpatialInteractionANDROIDX(
    XrSession                                   session);

XRAPI_ATTR XrResult XRAPI_CALL xrDisableSpatialInteractionANDROIDX(
    XrSession                                   session);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_FUNCTIONS_XR_ANDROIDX_spatial_interaction_lifecycle(_) \
    _(EnableSpatialInteractionANDROIDX, ANDROIDX_spatial_interaction_lifecycle) \
    _(DisableSpatialInteractionANDROIDX, ANDROIDX_spatial_interaction_lifecycle)

#endif /* XR_ANDROIDX_spatial_interaction_lifecycle */

#ifdef __cplusplus
}
#endif

#endif
