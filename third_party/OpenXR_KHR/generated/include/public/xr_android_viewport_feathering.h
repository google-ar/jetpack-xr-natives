#ifndef XR_ANDROID_VIEWPORT_FEATHERING_H_
#define XR_ANDROID_VIEWPORT_FEATHERING_H_ 1

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


#ifndef XR_ANDROID_viewport_feathering

// XR_ANDROID_viewport_feathering is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_viewport_feathering 1
#define XR_TYPE_VIEWPORT_FEATHERING_CONFIG_VIEW_ANDROID ((XrStructureType) 1000806000U)

#define XR_ANDROID_viewport_feathering_SPEC_VERSION 1
#define XR_ANDROID_VIEWPORT_FEATHERING_EXTENSION_NAME "XR_ANDROID_viewport_feathering"
typedef struct XrViewportFeatheringConfigViewANDROID {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    uint32_t              insetLeft;
    uint32_t              insetRight;
    uint32_t              insetTop;
    uint32_t              insetBottom;
} XrViewportFeatheringConfigViewANDROID;


// Reflection macros
#define XR_LIST_STRUCT_XrViewportFeatheringConfigViewANDROID(_) \
    _(type) \
    _(next) \
    _(insetLeft) \
    _(insetRight) \
    _(insetTop) \
    _(insetBottom)

#endif /* XR_ANDROID_viewport_feathering */

#ifdef __cplusplus
}
#endif

#endif
