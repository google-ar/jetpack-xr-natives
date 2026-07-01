#ifndef XR_ANDROIDSYS_BACKGROUND_TRACKING_H_
#define XR_ANDROIDSYS_BACKGROUND_TRACKING_H_ 1

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


#ifndef XR_ANDROIDSYS_background_tracking

// XR_ANDROIDSYS_background_tracking is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_background_tracking 1
#define XR_TYPE_INSTANCE_CREATE_INFO_BACKGROUND_TRACKING_ANDROIDSYS ((XrStructureType) 1000725000U)
#ifdef XR_USE_PLATFORM_ANDROID

#define XR_ANDROIDSYS_background_tracking_SPEC_VERSION 1
#define XR_ANDROIDSYS_BACKGROUND_TRACKING_EXTENSION_NAME "XR_ANDROIDSYS_background_tracking"
typedef struct XrInstanceCreateInfoBackgroundTrackingANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    struct AIBinder*            token;
} XrInstanceCreateInfoBackgroundTrackingANDROIDSYS;


// Reflection macros
#define XR_LIST_STRUCT_XrInstanceCreateInfoBackgroundTrackingANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(token)

#endif /* XR_USE_PLATFORM_ANDROID */
#endif /* XR_ANDROIDSYS_background_tracking */

#ifdef __cplusplus
}
#endif

#endif
