#ifndef XR_ANDROIDX_ANDROID_SURFACE_SWAPCHAIN_ACQUIRE_IMAGE_H_
#define XR_ANDROIDX_ANDROID_SURFACE_SWAPCHAIN_ACQUIRE_IMAGE_H_ 1

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


#ifndef XR_ANDROIDX_android_surface_swapchain_acquire_image

// XR_ANDROIDX_android_surface_swapchain_acquire_image is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX_android_surface_swapchain_acquire_image 1
#define XR_TYPE_ANDROID_SURFACE_SWAPCHAIN_CREATE_INFO_ANDROIDX ((XrStructureType) 1000732000U)

#define XR_ANDROIDX_android_surface_swapchain_acquire_image_SPEC_VERSION 1
#define XR_ANDROIDX_ANDROID_SURFACE_SWAPCHAIN_ACQUIRE_IMAGE_EXTENSION_NAME "XR_ANDROIDX_android_surface_swapchain_acquire_image"
typedef struct XrAndroidSurfaceSwapchainCreateInfoANDROIDX {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    uint32_t                    bufferCount;
} XrAndroidSurfaceSwapchainCreateInfoANDROIDX;

typedef XrResult (XRAPI_PTR *PFN_xrAcquireAndroidSurfaceSwapchainImageANDROIDX)(XrSwapchain swapchain, XrTime* timestamp);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrAcquireAndroidSurfaceSwapchainImageANDROIDX(
    XrSwapchain                                 swapchain,
    XrTime*                                     timestamp);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_STRUCT_XrAndroidSurfaceSwapchainCreateInfoANDROIDX(_) \
    _(type) \
    _(next) \
    _(bufferCount)

#define XR_LIST_FUNCTIONS_XR_ANDROIDX_android_surface_swapchain_acquire_image(_) \
    _(AcquireAndroidSurfaceSwapchainImageANDROIDX, ANDROIDX_android_surface_swapchain_acquire_image)

#endif /* XR_ANDROIDX_android_surface_swapchain_acquire_image */

#ifdef __cplusplus
}
#endif

#endif
