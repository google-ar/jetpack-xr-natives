#ifndef XR_ANDROID_DEPTH_TEXTURE_H_
#define XR_ANDROID_DEPTH_TEXTURE_H_ 1

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


#ifndef XR_ANDROID_depth_texture

// XR_ANDROID_depth_texture is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_depth_texture 1
#define XR_TYPE_DEPTH_SWAPCHAIN_CREATE_INFO_ANDROID ((XrStructureType) 1000702000U)
#define XR_TYPE_DEPTH_VIEW_ANDROID        ((XrStructureType) 1000702001U)
#define XR_TYPE_DEPTH_ACQUIRE_INFO_ANDROID ((XrStructureType) 1000702002U)
#define XR_TYPE_DEPTH_ACQUIRE_RESULT_ANDROID ((XrStructureType) 1000702003U)
#define XR_TYPE_SYSTEM_DEPTH_TRACKING_PROPERTIES_ANDROID ((XrStructureType) 1000702004U)
#define XR_TYPE_DEPTH_SWAPCHAIN_IMAGE_ANDROID ((XrStructureType) 1000702005U)
// Depth images are not available.
#define XR_ERROR_DEPTH_NOT_AVAILABLE_ANDROID ((XrResult) -1000702000U)
// XrDepthSwapchainANDROID
#define XR_OBJECT_TYPE_DEPTH_SWAPCHAIN_ANDROID ((XrObjectType) 1000702001U)

XR_DEFINE_HANDLE(XrDepthSwapchainANDROID)
#define XR_ANDROID_depth_texture_SPEC_VERSION 1
#define XR_ANDROID_DEPTH_TEXTURE_EXTENSION_NAME "XR_ANDROID_depth_texture"

typedef enum XrDepthCameraResolutionANDROID {
    // The resolution of the depth and confidence images is 80x80.
    XR_DEPTH_CAMERA_RESOLUTION_80x80_ANDROID = 0,
    // The resolution of the depth and confidence images is 160x160.
    XR_DEPTH_CAMERA_RESOLUTION_160x160_ANDROID = 1,
    // The resolution of the depth and confidence images is 320x320.
    XR_DEPTH_CAMERA_RESOLUTION_320x320_ANDROID = 2,
    XR_DEPTH_CAMERA_RESOLUTION_MAX_ENUM_ANDROID = 0x7FFFFFFF
} XrDepthCameraResolutionANDROID;
typedef XrFlags64 XrDepthSwapchainCreateFlagsANDROID;

// Flag bits for XrDepthSwapchainCreateFlagsANDROID
// Indicates the swapchain will provide smooth depth imagess.
static const XrDepthSwapchainCreateFlagsANDROID XR_DEPTH_SWAPCHAIN_CREATE_SMOOTH_DEPTH_IMAGE_BIT_ANDROID = 0x00000001;
// Indicates the swapchain will provide smooth depth confidence imagess.
static const XrDepthSwapchainCreateFlagsANDROID XR_DEPTH_SWAPCHAIN_CREATE_SMOOTH_CONFIDENCE_IMAGE_BIT_ANDROID = 0x00000002;
// Indicates the swapchain will provide raw depth imagess.
static const XrDepthSwapchainCreateFlagsANDROID XR_DEPTH_SWAPCHAIN_CREATE_RAW_DEPTH_IMAGE_BIT_ANDROID = 0x00000004;
// Indicates the swapchain will provide raw depth confidence imagess.
static const XrDepthSwapchainCreateFlagsANDROID XR_DEPTH_SWAPCHAIN_CREATE_RAW_CONFIDENCE_IMAGE_BIT_ANDROID = 0x00000008;

typedef struct XrDepthSwapchainCreateInfoANDROID {
    XrStructureType                       type;
    const void* XR_MAY_ALIAS              next;
    XrDepthCameraResolutionANDROID        resolution;
    XrDepthSwapchainCreateFlagsANDROID    createFlags;
} XrDepthSwapchainCreateInfoANDROID;

typedef struct XrDepthSwapchainImageANDROID {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    const float*          rawDepthImage;
    const uint8_t*        rawDepthConfidenceImage;
    const float*          smoothDepthImage;
    const uint8_t*        smoothDepthConfidenceImage;
} XrDepthSwapchainImageANDROID;

typedef struct XrDepthAcquireInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrSpace                     space;
    XrTime                      displayTime;
} XrDepthAcquireInfoANDROID;

typedef struct XrDepthViewANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrFovf                      fov;
    XrPosef                     pose;
} XrDepthViewANDROID;

typedef struct XrDepthAcquireResultANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    uint32_t                    acquiredIndex;
    XrTime                      exposureTimestamp;
    XrDepthViewANDROID          views[2];
} XrDepthAcquireResultANDROID;

typedef struct XrSystemDepthTrackingPropertiesANDROID {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    XrBool32              supportsDepthTracking;
} XrSystemDepthTrackingPropertiesANDROID;

typedef XrResult (XRAPI_PTR *PFN_xrCreateDepthSwapchainANDROID)(XrSession session, const XrDepthSwapchainCreateInfoANDROID* createInfo, XrDepthSwapchainANDROID* swapchain);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyDepthSwapchainANDROID)(XrDepthSwapchainANDROID swapchain);
typedef XrResult (XRAPI_PTR *PFN_xrEnumerateDepthSwapchainImagesANDROID)(XrDepthSwapchainANDROID depthSwapchain, uint32_t depthImageCapacityInput, uint32_t* depthImageCountOutput, XrDepthSwapchainImageANDROID* depthImages);
typedef XrResult (XRAPI_PTR *PFN_xrEnumerateDepthResolutionsANDROID)(XrSession session, uint32_t resolutionCapacityInput, uint32_t* resolutionCountOutput, XrDepthCameraResolutionANDROID* resolutions);
typedef XrResult (XRAPI_PTR *PFN_xrAcquireDepthSwapchainImagesANDROID)(XrDepthSwapchainANDROID depthSwapchain, const XrDepthAcquireInfoANDROID* acquireInfo, XrDepthAcquireResultANDROID* acquireResult);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateDepthSwapchainANDROID(
    XrSession                                   session,
    const XrDepthSwapchainCreateInfoANDROID*    createInfo,
    XrDepthSwapchainANDROID*                    swapchain);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyDepthSwapchainANDROID(
    XrDepthSwapchainANDROID                     swapchain);

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateDepthSwapchainImagesANDROID(
    XrDepthSwapchainANDROID                     depthSwapchain,
    uint32_t                                    depthImageCapacityInput,
    uint32_t*                                   depthImageCountOutput,
    XrDepthSwapchainImageANDROID*               depthImages);

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateDepthResolutionsANDROID(
    XrSession                                   session,
    uint32_t                                    resolutionCapacityInput,
    uint32_t*                                   resolutionCountOutput,
    XrDepthCameraResolutionANDROID*             resolutions);

XRAPI_ATTR XrResult XRAPI_CALL xrAcquireDepthSwapchainImagesANDROID(
    XrDepthSwapchainANDROID                     depthSwapchain,
    const XrDepthAcquireInfoANDROID*            acquireInfo,
    XrDepthAcquireResultANDROID*                acquireResult);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_ENUM_XrDepthCameraResolutionANDROID(_) \
    _(XR_DEPTH_CAMERA_RESOLUTION_80x80_ANDROID, 0) \
    _(XR_DEPTH_CAMERA_RESOLUTION_160x160_ANDROID, 1) \
    _(XR_DEPTH_CAMERA_RESOLUTION_320x320_ANDROID, 2) \
    _(XR_DEPTH_CAMERA_RESOLUTION_MAX_ENUM_ANDROID, 0x7FFFFFFF)

#define XR_LIST_BITS_XrDepthSwapchainCreateFlagsANDROID(_) \
    _(XR_DEPTH_SWAPCHAIN_CREATE_SMOOTH_DEPTH_IMAGE_BIT_ANDROID, 0x00000001) \
    _(XR_DEPTH_SWAPCHAIN_CREATE_SMOOTH_CONFIDENCE_IMAGE_BIT_ANDROID, 0x00000002) \
    _(XR_DEPTH_SWAPCHAIN_CREATE_RAW_DEPTH_IMAGE_BIT_ANDROID, 0x00000004) \
    _(XR_DEPTH_SWAPCHAIN_CREATE_RAW_CONFIDENCE_IMAGE_BIT_ANDROID, 0x00000008)

#define XR_LIST_STRUCT_XrDepthSwapchainCreateInfoANDROID(_) \
    _(type) \
    _(next) \
    _(resolution) \
    _(createFlags)

#define XR_LIST_STRUCT_XrDepthSwapchainImageANDROID(_) \
    _(type) \
    _(next) \
    _(rawDepthImage) \
    _(rawDepthConfidenceImage) \
    _(smoothDepthImage) \
    _(smoothDepthConfidenceImage)

#define XR_LIST_STRUCT_XrDepthAcquireInfoANDROID(_) \
    _(type) \
    _(next) \
    _(space) \
    _(displayTime)

#define XR_LIST_STRUCT_XrDepthViewANDROID(_) \
    _(type) \
    _(next) \
    _(fov) \
    _(pose)

#define XR_LIST_STRUCT_XrDepthAcquireResultANDROID(_) \
    _(type) \
    _(next) \
    _(acquiredIndex) \
    _(exposureTimestamp) \
    _(views)

#define XR_LIST_STRUCT_XrSystemDepthTrackingPropertiesANDROID(_) \
    _(type) \
    _(next) \
    _(supportsDepthTracking)

#define XR_LIST_FUNCTIONS_XR_ANDROID_depth_texture(_) \
    _(CreateDepthSwapchainANDROID, ANDROID_depth_texture) \
    _(DestroyDepthSwapchainANDROID, ANDROID_depth_texture) \
    _(EnumerateDepthSwapchainImagesANDROID, ANDROID_depth_texture) \
    _(EnumerateDepthResolutionsANDROID, ANDROID_depth_texture) \
    _(AcquireDepthSwapchainImagesANDROID, ANDROID_depth_texture)

#endif /* XR_ANDROID_depth_texture */

#ifdef __cplusplus
}
#endif

#endif
