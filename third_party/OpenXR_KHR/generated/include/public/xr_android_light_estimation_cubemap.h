#ifndef XR_ANDROID_LIGHT_ESTIMATION_CUBEMAP_H_
#define XR_ANDROID_LIGHT_ESTIMATION_CUBEMAP_H_ 1

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


#ifndef XR_ANDROID_light_estimation_cubemap

// XR_ANDROID_light_estimation_cubemap is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_light_estimation_cubemap 1
#define XR_TYPE_SYSTEM_CUBEMAP_LIGHT_ESTIMATION_PROPERTIES_ANDROID ((XrStructureType) 1000721000U)
#define XR_TYPE_CUBEMAP_LIGHT_ESTIMATOR_CREATE_INFO_ANDROID ((XrStructureType) 1000721001U)
#define XR_TYPE_CUBEMAP_LIGHTING_DATA_ANDROID ((XrStructureType) 1000721002U)

#define XR_ANDROID_light_estimation_cubemap_SPEC_VERSION 1
#define XR_ANDROID_LIGHT_ESTIMATION_CUBEMAP_EXTENSION_NAME "XR_ANDROID_light_estimation_cubemap"

typedef enum XrCubemapLightingColorFormatANDROID {
    // A color format with 3 channels where each channel is a 32-bit floating point value.
    XR_CUBEMAP_LIGHTING_COLOR_FORMAT_R32G32B32_SFLOAT_ANDROID = 1,
    // A color format with 4 channels where each channel is a 32-bit floating point value.
    XR_CUBEMAP_LIGHTING_COLOR_FORMAT_R32G32B32A32_SFLOAT_ANDROID = 2,
    // A color format with 4 channels where each channel is a 16-bit floating point value.
    XR_CUBEMAP_LIGHTING_COLOR_FORMAT_R16G16B16A16_SFLOAT_ANDROID = 3,
    XR_CUBEMAP_LIGHTING_COLOR_FORMAT_MAX_ENUM_ANDROID = 0x7FFFFFFF
} XrCubemapLightingColorFormatANDROID;
typedef struct XrSystemCubemapLightEstimationPropertiesANDROID {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    XrBool32              supportsCubemapLightEstimation;
} XrSystemCubemapLightEstimationPropertiesANDROID;

// XrCubemapLightEstimatorCreateInfoANDROID extends XrLightEstimatorCreateInfoANDROID
typedef struct XrCubemapLightEstimatorCreateInfoANDROID {
    XrStructureType                        type;
    const void* XR_MAY_ALIAS               next;
    uint32_t                               cubemapResolution;
    XrCubemapLightingColorFormatANDROID    colorFormat;
    XrBool32                               reproject;
} XrCubemapLightEstimatorCreateInfoANDROID;

// XrCubemapLightingDataANDROID extends XrLightEstimateANDROID
typedef struct XrCubemapLightingDataANDROID {
    XrStructureType                type;
    void* XR_MAY_ALIAS             next;
    XrLightEstimateStateANDROID    state;
    uint32_t                       imageBufferSize;
    uint8_t*                       imageBufferRight;
    uint8_t*                       imageBufferLeft;
    uint8_t*                       imageBufferTop;
    uint8_t*                       imageBufferBottom;
    uint8_t*                       imageBufferFront;
    uint8_t*                       imageBufferBack;
    XrQuaternionf                  rotation;
    XrTime                         centerExposureTime;
} XrCubemapLightingDataANDROID;

typedef XrResult (XRAPI_PTR *PFN_xrEnumerateCubemapLightingResolutionsANDROID)(XrInstance instance, XrSystemId systemId, uint32_t resolutionCapacityInput, uint32_t* resolutionCountOutput, uint32_t* resolutions);
typedef XrResult (XRAPI_PTR *PFN_xrEnumerateCubemapLightingColorFormatsANDROID)(XrInstance instance, XrSystemId systemId, uint32_t colorFormatCapacityInput, uint32_t* colorFormatCountOutput, XrCubemapLightingColorFormatANDROID* colorFormats);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateCubemapLightingResolutionsANDROID(
    XrInstance                                  instance,
    XrSystemId                                  systemId,
    uint32_t                                    resolutionCapacityInput,
    uint32_t*                                   resolutionCountOutput,
    uint32_t*                                   resolutions);

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateCubemapLightingColorFormatsANDROID(
    XrInstance                                  instance,
    XrSystemId                                  systemId,
    uint32_t                                    colorFormatCapacityInput,
    uint32_t*                                   colorFormatCountOutput,
    XrCubemapLightingColorFormatANDROID*        colorFormats);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_ENUM_XrCubemapLightingColorFormatANDROID(_) \
    _(XR_CUBEMAP_LIGHTING_COLOR_FORMAT_R32G32B32_SFLOAT_ANDROID, 1) \
    _(XR_CUBEMAP_LIGHTING_COLOR_FORMAT_R32G32B32A32_SFLOAT_ANDROID, 2) \
    _(XR_CUBEMAP_LIGHTING_COLOR_FORMAT_R16G16B16A16_SFLOAT_ANDROID, 3) \
    _(XR_CUBEMAP_LIGHTING_COLOR_FORMAT_MAX_ENUM_ANDROID, 0x7FFFFFFF)

#define XR_LIST_STRUCT_XrSystemCubemapLightEstimationPropertiesANDROID(_) \
    _(type) \
    _(next) \
    _(supportsCubemapLightEstimation)

#define XR_LIST_STRUCT_XrCubemapLightEstimatorCreateInfoANDROID(_) \
    _(type) \
    _(next) \
    _(cubemapResolution) \
    _(colorFormat) \
    _(reproject)

#define XR_LIST_STRUCT_XrCubemapLightingDataANDROID(_) \
    _(type) \
    _(next) \
    _(state) \
    _(imageBufferSize) \
    _(imageBufferRight) \
    _(imageBufferLeft) \
    _(imageBufferTop) \
    _(imageBufferBottom) \
    _(imageBufferFront) \
    _(imageBufferBack) \
    _(rotation) \
    _(centerExposureTime)

#define XR_LIST_FUNCTIONS_XR_ANDROID_light_estimation_cubemap(_) \
    _(EnumerateCubemapLightingResolutionsANDROID, ANDROID_light_estimation_cubemap) \
    _(EnumerateCubemapLightingColorFormatsANDROID, ANDROID_light_estimation_cubemap)

#endif /* XR_ANDROID_light_estimation_cubemap */

#ifdef __cplusplus
}
#endif

#endif
