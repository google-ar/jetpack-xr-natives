#ifndef XR_ANDROIDX_COMPOSITION_LAYER_AXIS_ALIGNED_DISTORTION_H_
#define XR_ANDROIDX_COMPOSITION_LAYER_AXIS_ALIGNED_DISTORTION_H_ 1

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


#ifndef XR_ANDROIDX_composition_layer_axis_aligned_distortion

// XR_ANDROIDX_composition_layer_axis_aligned_distortion is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX_composition_layer_axis_aligned_distortion 1
#define XR_TYPE_COMPOSITION_LAYER_AXIS_ALIGNED_DISTORTION_ANDROIDX ((XrStructureType) 1000733000U)

#define XR_ANDROIDX_composition_layer_axis_aligned_distortion_SPEC_VERSION 1
#define XR_ANDROIDX_COMPOSITION_LAYER_AXIS_ALIGNED_DISTORTION_EXTENSION_NAME "XR_ANDROIDX_composition_layer_axis_aligned_distortion"
typedef struct XrCompositionLayerAxisAlignedDistortionANDROIDX {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrCompositionLayerFlags     layerFlags;
    XrSpace                     space;
    XrExtent2Di                 nativeDimensions;
    XrVector2f                  interiorExtent;
    XrVector2f                  interiorLocation;
    XrVector2f                  peripheryCompressionFactor;
} XrCompositionLayerAxisAlignedDistortionANDROIDX;


// Reflection macros
#define XR_LIST_STRUCT_XrCompositionLayerAxisAlignedDistortionANDROIDX(_) \
    _(type) \
    _(next) \
    _(layerFlags) \
    _(space) \
    _(nativeDimensions) \
    _(interiorExtent) \
    _(interiorLocation) \
    _(peripheryCompressionFactor)

#endif /* XR_ANDROIDX_composition_layer_axis_aligned_distortion */

#ifdef __cplusplus
}
#endif

#endif
