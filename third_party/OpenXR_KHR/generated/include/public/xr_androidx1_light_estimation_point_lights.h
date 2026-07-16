#ifndef XR_ANDROIDX1_LIGHT_ESTIMATION_POINT_LIGHTS_H_
#define XR_ANDROIDX1_LIGHT_ESTIMATION_POINT_LIGHTS_H_ 1

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


#ifndef XR_ANDROIDX1_light_estimation_point_lights

// XR_ANDROIDX1_light_estimation_point_lights is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX1_light_estimation_point_lights 1
#define XR_ANDROIDX1_light_estimation_point_lights_SPEC_VERSION 1
#define XR_ANDROIDX1_LIGHT_ESTIMATION_POINT_LIGHTS_EXTENSION_NAME "XR_ANDROIDX1_light_estimation_point_lights"
#define XR_TYPE_SYSTEM_POINT_LIGHTS_ESTIMATION_PROPERTIES_ANDROIDX1 ((XrStructureType) 1000731000U)
#define XR_TYPE_POINT_LIGHTS_ESTIMATOR_CREATE_INFO_ANDROIDX1 ((XrStructureType) 1000731001U)
#define XR_TYPE_POINT_LIGHTS_ANDROIDX1    ((XrStructureType) 1000731002U)
typedef struct XrSystemPointLightsEstimationPropertiesANDROIDX1 {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    XrBool32              supportsPointLightsEstimation;
    uint32_t              maxPointLightCount;
} XrSystemPointLightsEstimationPropertiesANDROIDX1;

// XrPointLightsEstimatorCreateInfoANDROIDX1 extends XrLightEstimatorCreateInfoANDROID
typedef struct XrPointLightsEstimatorCreateInfoANDROIDX1 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrPointLightsEstimatorCreateInfoANDROIDX1;

typedef struct XrPointLightDataANDROIDX1 {
    XrVector3f    position;
    XrColor3f     color;
} XrPointLightDataANDROIDX1;

// XrPointLightsANDROIDX1 extends XrLightEstimateANDROID
typedef struct XrPointLightsANDROIDX1 {
    XrStructureType                type;
    void* XR_MAY_ALIAS             next;
    XrLightEstimateStateANDROID    state;
    XrColor3f                      ambientIntensity;
    uint32_t                       pointLightCapacityInput;
    uint32_t                       pointLightCountOutput;
    XrPointLightDataANDROIDX1*     pointLights;
} XrPointLightsANDROIDX1;

#endif /* XR_ANDROIDX1_light_estimation_point_lights */

#ifdef __cplusplus
}
#endif

#endif
