#ifndef XR_ANDROID_GLOBAL_PASSTHROUGH_DIMMING_H_
#define XR_ANDROID_GLOBAL_PASSTHROUGH_DIMMING_H_ 1

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


#ifndef XR_ANDROID_global_passthrough_dimming

// XR_ANDROID_global_passthrough_dimming is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_global_passthrough_dimming 1
#define XR_ANDROID_global_passthrough_dimming_SPEC_VERSION 1
#define XR_ANDROID_GLOBAL_PASSTHROUGH_DIMMING_EXTENSION_NAME "XR_ANDROID_global_passthrough_dimming"
#define XR_TYPE_SYSTEM_GLOBAL_DIMMING_PROPERTIES_ANDROID ((XrStructureType) 1000796000U)
#define XR_TYPE_EVENT_DATA_GLOBAL_DIMMING_LEVEL_CHANGED_ANDROID ((XrStructureType) 1000796001U)
#define XR_TYPE_GLOBAL_DIMMING_FRAME_END_INFO_ANDROID ((XrStructureType) 1000796002U)
typedef struct XrSystemGlobalDimmingPropertiesANDROID {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    XrBool32              supportsGlobalDimming;
} XrSystemGlobalDimmingPropertiesANDROID;

typedef struct XrEventDataGlobalDimmingLevelChangedANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrSession                   session;
} XrEventDataGlobalDimmingLevelChangedANDROID;

typedef struct XrGlobalDimmingFrameEndInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    float                       globalDimmingLevel;
} XrGlobalDimmingFrameEndInfoANDROID;

typedef XrResult (XRAPI_PTR *PFN_xrGetGlobalDimmingLevelANDROID)(XrSession session, float* dimmingLevel);
typedef XrResult (XRAPI_PTR *PFN_xrEnumerateSupportedGlobalDimmingLevelsANDROID)(XrInstance instance, XrSystemId systemId, uint32_t supportedGlobalDimmingLevelCapacityInput, uint32_t* supportedGlobalDimmingLevelCountOutput, float* supportedGlobalDimmingLevels);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrGetGlobalDimmingLevelANDROID(
    XrSession                                   session,
    float*                                      dimmingLevel);

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateSupportedGlobalDimmingLevelsANDROID(
    XrInstance                                  instance,
    XrSystemId                                  systemId,
    uint32_t                                    supportedGlobalDimmingLevelCapacityInput,
    uint32_t*                                   supportedGlobalDimmingLevelCountOutput,
    float*                                      supportedGlobalDimmingLevels);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROID_global_passthrough_dimming */

#ifdef __cplusplus
}
#endif

#endif
