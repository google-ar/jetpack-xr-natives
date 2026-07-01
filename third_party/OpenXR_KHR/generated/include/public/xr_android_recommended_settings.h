#ifndef XR_ANDROID_RECOMMENDED_SETTINGS_H_
#define XR_ANDROID_RECOMMENDED_SETTINGS_H_ 1

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


#ifndef XR_ANDROID_recommended_settings

// XR_ANDROID_recommended_settings is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_recommended_settings 1
#define XR_TYPE_SYSTEM_INPUT_MODALITY_PROPERTIES_ANDROID ((XrStructureType) 1000454000U)
#define XR_TYPE_RECOMMENDED_SETTINGS_ANDROID ((XrStructureType) 1000454001U)

#define XR_ANDROID_recommended_settings_SPEC_VERSION 1
#define XR_ANDROID_RECOMMENDED_SETTINGS_EXTENSION_NAME "XR_ANDROID_recommended_settings"
typedef XrFlags64 XrInputModalityFlagsANDROID;

// Flag bits for XrInputModalityFlagsANDROID
// Indicates an input modality that is using head tracking.
static const XrInputModalityFlagsANDROID XR_INPUT_MODALITY_HEAD_BIT_ANDROID = 0x00000001;
// Indicates an input modality that is using any XR controller inputs.
static const XrInputModalityFlagsANDROID XR_INPUT_MODALITY_CONTROLLER_BIT_ANDROID = 0x00000002;
// Indicates an input modality that is using hand tracking.
static const XrInputModalityFlagsANDROID XR_INPUT_MODALITY_HANDS_BIT_ANDROID = 0x00000004;
// Indicates an input modality that is using mouse inputs.
static const XrInputModalityFlagsANDROID XR_INPUT_MODALITY_MOUSE_BIT_ANDROID = 0x00000008;
// Indicates an input modality that combines eye tracking and hand tracking.
static const XrInputModalityFlagsANDROID XR_INPUT_MODALITY_GAZE_AND_GESTURE_BIT_ANDROID = 0x00000010;

typedef struct XrSystemInputModalityPropertiesANDROID {
    XrStructureType                type;
    void* XR_MAY_ALIAS             next;
    XrInputModalityFlagsANDROID    supportedInputModalities;
} XrSystemInputModalityPropertiesANDROID;

typedef struct XrRecommendedSettingsANDROID {
    XrStructureType                type;
    void* XR_MAY_ALIAS             next;
    XrEnvironmentBlendMode         blendMode;
    float                          passthroughOpacity;
    XrInputModalityFlagsANDROID    activeInputModalities;
} XrRecommendedSettingsANDROID;

typedef XrResult (XRAPI_PTR *PFN_xrGetRecommendedSettingsANDROID)(XrInstance instance, XrSystemId systemId, XrRecommendedSettingsANDROID* output);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrGetRecommendedSettingsANDROID(
    XrInstance                                  instance,
    XrSystemId                                  systemId,
    XrRecommendedSettingsANDROID*               output);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_BITS_XrInputModalityFlagsANDROID(_) \
    _(XR_INPUT_MODALITY_HEAD_BIT_ANDROID, 0x00000001) \
    _(XR_INPUT_MODALITY_CONTROLLER_BIT_ANDROID, 0x00000002) \
    _(XR_INPUT_MODALITY_HANDS_BIT_ANDROID, 0x00000004) \
    _(XR_INPUT_MODALITY_MOUSE_BIT_ANDROID, 0x00000008) \
    _(XR_INPUT_MODALITY_GAZE_AND_GESTURE_BIT_ANDROID, 0x00000010)

#define XR_LIST_STRUCT_XrSystemInputModalityPropertiesANDROID(_) \
    _(type) \
    _(next) \
    _(supportedInputModalities)

#define XR_LIST_STRUCT_XrRecommendedSettingsANDROID(_) \
    _(type) \
    _(next) \
    _(blendMode) \
    _(passthroughOpacity) \
    _(activeInputModalities)

#define XR_LIST_FUNCTIONS_XR_ANDROID_recommended_settings(_) \
    _(GetRecommendedSettingsANDROID, ANDROID_recommended_settings)

#endif /* XR_ANDROID_recommended_settings */

#ifdef __cplusplus
}
#endif

#endif
