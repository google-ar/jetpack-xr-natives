#ifndef XR_ANDROIDX_SYSTEM_STATE_H_
#define XR_ANDROIDX_SYSTEM_STATE_H_ 1

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


#ifndef XR_ANDROIDX_system_state

// XR_ANDROIDX_system_state is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX_system_state 1
#define XR_ANDROIDX_system_state_SPEC_VERSION 1
#define XR_ANDROIDX_SYSTEM_STATE_EXTENSION_NAME "XR_ANDROIDX_system_state"
#define XR_TYPE_SYSTEM_STATE_ANDROIDX     ((XrStructureType) 1000454000U)

typedef enum XrInputModalityANDROIDX {
    // Indicates an input modality that cannot be determined.
    XR_INPUT_MODALITY_UNKNOWN_ANDROIDX = 0,
    // Indicates an input modality that is using head tracking.
    XR_INPUT_MODALITY_HEAD_ANDROIDX = 1,
    // Indicates an input modality that is using some XR controller inputs.
    XR_INPUT_MODALITY_CONTROLLER_ANDROIDX = 2,
    // Indicates an input modality that is using hand tracking.
    XR_INPUT_MODALITY_HANDS_ANDROIDX = 3,
    // Indicates an input modality that is using a mouse inputs.
    XR_INPUT_MODALITY_MOUSE_ANDROIDX = 4,
    // Indicates an input modality that combines eye tracking and hand tracking.
    XR_INPUT_MODALITY_GAZE_AND_GESTURE_ANDROIDX = 5,
    XR_INPUT_MODALITY_MAX_ENUM_ANDROIDX = 0x7FFFFFFF
} XrInputModalityANDROIDX;
typedef struct XrSystemStateANDROIDX {
    XrStructureType            type;
    void* XR_MAY_ALIAS         next;
    XrEnvironmentBlendMode     currentBlendMode;
    float                      passthroughOpacity;
    XrInputModalityANDROIDX    currentInputModality;
} XrSystemStateANDROIDX;

typedef XrResult (XRAPI_PTR *PFN_xrEnumerateInputModalityANDROIDX)(XrInstance instance, XrSystemId systemId, uint32_t inputModalityCapacityInput, uint32_t* inputModalityCountOutput, XrInputModalityANDROIDX*   inputModalities);
typedef XrResult (XRAPI_PTR *PFN_xrGetSuggestedSettingsANDROIDX)(XrInstance instance, XrSystemId systemId, XrSystemStateANDROIDX* output);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateInputModalityANDROIDX(
    XrInstance                                  instance,
    XrSystemId                                  systemId,
    uint32_t                                    inputModalityCapacityInput,
    uint32_t*                                   inputModalityCountOutput,
    XrInputModalityANDROIDX*                    inputModalities);

XRAPI_ATTR XrResult XRAPI_CALL xrGetSuggestedSettingsANDROIDX(
    XrInstance                                  instance,
    XrSystemId                                  systemId,
    XrSystemStateANDROIDX*                      output);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROIDX_system_state */

#ifdef __cplusplus
}
#endif

#endif
