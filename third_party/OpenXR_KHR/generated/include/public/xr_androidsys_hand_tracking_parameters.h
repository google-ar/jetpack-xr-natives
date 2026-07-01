#ifndef XR_ANDROIDSYS_HAND_TRACKING_PARAMETERS_H_
#define XR_ANDROIDSYS_HAND_TRACKING_PARAMETERS_H_ 1

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


#ifndef XR_ANDROIDSYS_hand_tracking_parameters

// XR_ANDROIDSYS_hand_tracking_parameters is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_hand_tracking_parameters 1
#define XR_TYPE_HAND_TRACKING_PARAMETERS_GET_INFO_ANDROIDSYS ((XrStructureType) 1000464000U)
#define XR_TYPE_HAND_TRACKING_PARAMETERS_ANDROIDSYS ((XrStructureType) 1000464001U)

#define XR_ANDROIDSYS_hand_tracking_parameters_SPEC_VERSION 1
#define XR_ANDROIDSYS_HAND_TRACKING_PARAMETERS_EXTENSION_NAME "XR_ANDROIDSYS_hand_tracking_parameters"
#define XR_HAND_IDENTITY_PARAMETERS_COUNT_ANDROIDSYS 6
#define XR_HAND_JOINT_ANGLES_COUNT_ANDROIDSYS 20
typedef struct XrHandTrackingParametersGetInfoANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrSpace                     baseSpace;
    XrTime                      time;
} XrHandTrackingParametersGetInfoANDROIDSYS;

typedef struct XrHandTrackingParametersANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrBool32                    isHandActive;
    uint32_t                    identityParametersCount;
    float*                      identityParameters;
    XrSpaceLocationFlags        rootJointLocationFlags;
    XrPosef                     rootJointPose;
    uint32_t                    jointAnglesCount;
    float*                      jointAngles;
} XrHandTrackingParametersANDROIDSYS;

typedef XrResult (XRAPI_PTR *PFN_xrGetHandTrackingParametersANDROIDSYS)(XrHandTrackerEXT handTracker, const XrHandTrackingParametersGetInfoANDROIDSYS* getInfo, XrHandTrackingParametersANDROIDSYS* handTrackingParameters);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrGetHandTrackingParametersANDROIDSYS(
    XrHandTrackerEXT                            handTracker,
    const XrHandTrackingParametersGetInfoANDROIDSYS* getInfo,
    XrHandTrackingParametersANDROIDSYS*         handTrackingParameters);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_STRUCT_XrHandTrackingParametersGetInfoANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(baseSpace) \
    _(time)

#define XR_LIST_STRUCT_XrHandTrackingParametersANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(isHandActive) \
    _(identityParametersCount) \
    _(identityParameters) \
    _(rootJointLocationFlags) \
    _(rootJointPose) \
    _(jointAnglesCount) \
    _(jointAngles)

#define XR_LIST_FUNCTIONS_XR_ANDROIDSYS_hand_tracking_parameters(_) \
    _(GetHandTrackingParametersANDROIDSYS, ANDROIDSYS_hand_tracking_parameters)

#endif /* XR_ANDROIDSYS_hand_tracking_parameters */

#ifdef __cplusplus
}
#endif

#endif
