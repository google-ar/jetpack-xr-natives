#ifndef XR_ANDROIDSYS_IPD_CALIBRATION_H_
#define XR_ANDROIDSYS_IPD_CALIBRATION_H_ 1

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


#ifndef XR_ANDROIDSYS_ipd_calibration

// XR_ANDROIDSYS_ipd_calibration is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_ipd_calibration 1
// XrIpdCalibrationTrackerANDROIDSYS
#define XR_OBJECT_TYPE_IPD_CALIBRATION_TRACKER_ANDROIDSYS ((XrObjectType) 1000719000U)
#define XR_TYPE_IPD_CALIBRATION_TRACKER_CREATE_INFO_ANDROIDSYS ((XrStructureType) 1000719000U)
#define XR_TYPE_IPD_CALIBRATION_STATE_ANDROIDSYS ((XrStructureType) 1000719001U)
#define XR_TYPE_IPD_CALIBRATION_STATE_GET_INFO_ANDROIDSYS ((XrStructureType) 1000719002U)
#define XR_TYPE_IPD_CALIBRATION_RENDER_ORIGIN_ANDROIDSYS ((XrStructureType) 1000719003U)

XR_DEFINE_HANDLE(XrIpdCalibrationTrackerANDROIDSYS)
#define XR_ANDROIDSYS_ipd_calibration_SPEC_VERSION 1
#define XR_ANDROIDSYS_IPD_CALIBRATION_EXTENSION_NAME "XR_ANDROIDSYS_ipd_calibration"
#define XR_MAX_RX_PROD_ID_STRING_LENGTH_ANDROIDSYS 64
#define XR_MAX_LENS_ID_STRING_LENGTH_ANDROIDSYS 64

typedef enum XrIpdCalibrationStatusANDROIDSYS {
    // The eye tracker is not able to operate normally.
    XR_IPD_CALIBRATION_STATUS_DISABLED_ANDROIDSYS = 0,
    // The eye tracker is initializing and/or no valid calibration is present.
    XR_IPD_CALIBRATION_STATUS_INITIALIZING_ANDROIDSYS = 1,
    // The eye tracker is in the process of adjusting IPD for a user.
    XR_IPD_CALIBRATION_STATUS_ADJUSTING_IPD_ANDROIDSYS = 2,
    // The eye tracker is calibrating eye tracking.
    XR_IPD_CALIBRATION_STATUS_CALIBRATING_EYE_TRACKER_ANDROIDSYS = 3,
    // The eye tracker is calibrated and ready to use.
    XR_IPD_CALIBRATION_STATUS_READY_ANDROIDSYS = 4,
    XR_IPD_CALIBRATION_STATUS_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrIpdCalibrationStatusANDROIDSYS;

typedef enum XrIpdCalibrationErrorANDROIDSYS {
    // The eye tracker calibration finished successfully.
    XR_IPD_CALIBRATION_ERROR_UNKNOWN_ANDROIDSYS = 0,
    // The eye tracker calibration failed to adjust IPD. This can be cause by a hardware problem or an algorithmic error.
    XR_IPD_CALIBRATION_ERROR_NONE_ANDROIDSYS = 1,
    // The user's IPD is out of the supported range.
    XR_IPD_CALIBRATION_ERROR_IPD_CALIBRATION_FAILED_ANDROIDSYS = 2,
    // The eye tracker calibration failed.
    XR_IPD_CALIBRATION_ERROR_IPD_OUT_OF_RANGE_ANDROIDSYS = 3,
    // The eye tracker calibration failed for an unknown reason.
    XR_IPD_CALIBRATION_ERROR_EYE_CALIBRATION_FAILED_ANDROIDSYS = 4,
    XR_IPD_CALIBRATION_ERROR_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrIpdCalibrationErrorANDROIDSYS;
typedef struct XrIpdCalibrationTrackerCreateInfoANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrIpdCalibrationTrackerCreateInfoANDROIDSYS;

typedef struct XrIpdCalibrationStateGetInfoANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrIpdCalibrationStateGetInfoANDROIDSYS;

typedef struct XrIpdCalibrationStateANDROIDSYS {
    XrStructureType                     type;
    void* XR_MAY_ALIAS                  next;
    XrIpdCalibrationStatusANDROIDSYS    status;
    XrIpdCalibrationErrorANDROIDSYS     error;
} XrIpdCalibrationStateANDROIDSYS;

typedef struct XrIpdCalibrationRenderOriginANDROIDSYS {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    float                 renderOriginLeftEye;
    float                 renderOriginRightEye;
} XrIpdCalibrationRenderOriginANDROIDSYS;

typedef XrResult (XRAPI_PTR *PFN_xrCreateIpdCalibrationTrackerANDROIDSYS)(XrSession session, const XrIpdCalibrationTrackerCreateInfoANDROIDSYS* createInfo, XrIpdCalibrationTrackerANDROIDSYS* tracker);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyIpdCalibrationTrackerANDROIDSYS)(XrIpdCalibrationTrackerANDROIDSYS tracker);
typedef XrResult (XRAPI_PTR *PFN_xrInitiateAutomaticIpdCalibrationANDROIDSYS)(XrIpdCalibrationTrackerANDROIDSYS tracker);
typedef XrResult (XRAPI_PTR *PFN_xrSendHintHardwareIpdANDROIDSYS)(XrIpdCalibrationTrackerANDROIDSYS tracker, const XrTime time, float ipd);
typedef XrResult (XRAPI_PTR *PFN_xrGetHardwareIpdANDROIDSYS)(XrIpdCalibrationTrackerANDROIDSYS tracker, float* ipdOutput);
typedef XrResult (XRAPI_PTR *PFN_xrGetIpdCalibrationStateANDROIDSYS)(XrIpdCalibrationTrackerANDROIDSYS tracker, const XrIpdCalibrationStateGetInfoANDROIDSYS* getInfo, XrIpdCalibrationStateANDROIDSYS* state);
typedef XrResult (XRAPI_PTR *PFN_xrUpdateRenderOriginANDROIDSYS)(XrIpdCalibrationTrackerANDROIDSYS adjustment, const XrIpdCalibrationRenderOriginANDROIDSYS* renderOrigin);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateIpdCalibrationTrackerANDROIDSYS(
    XrSession                                   session,
    const XrIpdCalibrationTrackerCreateInfoANDROIDSYS* createInfo,
    XrIpdCalibrationTrackerANDROIDSYS*          tracker);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyIpdCalibrationTrackerANDROIDSYS(
    XrIpdCalibrationTrackerANDROIDSYS           tracker);

XRAPI_ATTR XrResult XRAPI_CALL xrInitiateAutomaticIpdCalibrationANDROIDSYS(
    XrIpdCalibrationTrackerANDROIDSYS           tracker);

XRAPI_ATTR XrResult XRAPI_CALL xrSendHintHardwareIpdANDROIDSYS(
    XrIpdCalibrationTrackerANDROIDSYS           tracker,
    const XrTime                                time,
    float                                       ipd);

XRAPI_ATTR XrResult XRAPI_CALL xrGetHardwareIpdANDROIDSYS(
    XrIpdCalibrationTrackerANDROIDSYS           tracker,
    float*                                      ipdOutput);

XRAPI_ATTR XrResult XRAPI_CALL xrGetIpdCalibrationStateANDROIDSYS(
    XrIpdCalibrationTrackerANDROIDSYS           tracker,
    const XrIpdCalibrationStateGetInfoANDROIDSYS* getInfo,
    XrIpdCalibrationStateANDROIDSYS*            state);

XRAPI_ATTR XrResult XRAPI_CALL xrUpdateRenderOriginANDROIDSYS(
    XrIpdCalibrationTrackerANDROIDSYS           adjustment,
    const XrIpdCalibrationRenderOriginANDROIDSYS* renderOrigin);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_ENUM_XrIpdCalibrationStatusANDROIDSYS(_) \
    _(XR_IPD_CALIBRATION_STATUS_DISABLED_ANDROIDSYS, 0) \
    _(XR_IPD_CALIBRATION_STATUS_INITIALIZING_ANDROIDSYS, 1) \
    _(XR_IPD_CALIBRATION_STATUS_ADJUSTING_IPD_ANDROIDSYS, 2) \
    _(XR_IPD_CALIBRATION_STATUS_CALIBRATING_EYE_TRACKER_ANDROIDSYS, 3) \
    _(XR_IPD_CALIBRATION_STATUS_READY_ANDROIDSYS, 4) \
    _(XR_IPD_CALIBRATION_STATUS_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_ENUM_XrIpdCalibrationErrorANDROIDSYS(_) \
    _(XR_IPD_CALIBRATION_ERROR_UNKNOWN_ANDROIDSYS, 0) \
    _(XR_IPD_CALIBRATION_ERROR_NONE_ANDROIDSYS, 1) \
    _(XR_IPD_CALIBRATION_ERROR_IPD_CALIBRATION_FAILED_ANDROIDSYS, 2) \
    _(XR_IPD_CALIBRATION_ERROR_IPD_OUT_OF_RANGE_ANDROIDSYS, 3) \
    _(XR_IPD_CALIBRATION_ERROR_EYE_CALIBRATION_FAILED_ANDROIDSYS, 4) \
    _(XR_IPD_CALIBRATION_ERROR_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_STRUCT_XrIpdCalibrationTrackerCreateInfoANDROIDSYS(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrIpdCalibrationStateGetInfoANDROIDSYS(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrIpdCalibrationStateANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(status) \
    _(error)

#define XR_LIST_STRUCT_XrIpdCalibrationRenderOriginANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(renderOriginLeftEye) \
    _(renderOriginRightEye)

#define XR_LIST_FUNCTIONS_XR_ANDROIDSYS_ipd_calibration(_) \
    _(CreateIpdCalibrationTrackerANDROIDSYS, ANDROIDSYS_ipd_calibration) \
    _(DestroyIpdCalibrationTrackerANDROIDSYS, ANDROIDSYS_ipd_calibration) \
    _(InitiateAutomaticIpdCalibrationANDROIDSYS, ANDROIDSYS_ipd_calibration) \
    _(SendHintHardwareIpdANDROIDSYS, ANDROIDSYS_ipd_calibration) \
    _(GetHardwareIpdANDROIDSYS, ANDROIDSYS_ipd_calibration) \
    _(GetIpdCalibrationStateANDROIDSYS, ANDROIDSYS_ipd_calibration) \
    _(UpdateRenderOriginANDROIDSYS, ANDROIDSYS_ipd_calibration)

#endif /* XR_ANDROIDSYS_ipd_calibration */

#ifdef __cplusplus
}
#endif

#endif
