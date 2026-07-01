#ifndef XR_ANDROIDSYS_EYE_TRACKING_CALIBRATION_H_
#define XR_ANDROIDSYS_EYE_TRACKING_CALIBRATION_H_ 1

// Standalone dependencies:
#include <openxr/public/xr_androidsys_ipd_calibration.h>

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


#ifndef XR_ANDROIDSYS_eye_tracking_calibration

// XR_ANDROIDSYS_eye_tracking_calibration is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_eye_tracking_calibration 1
// XrEyeCalibrationANDROIDSYS
#define XR_OBJECT_TYPE_EYE_CALIBRATION_ANDROIDSYS ((XrObjectType) 1000720000U)
#define XR_TYPE_EYE_TRACKER_CALIBRATION_STATE_ANDROIDSYS ((XrStructureType) 1000720000U)
#define XR_TYPE_EYE_TRACKER_CALIBRATION_STATE_GET_INFO_ANDROIDSYS ((XrStructureType) 1000720001U)
#define XR_TYPE_EYE_CALIBRATION_CREATE_INFO_ANDROIDSYS ((XrStructureType) 1000720002U)
#define XR_TYPE_RX_LENS_OPTICAL_DESCRIPTION_ANDROIDSYS ((XrStructureType) 1000720003U)
#define XR_TYPE_RX_LENS_ENTRY_ANDROIDSYS  ((XrStructureType) 1000720004U)
#define XR_TYPE_EYE_TRACKER_REALTIME_CALIBRATION_DATA_ANDROIDSYS ((XrStructureType) 1000720005U)

XR_DEFINE_HANDLE(XrEyeCalibrationANDROIDSYS)
#define XR_ANDROIDSYS_eye_tracking_calibration_SPEC_VERSION 1
#define XR_ANDROIDSYS_EYE_TRACKING_CALIBRATION_EXTENSION_NAME "XR_ANDROIDSYS_eye_tracking_calibration"

typedef enum XrEyeTrackerCalibrationStatusANDROIDSYS {
    // The eye tracker is not able to operate normally.
    XR_EYE_TRACKER_CALIBRATION_STATUS_DISABLED_ANDROIDSYS = 0,
    // The eye tracker is initializing and/or no valid calibration is present.
    XR_EYE_TRACKER_CALIBRATION_STATUS_INITIALIZING_ANDROIDSYS = 1,
    // The eye tracker is in the process of calibrating a user.
    XR_EYE_TRACKER_CALIBRATION_STATUS_CALIBRATING_ANDROIDSYS = 2,
    // The eye tracker is calibrated and is producing gaze estimates for the left eye only.
    XR_EYE_TRACKER_CALIBRATION_STATUS_LEFT_ANDROIDSYS = 3,
    // The eye tracker is calibrated and is producing gaze estimates for the right eye only.
    XR_EYE_TRACKER_CALIBRATION_STATUS_RIGHT_ANDROIDSYS = 4,
    // The eye tracker is calibrated and is producing gaze estimates for both eyes.
    XR_EYE_TRACKER_CALIBRATION_STATUS_BOTH_ANDROIDSYS = 5,
    // The eye tracker is performing user calibration computation.
    XR_EYE_TRACKER_CALIBRATION_STATUS_COMPUTING_CALIBRATION_ANDROIDSYS = 6,
    XR_EYE_TRACKER_CALIBRATION_STATUS_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrEyeTrackerCalibrationStatusANDROIDSYS;

typedef enum XrEyeTrackerCalibrationErrorANDROIDSYS {
    // The eye tracker calibration failed for an unknown reason.
    XR_EYE_TRACKER_CALIBRATION_ERROR_UNKNOWN_ERROR_ANDROIDSYS = 0,
    // The eye tracker calibration finished successfully.
    XR_EYE_TRACKER_CALIBRATION_ERROR_NONE_ANDROIDSYS = 1,
    // The eye tracker calibration failed to collect stable frames for calibration. This is most likely due to user not fixating on a target or blinking excessively.
    XR_EYE_TRACKER_CALIBRATION_ERROR_COLLECTION_TIMEOUT_ERROR_ANDROIDSYS = 2,
    // The eye tracker calibration computation failed.
    XR_EYE_TRACKER_CALIBRATION_ERROR_COMPUTATION_ERROR_ANDROIDSYS = 3,
    // The eye tracker failed to load the calibration file.
    XR_EYE_TRACKER_CALIBRATION_ERROR_LOAD_ERROR_ANDROIDSYS = 4,
    // The eye tracker failed to save into the calibration file.
    XR_EYE_TRACKER_CALIBRATION_ERROR_SAVE_ERROR_ANDROIDSYS = 5,
    XR_EYE_TRACKER_CALIBRATION_ERROR_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrEyeTrackerCalibrationErrorANDROIDSYS;
typedef struct XrEyeTrackerCalibrationStateANDROIDSYS {
    XrStructureType                            type;
    void* XR_MAY_ALIAS                         next;
    XrEyeTrackerCalibrationStatusANDROIDSYS    status;
    float                                      calibrationErrorDegrees;
    XrEyeTrackerCalibrationErrorANDROIDSYS     error;
    int32_t                                    numTargets;
    int32_t                                    currentTarget;
    float                                      currentTargetProgress;
    XrVector3f                                 currentTargetPosition;
} XrEyeTrackerCalibrationStateANDROIDSYS;

typedef struct XrEyeTrackerCalibrationStateGetInfoANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrTime                      time;
    XrSpace                     baseSpace;
} XrEyeTrackerCalibrationStateGetInfoANDROIDSYS;

typedef struct XrRxLensOpticalDescriptionANDROIDSYS {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    float                 sphericalPowerDiopters;
    float                 cylindricalPowerDiopters;
    float                 cylindricalAxisDegrees;
    float                 baseCurveParallelPowerDiopters;
    float                 baseCurvePerpendicularPowerDiopters;
    float                 centerThicknessMeters;
    float                 diameterMeters;
    float                 refractiveIndexVisible;
    float                 refractiveIndexIr;
    char                  productIdentifier[XR_MAX_RX_PROD_ID_STRING_LENGTH_ANDROIDSYS];
} XrRxLensOpticalDescriptionANDROIDSYS;

typedef struct XrRxLensEntryANDROIDSYS {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    XrEyeIndexANDROID     eye;
    char                  rxIdentifier[XR_MAX_LENS_ID_STRING_LENGTH_ANDROIDSYS];
} XrRxLensEntryANDROIDSYS;

typedef struct XrEyeCalibrationCreateInfoANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrEyeCalibrationCreateInfoANDROIDSYS;

typedef struct XrEyeTrackerRealtimeCalibrationDataANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrTime                      timestamp;
    XrSpace                     baseSpace;
    XrVector3f                  interactionCursorPosition;
    XrExtent3Df                 interactionCursorDimensions;
} XrEyeTrackerRealtimeCalibrationDataANDROIDSYS;

typedef XrResult (XRAPI_PTR *PFN_xrStartEyeTrackerCalibrationANDROIDSYS)(XrEyeTrackerANDROID eyeTracker);
typedef XrResult (XRAPI_PTR *PFN_xrGetEyeTrackerCalibrationStateANDROIDSYS)(XrEyeTrackerANDROID eyeTracker, const XrEyeTrackerCalibrationStateGetInfoANDROIDSYS* getInfo, XrEyeTrackerCalibrationStateANDROIDSYS* stateOutput);
typedef XrResult (XRAPI_PTR *PFN_xrStopEyeTrackerCalibrationANDROIDSYS)(XrEyeTrackerANDROID eyeTracker);
typedef XrResult (XRAPI_PTR *PFN_xrCreateEyeCalibrationANDROIDSYS)(XrSession session, const XrEyeCalibrationCreateInfoANDROIDSYS* createInfo, XrEyeCalibrationANDROIDSYS* calibrationOutput);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyEyeCalibrationANDROIDSYS)(XrEyeCalibrationANDROIDSYS calibration);
typedef XrResult (XRAPI_PTR *PFN_xrRegisterRxInsertANDROIDSYS)(XrEyeCalibrationANDROIDSYS calibration, XrRxLensOpticalDescriptionANDROIDSYS opticalDescription, XrRxLensEntryANDROIDSYS lensEntry);
typedef XrResult (XRAPI_PTR *PFN_xrUnregisterRxInsertANDROIDSYS)(XrEyeCalibrationANDROIDSYS calibration, XrRxLensEntryANDROIDSYS lensEntry);
typedef XrResult (XRAPI_PTR *PFN_xrEnumerateRxInsertsANDROIDSYS)(XrEyeCalibrationANDROIDSYS calibration, uint32_t rxInsertCapacityInput, uint32_t* rxInsertCountOutput, XrRxLensEntryANDROIDSYS* rxInserts);
typedef XrResult (XRAPI_PTR *PFN_xrLoadRxInsertANDROIDSYS)(XrEyeCalibrationANDROIDSYS calibration, XrRxLensEntryANDROIDSYS lensEntry, XrBool32* outRetryRequired);
typedef XrResult (XRAPI_PTR *PFN_xrSubmitRealtimeEyeCalibrationDataANDROIDSYS)(XrEyeTrackerANDROID eyeTracker, const XrEyeTrackerRealtimeCalibrationDataANDROIDSYS* data);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrStartEyeTrackerCalibrationANDROIDSYS(
    XrEyeTrackerANDROID                         eyeTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrGetEyeTrackerCalibrationStateANDROIDSYS(
    XrEyeTrackerANDROID                         eyeTracker,
    const XrEyeTrackerCalibrationStateGetInfoANDROIDSYS* getInfo,
    XrEyeTrackerCalibrationStateANDROIDSYS*     stateOutput);

XRAPI_ATTR XrResult XRAPI_CALL xrStopEyeTrackerCalibrationANDROIDSYS(
    XrEyeTrackerANDROID                         eyeTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrCreateEyeCalibrationANDROIDSYS(
    XrSession                                   session,
    const XrEyeCalibrationCreateInfoANDROIDSYS* createInfo,
    XrEyeCalibrationANDROIDSYS*                 calibrationOutput);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyEyeCalibrationANDROIDSYS(
    XrEyeCalibrationANDROIDSYS                  calibration);

XRAPI_ATTR XrResult XRAPI_CALL xrRegisterRxInsertANDROIDSYS(
    XrEyeCalibrationANDROIDSYS                  calibration,
    XrRxLensOpticalDescriptionANDROIDSYS        opticalDescription,
    XrRxLensEntryANDROIDSYS                     lensEntry);

XRAPI_ATTR XrResult XRAPI_CALL xrUnregisterRxInsertANDROIDSYS(
    XrEyeCalibrationANDROIDSYS                  calibration,
    XrRxLensEntryANDROIDSYS                     lensEntry);

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateRxInsertsANDROIDSYS(
    XrEyeCalibrationANDROIDSYS                  calibration,
    uint32_t                                    rxInsertCapacityInput,
    uint32_t*                                   rxInsertCountOutput,
    XrRxLensEntryANDROIDSYS*                    rxInserts);

XRAPI_ATTR XrResult XRAPI_CALL xrLoadRxInsertANDROIDSYS(
    XrEyeCalibrationANDROIDSYS                  calibration,
    XrRxLensEntryANDROIDSYS                     lensEntry,
    XrBool32*                                   outRetryRequired);

XRAPI_ATTR XrResult XRAPI_CALL xrSubmitRealtimeEyeCalibrationDataANDROIDSYS(
    XrEyeTrackerANDROID                         eyeTracker,
    const XrEyeTrackerRealtimeCalibrationDataANDROIDSYS* data);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_ENUM_XrEyeTrackerCalibrationStatusANDROIDSYS(_) \
    _(XR_EYE_TRACKER_CALIBRATION_STATUS_DISABLED_ANDROIDSYS, 0) \
    _(XR_EYE_TRACKER_CALIBRATION_STATUS_INITIALIZING_ANDROIDSYS, 1) \
    _(XR_EYE_TRACKER_CALIBRATION_STATUS_CALIBRATING_ANDROIDSYS, 2) \
    _(XR_EYE_TRACKER_CALIBRATION_STATUS_LEFT_ANDROIDSYS, 3) \
    _(XR_EYE_TRACKER_CALIBRATION_STATUS_RIGHT_ANDROIDSYS, 4) \
    _(XR_EYE_TRACKER_CALIBRATION_STATUS_BOTH_ANDROIDSYS, 5) \
    _(XR_EYE_TRACKER_CALIBRATION_STATUS_COMPUTING_CALIBRATION_ANDROIDSYS, 6) \
    _(XR_EYE_TRACKER_CALIBRATION_STATUS_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_ENUM_XrEyeTrackerCalibrationErrorANDROIDSYS(_) \
    _(XR_EYE_TRACKER_CALIBRATION_ERROR_UNKNOWN_ERROR_ANDROIDSYS, 0) \
    _(XR_EYE_TRACKER_CALIBRATION_ERROR_NONE_ANDROIDSYS, 1) \
    _(XR_EYE_TRACKER_CALIBRATION_ERROR_COLLECTION_TIMEOUT_ERROR_ANDROIDSYS, 2) \
    _(XR_EYE_TRACKER_CALIBRATION_ERROR_COMPUTATION_ERROR_ANDROIDSYS, 3) \
    _(XR_EYE_TRACKER_CALIBRATION_ERROR_LOAD_ERROR_ANDROIDSYS, 4) \
    _(XR_EYE_TRACKER_CALIBRATION_ERROR_SAVE_ERROR_ANDROIDSYS, 5) \
    _(XR_EYE_TRACKER_CALIBRATION_ERROR_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_STRUCT_XrEyeTrackerCalibrationStateANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(status) \
    _(calibrationErrorDegrees) \
    _(error) \
    _(numTargets) \
    _(currentTarget) \
    _(currentTargetProgress) \
    _(currentTargetPosition)

#define XR_LIST_STRUCT_XrEyeTrackerCalibrationStateGetInfoANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(time) \
    _(baseSpace)

#define XR_LIST_STRUCT_XrRxLensOpticalDescriptionANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(sphericalPowerDiopters) \
    _(cylindricalPowerDiopters) \
    _(cylindricalAxisDegrees) \
    _(baseCurveParallelPowerDiopters) \
    _(baseCurvePerpendicularPowerDiopters) \
    _(centerThicknessMeters) \
    _(diameterMeters) \
    _(refractiveIndexVisible) \
    _(refractiveIndexIr) \
    _(productIdentifier)

#define XR_LIST_STRUCT_XrRxLensEntryANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(eye) \
    _(rxIdentifier)

#define XR_LIST_STRUCT_XrEyeCalibrationCreateInfoANDROIDSYS(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrEyeTrackerRealtimeCalibrationDataANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(timestamp) \
    _(baseSpace) \
    _(interactionCursorPosition) \
    _(interactionCursorDimensions)

#define XR_LIST_FUNCTIONS_XR_ANDROIDSYS_eye_tracking_calibration(_) \
    _(StartEyeTrackerCalibrationANDROIDSYS, ANDROIDSYS_eye_tracking_calibration) \
    _(GetEyeTrackerCalibrationStateANDROIDSYS, ANDROIDSYS_eye_tracking_calibration) \
    _(StopEyeTrackerCalibrationANDROIDSYS, ANDROIDSYS_eye_tracking_calibration) \
    _(CreateEyeCalibrationANDROIDSYS, ANDROIDSYS_eye_tracking_calibration) \
    _(DestroyEyeCalibrationANDROIDSYS, ANDROIDSYS_eye_tracking_calibration) \
    _(RegisterRxInsertANDROIDSYS, ANDROIDSYS_eye_tracking_calibration) \
    _(UnregisterRxInsertANDROIDSYS, ANDROIDSYS_eye_tracking_calibration) \
    _(EnumerateRxInsertsANDROIDSYS, ANDROIDSYS_eye_tracking_calibration) \
    _(LoadRxInsertANDROIDSYS, ANDROIDSYS_eye_tracking_calibration) \
    _(SubmitRealtimeEyeCalibrationDataANDROIDSYS, ANDROIDSYS_eye_tracking_calibration)

#endif /* XR_ANDROIDSYS_eye_tracking_calibration */

#ifdef __cplusplus
}
#endif

#endif
