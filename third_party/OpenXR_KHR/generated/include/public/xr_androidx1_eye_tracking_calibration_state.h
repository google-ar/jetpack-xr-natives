#ifndef XR_ANDROIDX1_EYE_TRACKING_CALIBRATION_STATE_H_
#define XR_ANDROIDX1_EYE_TRACKING_CALIBRATION_STATE_H_ 1

/*
** Copyright 2017-2025 The Khronos Group Inc.
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


#ifndef XR_ANDROIDX1_eye_tracking_calibration_state

// XR_ANDROIDX1_eye_tracking_calibration_state is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX1_eye_tracking_calibration_state 1
#define XR_ANDROIDX1_eye_tracking_calibration_state_SPEC_VERSION 1
#define XR_ANDROIDX1_EYE_TRACKING_CALIBRATION_STATE_EXTENSION_NAME "XR_ANDROIDX1_eye_tracking_calibration_state"

typedef enum XrEyeTrackerCalibrationStateANDROIDX1 {
    // The eye tracker calibration state is unknown. This can happen when the call fails or the eye tracking service could not determine the state.
    XR_EYE_TRACKER_CALIBRATION_STATE_UNKNOWN_ANDROIDX1 = 0,
    // The eye tracker is using the default calibration.
    XR_EYE_TRACKER_CALIBRATION_STATE_DEFAULT_ANDROIDX1 = 1,
    // The eye tracker is calibrated for the user.
    XR_EYE_TRACKER_CALIBRATION_STATE_USER_ANDROIDX1 = 2,
    XR_EYE_TRACKER_CALIBRATION_STATE_ANDROIDX1_MAX_ENUM = 0x7FFFFFFF
} XrEyeTrackerCalibrationStateANDROIDX1;
typedef XrResult (XRAPI_PTR *PFN_xrGetEyeTrackerCalibrationStateANDROIDX1)(XrEyeTrackerANDROID eyeTracker, XrEyeTrackerCalibrationStateANDROIDX1* stateOutput);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrGetEyeTrackerCalibrationStateANDROIDX1(
    XrEyeTrackerANDROID                         eyeTracker,
    XrEyeTrackerCalibrationStateANDROIDX1*      stateOutput);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROIDX1_eye_tracking_calibration_state */

#ifdef __cplusplus
}
#endif

#endif
