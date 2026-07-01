#ifndef XR_ANDROIDSYS_FACE_TRACKING_CALIBRATION_H_
#define XR_ANDROIDSYS_FACE_TRACKING_CALIBRATION_H_ 1

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


#ifndef XR_ANDROIDSYS_face_tracking_calibration

// XR_ANDROIDSYS_face_tracking_calibration is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_face_tracking_calibration 1

#define XR_ANDROIDSYS_face_tracking_calibration_SPEC_VERSION 1
#define XR_ANDROIDSYS_FACE_TRACKING_CALIBRATION_EXTENSION_NAME "XR_ANDROIDSYS_face_tracking_calibration"
typedef XrResult (XRAPI_PTR *PFN_xrActivateFaceCalibrationModeANDROIDSYS)(XrFaceTrackerANDROID faceTracker);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrActivateFaceCalibrationModeANDROIDSYS(
    XrFaceTrackerANDROID                        faceTracker);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_FUNCTIONS_XR_ANDROIDSYS_face_tracking_calibration(_) \
    _(ActivateFaceCalibrationModeANDROIDSYS, ANDROIDSYS_face_tracking_calibration)

#endif /* XR_ANDROIDSYS_face_tracking_calibration */

#ifdef __cplusplus
}
#endif

#endif
