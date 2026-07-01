#ifndef XR_ANDROIDSYS_HEADTRACKING_ERROR_CAUSE_H_
#define XR_ANDROIDSYS_HEADTRACKING_ERROR_CAUSE_H_ 1

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


#ifndef XR_ANDROIDSYS_headtracking_error_cause

// XR_ANDROIDSYS_headtracking_error_cause is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_headtracking_error_cause 1
#define XR_TYPE_VIEW_STATE_TRACKING_ERROR_ANDROIDSYS ((XrStructureType) 1000734000U)

#define XR_ANDROIDSYS_headtracking_error_cause_SPEC_VERSION 1
#define XR_ANDROIDSYS_HEADTRACKING_ERROR_CAUSE_EXTENSION_NAME "XR_ANDROIDSYS_headtracking_error_cause"

typedef enum XrHeadTrackingErrorCauseANDROIDSYS {
    XR_HEAD_TRACKING_ERROR_CAUSE_NONE_ANDROIDSYS = 0,
    XR_HEAD_TRACKING_ERROR_CAUSE_LOW_LIGHT_ANDROIDSYS = -1,
    XR_HEAD_TRACKING_ERROR_CAUSE_EXCESSIVE_MOTION_ANDROIDSYS = -2,
    XR_HEAD_TRACKING_ERROR_CAUSE_OTHER_ANDROIDSYS = -3,
    XR_HEAD_TRACKING_ERROR_CAUSE_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrHeadTrackingErrorCauseANDROIDSYS;
typedef struct XrViewStateTrackingErrorANDROIDSYS {
    XrStructureType                       type;
    void* XR_MAY_ALIAS                    next;
    XrHeadTrackingErrorCauseANDROIDSYS    cause;
} XrViewStateTrackingErrorANDROIDSYS;


// Reflection macros
#define XR_LIST_ENUM_XrHeadTrackingErrorCauseANDROIDSYS(_) \
    _(XR_HEAD_TRACKING_ERROR_CAUSE_NONE_ANDROIDSYS, 0) \
    _(XR_HEAD_TRACKING_ERROR_CAUSE_LOW_LIGHT_ANDROIDSYS, -1) \
    _(XR_HEAD_TRACKING_ERROR_CAUSE_EXCESSIVE_MOTION_ANDROIDSYS, -2) \
    _(XR_HEAD_TRACKING_ERROR_CAUSE_OTHER_ANDROIDSYS, -3) \
    _(XR_HEAD_TRACKING_ERROR_CAUSE_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_STRUCT_XrViewStateTrackingErrorANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(cause)

#endif /* XR_ANDROIDSYS_headtracking_error_cause */

#ifdef __cplusplus
}
#endif

#endif
