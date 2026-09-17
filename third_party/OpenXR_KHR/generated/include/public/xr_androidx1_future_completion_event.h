#ifndef XR_ANDROIDX1_FUTURE_COMPLETION_EVENT_H_
#define XR_ANDROIDX1_FUTURE_COMPLETION_EVENT_H_ 1

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


#ifndef XR_ANDROIDX1_future_completion_event

// XR_ANDROIDX1_future_completion_event is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX1_future_completion_event 1
#define XR_TYPE_EVENT_DATA_FUTURE_COMPLETE_ANDROIDX1 ((XrStructureType) 1000805000U)

#define XR_ANDROIDX1_future_completion_event_SPEC_VERSION 1
#define XR_ANDROIDX1_FUTURE_COMPLETION_EVENT_EXTENSION_NAME "XR_ANDROIDX1_future_completion_event"
typedef struct XrEventDataFutureCompleteANDROIDX1 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrFutureEXT                 future;
} XrEventDataFutureCompleteANDROIDX1;


// Reflection macros
#define XR_LIST_STRUCT_XrEventDataFutureCompleteANDROIDX1(_) \
    _(type) \
    _(next) \
    _(future)

#endif /* XR_ANDROIDX1_future_completion_event */

#ifdef __cplusplus
}
#endif

#endif
