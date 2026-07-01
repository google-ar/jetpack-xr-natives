#ifndef XR_ANDROIDSYS_INPUT_TRACING_H_
#define XR_ANDROIDSYS_INPUT_TRACING_H_ 1

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


#ifndef XR_ANDROIDSYS_input_tracing

// XR_ANDROIDSYS_input_tracing is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_input_tracing 1
#define XR_TYPE_INPUT_TRACING_DATA_ANDROIDSYS ((XrStructureType) 1000729000U)
#ifdef XR_USE_PLATFORM_ANDROID

#define XR_ANDROIDSYS_input_tracing_SPEC_VERSION 1
#define XR_ANDROIDSYS_INPUT_TRACING_EXTENSION_NAME "XR_ANDROIDSYS_input_tracing"
typedef struct XrInputTracingDataANDROIDSYS {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    int64_t               eventTracingId;
} XrInputTracingDataANDROIDSYS;


// Reflection macros
#define XR_LIST_STRUCT_XrInputTracingDataANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(eventTracingId)

#endif /* XR_USE_PLATFORM_ANDROID */
#endif /* XR_ANDROIDSYS_input_tracing */

#ifdef __cplusplus
}
#endif

#endif
