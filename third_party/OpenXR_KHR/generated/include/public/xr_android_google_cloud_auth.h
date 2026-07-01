#ifndef XR_ANDROID_GOOGLE_CLOUD_AUTH_H_
#define XR_ANDROID_GOOGLE_CLOUD_AUTH_H_ 1

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


#ifndef XR_ANDROID_google_cloud_auth

// XR_ANDROID_google_cloud_auth is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_google_cloud_auth 1
#define XR_ERROR_KEYLESS_AUTH_NOT_SETUP_ANDROID ((XrResult) -1000787000U)
#define XR_ERROR_KEYLESS_AUTH_FAILED_ANDROID ((XrResult) -1000787001U)
#define XR_TYPE_GOOGLE_CLOUD_AUTH_INFO_API_KEY_ANDROID ((XrStructureType) 1000787000U)
#define XR_TYPE_GOOGLE_CLOUD_AUTH_INFO_TOKEN_ANDROID ((XrStructureType) 1000787001U)
#define XR_TYPE_GOOGLE_CLOUD_AUTH_INFO_KEYLESS_ANDROID ((XrStructureType) 1000787002U)
#define XR_TYPE_GOOGLE_CLOUD_AUTH_ERROR_RESULT_ANDROID ((XrStructureType) 1000787003U)

#define XR_ANDROID_google_cloud_auth_SPEC_VERSION 1
#define XR_ANDROID_GOOGLE_CLOUD_AUTH_EXTENSION_NAME "XR_ANDROID_google_cloud_auth"

typedef enum XrGoogleCloudAuthErrorANDROID {
    // No error occurred when invoking a Google Cloud API.
    XR_GOOGLE_CLOUD_AUTH_ERROR_NONE_ANDROID = 0,
    // Quota exceeded when invoking a Google Cloud API.
    XR_GOOGLE_CLOUD_AUTH_ERROR_QUOTA_EXCEEDED_ANDROID = -1,
    // Failed to reach a Google Cloud API, possibly due to network connectivity issues or server availability.
    XR_GOOGLE_CLOUD_AUTH_ERROR_UNREACHABLE_ANDROID = -2,
    // An auth error occurred when invoking a Google Cloud API.
    XR_GOOGLE_CLOUD_AUTH_ERROR_ANDROID = -3,
    XR_GOOGLE_CLOUD_AUTH_ERROR_MAX_ENUM_ANDROID = 0x7FFFFFFF
} XrGoogleCloudAuthErrorANDROID;
typedef struct XR_MAY_ALIAS XrGoogleCloudAuthInfoBaseHeaderANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrGoogleCloudAuthInfoBaseHeaderANDROID;

typedef struct XrGoogleCloudAuthInfoApiKeyANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    const char*                 apiKey;
} XrGoogleCloudAuthInfoApiKeyANDROID;

typedef struct XrGoogleCloudAuthInfoTokenANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    const char*                 authToken;
} XrGoogleCloudAuthInfoTokenANDROID;

typedef struct XrGoogleCloudAuthInfoKeylessANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrGoogleCloudAuthInfoKeylessANDROID;

// XrGoogleCloudAuthErrorResultANDROID extends XrGoogleCloudAuthInfoBaseHeaderANDROID
typedef struct XrGoogleCloudAuthErrorResultANDROID {
    XrStructureType                  type;
    void* XR_MAY_ALIAS               next;
    XrGoogleCloudAuthErrorANDROID    error;
} XrGoogleCloudAuthErrorResultANDROID;

typedef XrResult (XRAPI_PTR *PFN_xrSetGoogleCloudAuthAsyncANDROID)(XrSession session, const XrGoogleCloudAuthInfoBaseHeaderANDROID* authInfo, XrFutureEXT* future);
typedef XrResult (XRAPI_PTR *PFN_xrSetGoogleCloudAuthCompleteANDROID)(XrSession session, XrFutureEXT future, XrFutureCompletionEXT* completion);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrSetGoogleCloudAuthAsyncANDROID(
    XrSession                                   session,
    const XrGoogleCloudAuthInfoBaseHeaderANDROID* authInfo,
    XrFutureEXT*                                future);

XRAPI_ATTR XrResult XRAPI_CALL xrSetGoogleCloudAuthCompleteANDROID(
    XrSession                                   session,
    XrFutureEXT                                 future,
    XrFutureCompletionEXT*                      completion);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_ENUM_XrGoogleCloudAuthErrorANDROID(_) \
    _(XR_GOOGLE_CLOUD_AUTH_ERROR_NONE_ANDROID, 0) \
    _(XR_GOOGLE_CLOUD_AUTH_ERROR_QUOTA_EXCEEDED_ANDROID, -1) \
    _(XR_GOOGLE_CLOUD_AUTH_ERROR_UNREACHABLE_ANDROID, -2) \
    _(XR_GOOGLE_CLOUD_AUTH_ERROR_ANDROID, -3) \
    _(XR_GOOGLE_CLOUD_AUTH_ERROR_MAX_ENUM_ANDROID, 0x7FFFFFFF)

#define XR_LIST_STRUCT_XrGoogleCloudAuthInfoBaseHeaderANDROID(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrGoogleCloudAuthInfoApiKeyANDROID(_) \
    _(type) \
    _(next) \
    _(apiKey)

#define XR_LIST_STRUCT_XrGoogleCloudAuthInfoTokenANDROID(_) \
    _(type) \
    _(next) \
    _(authToken)

#define XR_LIST_STRUCT_XrGoogleCloudAuthInfoKeylessANDROID(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrGoogleCloudAuthErrorResultANDROID(_) \
    _(type) \
    _(next) \
    _(error)

#define XR_LIST_FUNCTIONS_XR_ANDROID_google_cloud_auth(_) \
    _(SetGoogleCloudAuthAsyncANDROID, ANDROID_google_cloud_auth) \
    _(SetGoogleCloudAuthCompleteANDROID, ANDROID_google_cloud_auth)

#endif /* XR_ANDROID_google_cloud_auth */

#ifdef __cplusplus
}
#endif

#endif
