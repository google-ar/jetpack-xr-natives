#ifndef XR_ANDROIDX2_GOOGLE_CLOUD_AUTH_H_
#define XR_ANDROIDX2_GOOGLE_CLOUD_AUTH_H_ 1

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


#ifndef XR_ANDROIDX2_google_cloud_auth

// XR_ANDROIDX2_google_cloud_auth is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX2_google_cloud_auth 1
#define XR_ANDROIDX2_google_cloud_auth_SPEC_VERSION 1
#define XR_ANDROIDX2_GOOGLE_CLOUD_AUTH_EXTENSION_NAME "XR_ANDROIDX2_google_cloud_auth"
#define XR_ERROR_KEYLESS_AUTH_NOT_SETUP_ANDROIDX2 ((XrResult) -1000787000U)
#define XR_ERROR_KEYLESS_AUTH_FAILED_ANDROIDX2 ((XrResult) -1000787001U)
#define XR_TYPE_GOOGLE_CLOUD_AUTH_API_KEY_ANDROIDX2 ((XrStructureType) 1000787000U)
#define XR_TYPE_GOOGLE_CLOUD_AUTH_TOKEN_ANDROIDX2 ((XrStructureType) 1000787001U)
#define XR_TYPE_GOOGLE_CLOUD_AUTH_KEYLESS_ANDROIDX2 ((XrStructureType) 1000787002U)
#define XR_TYPE_GOOGLE_CLOUD_AUTH_ERROR_RESULT_ANDROIDX2 ((XrStructureType) 1000787003U)

typedef enum XrGoogleCloudAuthErrorANDROIDX2 {
    // No error occurred when invoking a Google Cloud API.
    XR_GOOGLE_CLOUD_AUTH_ERROR_NONE_ANDROIDX2 = 0,
    // Quota exceeded when invoking a Google Cloud API.
    XR_GOOGLE_CLOUD_AUTH_ERROR_QUOTA_EXCEEDED_ANDROIDX2 = -1,
    // Failed to reach a Google Cloud API, possibly due to network connectivity issues or server availability.
    XR_GOOGLE_CLOUD_AUTH_ERROR_UNREACHABLE_ANDROIDX2 = -2,
    // An auth error occurred when invoking a Google Cloud API.
    XR_GOOGLE_CLOUD_AUTH_ERROR_ANDROIDX2 = -3,
    XR_GOOGLE_CLOUD_AUTH_ERROR_ANDROIDX2_MAX_ENUM = 0x7FFFFFFF
} XrGoogleCloudAuthErrorANDROIDX2;
typedef struct XrGoogleCloudAuthInfoBaseHeaderANDROIDX2 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrGoogleCloudAuthInfoBaseHeaderANDROIDX2;

// XrGoogleCloudAuthApiKeyANDROIDX2 extends XrGoogleCloudAuthInfoBaseHeaderANDROIDX2
typedef struct XrGoogleCloudAuthApiKeyANDROIDX2 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    const char*                 apiKey;
} XrGoogleCloudAuthApiKeyANDROIDX2;

// XrGoogleCloudAuthTokenANDROIDX2 extends XrGoogleCloudAuthInfoBaseHeaderANDROIDX2
typedef struct XrGoogleCloudAuthTokenANDROIDX2 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    const char*                 authToken;
} XrGoogleCloudAuthTokenANDROIDX2;

// XrGoogleCloudAuthKeylessANDROIDX2 extends XrGoogleCloudAuthInfoBaseHeaderANDROIDX2
typedef struct XrGoogleCloudAuthKeylessANDROIDX2 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrGoogleCloudAuthKeylessANDROIDX2;

// XrGoogleCloudAuthErrorResultANDROIDX2 extends XrSurfaceAnchorCreateCompletionANDROIDX2,XrVPSAvailabilityCheckCompletionANDROIDX2
typedef struct XrGoogleCloudAuthErrorResultANDROIDX2 {
    XrStructureType                    type;
    void* XR_MAY_ALIAS                 next;
    XrGoogleCloudAuthErrorANDROIDX2    error;
} XrGoogleCloudAuthErrorResultANDROIDX2;

typedef XrResult (XRAPI_PTR *PFN_xrSetGoogleCloudAuthAsyncANDROIDX2)(XrSession session, const XrGoogleCloudAuthInfoBaseHeaderANDROIDX2* authInfo, XrFutureEXT* future);
typedef XrResult (XRAPI_PTR *PFN_xrSetGoogleCloudAuthCompleteANDROIDX2)(XrSession session, XrFutureEXT future, XrFutureCompletionEXT* completion);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrSetGoogleCloudAuthAsyncANDROIDX2(
    XrSession                                   session,
    const XrGoogleCloudAuthInfoBaseHeaderANDROIDX2* authInfo,
    XrFutureEXT*                                future);

XRAPI_ATTR XrResult XRAPI_CALL xrSetGoogleCloudAuthCompleteANDROIDX2(
    XrSession                                   session,
    XrFutureEXT                                 future,
    XrFutureCompletionEXT*                      completion);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROIDX2_google_cloud_auth */

#ifdef __cplusplus
}
#endif

#endif
