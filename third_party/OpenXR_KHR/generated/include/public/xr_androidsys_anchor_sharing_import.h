#ifndef XR_ANDROIDSYS_ANCHOR_SHARING_IMPORT_H_
#define XR_ANDROIDSYS_ANCHOR_SHARING_IMPORT_H_ 1

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


#ifndef XR_ANDROIDSYS_anchor_sharing_import

// XR_ANDROIDSYS_anchor_sharing_import is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_anchor_sharing_import 1
#define XR_TYPE_SHARED_ANCHOR_SPACE_CREATE_INFO_ANDROIDSYS ((XrStructureType) 1000726000U)
#ifdef XR_USE_PLATFORM_ANDROID

#define XR_ANDROIDSYS_anchor_sharing_import_SPEC_VERSION 1
#define XR_ANDROIDSYS_ANCHOR_SHARING_IMPORT_EXTENSION_NAME "XR_ANDROIDSYS_anchor_sharing_import"
typedef struct XrSharedAnchorSpaceCreateInfoANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    struct AIBinder*            anchorToken;
} XrSharedAnchorSpaceCreateInfoANDROIDSYS;

typedef XrResult (XRAPI_PTR *PFN_xrCreateSharedAnchorSpaceANDROIDSYS)(XrSession session, const XrSharedAnchorSpaceCreateInfoANDROIDSYS* sharingInfo, XrSpace* anchor);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateSharedAnchorSpaceANDROIDSYS(
    XrSession                                   session,
    const XrSharedAnchorSpaceCreateInfoANDROIDSYS* sharingInfo,
    XrSpace*                                    anchor);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_STRUCT_XrSharedAnchorSpaceCreateInfoANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(anchorToken)

#define XR_LIST_FUNCTIONS_XR_ANDROIDSYS_anchor_sharing_import(_) \
    _(CreateSharedAnchorSpaceANDROIDSYS, ANDROIDSYS_anchor_sharing_import)

#endif /* XR_USE_PLATFORM_ANDROID */
#endif /* XR_ANDROIDSYS_anchor_sharing_import */

#ifdef __cplusplus
}
#endif

#endif
