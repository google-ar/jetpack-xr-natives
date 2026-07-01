#ifndef XR_ANDROIDSYS_FIT_TRACKING_H_
#define XR_ANDROIDSYS_FIT_TRACKING_H_ 1

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


#ifndef XR_ANDROIDSYS_fit_tracking

// XR_ANDROIDSYS_fit_tracking is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_fit_tracking 1
#define XR_OBJECT_TYPE_FIT_TRACKER_ANDROIDSYS ((XrObjectType) 1000468000U)
#define XR_TYPE_FIT_TRACKER_CREATE_INFO_ANDROIDSYS ((XrStructureType) 1000468000U)
#define XR_TYPE_FIT_TRACKER_GET_INFO_ANDROIDSYS ((XrStructureType) 1000468001U)
#define XR_TYPE_FIT_TRACKER_RX_INSERT_ANDROIDSYS ((XrStructureType) 1000468002U)
#define XR_TYPE_FIT_TRACKER_COMPARISON_INFO_ANDROIDSYS ((XrStructureType) 1000468003U)
#define XR_TYPE_FIT_TRACKER_INSERT_CHANGE_ANDROIDSYS ((XrStructureType) 1000468004U)
#define XR_TYPE_FIT_TRACKER_MOUNT_ANDROIDSYS ((XrStructureType) 1000468005U)
#define XR_TYPE_FIT_TRACKER_FIT_DATA_ANDROIDSYS ((XrStructureType) 1000468006U)

XR_DEFINE_HANDLE(XrFitTrackerANDROIDSYS)
#define XR_ANDROIDSYS_fit_tracking_SPEC_VERSION 1
#define XR_ANDROIDSYS_FIT_TRACKING_EXTENSION_NAME "XR_ANDROIDSYS_fit_tracking"

// The current overall status of fit tracking
typedef enum XrFitTrackerStatusANDROIDSYS {
    // The fit tracker is initialized without any errors
    XR_FIT_TRACKER_STATUS_INITIALIZED_ANDROIDSYS = 0,
    // The fit tracker is not enabled due to error in initializing model
    XR_FIT_TRACKER_STATUS_ERROR_ANDROIDSYS = 1,
    // The fit tracker is not enabled due to error in initializing camera
    XR_FIT_TRACKER_STATUS_CAMERA_ERROR_ANDROIDSYS = 2,
    // The fit tracker is not enabled due to undefined error
    XR_FIT_TRACKER_STATUS_UNDEFINED_ERROR_ANDROIDSYS = 3,
    XR_FIT_TRACKER_STATUS_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrFitTrackerStatusANDROIDSYS;

// The status of Rx Insert detection
typedef enum XrFitTrackerRxInsertStatusANDROIDSYS {
    // Detected the user does not put on Rx insert
    XR_FIT_TRACKER_RX_INSERT_STATUS_NOT_PRESENT_ANDROIDSYS = 0,
    // Detected the user puts on Rx insert
    XR_FIT_TRACKER_RX_INSERT_STATUS_PRESENT_ANDROIDSYS = 1,
    // Detected the user puts on glasses
    XR_FIT_TRACKER_RX_INSERT_STATUS_GLASSES_PRESENT_ANDROIDSYS = 2,
    XR_FIT_TRACKER_RX_INSERT_STATUS_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrFitTrackerRxInsertStatusANDROIDSYS;

// The status of Rx insert change detection
typedef enum XrFitTrackerRxInsertChangeStatusANDROIDSYS {
    // The user puts on the existing Rx insert
    XR_FIT_TRACKER_RX_INSERT_CHANGE_STATUS_NOT_DETECTED_ANDROIDSYS = 0,
    // Detected the user puts on a new Rx insert
    XR_FIT_TRACKER_RX_INSERT_CHANGE_STATUS_DETECTED_ANDROIDSYS = 1,
    XR_FIT_TRACKER_RX_INSERT_CHANGE_STATUS_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrFitTrackerRxInsertChangeStatusANDROIDSYS;

// The status of mount detection - indicates whether the device is mounted on the user's head or not
typedef enum XrFitTrackerHeadMountedStatusANDROIDSYS {
    // The user has not mounted the device
    XR_FIT_TRACKER_HEAD_MOUNTED_STATUS_NOT_DETECTED_ANDROIDSYS = 0,
    // The user has mounted the device
    XR_FIT_TRACKER_HEAD_MOUNTED_STATUS_DETECTED_ANDROIDSYS = 1,
    XR_FIT_TRACKER_HEAD_MOUNTED_STATUS_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrFitTrackerHeadMountedStatusANDROIDSYS;
typedef struct XrFitTrackerCreateInfoANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrFitTrackerCreateInfoANDROIDSYS;

typedef struct XrFitTrackerGetInfoANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrFitTrackerGetInfoANDROIDSYS;

typedef struct XrFitTrackerRxInsertANDROIDSYS {
    XrStructureType                         type;
    void* XR_MAY_ALIAS                      next;
    XrFitTrackerStatusANDROIDSYS            status;
    int64_t                                 referenceFrameId;
    XrFitTrackerRxInsertStatusANDROIDSYS    rxInsertStatusLeftEye;
    XrFitTrackerRxInsertStatusANDROIDSYS    rxInsertStatusRightEye;
} XrFitTrackerRxInsertANDROIDSYS;

typedef struct XrFitTrackerComparisonInfoANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    int64_t                     referenceImageA;
    int64_t                     referenceImageB;
} XrFitTrackerComparisonInfoANDROIDSYS;

typedef struct XrFitTrackerInsertChangeANDROIDSYS {
    XrStructureType                               type;
    void* XR_MAY_ALIAS                            next;
    XrFitTrackerStatusANDROIDSYS                  status;
    int64_t                                       referenceFrameId;
    XrFitTrackerRxInsertChangeStatusANDROIDSYS    rxInsertChangeStatusLeftEye;
    XrFitTrackerRxInsertChangeStatusANDROIDSYS    rxInsertChangeStatusRightEye;
} XrFitTrackerInsertChangeANDROIDSYS;

typedef struct XrFitTrackerMountANDROIDSYS {
    XrStructureType                            type;
    void* XR_MAY_ALIAS                         next;
    XrFitTrackerStatusANDROIDSYS               status;
    int64_t                                    referenceFrameId;
    XrFitTrackerHeadMountedStatusANDROIDSYS    mountStatus;
} XrFitTrackerMountANDROIDSYS;

typedef struct XrFitTrackerFitDataANDROIDSYS {
    XrStructureType                 type;
    void* XR_MAY_ALIAS              next;
    XrFitTrackerStatusANDROIDSYS    status;
    int64_t                         referenceFrameId;
    XrOffset2Di                     eyeImageCenterLeft;
    XrOffset2Di                     eyeImageCenterRight;
    float                           eyeReliefLeft;
    float                           eyeReliefRight;
} XrFitTrackerFitDataANDROIDSYS;

typedef XrResult (XRAPI_PTR *PFN_xrCreateFitTrackerANDROIDSYS)(XrSession session, const XrFitTrackerCreateInfoANDROIDSYS* createInfo, XrFitTrackerANDROIDSYS* fitTrackerOut);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyFitTrackerANDROIDSYS)(XrFitTrackerANDROIDSYS fitTracker);
typedef XrResult (XRAPI_PTR *PFN_xrGetFitTrackerCurrentRxInsertANDROIDSYS)(XrFitTrackerANDROIDSYS fitTracker, const XrFitTrackerGetInfoANDROIDSYS* getInfo, XrFitTrackerRxInsertANDROIDSYS* rxInsertOut);
typedef XrResult (XRAPI_PTR *PFN_xrCompareFitTrackerRxInsertsANDROIDSYS)(XrFitTrackerANDROIDSYS fitTracker, const XrFitTrackerComparisonInfoANDROIDSYS* comparisonInfo, XrFitTrackerInsertChangeANDROIDSYS* rxInsertChangeOut);
typedef XrResult (XRAPI_PTR *PFN_xrGetFitTrackerMountANDROIDSYS)(XrFitTrackerANDROIDSYS fitTracker, const XrFitTrackerGetInfoANDROIDSYS* getInfo, XrFitTrackerMountANDROIDSYS* mountOut);
typedef XrResult (XRAPI_PTR *PFN_xrGetFitTrackerFitDataANDROIDSYS)(XrFitTrackerANDROIDSYS fitTracker, const XrFitTrackerGetInfoANDROIDSYS* getInfo, XrFitTrackerFitDataANDROIDSYS* fitDataOut);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateFitTrackerANDROIDSYS(
    XrSession                                   session,
    const XrFitTrackerCreateInfoANDROIDSYS*     createInfo,
    XrFitTrackerANDROIDSYS*                     fitTrackerOut);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyFitTrackerANDROIDSYS(
    XrFitTrackerANDROIDSYS                      fitTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrGetFitTrackerCurrentRxInsertANDROIDSYS(
    XrFitTrackerANDROIDSYS                      fitTracker,
    const XrFitTrackerGetInfoANDROIDSYS*        getInfo,
    XrFitTrackerRxInsertANDROIDSYS*             rxInsertOut);

XRAPI_ATTR XrResult XRAPI_CALL xrCompareFitTrackerRxInsertsANDROIDSYS(
    XrFitTrackerANDROIDSYS                      fitTracker,
    const XrFitTrackerComparisonInfoANDROIDSYS* comparisonInfo,
    XrFitTrackerInsertChangeANDROIDSYS*         rxInsertChangeOut);

XRAPI_ATTR XrResult XRAPI_CALL xrGetFitTrackerMountANDROIDSYS(
    XrFitTrackerANDROIDSYS                      fitTracker,
    const XrFitTrackerGetInfoANDROIDSYS*        getInfo,
    XrFitTrackerMountANDROIDSYS*                mountOut);

XRAPI_ATTR XrResult XRAPI_CALL xrGetFitTrackerFitDataANDROIDSYS(
    XrFitTrackerANDROIDSYS                      fitTracker,
    const XrFitTrackerGetInfoANDROIDSYS*        getInfo,
    XrFitTrackerFitDataANDROIDSYS*              fitDataOut);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_ENUM_XrFitTrackerStatusANDROIDSYS(_) \
    _(XR_FIT_TRACKER_STATUS_INITIALIZED_ANDROIDSYS, 0) \
    _(XR_FIT_TRACKER_STATUS_ERROR_ANDROIDSYS, 1) \
    _(XR_FIT_TRACKER_STATUS_CAMERA_ERROR_ANDROIDSYS, 2) \
    _(XR_FIT_TRACKER_STATUS_UNDEFINED_ERROR_ANDROIDSYS, 3) \
    _(XR_FIT_TRACKER_STATUS_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_ENUM_XrFitTrackerRxInsertStatusANDROIDSYS(_) \
    _(XR_FIT_TRACKER_RX_INSERT_STATUS_NOT_PRESENT_ANDROIDSYS, 0) \
    _(XR_FIT_TRACKER_RX_INSERT_STATUS_PRESENT_ANDROIDSYS, 1) \
    _(XR_FIT_TRACKER_RX_INSERT_STATUS_GLASSES_PRESENT_ANDROIDSYS, 2) \
    _(XR_FIT_TRACKER_RXINSERT_STATUS_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_ENUM_XrFitTrackerRxInsertChangeStatusANDROIDSYS(_) \
    _(XR_FIT_TRACKER_RX_INSERT_CHANGE_STATUS_NOT_DETECTED_ANDROIDSYS, 0) \
    _(XR_FIT_TRACKER_RX_INSERT_CHANGE_STATUS_DETECTED_ANDROIDSYS, 1) \
    _(XR_FIT_TRACKER_RXINSERT_CHANGE_STATUS_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_ENUM_XrFitTrackerHeadMountedStatusANDROIDSYS(_) \
    _(XR_FIT_TRACKER_HEAD_MOUNTED_STATUS_NOT_DETECTED_ANDROIDSYS, 0) \
    _(XR_FIT_TRACKER_HEAD_MOUNTED_STATUS_DETECTED_ANDROIDSYS, 1) \
    _(XR_FIT_TRACKER_HEAD_MOUNTED_STATUS_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_STRUCT_XrFitTrackerCreateInfoANDROIDSYS(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrFitTrackerGetInfoANDROIDSYS(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrFitTrackerRxInsertANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(status) \
    _(referenceFrameId) \
    _(rxInsertStatusLeftEye) \
    _(rxInsertStatusRightEye)

#define XR_LIST_STRUCT_XrFitTrackerComparisonInfoANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(referenceImageA) \
    _(referenceImageB)

#define XR_LIST_STRUCT_XrFitTrackerInsertChangeANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(status) \
    _(referenceFrameId) \
    _(rxInsertChangeStatusLeftEye) \
    _(rxInsertChangeStatusRightEye)

#define XR_LIST_STRUCT_XrFitTrackerMountANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(status) \
    _(referenceFrameId) \
    _(mountStatus)

#define XR_LIST_STRUCT_XrFitTrackerFitDataANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(status) \
    _(referenceFrameId) \
    _(eyeImageCenterLeft) \
    _(eyeImageCenterRight) \
    _(eyeReliefLeft) \
    _(eyeReliefRight)

#define XR_LIST_FUNCTIONS_XR_ANDROIDSYS_fit_tracking(_) \
    _(CreateFitTrackerANDROIDSYS, ANDROIDSYS_fit_tracking) \
    _(DestroyFitTrackerANDROIDSYS, ANDROIDSYS_fit_tracking) \
    _(GetFitTrackerCurrentRxInsertANDROIDSYS, ANDROIDSYS_fit_tracking) \
    _(CompareFitTrackerRxInsertsANDROIDSYS, ANDROIDSYS_fit_tracking) \
    _(GetFitTrackerMountANDROIDSYS, ANDROIDSYS_fit_tracking) \
    _(GetFitTrackerFitDataANDROIDSYS, ANDROIDSYS_fit_tracking)

#endif /* XR_ANDROIDSYS_fit_tracking */

#ifdef __cplusplus
}
#endif

#endif
