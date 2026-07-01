#ifndef XR_ANDROIDSYS_BODY_TRACKING_H_
#define XR_ANDROIDSYS_BODY_TRACKING_H_ 1

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


#ifndef XR_ANDROIDSYS_body_tracking

// XR_ANDROIDSYS_body_tracking is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_body_tracking 1
#define XR_TYPE_BODY_JOINT_LOCATIONS_ANDROIDSYS ((XrStructureType) 1000717003U)
#define XR_TYPE_BODY_TRACKER_CREATE_INFO_ANDROIDSYS ((XrStructureType) 1000717004U)
#define XR_TYPE_BODY_JOINTS_LOCATE_INFO_ANDROIDSYS ((XrStructureType) 1000717005U)
#define XR_TYPE_BODY_TRACKER_DIMENSIONS_BASE_HEADER_ANDROIDSYS ((XrStructureType) 1000717006U)
#define XR_TYPE_BODY_TRACKER_DIMENSIONS_AUTO_CALIBRATION_ANDROIDSYS ((XrStructureType) 1000717007U)
#define XR_TYPE_BODY_TRACKER_DIMENSIONS_BODY_HEIGHT_ANDROIDSYS ((XrStructureType) 1000717008U)
#define XR_TYPE_BODY_TRACKER_DIMENSIONS_JOINT_POSITIONS_ANDROIDSYS ((XrStructureType) 1000717009U)
#define XR_TYPE_BODY_TRACKER_DIMENSIONS_BONE_LENGTHS_ANDROIDSYS ((XrStructureType) 1000717010U)
// XrBodyTrackerANDROIDSYS
#define XR_OBJECT_TYPE_BODY_TRACKER_ANDROIDSYS ((XrObjectType) 1000717000U)

XR_DEFINE_HANDLE(XrBodyTrackerANDROIDSYS)
#define XR_ANDROIDSYS_body_tracking_SPEC_VERSION 2
#define XR_ANDROIDSYS_BODY_TRACKING_EXTENSION_NAME "XR_ANDROIDSYS_body_tracking"
#define XR_BODY_UPPER_BODY_JOINT_COUNT_ANDROIDSYS 14
#define XR_BODY_FULL_BODY_JOINT_COUNT_ANDROIDSYS 22

typedef enum XrUpperBodyJointTypeANDROIDSYS {
    XR_UPPER_BODY_JOINT_TYPE_HIPS_ANDROIDSYS = 0,
    XR_UPPER_BODY_JOINT_TYPE_SPINE_ANDROIDSYS = 1,
    XR_UPPER_BODY_JOINT_TYPE_RIBS_ANDROIDSYS = 2,
    XR_UPPER_BODY_JOINT_TYPE_CHEST_ANDROIDSYS = 3,
    XR_UPPER_BODY_JOINT_TYPE_NECK_ANDROIDSYS = 4,
    XR_UPPER_BODY_JOINT_TYPE_HEAD_ANDROIDSYS = 5,
    XR_UPPER_BODY_JOINT_TYPE_LEFT_SHOULDER_ANDROIDSYS = 6,
    XR_UPPER_BODY_JOINT_TYPE_RIGHT_SHOULDER_ANDROIDSYS = 7,
    XR_UPPER_BODY_JOINT_TYPE_LEFT_UPPER_ARM_ANDROIDSYS = 8,
    XR_UPPER_BODY_JOINT_TYPE_RIGHT_UPPER_ARM_ANDROIDSYS = 9,
    XR_UPPER_BODY_JOINT_TYPE_LEFT_LOWER_ARM_ANDROIDSYS = 10,
    XR_UPPER_BODY_JOINT_TYPE_RIGHT_LOWER_ARM_ANDROIDSYS = 11,
    XR_UPPER_BODY_JOINT_TYPE_LEFT_HAND_ANDROIDSYS = 12,
    XR_UPPER_BODY_JOINT_TYPE_RIGHT_HAND_ANDROIDSYS = 13,
    XR_UPPER_BODY_JOINT_TYPE_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrUpperBodyJointTypeANDROIDSYS;

typedef enum XrFullBodyJointTypeANDROIDSYS {
    XR_FULL_BODY_JOINT_TYPE_HIPS_ANDROIDSYS = 0,
    XR_FULL_BODY_JOINT_TYPE_SPINE_ANDROIDSYS = 1,
    XR_FULL_BODY_JOINT_TYPE_RIBS_ANDROIDSYS = 2,
    XR_FULL_BODY_JOINT_TYPE_CHEST_ANDROIDSYS = 3,
    XR_FULL_BODY_JOINT_TYPE_NECK_ANDROIDSYS = 4,
    XR_FULL_BODY_JOINT_TYPE_HEAD_ANDROIDSYS = 5,
    XR_FULL_BODY_JOINT_TYPE_LEFT_SHOULDER_ANDROIDSYS = 6,
    XR_FULL_BODY_JOINT_TYPE_RIGHT_SHOULDER_ANDROIDSYS = 7,
    XR_FULL_BODY_JOINT_TYPE_LEFT_UPPER_ARM_ANDROIDSYS = 8,
    XR_FULL_BODY_JOINT_TYPE_RIGHT_UPPER_ARM_ANDROIDSYS = 9,
    XR_FULL_BODY_JOINT_TYPE_LEFT_LOWER_ARM_ANDROIDSYS = 10,
    XR_FULL_BODY_JOINT_TYPE_RIGHT_LOWER_ARM_ANDROIDSYS = 11,
    XR_FULL_BODY_JOINT_TYPE_LEFT_HAND_ANDROIDSYS = 12,
    XR_FULL_BODY_JOINT_TYPE_RIGHT_HAND_ANDROIDSYS = 13,
    XR_FULL_BODY_JOINT_TYPE_LEFT_UPPER_LEG_ANDROIDSYS = 14,
    XR_FULL_BODY_JOINT_TYPE_RIGHT_UPPER_LEG_ANDROIDSYS = 15,
    XR_FULL_BODY_JOINT_TYPE_LEFT_LOWER_LEG_ANDROIDSYS = 16,
    XR_FULL_BODY_JOINT_TYPE_RIGHT_LOWER_LEG_ANDROIDSYS = 17,
    XR_FULL_BODY_JOINT_TYPE_LEFT_FOOT_ANDROIDSYS = 18,
    XR_FULL_BODY_JOINT_TYPE_RIGHT_FOOT_ANDROIDSYS = 19,
    XR_FULL_BODY_JOINT_TYPE_LEFT_TOES_ANDROIDSYS = 20,
    XR_FULL_BODY_JOINT_TYPE_RIGHT_TOES_ANDROIDSYS = 21,
    XR_FULL_BODY_JOINT_TYPE_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrFullBodyJointTypeANDROIDSYS;

typedef enum XrBodyJointSetANDROIDSYS {
    // The joint set covering the upper body.
    XR_BODY_JOINT_SET_UPPER_BODY_ANDROIDSYS = 0,
    // The joint set covering the full body.
    XR_BODY_JOINT_SET_FULL_BODY_ANDROIDSYS = 1,
    XR_BODY_JOINT_SET_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrBodyJointSetANDROIDSYS;
typedef struct XrBodyJointLocationsANDROIDSYS {
    XrStructureType         type;
    void *                  next;
    XrTime                  lastUpdateTime;
    XrUuid                  restSkeletonGenerationUuid;
    uint32_t                jointCount;
    XrSpaceLocationData*    joints;
} XrBodyJointLocationsANDROIDSYS;

typedef struct XrBodyTrackerCreateInfoANDROIDSYS {
    XrStructureType             type;
    const void *                next;
    XrBodyJointSetANDROIDSYS    jointSet;
} XrBodyTrackerCreateInfoANDROIDSYS;

typedef struct XrBodyJointsLocateInfoANDROIDSYS {
    XrStructureType    type;
    const void *       next;
    XrTime             time;
    XrSpace            space;
} XrBodyJointsLocateInfoANDROIDSYS;

typedef struct XR_MAY_ALIAS XrBodyTrackerDimensionsBaseHeaderANDROIDSYS {
    XrStructureType    type;
    const void *       next;
} XrBodyTrackerDimensionsBaseHeaderANDROIDSYS;

typedef struct XrBodyTrackerDimensionsAutoCalibrationANDROIDSYS {
    XrStructureType    type;
    const void *       next;
} XrBodyTrackerDimensionsAutoCalibrationANDROIDSYS;

typedef struct XrBodyTrackerDimensionsBodyHeightANDROIDSYS {
    XrStructureType    type;
    const void *       next;
    float              bodyHeight;
} XrBodyTrackerDimensionsBodyHeightANDROIDSYS;

typedef struct XrBodyTrackerDimensionsJointPositionsANDROIDSYS {
    XrStructureType      type;
    const void *         next;
    XrTime               time;
    XrSpace              space;
    uint32_t             jointPositionCount;
    const XrVector3f*    jointPositions;
} XrBodyTrackerDimensionsJointPositionsANDROIDSYS;

typedef struct XrBodyTrackerDimensionsBoneLengthsANDROIDSYS {
    XrStructureType    type;
    const void *       next;
    uint32_t           boneLengthCount;
    const float*       boneLengths;
} XrBodyTrackerDimensionsBoneLengthsANDROIDSYS;

typedef XrResult (XRAPI_PTR *PFN_xrEnumerateBodyJointSetANDROIDSYS)(XrInstance instance, XrSystemId systemId, uint32_t jointSetCapacityInput, uint32_t* jointSetCountOutput, XrBodyJointSetANDROIDSYS* jointSets);
typedef XrResult (XRAPI_PTR *PFN_xrCreateBodyTrackerANDROIDSYS)(XrSession session, const XrBodyTrackerCreateInfoANDROIDSYS* createInfo, XrBodyTrackerANDROIDSYS* bodyTracker);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyBodyTrackerANDROIDSYS)(XrBodyTrackerANDROIDSYS bodyTracker);
typedef XrResult (XRAPI_PTR *PFN_xrLocateBodyJointsANDROIDSYS)(XrBodyTrackerANDROIDSYS bodyTracker, const XrBodyJointsLocateInfoANDROIDSYS* locateInfo, XrBodyJointLocationsANDROIDSYS* locations);
typedef XrResult (XRAPI_PTR *PFN_xrLocateRestBodyJointsANDROIDSYS)(XrBodyTrackerANDROIDSYS bodyTracker, const XrBodyJointsLocateInfoANDROIDSYS* locateInfo, XrBodyJointLocationsANDROIDSYS* locations);
typedef XrResult (XRAPI_PTR *PFN_xrSetBodyTrackerDimensionsANDROIDSYS)(XrBodyTrackerANDROIDSYS bodyTracker, const XrBodyTrackerDimensionsBaseHeaderANDROIDSYS* calibration);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateBodyJointSetANDROIDSYS(
    XrInstance                                  instance,
    XrSystemId                                  systemId,
    uint32_t                                    jointSetCapacityInput,
    uint32_t*                                   jointSetCountOutput,
    XrBodyJointSetANDROIDSYS*                   jointSets);

XRAPI_ATTR XrResult XRAPI_CALL xrCreateBodyTrackerANDROIDSYS(
    XrSession                                   session,
    const XrBodyTrackerCreateInfoANDROIDSYS*    createInfo,
    XrBodyTrackerANDROIDSYS*                    bodyTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyBodyTrackerANDROIDSYS(
    XrBodyTrackerANDROIDSYS                     bodyTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrLocateBodyJointsANDROIDSYS(
    XrBodyTrackerANDROIDSYS                     bodyTracker,
    const XrBodyJointsLocateInfoANDROIDSYS*     locateInfo,
    XrBodyJointLocationsANDROIDSYS*             locations);

XRAPI_ATTR XrResult XRAPI_CALL xrLocateRestBodyJointsANDROIDSYS(
    XrBodyTrackerANDROIDSYS                     bodyTracker,
    const XrBodyJointsLocateInfoANDROIDSYS*     locateInfo,
    XrBodyJointLocationsANDROIDSYS*             locations);

XRAPI_ATTR XrResult XRAPI_CALL xrSetBodyTrackerDimensionsANDROIDSYS(
    XrBodyTrackerANDROIDSYS                     bodyTracker,
    const XrBodyTrackerDimensionsBaseHeaderANDROIDSYS* calibration);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_ENUM_XrUpperBodyJointTypeANDROIDSYS(_) \
    _(XR_UPPER_BODY_JOINT_TYPE_HIPS_ANDROIDSYS, 0) \
    _(XR_UPPER_BODY_JOINT_TYPE_SPINE_ANDROIDSYS, 1) \
    _(XR_UPPER_BODY_JOINT_TYPE_RIBS_ANDROIDSYS, 2) \
    _(XR_UPPER_BODY_JOINT_TYPE_CHEST_ANDROIDSYS, 3) \
    _(XR_UPPER_BODY_JOINT_TYPE_NECK_ANDROIDSYS, 4) \
    _(XR_UPPER_BODY_JOINT_TYPE_HEAD_ANDROIDSYS, 5) \
    _(XR_UPPER_BODY_JOINT_TYPE_LEFT_SHOULDER_ANDROIDSYS, 6) \
    _(XR_UPPER_BODY_JOINT_TYPE_RIGHT_SHOULDER_ANDROIDSYS, 7) \
    _(XR_UPPER_BODY_JOINT_TYPE_LEFT_UPPER_ARM_ANDROIDSYS, 8) \
    _(XR_UPPER_BODY_JOINT_TYPE_RIGHT_UPPER_ARM_ANDROIDSYS, 9) \
    _(XR_UPPER_BODY_JOINT_TYPE_LEFT_LOWER_ARM_ANDROIDSYS, 10) \
    _(XR_UPPER_BODY_JOINT_TYPE_RIGHT_LOWER_ARM_ANDROIDSYS, 11) \
    _(XR_UPPER_BODY_JOINT_TYPE_LEFT_HAND_ANDROIDSYS, 12) \
    _(XR_UPPER_BODY_JOINT_TYPE_RIGHT_HAND_ANDROIDSYS, 13) \
    _(XR_UPPER_BODY_JOINT_TYPE_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_ENUM_XrFullBodyJointTypeANDROIDSYS(_) \
    _(XR_FULL_BODY_JOINT_TYPE_HIPS_ANDROIDSYS, 0) \
    _(XR_FULL_BODY_JOINT_TYPE_SPINE_ANDROIDSYS, 1) \
    _(XR_FULL_BODY_JOINT_TYPE_RIBS_ANDROIDSYS, 2) \
    _(XR_FULL_BODY_JOINT_TYPE_CHEST_ANDROIDSYS, 3) \
    _(XR_FULL_BODY_JOINT_TYPE_NECK_ANDROIDSYS, 4) \
    _(XR_FULL_BODY_JOINT_TYPE_HEAD_ANDROIDSYS, 5) \
    _(XR_FULL_BODY_JOINT_TYPE_LEFT_SHOULDER_ANDROIDSYS, 6) \
    _(XR_FULL_BODY_JOINT_TYPE_RIGHT_SHOULDER_ANDROIDSYS, 7) \
    _(XR_FULL_BODY_JOINT_TYPE_LEFT_UPPER_ARM_ANDROIDSYS, 8) \
    _(XR_FULL_BODY_JOINT_TYPE_RIGHT_UPPER_ARM_ANDROIDSYS, 9) \
    _(XR_FULL_BODY_JOINT_TYPE_LEFT_LOWER_ARM_ANDROIDSYS, 10) \
    _(XR_FULL_BODY_JOINT_TYPE_RIGHT_LOWER_ARM_ANDROIDSYS, 11) \
    _(XR_FULL_BODY_JOINT_TYPE_LEFT_HAND_ANDROIDSYS, 12) \
    _(XR_FULL_BODY_JOINT_TYPE_RIGHT_HAND_ANDROIDSYS, 13) \
    _(XR_FULL_BODY_JOINT_TYPE_LEFT_UPPER_LEG_ANDROIDSYS, 14) \
    _(XR_FULL_BODY_JOINT_TYPE_RIGHT_UPPER_LEG_ANDROIDSYS, 15) \
    _(XR_FULL_BODY_JOINT_TYPE_LEFT_LOWER_LEG_ANDROIDSYS, 16) \
    _(XR_FULL_BODY_JOINT_TYPE_RIGHT_LOWER_LEG_ANDROIDSYS, 17) \
    _(XR_FULL_BODY_JOINT_TYPE_LEFT_FOOT_ANDROIDSYS, 18) \
    _(XR_FULL_BODY_JOINT_TYPE_RIGHT_FOOT_ANDROIDSYS, 19) \
    _(XR_FULL_BODY_JOINT_TYPE_LEFT_TOES_ANDROIDSYS, 20) \
    _(XR_FULL_BODY_JOINT_TYPE_RIGHT_TOES_ANDROIDSYS, 21) \
    _(XR_FULL_BODY_JOINT_TYPE_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_ENUM_XrBodyJointSetANDROIDSYS(_) \
    _(XR_BODY_JOINT_SET_UPPER_BODY_ANDROIDSYS, 0) \
    _(XR_BODY_JOINT_SET_FULL_BODY_ANDROIDSYS, 1) \
    _(XR_BODY_JOINT_SET_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_STRUCT_XrBodyJointLocationsANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(lastUpdateTime) \
    _(restSkeletonGenerationUuid) \
    _(jointCount) \
    _(joints)

#define XR_LIST_STRUCT_XrBodyTrackerCreateInfoANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(jointSet)

#define XR_LIST_STRUCT_XrBodyJointsLocateInfoANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(time) \
    _(space)

#define XR_LIST_STRUCT_XrBodyTrackerDimensionsBaseHeaderANDROIDSYS(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrBodyTrackerDimensionsAutoCalibrationANDROIDSYS(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrBodyTrackerDimensionsBodyHeightANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(bodyHeight)

#define XR_LIST_STRUCT_XrBodyTrackerDimensionsJointPositionsANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(time) \
    _(space) \
    _(jointPositionCount) \
    _(jointPositions)

#define XR_LIST_STRUCT_XrBodyTrackerDimensionsBoneLengthsANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(boneLengthCount) \
    _(boneLengths)

#define XR_LIST_FUNCTIONS_XR_ANDROIDSYS_body_tracking(_) \
    _(EnumerateBodyJointSetANDROIDSYS, ANDROIDSYS_body_tracking) \
    _(CreateBodyTrackerANDROIDSYS, ANDROIDSYS_body_tracking) \
    _(DestroyBodyTrackerANDROIDSYS, ANDROIDSYS_body_tracking) \
    _(LocateBodyJointsANDROIDSYS, ANDROIDSYS_body_tracking) \
    _(LocateRestBodyJointsANDROIDSYS, ANDROIDSYS_body_tracking) \
    _(SetBodyTrackerDimensionsANDROIDSYS, ANDROIDSYS_body_tracking)

#endif /* XR_ANDROIDSYS_body_tracking */

#ifdef __cplusplus
}
#endif

#endif
