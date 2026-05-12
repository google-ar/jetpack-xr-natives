#ifndef XR_ANDROIDX1_BODY_TRACKING_H_
#define XR_ANDROIDX1_BODY_TRACKING_H_ 1

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


#ifndef XR_ANDROIDX1_body_tracking

// XR_ANDROIDX1_body_tracking is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX1_body_tracking 1
XR_DEFINE_HANDLE(XrBodyTrackerANDROIDX1)
#define XR_ANDROIDX_BODY_TRACKING_MAX_NUM_JOINTS_ANDROIDX1 22
#define XR_ANDROIDX1_body_tracking_SPEC_VERSION 1
#define XR_ANDROIDX1_BODY_TRACKING_EXTENSION_NAME "XR_ANDROIDX1_body_tracking"
#define XR_TYPE_BODY_TRACKER_CREATE_INFO_ANDROIDX1 ((XrStructureType) 1000716000U)
#define XR_TYPE_AVATAR_SKELETON_ANDROIDX1 ((XrStructureType) 1000716001U)
#define XR_TYPE_BODY_TRACKER_GET_INFO_ANDROIDX1 ((XrStructureType) 1000716002U)
#define XR_TYPE_BODY_TRACKER_AVATAR_PROPORTIONS_ANDROIDX1 ((XrStructureType) 1000716003U)
#define XR_TYPE_AVATAR_SKELETON_JOINT_ANDROIDX1 ((XrStructureType) 1000716004U)
#define XR_TYPE_BODY_TRACKER_CALIBRATION_ANDROIDX1 ((XrStructureType) 1000716005U)
// XrBodyTrackerANDROIDX1
#define XR_OBJECT_TYPE_BODY_TRACKER_ANDROIDX1 ((XrObjectType) 1000716000U)
#define XR_ANDROIDX_BODY_TRACKING_UPPER_BODY_JOINT_COUNT_ANDROIDX1 14
#define XR_ANDROIDX_BODY_TRACKING_FULL_BODY_JOINT_COUNT_ANDROIDX1 22

typedef enum XrAvatarSkeletonJointStatusANDROIDX1 {
    // The joint pose has been explicitly tracked by either hand tracking or head tracking services.
    XR_AVATAR_SKELETON_JOINT_STATUS_TRACKED_ANDROIDX1 = 0,
    // The joint pose has been estimated via a synthesis process using inverse kinematics and generative models.
    XR_AVATAR_SKELETON_JOINT_STATUS_ESTIMATED_ANDROIDX1 = 1,
    // The joint pose is invalid.
    XR_AVATAR_SKELETON_JOINT_STATUS_INVALID_ANDROIDX1 = 2,
    XR_AVATAR_SKELETON_JOINT_STATUS_ANDROIDX1_MAX_ENUM = 0x7FFFFFFF
} XrAvatarSkeletonJointStatusANDROIDX1;

typedef enum XrAvatarSkeletonJointTypeANDROIDX1 {
    XR_AVATAR_SKELETON_JOINT_TYPE_HIPS_ANDROIDX1 = 0,
    XR_AVATAR_SKELETON_JOINT_TYPE_SPINE_ANDROIDX1 = 1,
    XR_AVATAR_SKELETON_JOINT_TYPE_RIBS_ANDROIDX1 = 2,
    XR_AVATAR_SKELETON_JOINT_TYPE_CHEST_ANDROIDX1 = 3,
    XR_AVATAR_SKELETON_JOINT_TYPE_NECK_ANDROIDX1 = 4,
    XR_AVATAR_SKELETON_JOINT_TYPE_HEAD_ANDROIDX1 = 5,
    XR_AVATAR_SKELETON_JOINT_TYPE_LEFT_SHOULDER_ANDROIDX1 = 6,
    XR_AVATAR_SKELETON_JOINT_TYPE_RIGHT_SHOULDER_ANDROIDX1 = 7,
    XR_AVATAR_SKELETON_JOINT_TYPE_LEFT_UPPER_ARM_ANDROIDX1 = 8,
    XR_AVATAR_SKELETON_JOINT_TYPE_RIGHT_UPPER_ARM_ANDROIDX1 = 9,
    XR_AVATAR_SKELETON_JOINT_TYPE_LEFT_LOWER_ARM_ANDROIDX1 = 10,
    XR_AVATAR_SKELETON_JOINT_TYPE_RIGHT_LOWER_ARM_ANDROIDX1 = 11,
    XR_AVATAR_SKELETON_JOINT_TYPE_LEFT_HAND_ANDROIDX1 = 12,
    XR_AVATAR_SKELETON_JOINT_TYPE_RIGHT_HAND_ANDROIDX1 = 13,
    XR_AVATAR_SKELETON_JOINT_TYPE_LEFT_UPPER_LEG_ANDROIDX1 = 14,
    XR_AVATAR_SKELETON_JOINT_TYPE_RIGHT_UPPER_LEG_ANDROIDX1 = 15,
    XR_AVATAR_SKELETON_JOINT_TYPE_LEFT_LOWER_LEG_ANDROIDX1 = 16,
    XR_AVATAR_SKELETON_JOINT_TYPE_RIGHT_LOWER_LEG_ANDROIDX1 = 17,
    XR_AVATAR_SKELETON_JOINT_TYPE_LEFT_FOOT_ANDROIDX1 = 18,
    XR_AVATAR_SKELETON_JOINT_TYPE_RIGHT_FOOT_ANDROIDX1 = 19,
    XR_AVATAR_SKELETON_JOINT_TYPE_LEFT_TOES_ANDROIDX1 = 20,
    XR_AVATAR_SKELETON_JOINT_TYPE_RIGHT_TOES_ANDROIDX1 = 21,
    XR_AVATAR_SKELETON_JOINT_TYPE_ANDROIDX1_MAX_ENUM = 0x7FFFFFFF
} XrAvatarSkeletonJointTypeANDROIDX1;

typedef enum XrBodyJointSetANDROIDX1 {
    // The joint set covering the upper body.
    XR_BODY_JOINT_SET_UPPER_BODY_ANDROIDX1 = 0,
    // The joint set covering the full body.
    XR_BODY_JOINT_SET_FULL_BODY_ANDROIDX1 = 1,
    XR_BODY_JOINT_SET_ANDROIDX1_MAX_ENUM = 0x7FFFFFFF
} XrBodyJointSetANDROIDX1;
typedef struct XrAvatarSkeletonJointANDROIDX1 {
    XrStructureType                         type;
    void *                                  next;
    XrAvatarSkeletonJointTypeANDROIDX1      jointType;
    XrAvatarSkeletonJointTypeANDROIDX1      parent;
    XrAvatarSkeletonJointStatusANDROIDX1    status;
    XrPosef                                 pose;
} XrAvatarSkeletonJointANDROIDX1;

typedef struct XrAvatarSkeletonANDROIDX1 {
    XrStructureType                   type;
    void *                            next;
    XrPosef                           rootPose;
    XrTime                            updateTime;
    uint32_t                          restSkeletonUpdateCount;
    uint32_t                          numJoints;
    XrAvatarSkeletonJointANDROIDX1    joints[XR_ANDROIDX_BODY_TRACKING_MAX_NUM_JOINTS_ANDROIDX1];
} XrAvatarSkeletonANDROIDX1;

typedef struct XrBodyTrackerCreateInfoANDROIDX1 {
    XrStructureType            type;
    const void *               next;
    XrBodyJointSetANDROIDX1    jointSet;
} XrBodyTrackerCreateInfoANDROIDX1;

typedef struct XrBodyTrackerGetInfoANDROIDX1 {
    XrStructureType    type;
    const void *       next;
    XrTime             time;
    XrSpace            space;
} XrBodyTrackerGetInfoANDROIDX1;

typedef struct XrBodyTrackerAvatarProportionsANDROIDX1 {
    XrStructureType    type;
    void *             next;
    float              headHeight;
    float              hipsWidth;
    float              hipsLength;
    float              torsoLength;
    float              neckLength;
    float              shoulderWidth;
    float              upperArmLength;
    float              lowerArmLength;
    float              upperLegLength;
    float              lowerLegLength;
    float              ankleHeight;
    float              footLength;
} XrBodyTrackerAvatarProportionsANDROIDX1;

typedef struct XrBodyTrackerCalibrationANDROIDX1 {
    XrStructureType                            type;
    void *                                     next;
    XrBool32                                   enableAutoCalibration;
    XrBodyTrackerAvatarProportionsANDROIDX1    proportions;
} XrBodyTrackerCalibrationANDROIDX1;

typedef XrResult (XRAPI_PTR *PFN_xrCreateBodyTrackerANDROIDX1)(XrSession session, const XrBodyTrackerCreateInfoANDROIDX1* createInfo, XrBodyTrackerANDROIDX1* bodyTracker);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyBodyTrackerANDROIDX1)(XrBodyTrackerANDROIDX1 bodyTracker);
typedef XrResult (XRAPI_PTR *PFN_xrGetBodyTrackerSkeletonANDROIDX1)(XrBodyTrackerANDROIDX1 bodyTracker, const XrBodyTrackerGetInfoANDROIDX1* getInfo, XrAvatarSkeletonANDROIDX1* skeletonPoseOutput);
typedef XrResult (XRAPI_PTR *PFN_xrGetBodyTrackerRestSkeletonANDROIDX1)(XrBodyTrackerANDROIDX1 bodyTracker, const XrBodyTrackerGetInfoANDROIDX1* getInfo, XrAvatarSkeletonANDROIDX1* skeletonRestOutput);
typedef XrResult (XRAPI_PTR *PFN_xrSetBodyTrackerCalibrationANDROIDX1)(XrBodyTrackerANDROIDX1 bodyTracker, const XrBodyTrackerCalibrationANDROIDX1* calibration);
typedef XrResult (XRAPI_PTR *PFN_xrGetBodyTrackerCalibrationANDROIDX1)(XrBodyTrackerANDROIDX1 bodyTracker, XrBodyTrackerCalibrationANDROIDX1* calibration);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateBodyTrackerANDROIDX1(
    XrSession                                   session,
    const XrBodyTrackerCreateInfoANDROIDX1*     createInfo,
    XrBodyTrackerANDROIDX1*                     bodyTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyBodyTrackerANDROIDX1(
    XrBodyTrackerANDROIDX1                      bodyTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrGetBodyTrackerSkeletonANDROIDX1(
    XrBodyTrackerANDROIDX1                      bodyTracker,
    const XrBodyTrackerGetInfoANDROIDX1*        getInfo,
    XrAvatarSkeletonANDROIDX1*                  skeletonPoseOutput);

XRAPI_ATTR XrResult XRAPI_CALL xrGetBodyTrackerRestSkeletonANDROIDX1(
    XrBodyTrackerANDROIDX1                      bodyTracker,
    const XrBodyTrackerGetInfoANDROIDX1*        getInfo,
    XrAvatarSkeletonANDROIDX1*                  skeletonRestOutput);

XRAPI_ATTR XrResult XRAPI_CALL xrSetBodyTrackerCalibrationANDROIDX1(
    XrBodyTrackerANDROIDX1                      bodyTracker,
    const XrBodyTrackerCalibrationANDROIDX1*    calibration);

XRAPI_ATTR XrResult XRAPI_CALL xrGetBodyTrackerCalibrationANDROIDX1(
    XrBodyTrackerANDROIDX1                      bodyTracker,
    XrBodyTrackerCalibrationANDROIDX1*          calibration);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROIDX1_body_tracking */

#ifdef __cplusplus
}
#endif

#endif
