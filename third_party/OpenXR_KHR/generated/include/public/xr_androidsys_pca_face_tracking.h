#ifndef XR_ANDROIDSYS_PCA_FACE_TRACKING_H_
#define XR_ANDROIDSYS_PCA_FACE_TRACKING_H_ 1

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


#ifndef XR_ANDROIDSYS_pca_face_tracking

// XR_ANDROIDSYS_pca_face_tracking is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDSYS_pca_face_tracking 1
#define XR_TYPE_PCA_FACE_TRACKER_CREATE_INFO_ANDROIDSYS ((XrStructureType) 1000730000U)
#define XR_TYPE_PCA_FACE_STATE_ANDROIDSYS ((XrStructureType) 1000730001U)
#define XR_TYPE_FACE_JOINT_ANDROIDSYS     ((XrStructureType) 1000730002U)

#define XR_ANDROIDSYS_pca_face_tracking_SPEC_VERSION 1
#define XR_ANDROIDSYS_PCA_FACE_TRACKING_EXTENSION_NAME "XR_ANDROIDSYS_pca_face_tracking"
#define XR_FACE_JOINT_COUNT_ANDROIDSYS    4
#define XR_PCA_FACE_PARAMETER_COUNT_ANDROIDSYS 262
#define XR_FACE_SHAPE_PARAMETER_COUNT_ANDROIDSYS 126

typedef enum XrFaceJointTypeANDROIDSYS {
    // A joint for the neck pose.
    XR_FACE_JOINT_TYPE_NECK_ANDROIDSYS = 0,
    // A joint for the head pose.
    XR_FACE_JOINT_TYPE_HEAD_ANDROIDSYS = 1,
    // A joint for the left eye pose.
    XR_FACE_JOINT_TYPE_LEFT_EYE_ANDROIDSYS = 2,
    // A joint for the right eye pose.
    XR_FACE_JOINT_TYPE_RIGHT_EYE_ANDROIDSYS = 3,
    XR_FACE_JOINT_TYPE_MAX_ENUM_ANDROIDSYS = 0x7FFFFFFF
} XrFaceJointTypeANDROIDSYS;
// XrPcaFaceTrackerCreateInfoANDROIDSYS extends XrFaceTrackerCreateInfoANDROID
typedef struct XrPcaFaceTrackerCreateInfoANDROIDSYS {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    float*                      faceShape;
    uint32_t                    numFaceShapeParameters;
} XrPcaFaceTrackerCreateInfoANDROIDSYS;

typedef struct XrFaceJointANDROIDSYS {
    XrStructureType              type;
    void* XR_MAY_ALIAS           next;
    XrFaceJointTypeANDROIDSYS    jointType;
    XrPosef                      pose;
} XrFaceJointANDROIDSYS;

// XrPcaFaceStateANDROIDSYS extends XrFaceStateANDROID
typedef struct XrPcaFaceStateANDROIDSYS {
    XrStructureType           type;
    void* XR_MAY_ALIAS        next;
    uint32_t                  jointCapacityInput;
    uint32_t                  jointCountOutput;
    XrFaceJointANDROIDSYS*    joints;
    uint32_t                  parametersCapacityInput;
    uint32_t                  parametersCountOutput;
    float*                    pcaParameters;
} XrPcaFaceStateANDROIDSYS;


// Reflection macros
#define XR_LIST_ENUM_XrFaceJointTypeANDROIDSYS(_) \
    _(XR_FACE_JOINT_TYPE_NECK_ANDROIDSYS, 0) \
    _(XR_FACE_JOINT_TYPE_HEAD_ANDROIDSYS, 1) \
    _(XR_FACE_JOINT_TYPE_LEFT_EYE_ANDROIDSYS, 2) \
    _(XR_FACE_JOINT_TYPE_RIGHT_EYE_ANDROIDSYS, 3) \
    _(XR_FACE_JOINT_TYPE_MAX_ENUM_ANDROIDSYS, 0x7FFFFFFF)

#define XR_LIST_STRUCT_XrPcaFaceTrackerCreateInfoANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(faceShape) \
    _(numFaceShapeParameters)

#define XR_LIST_STRUCT_XrFaceJointANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(jointType) \
    _(pose)

#define XR_LIST_STRUCT_XrPcaFaceStateANDROIDSYS(_) \
    _(type) \
    _(next) \
    _(jointCapacityInput) \
    _(jointCountOutput) \
    _(joints) \
    _(parametersCapacityInput) \
    _(parametersCountOutput) \
    _(pcaParameters)

#endif /* XR_ANDROIDSYS_pca_face_tracking */

#ifdef __cplusplus
}
#endif

#endif
