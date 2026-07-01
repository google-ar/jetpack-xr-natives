#ifndef XR_ANDROID_HAND_MESH_H_
#define XR_ANDROID_HAND_MESH_H_ 1

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


#ifndef XR_ANDROID_hand_mesh

// XR_ANDROID_hand_mesh is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_hand_mesh 1
#define XR_TYPE_SYSTEM_HAND_MESH_TRACKING_PROPERTIES_ANDROID ((XrStructureType) 1000703000U)
#define XR_TYPE_HAND_MESH_TRACKER_CREATE_INFO_ANDROID ((XrStructureType) 1000703001U)
#define XR_TYPE_HAND_MESH_GET_INFO_ANDROID ((XrStructureType) 1000703003U)
#define XR_TYPE_HAND_TRACKING_MESHES_ANDROID ((XrStructureType) 1000703004U)
// XrHandMeshTrackerANDROID
#define XR_OBJECT_TYPE_HAND_MESH_TRACKER_ANDROID ((XrObjectType) 1000703000U)

XR_DEFINE_HANDLE(XrHandMeshTrackerANDROID)
#define XR_ANDROID_hand_mesh_SPEC_VERSION 1
#define XR_ANDROID_HAND_MESH_EXTENSION_NAME "XR_ANDROID_hand_mesh"
typedef struct XrSystemHandMeshTrackingPropertiesANDROID {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    XrBool32              supportsHandMeshTracking;
    XrBool32              supportsTextureUV;
    XrBool32              supportsVertexNormal;
} XrSystemHandMeshTrackingPropertiesANDROID;

typedef struct XrHandMeshTrackerCreateInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrHandMeshTrackerCreateInfoANDROID;

typedef struct XrHandMeshGetInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrSpace                     baseSpace;
    XrTime                      time;
} XrHandMeshGetInfoANDROID;

typedef struct XrHandMeshANDROID {
    XrBool32             isActive;
    XrTime               dynamicLastUpdateTime;
    uint32_t             indexCount;
    uint32_t             vertexCount;
    const uint32_t*      indices;
    const XrVector2f*    textureUVs;
    const XrVector3f*    positions;
    const XrVector3f*    normals;
    XrPosef              baseSpaceFromVertexSpace;
} XrHandMeshANDROID;

typedef struct XrHandTrackingMeshesANDROID {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    XrHandMeshANDROID     leftHandMesh;
    XrHandMeshANDROID     rightHandMesh;
} XrHandTrackingMeshesANDROID;

typedef XrResult (XRAPI_PTR *PFN_xrCreateHandMeshTrackerANDROID)(XrSession session, const XrHandMeshTrackerCreateInfoANDROID* createInfo, XrHandMeshTrackerANDROID* handMeshTracker);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyHandMeshTrackerANDROID)(XrHandMeshTrackerANDROID handMeshTracker);
typedef XrResult (XRAPI_PTR *PFN_xrGetHandMeshANDROID)(XrHandMeshTrackerANDROID handMeshTracker, const XrHandMeshGetInfoANDROID* getInfo, XrHandTrackingMeshesANDROID* handMeshes);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateHandMeshTrackerANDROID(
    XrSession                                   session,
    const XrHandMeshTrackerCreateInfoANDROID*   createInfo,
    XrHandMeshTrackerANDROID*                   handMeshTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyHandMeshTrackerANDROID(
    XrHandMeshTrackerANDROID                    handMeshTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrGetHandMeshANDROID(
    XrHandMeshTrackerANDROID                    handMeshTracker,
    const XrHandMeshGetInfoANDROID*             getInfo,
    XrHandTrackingMeshesANDROID*                handMeshes);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_STRUCT_XrSystemHandMeshTrackingPropertiesANDROID(_) \
    _(type) \
    _(next) \
    _(supportsHandMeshTracking) \
    _(supportsTextureUV) \
    _(supportsVertexNormal)

#define XR_LIST_STRUCT_XrHandMeshTrackerCreateInfoANDROID(_) \
    _(type) \
    _(next)

#define XR_LIST_STRUCT_XrHandMeshGetInfoANDROID(_) \
    _(type) \
    _(next) \
    _(baseSpace) \
    _(time)

#define XR_LIST_STRUCT_XrHandMeshANDROID(_) \
    _(isActive) \
    _(dynamicLastUpdateTime) \
    _(indexCount) \
    _(vertexCount) \
    _(indices) \
    _(textureUVs) \
    _(positions) \
    _(normals) \
    _(baseSpaceFromVertexSpace)

#define XR_LIST_STRUCT_XrHandTrackingMeshesANDROID(_) \
    _(type) \
    _(next) \
    _(leftHandMesh) \
    _(rightHandMesh)

#define XR_LIST_FUNCTIONS_XR_ANDROID_hand_mesh(_) \
    _(CreateHandMeshTrackerANDROID, ANDROID_hand_mesh) \
    _(DestroyHandMeshTrackerANDROID, ANDROID_hand_mesh) \
    _(GetHandMeshANDROID, ANDROID_hand_mesh)

#endif /* XR_ANDROID_hand_mesh */

#ifdef __cplusplus
}
#endif

#endif
