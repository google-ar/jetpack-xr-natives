#ifndef XR_ANDROIDX1_SPATIAL_MESH_RAYCAST_H_
#define XR_ANDROIDX1_SPATIAL_MESH_RAYCAST_H_ 1

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


#ifndef XR_ANDROIDX1_spatial_mesh_raycast

// XR_ANDROIDX1_spatial_mesh_raycast is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX1_spatial_mesh_raycast 1
// Mesh raycast capability
#define XR_SPATIAL_CAPABILITY_MESH_RAYCAST_ANDROIDX1 ((XrSpatialCapabilityEXT) 1000803000U)
// Component that identifies an entity as a mesh point. There is no corresponding list or data structure for this component type. The presence of this component is merely indicative of an entity's ability to be attachable to an anchor.
#define XR_SPATIAL_COMPONENT_TYPE_MESH_POINT_ANDROIDX1 ((XrSpatialComponentTypeEXT) 1000803000U)
#define XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_MESH_RAYCAST_ANDROIDX1 ((XrStructureType) 1000803000U)

#define XR_ANDROIDX1_spatial_mesh_raycast_SPEC_VERSION 1
#define XR_ANDROIDX1_SPATIAL_MESH_RAYCAST_EXTENSION_NAME "XR_ANDROIDX1_spatial_mesh_raycast"
typedef struct XrSpatialCapabilityConfigurationMeshRaycastANDROIDX1 {
    XrStructureType                     type;
    const void* XR_MAY_ALIAS            next;
    XrSpatialCapabilityEXT              capability;
    uint32_t                            enabledComponentCount;
    const XrSpatialComponentTypeEXT*    enabledComponents;
} XrSpatialCapabilityConfigurationMeshRaycastANDROIDX1;


// Reflection macros
#define XR_LIST_STRUCT_XrSpatialCapabilityConfigurationMeshRaycastANDROIDX1(_) \
    _(type) \
    _(next) \
    _(capability) \
    _(enabledComponentCount) \
    _(enabledComponents)

#endif /* XR_ANDROIDX1_spatial_mesh_raycast */

#ifdef __cplusplus
}
#endif

#endif
