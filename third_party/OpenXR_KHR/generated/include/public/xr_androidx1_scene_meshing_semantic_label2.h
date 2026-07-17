#ifndef XR_ANDROIDX1_SCENE_MESHING_SEMANTIC_LABEL2_H_
#define XR_ANDROIDX1_SCENE_MESHING_SEMANTIC_LABEL2_H_ 1

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


#ifndef XR_ANDROIDX1_scene_meshing_semantic_label2

// XR_ANDROIDX1_scene_meshing_semantic_label2 is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX1_scene_meshing_semantic_label2 1
#define XR_ANDROIDX1_scene_meshing_semantic_label2_SPEC_VERSION 1
#define XR_ANDROIDX1_SCENE_MESHING_SEMANTIC_LABEL2_EXTENSION_NAME "XR_ANDROIDX1_scene_meshing_semantic_label2"
// This semantic label set represents XrSceneMeshSemanticLabel2ANDROIDX1.
#define XR_SCENE_MESH_SEMANTIC_LABEL_SET_DEFAULT2_ANDROIDX1 ((XrSceneMeshSemanticLabelSetANDROID) 1000801000U)

typedef enum XrSceneMeshSemanticLabel2ANDROIDX1 {
    // This semantic indicates that the corresponding mesh element represents an unknown object.
    XR_SCENE_MESH_SEMANTIC_LABEL2_OTHER_ANDROIDX1 = 0,
    // This semantic indicates that the corresponding mesh element represents a floor.
    XR_SCENE_MESH_SEMANTIC_LABEL2_FLOOR_ANDROIDX1 = 1,
    // This semantic indicates that the corresponding mesh element represents a ceiling.
    XR_SCENE_MESH_SEMANTIC_LABEL2_CEILING_ANDROIDX1 = 2,
    // This semantic indicates that the corresponding mesh element represents a wall.
    XR_SCENE_MESH_SEMANTIC_LABEL2_WALL_ANDROIDX1 = 3,
    // This semantic indicates that the corresponding mesh element represents a table.
    XR_SCENE_MESH_SEMANTIC_LABEL2_TABLE_ANDROIDX1 = 4,
    // This semantic indicates that the corresponding mesh element represents a chair.
    XR_SCENE_MESH_SEMANTIC_LABEL2_CHAIR_ANDROIDX1 = 5,
    // This semantic indicates that the corresponding mesh element represents a sofa.
    XR_SCENE_MESH_SEMANTIC_LABEL2_SOFA_ANDROIDX1 = 6,
    // This semantic indicates that the corresponding mesh element represents a bed.
    XR_SCENE_MESH_SEMANTIC_LABEL2_BED_ANDROIDX1 = 7,
    XR_SCENE_MESH_SEMANTIC_LABEL_2ANDROIDX1_MAX_ENUM = 0x7FFFFFFF
} XrSceneMeshSemanticLabel2ANDROIDX1;
#endif /* XR_ANDROIDX1_scene_meshing_semantic_label2 */

#ifdef __cplusplus
}
#endif

#endif
