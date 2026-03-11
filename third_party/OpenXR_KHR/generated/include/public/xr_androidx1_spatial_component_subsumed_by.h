#ifndef XR_ANDROIDX1_SPATIAL_COMPONENT_SUBSUMED_BY_H_
#define XR_ANDROIDX1_SPATIAL_COMPONENT_SUBSUMED_BY_H_ 1

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


#ifndef XR_ANDROIDX1_spatial_component_subsumed_by

// XR_ANDROIDX1_spatial_component_subsumed_by is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX1_spatial_component_subsumed_by 1
#define XR_ANDROIDX1_spatial_component_subsumed_by_SPEC_VERSION 1
#define XR_ANDROIDX1_SPATIAL_COMPONENT_SUBSUMED_BY_EXTENSION_NAME "XR_ANDROIDX1_spatial_component_subsumed_by"
// Component that provides entity ID of the entity subsuming the attached entity. Corresponding list structure is slink:XrSpatialComponentSubsumedByListANDROIDX1; Corresponding data structure is basetype:XrSpatialEntityIdEXT
#define XR_SPATIAL_COMPONENT_TYPE_SUBSUMED_BY_ANDROIDX1 ((XrSpatialComponentTypeEXT) 1000791000U)
#define XR_TYPE_SPATIAL_DISCOVERY_NOT_SUBSUMED_FILTER_ANDROIDX1 ((XrStructureType) 1000791001U)
#define XR_TYPE_SPATIAL_COMPONENT_SUBSUMED_BY_LIST_ANDROIDX1 ((XrStructureType) 1000791002U)
// XrSpatialDiscoveryNotSubsumedFilterANDROIDX1 extends XrSpatialDiscoverySnapshotCreateInfoEXT
typedef struct XrSpatialDiscoveryNotSubsumedFilterANDROIDX1 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrSpatialDiscoveryNotSubsumedFilterANDROIDX1;

// XrSpatialComponentSubsumedByListANDROIDX1 extends XrSpatialComponentDataQueryResultEXT
typedef struct XrSpatialComponentSubsumedByListANDROIDX1 {
    XrStructureType          type;
    void* XR_MAY_ALIAS       next;
    uint32_t                 subsumedByCount;
    XrSpatialEntityIdEXT*    subsumedByIds;
} XrSpatialComponentSubsumedByListANDROIDX1;

#endif /* XR_ANDROIDX1_spatial_component_subsumed_by */

#ifdef __cplusplus
}
#endif

#endif
