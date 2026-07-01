#ifndef XR_ANDROIDX1_SPATIAL_OCCUPANCY_GRID_H_
#define XR_ANDROIDX1_SPATIAL_OCCUPANCY_GRID_H_ 1

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


#ifndef XR_ANDROIDX1_spatial_occupancy_grid

// XR_ANDROIDX1_spatial_occupancy_grid is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX1_spatial_occupancy_grid 1
// Component that provides information of occupancy grid on the attached entity. Corresponding list structure is slink:XrSpatialComponentOccupancyGridListANDROIDX1; Corresponding data structure is slink:XrSpatialOccupancyGridDataANDROIDX1
#define XR_SPATIAL_COMPONENT_TYPE_OCCUPANCY_GRID_ANDROIDX1 ((XrSpatialComponentTypeEXT) 1000793000U)
#define XR_TYPE_SPATIAL_COMPONENT_OCCUPANCY_GRID_LIST_ANDROIDX1 ((XrStructureType) 1000793000U)

#define XR_ANDROIDX1_spatial_occupancy_grid_SPEC_VERSION 1
#define XR_ANDROIDX1_SPATIAL_OCCUPANCY_GRID_EXTENSION_NAME "XR_ANDROIDX1_spatial_occupancy_grid"
typedef struct XrSpatialOccupancyGridDataANDROIDX1 {
    XrPosef               originPose;
    float                 scale;
    XrSpatialBufferEXT    cellBuffer;
    XrSpatialBufferEXT    maxHeightBuffer;
} XrSpatialOccupancyGridDataANDROIDX1;

// XrSpatialComponentOccupancyGridListANDROIDX1 extends XrSpatialComponentDataQueryResultEXT
typedef struct XrSpatialComponentOccupancyGridListANDROIDX1 {
    XrStructureType                         type;
    void* XR_MAY_ALIAS                      next;
    uint32_t                                gridCount;
    XrSpatialOccupancyGridDataANDROIDX1*    occupancyGrids;
} XrSpatialComponentOccupancyGridListANDROIDX1;


// Reflection macros
#define XR_LIST_STRUCT_XrSpatialOccupancyGridDataANDROIDX1(_) \
    _(originPose) \
    _(scale) \
    _(cellBuffer) \
    _(maxHeightBuffer)

#define XR_LIST_STRUCT_XrSpatialComponentOccupancyGridListANDROIDX1(_) \
    _(type) \
    _(next) \
    _(gridCount) \
    _(occupancyGrids)

#endif /* XR_ANDROIDX1_spatial_occupancy_grid */

#ifdef __cplusplus
}
#endif

#endif
