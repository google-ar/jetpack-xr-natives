#ifndef XR_ANDROIDX_OCCUPANCY_GRID_H_
#define XR_ANDROIDX_OCCUPANCY_GRID_H_ 1

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


#ifndef XR_ANDROIDX_occupancy_grid

// XR_ANDROIDX_occupancy_grid is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX_occupancy_grid 1
#define XR_TYPE_OCCUPANCY_GRID_ANDROIDX   ((XrStructureType) 1000715000U)
#define XR_TYPE_PLANE_TRACKABLE_TRACKER_CREATE_INFO_ANDROIDX ((XrStructureType) 1000715001U)

#define XR_MAX_OCCUPANCY_GRID_CELLS_ANDROIDX 2500
#define XR_ANDROIDX_occupancy_grid_SPEC_VERSION 1
#define XR_ANDROIDX_OCCUPANCY_GRID_EXTENSION_NAME "XR_ANDROIDX_occupancy_grid"
typedef XrFlags64 XrPlaneTrackableTrackerFlagsANDROIDX;

// Flag bits for XrPlaneTrackableTrackerFlagsANDROIDX
// Flag to enable occupancy grid detection for plane trackable
static const XrPlaneTrackableTrackerFlagsANDROIDX XR_PLANE_TRACKABLE_TRACKER_FLAGS_ENABLE_OCCUPANCY_GRID_BIT_ANDROIDX = 0x00000001;

typedef struct XrOccupancyGridCellANDROIDX {
    uint8_t     row;
    uint8_t     col;
    uint16_t    max_height;
} XrOccupancyGridCellANDROIDX;

typedef struct XrOccupancyGridANDROIDX {
    XrStructureType                type;
    void* XR_MAY_ALIAS             next;
    XrPosef                        originPose;
    float                          resolution;
    XrTime                         lastUpdatedTime;
    uint32_t                       numGridCellsCapacityInput;
    uint32_t*                      numGridCellsCountOutput;
    XrOccupancyGridCellANDROIDX    occupancyGridCells[XR_MAX_OCCUPANCY_GRID_CELLS_ANDROIDX];
} XrOccupancyGridANDROIDX;

typedef struct XrPlaneTrackableTrackerCreateInfoANDROIDX {
    XrStructureType                         type;
    const void* XR_MAY_ALIAS                next;
    XrPlaneTrackableTrackerFlagsANDROIDX    createFlags;
} XrPlaneTrackableTrackerCreateInfoANDROIDX;

typedef XrResult (XRAPI_PTR *PFN_xrGetOccupancyGridForPlaneANDROIDX)(XrTrackableTrackerANDROID trackableTracker, const XrTrackableGetInfoANDROID* getInfo, XrOccupancyGridANDROIDX* occupancyGridOutput);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrGetOccupancyGridForPlaneANDROIDX(
    XrTrackableTrackerANDROID                   trackableTracker,
    const XrTrackableGetInfoANDROID*            getInfo,
    XrOccupancyGridANDROIDX*                    occupancyGridOutput);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_BITS_XrPlaneTrackableTrackerFlagsANDROIDX(_) \
    _(XR_PLANE_TRACKABLE_TRACKER_FLAGS_ENABLE_OCCUPANCY_GRID_BIT_ANDROIDX, 0x00000001)

#define XR_LIST_STRUCT_XrOccupancyGridCellANDROIDX(_) \
    _(row) \
    _(col) \
    _(max_height)

#define XR_LIST_STRUCT_XrOccupancyGridANDROIDX(_) \
    _(type) \
    _(next) \
    _(originPose) \
    _(resolution) \
    _(lastUpdatedTime) \
    _(numGridCellsCapacityInput) \
    _(numGridCellsCountOutput) \
    _(occupancyGridCells)

#define XR_LIST_STRUCT_XrPlaneTrackableTrackerCreateInfoANDROIDX(_) \
    _(type) \
    _(next) \
    _(createFlags)

#define XR_LIST_FUNCTIONS_XR_ANDROIDX_occupancy_grid(_) \
    _(GetOccupancyGridForPlaneANDROIDX, ANDROIDX_occupancy_grid)

#endif /* XR_ANDROIDX_occupancy_grid */

#ifdef __cplusplus
}
#endif

#endif
