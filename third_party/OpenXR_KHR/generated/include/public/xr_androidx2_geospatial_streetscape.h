#ifndef XR_ANDROIDX2_GEOSPATIAL_STREETSCAPE_H_
#define XR_ANDROIDX2_GEOSPATIAL_STREETSCAPE_H_ 1

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


#ifndef XR_ANDROIDX2_geospatial_streetscape

// XR_ANDROIDX2_geospatial_streetscape is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX2_geospatial_streetscape 1
#define XR_ANDROIDX2_geospatial_streetscape_SPEC_VERSION 1
#define XR_ANDROIDX2_GEOSPATIAL_STREETSCAPE_EXTENSION_NAME "XR_ANDROIDX2_geospatial_streetscape"
#define XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_STREETSCAPE_GEOMETRY_ANDROIDX2 ((XrStructureType) 1000798000U)
#define XR_TYPE_SPATIAL_COMPONENT_STREETSCAPE_GEOMETRY_METADATA_LIST_ANDROIDX2 ((XrStructureType) 1000798001U)
// Streetscape geometry
#define XR_SPATIAL_CAPABILITY_STREETSCAPE_GEOMETRY_ANDROIDX2 ((XrSpatialCapabilityEXT) 1000798000U)
// Component that provides the metadata for a streetscape geometry; Corresponding list structure is slink:XrSpatialComponentStreetscapeGeometryMetadataListANDROIDX2; Corresponding data structure is slink:XrSpatialStreetscapeGeometryMetadataANDROIDX2
#define XR_SPATIAL_COMPONENT_TYPE_STREETSCAPE_GEOMETRY_METADATA_ANDROIDX2 ((XrSpatialComponentTypeEXT) 1000798000U)

typedef enum XrStreetscapeGeometrySemanticLabelANDROIDX2 {
    // This geometry represents the ground or floor.
    XR_STREETSCAPE_GEOMETRY_SEMANTIC_LABEL_TERRAIN_ANDROIDX2 = 1,
    // This geometry represents a building or other structure.
    XR_STREETSCAPE_GEOMETRY_SEMANTIC_LABEL_BUILDING_ANDROIDX2 = 2,
    XR_STREETSCAPE_GEOMETRY_SEMANTIC_LABEL_ANDROIDX2_MAX_ENUM = 0x7FFFFFFF
} XrStreetscapeGeometrySemanticLabelANDROIDX2;

typedef enum XrStreetscapeGeometryQualityANDROIDX2 {
    // The quality of the geometry is not defined, e.g. when the geometryType is ename:XR_STREETSCAPE_GEOMETRY_TYPE_TERRAIN_ANDROIDX2.
    XR_STREETSCAPE_GEOMETRY_QUALITY_NONE_ANDROIDX2 = 0,
    // The geometry of type ename:XR_STREETSCAPE_GEOMETRY_TYPE_BUILDING_ANDROIDX2 is the building footprint extruded up to a single flat top. The building mesh contains empty space above any angled roofs.
    XR_STREETSCAPE_GEOMETRY_QUALITY_BUILDING_LOD1_ANDROIDX2 = 1,
    // The geometry of type ename:XR_STREETSCAPE_GEOMETRY_TYPE_BUILDING_ANDROIDX2 is the building footprint with a rough heightmap. The geometry will closely follow simple angled roofs. Chimneys and roof vents on top of roofs will poke outside of the mesh.
    XR_STREETSCAPE_GEOMETRY_QUALITY_BUILDING_LOD2_ANDROIDX2 = 2,
    XR_STREETSCAPE_GEOMETRY_QUALITY_ANDROIDX2_MAX_ENUM = 0x7FFFFFFF
} XrStreetscapeGeometryQualityANDROIDX2;
typedef struct XrSpatialCapabilityConfigurationStreetscapeGeometryANDROIDX2 {
    XrStructureType                     type;
    const void* XR_MAY_ALIAS            next;
    XrSpatialCapabilityEXT              capability;
    uint32_t                            enabledComponentCount;
    const XrSpatialComponentTypeEXT*    enabledComponents;
} XrSpatialCapabilityConfigurationStreetscapeGeometryANDROIDX2;

typedef struct XrSpatialStreetscapeGeometryMetadataANDROIDX2 {
    XrStreetscapeGeometrySemanticLabelANDROIDX2    semanticLabel;
    XrStreetscapeGeometryQualityANDROIDX2          quality;
} XrSpatialStreetscapeGeometryMetadataANDROIDX2;

// XrSpatialComponentStreetscapeGeometryMetadataListANDROIDX2 extends XrSpatialComponentDataQueryResultEXT
typedef struct XrSpatialComponentStreetscapeGeometryMetadataListANDROIDX2 {
    XrStructureType                                   type;
    void* XR_MAY_ALIAS                                next;
    uint32_t                                          metadataCount;
    XrSpatialStreetscapeGeometryMetadataANDROIDX2*    metadatas;
} XrSpatialComponentStreetscapeGeometryMetadataListANDROIDX2;

#endif /* XR_ANDROIDX2_geospatial_streetscape */

#ifdef __cplusplus
}
#endif

#endif
