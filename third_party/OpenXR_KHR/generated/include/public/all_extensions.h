#ifndef OPENXR_EXTENSIONS_HELPER_PUBLIC_H_
#define OPENXR_EXTENSIONS_HELPER_PUBLIC_H_

// IWYU pragma: begin_exports

/* These include helps graduating extensions into openxr.h */
#include <openxr/openxr.h>

#include <openxr/public/xr_android_enumerate_system_extension_properties.h>
#include <openxr/public/xr_android_face_tracking_data_source.h>
#include <openxr/public/xr_android_geospatial.h>
#include <openxr/public/xr_android_geospatial_anchor.h>
#include <openxr/public/xr_android_global_passthrough_dimming.h>
#include <openxr/public/xr_android_google_cloud_auth.h>
#include <openxr/public/xr_android_spatial_anchor_space.h>

// IWYU pragma: end_exports

// These extensions are experimental and MUST be included explicitly by code
// that needs them to avoid breaks on updates.
// Do not rely on this header to include them.
#ifdef DO_NOT_USE_INCLUDE_LATEST_EXPERIMENTAL_EXTENSIONS
#include <openxr/public/xr_androidx1_body_tracking.h>
#include <openxr/public/xr_androidx1_eye_tracking_calibration_state.h>
#include <openxr/public/xr_androidx1_spatial_occupancy_grid.h>
#include <openxr/public/xr_androidx2_geospatial_streetscape.h>
#include <openxr/public/xr_androidx_spatial_interaction.h>
#include <openxr/public/xr_androidx_system_state.h>
#endif  // DO_NOT_USE_INCLUDE_LATEST_EXPERIMENTAL_EXTENSIONS

// This macro extends the base XR_LIST_EXTENSIONS macro with the extensions in
// this header.
#define XR_LIST_PUBLIC_EXTENSIONS(_) \
    _(XR_ANDROIDX_system_state, 455)\
    _(XR_ANDROIDX_spatial_interaction, 706)\
    _(XR_ANDROID_face_tracking_data_source, 707)\
    _(XR_ANDROIDX1_body_tracking, 717)\
    _(XR_ANDROIDX1_eye_tracking_calibration_state, 723)\
    _(XR_ANDROID_enumerate_system_extension_properties, 725)\
    _(XR_ANDROID_google_cloud_auth, 788)\
    _(XR_ANDROID_geospatial, 790)\
    _(XR_ANDROIDX1_spatial_occupancy_grid, 794)\
    _(XR_ANDROID_spatial_anchor_space, 796)\
    _(XR_ANDROID_global_passthrough_dimming, 797)\
    _(XR_ANDROID_geospatial_anchor, 798)\
    _(XR_ANDROIDX2_geospatial_streetscape, 799)\

#endif  // OPENXR_EXTENSIONS_HELPER_PUBLIC_H_
