#ifndef OPENXR_EXTENSIONS_HELPER_PUBLIC_H_
#define OPENXR_EXTENSIONS_HELPER_PUBLIC_H_

// IWYU pragma: begin_exports

/* These include helps graduating extensions into openxr.h */
#include <openxr/openxr.h>

#include <openxr/public/xr_androidsys_body_tracking.h>
#include <openxr/public/xr_androidsys_face_tracking_calibration.h>
#include <openxr/public/xr_androidsys_fit_tracking.h>
#include <openxr/public/xr_androidsys_hand_tracking_parameters.h>
#include <openxr/public/xr_androidsys_headtracking_error_cause.h>
#include <openxr/public/xr_androidsys_ipd_calibration.h>
#include <openxr/public/xr_androidsys_pca_face_tracking.h>
#include <openxr/public/xr_androidsys_trackables_shoebox.h>
#include <openxr/public/xr_android_depth_texture.h>
#include <openxr/public/xr_android_enumerate_system_extension_properties.h>
#include <openxr/public/xr_android_face_tracking_data_source.h>
#include <openxr/public/xr_android_geospatial.h>
#include <openxr/public/xr_android_geospatial_anchor.h>
#include <openxr/public/xr_android_global_passthrough_dimming.h>
#include <openxr/public/xr_android_google_cloud_auth.h>
#include <openxr/public/xr_android_hand_mesh.h>
#include <openxr/public/xr_android_light_estimation_cubemap.h>
#include <openxr/public/xr_android_recommended_settings.h>
#include <openxr/public/xr_android_surface_control_swapchain.h>
#include <openxr/public/xr_android_viewport_feathering.h>
#include <openxr/public/xr_androidsys_eye_tracking_calibration.h>


// These extensions are experimental and MUST be included explicitly by code
// that needs them to avoid breaks on updates.
// Do not rely on this header to include them.
#ifdef DO_NOT_USE_INCLUDE_LATEST_EXPERIMENTAL_EXTENSIONS
#include <openxr/public/xr_androidx1_body_tracking.h>
#include <openxr/public/xr_androidx1_eye_tracking_calibration_state.h>
#include <openxr/public/xr_androidx1_light_estimation_point_lights.h>
#include <openxr/public/xr_androidx1_scene_meshing_semantic_label2.h>
#include <openxr/public/xr_androidx1_spatial_mesh_raycast.h>
#include <openxr/public/xr_androidx1_spatial_occupancy_grid.h>
#include <openxr/public/xr_androidx2_geospatial_streetscape.h>
#include <openxr/public/xr_androidx_android_surface_swapchain_acquire_image.h>
#include <openxr/public/xr_androidx_composition_layer_axis_aligned_distortion.h>
#include <openxr/public/xr_androidx_occupancy_grid.h>
#include <openxr/public/xr_androidx_spatial_interaction.h>
#include <openxr/public/xr_androidx_spatial_interaction_lifecycle.h>
#endif  // DO_NOT_USE_INCLUDE_LATEST_EXPERIMENTAL_EXTENSIONS

// IWYU pragma: end_exports

// This macro extends the base XR_LIST_EXTENSIONS macro with the extensions in
// this header.
#define XR_LIST_PUBLIC_EXTENSIONS(_) \
    _(XR_ANDROID_recommended_settings, 455)\
    _(XR_ANDROID_surface_control_swapchain, 460)\
    _(XR_ANDROIDSYS_hand_tracking_parameters, 465)\
    _(XR_ANDROIDSYS_fit_tracking, 469)\
    _(XR_ANDROID_depth_texture, 703)\
    _(XR_ANDROID_hand_mesh, 704)\
    _(XR_ANDROIDX_spatial_interaction, 706)\
    _(XR_ANDROID_face_tracking_data_source, 707)\
    _(XR_ANDROIDX_occupancy_grid, 716)\
    _(XR_ANDROIDX1_body_tracking, 717)\
    _(XR_ANDROIDSYS_body_tracking, 718)\
    _(XR_ANDROIDSYS_ipd_calibration, 720)\
    _(XR_ANDROIDSYS_eye_tracking_calibration, 721)\
    _(XR_ANDROID_light_estimation_cubemap, 722)\
    _(XR_ANDROIDX1_eye_tracking_calibration_state, 723)\
    _(XR_ANDROIDX_spatial_interaction_lifecycle, 724)\
    _(XR_ANDROID_enumerate_system_extension_properties, 725)\
    _(XR_ANDROIDSYS_face_tracking_calibration, 728)\
    _(XR_ANDROIDSYS_trackables_shoebox, 729)\
    _(XR_ANDROIDSYS_pca_face_tracking, 731)\
    _(XR_ANDROIDX1_light_estimation_point_lights, 732)\
    _(XR_ANDROIDX_android_surface_swapchain_acquire_image, 733)\
    _(XR_ANDROIDX_composition_layer_axis_aligned_distortion, 734)\
    _(XR_ANDROIDSYS_headtracking_error_cause, 735)\
    _(XR_ANDROID_google_cloud_auth, 788)\
    _(XR_ANDROID_geospatial, 790)\
    _(XR_ANDROIDX1_spatial_occupancy_grid, 794)\
    _(XR_ANDROID_global_passthrough_dimming, 797)\
    _(XR_ANDROID_geospatial_anchor, 798)\
    _(XR_ANDROIDX2_geospatial_streetscape, 799)\
    _(XR_ANDROIDX1_scene_meshing_semantic_label2, 802)\
    _(XR_ANDROIDX1_spatial_mesh_raycast, 804)\
    _(XR_ANDROID_viewport_feathering, 807)\

#endif  // OPENXR_EXTENSIONS_HELPER_PUBLIC_H_
