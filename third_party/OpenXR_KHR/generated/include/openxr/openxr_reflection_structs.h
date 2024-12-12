#ifndef OPENXR_REFLECTION_STRUCTS_H_
#define OPENXR_REFLECTION_STRUCTS_H_ 1

/*
** Copyright (c) 2017-2024, The Khronos Group Inc.
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

/*
** This header is generated from the Khronos OpenXR XML API Registry.
**
*/

#include "openxr.h"

/*
This file contains expansion macros (X Macros) for OpenXR structures.
*/



/// Calls one of your macros with the structure type name and the XrStructureType constant for
/// each known structure type. The first macro (_avail) is called for those that are available,
/// while the second macro (_unavail) is called for those unavailable due to preprocessor definitions.
#define XR_LIST_ALL_STRUCTURE_TYPES(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_CORE(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_D3D11(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_D3D12(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_METAL(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_WAYLAND(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_WIN32(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_XCB(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_XLIB(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_ES(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_ES_XR_USE_PLATFORM_ANDROID(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_VULKAN(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_ANDROID(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_EGL(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_ML(_avail, _unavail) \
    _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_WIN32(_avail, _unavail) \


// Implementation detail of XR_LIST_ALL_STRUCTURE_TYPES()
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_CORE(_avail, _unavail) \
    _avail(XrApiLayerProperties, XR_TYPE_API_LAYER_PROPERTIES) \
    _avail(XrExtensionProperties, XR_TYPE_EXTENSION_PROPERTIES) \
    _avail(XrInstanceCreateInfo, XR_TYPE_INSTANCE_CREATE_INFO) \
    _avail(XrInstanceProperties, XR_TYPE_INSTANCE_PROPERTIES) \
    _avail(XrEventDataBuffer, XR_TYPE_EVENT_DATA_BUFFER) \
    _avail(XrSystemGetInfo, XR_TYPE_SYSTEM_GET_INFO) \
    _avail(XrSystemProperties, XR_TYPE_SYSTEM_PROPERTIES) \
    _avail(XrSessionCreateInfo, XR_TYPE_SESSION_CREATE_INFO) \
    _avail(XrSpaceVelocity, XR_TYPE_SPACE_VELOCITY) \
    _avail(XrReferenceSpaceCreateInfo, XR_TYPE_REFERENCE_SPACE_CREATE_INFO) \
    _avail(XrActionSpaceCreateInfo, XR_TYPE_ACTION_SPACE_CREATE_INFO) \
    _avail(XrSpaceLocation, XR_TYPE_SPACE_LOCATION) \
    _avail(XrViewConfigurationProperties, XR_TYPE_VIEW_CONFIGURATION_PROPERTIES) \
    _avail(XrViewConfigurationView, XR_TYPE_VIEW_CONFIGURATION_VIEW) \
    _avail(XrSwapchainCreateInfo, XR_TYPE_SWAPCHAIN_CREATE_INFO) \
    _avail(XrSwapchainImageAcquireInfo, XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO) \
    _avail(XrSwapchainImageWaitInfo, XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO) \
    _avail(XrSwapchainImageReleaseInfo, XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO) \
    _avail(XrSessionBeginInfo, XR_TYPE_SESSION_BEGIN_INFO) \
    _avail(XrFrameWaitInfo, XR_TYPE_FRAME_WAIT_INFO) \
    _avail(XrFrameState, XR_TYPE_FRAME_STATE) \
    _avail(XrFrameBeginInfo, XR_TYPE_FRAME_BEGIN_INFO) \
    _avail(XrFrameEndInfo, XR_TYPE_FRAME_END_INFO) \
    _avail(XrViewLocateInfo, XR_TYPE_VIEW_LOCATE_INFO) \
    _avail(XrViewState, XR_TYPE_VIEW_STATE) \
    _avail(XrView, XR_TYPE_VIEW) \
    _avail(XrActionSetCreateInfo, XR_TYPE_ACTION_SET_CREATE_INFO) \
    _avail(XrActionCreateInfo, XR_TYPE_ACTION_CREATE_INFO) \
    _avail(XrInteractionProfileSuggestedBinding, XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING) \
    _avail(XrSessionActionSetsAttachInfo, XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO) \
    _avail(XrInteractionProfileState, XR_TYPE_INTERACTION_PROFILE_STATE) \
    _avail(XrActionStateGetInfo, XR_TYPE_ACTION_STATE_GET_INFO) \
    _avail(XrActionStateBoolean, XR_TYPE_ACTION_STATE_BOOLEAN) \
    _avail(XrActionStateFloat, XR_TYPE_ACTION_STATE_FLOAT) \
    _avail(XrActionStateVector2f, XR_TYPE_ACTION_STATE_VECTOR2F) \
    _avail(XrActionStatePose, XR_TYPE_ACTION_STATE_POSE) \
    _avail(XrActionsSyncInfo, XR_TYPE_ACTIONS_SYNC_INFO) \
    _avail(XrBoundSourcesForActionEnumerateInfo, XR_TYPE_BOUND_SOURCES_FOR_ACTION_ENUMERATE_INFO) \
    _avail(XrInputSourceLocalizedNameGetInfo, XR_TYPE_INPUT_SOURCE_LOCALIZED_NAME_GET_INFO) \
    _avail(XrHapticActionInfo, XR_TYPE_HAPTIC_ACTION_INFO) \
    _avail(XrCompositionLayerProjectionView, XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW) \
    _avail(XrCompositionLayerProjection, XR_TYPE_COMPOSITION_LAYER_PROJECTION) \
    _avail(XrCompositionLayerQuad, XR_TYPE_COMPOSITION_LAYER_QUAD) \
    _avail(XrEventDataEventsLost, XR_TYPE_EVENT_DATA_EVENTS_LOST) \
    _avail(XrEventDataInstanceLossPending, XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) \
    _avail(XrEventDataSessionStateChanged, XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) \
    _avail(XrEventDataReferenceSpaceChangePending, XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING) \
    _avail(XrEventDataInteractionProfileChanged, XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED) \
    _avail(XrHapticVibration, XR_TYPE_HAPTIC_VIBRATION) \
    _avail(XrSpacesLocateInfo, XR_TYPE_SPACES_LOCATE_INFO) \
    _avail(XrSpaceLocations, XR_TYPE_SPACE_LOCATIONS) \
    _avail(XrSpaceVelocities, XR_TYPE_SPACE_VELOCITIES) \
    _avail(XrCompositionLayerCubeKHR, XR_TYPE_COMPOSITION_LAYER_CUBE_KHR) \
    _avail(XrCompositionLayerDepthInfoKHR, XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR) \
    _avail(XrCompositionLayerCylinderKHR, XR_TYPE_COMPOSITION_LAYER_CYLINDER_KHR) \
    _avail(XrCompositionLayerEquirectKHR, XR_TYPE_COMPOSITION_LAYER_EQUIRECT_KHR) \
    _avail(XrVisibilityMaskKHR, XR_TYPE_VISIBILITY_MASK_KHR) \
    _avail(XrEventDataVisibilityMaskChangedKHR, XR_TYPE_EVENT_DATA_VISIBILITY_MASK_CHANGED_KHR) \
    _avail(XrCompositionLayerColorScaleBiasKHR, XR_TYPE_COMPOSITION_LAYER_COLOR_SCALE_BIAS_KHR) \
    _avail(XrCompositionLayerEquirect2KHR, XR_TYPE_COMPOSITION_LAYER_EQUIRECT2_KHR) \
    _avail(XrBindingModificationsKHR, XR_TYPE_BINDING_MODIFICATIONS_KHR) \
    _avail(XrEventDataPerfSettingsEXT, XR_TYPE_EVENT_DATA_PERF_SETTINGS_EXT) \
    _avail(XrDebugUtilsObjectNameInfoEXT, XR_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT) \
    _avail(XrDebugUtilsLabelEXT, XR_TYPE_DEBUG_UTILS_LABEL_EXT) \
    _avail(XrDebugUtilsMessengerCallbackDataEXT, XR_TYPE_DEBUG_UTILS_MESSENGER_CALLBACK_DATA_EXT) \
    _avail(XrDebugUtilsMessengerCreateInfoEXT, XR_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT) \
    _avail(XrSystemEyeGazeInteractionPropertiesEXT, XR_TYPE_SYSTEM_EYE_GAZE_INTERACTION_PROPERTIES_EXT) \
    _avail(XrEyeGazeSampleTimeEXT, XR_TYPE_EYE_GAZE_SAMPLE_TIME_EXT) \
    _avail(XrSessionCreateInfoOverlayEXTX, XR_TYPE_SESSION_CREATE_INFO_OVERLAY_EXTX) \
    _avail(XrEventDataMainSessionVisibilityChangedEXTX, XR_TYPE_EVENT_DATA_MAIN_SESSION_VISIBILITY_CHANGED_EXTX) \
    _avail(XrSpatialAnchorCreateInfoMSFT, XR_TYPE_SPATIAL_ANCHOR_CREATE_INFO_MSFT) \
    _avail(XrSpatialAnchorSpaceCreateInfoMSFT, XR_TYPE_SPATIAL_ANCHOR_SPACE_CREATE_INFO_MSFT) \
    _avail(XrCompositionLayerImageLayoutFB, XR_TYPE_COMPOSITION_LAYER_IMAGE_LAYOUT_FB) \
    _avail(XrCompositionLayerAlphaBlendFB, XR_TYPE_COMPOSITION_LAYER_ALPHA_BLEND_FB) \
    _avail(XrViewConfigurationDepthRangeEXT, XR_TYPE_VIEW_CONFIGURATION_DEPTH_RANGE_EXT) \
    _avail(XrSpatialGraphNodeSpaceCreateInfoMSFT, XR_TYPE_SPATIAL_GRAPH_NODE_SPACE_CREATE_INFO_MSFT) \
    _avail(XrSpatialGraphStaticNodeBindingCreateInfoMSFT, XR_TYPE_SPATIAL_GRAPH_STATIC_NODE_BINDING_CREATE_INFO_MSFT) \
    _avail(XrSpatialGraphNodeBindingPropertiesGetInfoMSFT, XR_TYPE_SPATIAL_GRAPH_NODE_BINDING_PROPERTIES_GET_INFO_MSFT) \
    _avail(XrSpatialGraphNodeBindingPropertiesMSFT, XR_TYPE_SPATIAL_GRAPH_NODE_BINDING_PROPERTIES_MSFT) \
    _avail(XrSystemHandTrackingPropertiesEXT, XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT) \
    _avail(XrHandTrackerCreateInfoEXT, XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT) \
    _avail(XrHandJointsLocateInfoEXT, XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT) \
    _avail(XrHandJointLocationsEXT, XR_TYPE_HAND_JOINT_LOCATIONS_EXT) \
    _avail(XrHandJointVelocitiesEXT, XR_TYPE_HAND_JOINT_VELOCITIES_EXT) \
    _avail(XrSystemHandTrackingMeshPropertiesMSFT, XR_TYPE_SYSTEM_HAND_TRACKING_MESH_PROPERTIES_MSFT) \
    _avail(XrHandMeshSpaceCreateInfoMSFT, XR_TYPE_HAND_MESH_SPACE_CREATE_INFO_MSFT) \
    _avail(XrHandMeshUpdateInfoMSFT, XR_TYPE_HAND_MESH_UPDATE_INFO_MSFT) \
    _avail(XrHandMeshMSFT, XR_TYPE_HAND_MESH_MSFT) \
    _avail(XrHandPoseTypeInfoMSFT, XR_TYPE_HAND_POSE_TYPE_INFO_MSFT) \
    _avail(XrSecondaryViewConfigurationSessionBeginInfoMSFT, XR_TYPE_SECONDARY_VIEW_CONFIGURATION_SESSION_BEGIN_INFO_MSFT) \
    _avail(XrSecondaryViewConfigurationStateMSFT, XR_TYPE_SECONDARY_VIEW_CONFIGURATION_STATE_MSFT) \
    _avail(XrSecondaryViewConfigurationFrameStateMSFT, XR_TYPE_SECONDARY_VIEW_CONFIGURATION_FRAME_STATE_MSFT) \
    _avail(XrSecondaryViewConfigurationLayerInfoMSFT, XR_TYPE_SECONDARY_VIEW_CONFIGURATION_LAYER_INFO_MSFT) \
    _avail(XrSecondaryViewConfigurationFrameEndInfoMSFT, XR_TYPE_SECONDARY_VIEW_CONFIGURATION_FRAME_END_INFO_MSFT) \
    _avail(XrSecondaryViewConfigurationSwapchainCreateInfoMSFT, XR_TYPE_SECONDARY_VIEW_CONFIGURATION_SWAPCHAIN_CREATE_INFO_MSFT) \
    _avail(XrControllerModelKeyStateMSFT, XR_TYPE_CONTROLLER_MODEL_KEY_STATE_MSFT) \
    _avail(XrControllerModelNodePropertiesMSFT, XR_TYPE_CONTROLLER_MODEL_NODE_PROPERTIES_MSFT) \
    _avail(XrControllerModelPropertiesMSFT, XR_TYPE_CONTROLLER_MODEL_PROPERTIES_MSFT) \
    _avail(XrControllerModelNodeStateMSFT, XR_TYPE_CONTROLLER_MODEL_NODE_STATE_MSFT) \
    _avail(XrControllerModelStateMSFT, XR_TYPE_CONTROLLER_MODEL_STATE_MSFT) \
    _avail(XrViewConfigurationViewFovEPIC, XR_TYPE_VIEW_CONFIGURATION_VIEW_FOV_EPIC) \
    _avail(XrCompositionLayerReprojectionInfoMSFT, XR_TYPE_COMPOSITION_LAYER_REPROJECTION_INFO_MSFT) \
    _avail(XrCompositionLayerReprojectionPlaneOverrideMSFT, XR_TYPE_COMPOSITION_LAYER_REPROJECTION_PLANE_OVERRIDE_MSFT) \
    _avail(XrCompositionLayerSecureContentFB, XR_TYPE_COMPOSITION_LAYER_SECURE_CONTENT_FB) \
    _avail(XrSystemBodyTrackingPropertiesFB, XR_TYPE_SYSTEM_BODY_TRACKING_PROPERTIES_FB) \
    _avail(XrBodyTrackerCreateInfoFB, XR_TYPE_BODY_TRACKER_CREATE_INFO_FB) \
    _avail(XrBodySkeletonFB, XR_TYPE_BODY_SKELETON_FB) \
    _avail(XrBodyJointsLocateInfoFB, XR_TYPE_BODY_JOINTS_LOCATE_INFO_FB) \
    _avail(XrBodyJointLocationsFB, XR_TYPE_BODY_JOINT_LOCATIONS_FB) \
    _avail(XrInteractionProfileDpadBindingEXT, XR_TYPE_INTERACTION_PROFILE_DPAD_BINDING_EXT) \
    _avail(XrInteractionProfileAnalogThresholdVALVE, XR_TYPE_INTERACTION_PROFILE_ANALOG_THRESHOLD_VALVE) \
    _avail(XrHandJointsMotionRangeInfoEXT, XR_TYPE_HAND_JOINTS_MOTION_RANGE_INFO_EXT) \
    _avail(XrSceneObserverCreateInfoMSFT, XR_TYPE_SCENE_OBSERVER_CREATE_INFO_MSFT) \
    _avail(XrSceneCreateInfoMSFT, XR_TYPE_SCENE_CREATE_INFO_MSFT) \
    _avail(XrNewSceneComputeInfoMSFT, XR_TYPE_NEW_SCENE_COMPUTE_INFO_MSFT) \
    _avail(XrVisualMeshComputeLodInfoMSFT, XR_TYPE_VISUAL_MESH_COMPUTE_LOD_INFO_MSFT) \
    _avail(XrSceneComponentsMSFT, XR_TYPE_SCENE_COMPONENTS_MSFT) \
    _avail(XrSceneComponentsGetInfoMSFT, XR_TYPE_SCENE_COMPONENTS_GET_INFO_MSFT) \
    _avail(XrSceneComponentLocationsMSFT, XR_TYPE_SCENE_COMPONENT_LOCATIONS_MSFT) \
    _avail(XrSceneComponentsLocateInfoMSFT, XR_TYPE_SCENE_COMPONENTS_LOCATE_INFO_MSFT) \
    _avail(XrSceneObjectsMSFT, XR_TYPE_SCENE_OBJECTS_MSFT) \
    _avail(XrSceneComponentParentFilterInfoMSFT, XR_TYPE_SCENE_COMPONENT_PARENT_FILTER_INFO_MSFT) \
    _avail(XrSceneObjectTypesFilterInfoMSFT, XR_TYPE_SCENE_OBJECT_TYPES_FILTER_INFO_MSFT) \
    _avail(XrScenePlanesMSFT, XR_TYPE_SCENE_PLANES_MSFT) \
    _avail(XrScenePlaneAlignmentFilterInfoMSFT, XR_TYPE_SCENE_PLANE_ALIGNMENT_FILTER_INFO_MSFT) \
    _avail(XrSceneMeshesMSFT, XR_TYPE_SCENE_MESHES_MSFT) \
    _avail(XrSceneMeshBuffersGetInfoMSFT, XR_TYPE_SCENE_MESH_BUFFERS_GET_INFO_MSFT) \
    _avail(XrSceneMeshBuffersMSFT, XR_TYPE_SCENE_MESH_BUFFERS_MSFT) \
    _avail(XrSceneMeshVertexBufferMSFT, XR_TYPE_SCENE_MESH_VERTEX_BUFFER_MSFT) \
    _avail(XrSceneMeshIndicesUint32MSFT, XR_TYPE_SCENE_MESH_INDICES_UINT32_MSFT) \
    _avail(XrSceneMeshIndicesUint16MSFT, XR_TYPE_SCENE_MESH_INDICES_UINT16_MSFT) \
    _avail(XrSerializedSceneFragmentDataGetInfoMSFT, XR_TYPE_SERIALIZED_SCENE_FRAGMENT_DATA_GET_INFO_MSFT) \
    _avail(XrSceneDeserializeInfoMSFT, XR_TYPE_SCENE_DESERIALIZE_INFO_MSFT) \
    _avail(XrEventDataDisplayRefreshRateChangedFB, XR_TYPE_EVENT_DATA_DISPLAY_REFRESH_RATE_CHANGED_FB) \
    _avail(XrViveTrackerPathsHTCX, XR_TYPE_VIVE_TRACKER_PATHS_HTCX) \
    _avail(XrEventDataViveTrackerConnectedHTCX, XR_TYPE_EVENT_DATA_VIVE_TRACKER_CONNECTED_HTCX) \
    _avail(XrSystemFacialTrackingPropertiesHTC, XR_TYPE_SYSTEM_FACIAL_TRACKING_PROPERTIES_HTC) \
    _avail(XrFacialExpressionsHTC, XR_TYPE_FACIAL_EXPRESSIONS_HTC) \
    _avail(XrFacialTrackerCreateInfoHTC, XR_TYPE_FACIAL_TRACKER_CREATE_INFO_HTC) \
    _avail(XrSystemColorSpacePropertiesFB, XR_TYPE_SYSTEM_COLOR_SPACE_PROPERTIES_FB) \
    _avail(XrHandTrackingMeshFB, XR_TYPE_HAND_TRACKING_MESH_FB) \
    _avail(XrHandTrackingScaleFB, XR_TYPE_HAND_TRACKING_SCALE_FB) \
    _avail(XrHandTrackingAimStateFB, XR_TYPE_HAND_TRACKING_AIM_STATE_FB) \
    _avail(XrHandTrackingCapsulesStateFB, XR_TYPE_HAND_TRACKING_CAPSULES_STATE_FB) \
    _avail(XrSystemSpatialEntityPropertiesFB, XR_TYPE_SYSTEM_SPATIAL_ENTITY_PROPERTIES_FB) \
    _avail(XrSpatialAnchorCreateInfoFB, XR_TYPE_SPATIAL_ANCHOR_CREATE_INFO_FB) \
    _avail(XrSpaceComponentStatusSetInfoFB, XR_TYPE_SPACE_COMPONENT_STATUS_SET_INFO_FB) \
    _avail(XrSpaceComponentStatusFB, XR_TYPE_SPACE_COMPONENT_STATUS_FB) \
    _avail(XrEventDataSpatialAnchorCreateCompleteFB, XR_TYPE_EVENT_DATA_SPATIAL_ANCHOR_CREATE_COMPLETE_FB) \
    _avail(XrEventDataSpaceSetStatusCompleteFB, XR_TYPE_EVENT_DATA_SPACE_SET_STATUS_COMPLETE_FB) \
    _avail(XrFoveationProfileCreateInfoFB, XR_TYPE_FOVEATION_PROFILE_CREATE_INFO_FB) \
    _avail(XrSwapchainCreateInfoFoveationFB, XR_TYPE_SWAPCHAIN_CREATE_INFO_FOVEATION_FB) \
    _avail(XrSwapchainStateFoveationFB, XR_TYPE_SWAPCHAIN_STATE_FOVEATION_FB) \
    _avail(XrFoveationLevelProfileCreateInfoFB, XR_TYPE_FOVEATION_LEVEL_PROFILE_CREATE_INFO_FB) \
    _avail(XrSystemKeyboardTrackingPropertiesFB, XR_TYPE_SYSTEM_KEYBOARD_TRACKING_PROPERTIES_FB) \
    _avail(XrKeyboardSpaceCreateInfoFB, XR_TYPE_KEYBOARD_SPACE_CREATE_INFO_FB) \
    _avail(XrKeyboardTrackingQueryFB, XR_TYPE_KEYBOARD_TRACKING_QUERY_FB) \
    _avail(XrTriangleMeshCreateInfoFB, XR_TYPE_TRIANGLE_MESH_CREATE_INFO_FB) \
    _avail(XrSystemPassthroughPropertiesFB, XR_TYPE_SYSTEM_PASSTHROUGH_PROPERTIES_FB) \
    _avail(XrSystemPassthroughProperties2FB, XR_TYPE_SYSTEM_PASSTHROUGH_PROPERTIES2_FB) \
    _avail(XrPassthroughCreateInfoFB, XR_TYPE_PASSTHROUGH_CREATE_INFO_FB) \
    _avail(XrPassthroughLayerCreateInfoFB, XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB) \
    _avail(XrCompositionLayerPassthroughFB, XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB) \
    _avail(XrGeometryInstanceCreateInfoFB, XR_TYPE_GEOMETRY_INSTANCE_CREATE_INFO_FB) \
    _avail(XrGeometryInstanceTransformFB, XR_TYPE_GEOMETRY_INSTANCE_TRANSFORM_FB) \
    _avail(XrPassthroughStyleFB, XR_TYPE_PASSTHROUGH_STYLE_FB) \
    _avail(XrPassthroughColorMapMonoToRgbaFB, XR_TYPE_PASSTHROUGH_COLOR_MAP_MONO_TO_RGBA_FB) \
    _avail(XrPassthroughColorMapMonoToMonoFB, XR_TYPE_PASSTHROUGH_COLOR_MAP_MONO_TO_MONO_FB) \
    _avail(XrPassthroughBrightnessContrastSaturationFB, XR_TYPE_PASSTHROUGH_BRIGHTNESS_CONTRAST_SATURATION_FB) \
    _avail(XrEventDataPassthroughStateChangedFB, XR_TYPE_EVENT_DATA_PASSTHROUGH_STATE_CHANGED_FB) \
    _avail(XrRenderModelPathInfoFB, XR_TYPE_RENDER_MODEL_PATH_INFO_FB) \
    _avail(XrRenderModelPropertiesFB, XR_TYPE_RENDER_MODEL_PROPERTIES_FB) \
    _avail(XrRenderModelBufferFB, XR_TYPE_RENDER_MODEL_BUFFER_FB) \
    _avail(XrRenderModelLoadInfoFB, XR_TYPE_RENDER_MODEL_LOAD_INFO_FB) \
    _avail(XrSystemRenderModelPropertiesFB, XR_TYPE_SYSTEM_RENDER_MODEL_PROPERTIES_FB) \
    _avail(XrRenderModelCapabilitiesRequestFB, XR_TYPE_RENDER_MODEL_CAPABILITIES_REQUEST_FB) \
    _avail(XrViewLocateFoveatedRenderingVARJO, XR_TYPE_VIEW_LOCATE_FOVEATED_RENDERING_VARJO) \
    _avail(XrFoveatedViewConfigurationViewVARJO, XR_TYPE_FOVEATED_VIEW_CONFIGURATION_VIEW_VARJO) \
    _avail(XrSystemFoveatedRenderingPropertiesVARJO, XR_TYPE_SYSTEM_FOVEATED_RENDERING_PROPERTIES_VARJO) \
    _avail(XrCompositionLayerDepthTestVARJO, XR_TYPE_COMPOSITION_LAYER_DEPTH_TEST_VARJO) \
    _avail(XrSystemMarkerTrackingPropertiesVARJO, XR_TYPE_SYSTEM_MARKER_TRACKING_PROPERTIES_VARJO) \
    _avail(XrEventDataMarkerTrackingUpdateVARJO, XR_TYPE_EVENT_DATA_MARKER_TRACKING_UPDATE_VARJO) \
    _avail(XrMarkerSpaceCreateInfoVARJO, XR_TYPE_MARKER_SPACE_CREATE_INFO_VARJO) \
    _avail(XrFrameEndInfoML, XR_TYPE_FRAME_END_INFO_ML) \
    _avail(XrGlobalDimmerFrameEndInfoML, XR_TYPE_GLOBAL_DIMMER_FRAME_END_INFO_ML) \
    _avail(XrSystemMarkerUnderstandingPropertiesML, XR_TYPE_SYSTEM_MARKER_UNDERSTANDING_PROPERTIES_ML) \
    _avail(XrMarkerDetectorCreateInfoML, XR_TYPE_MARKER_DETECTOR_CREATE_INFO_ML) \
    _avail(XrMarkerDetectorArucoInfoML, XR_TYPE_MARKER_DETECTOR_ARUCO_INFO_ML) \
    _avail(XrMarkerDetectorSizeInfoML, XR_TYPE_MARKER_DETECTOR_SIZE_INFO_ML) \
    _avail(XrMarkerDetectorAprilTagInfoML, XR_TYPE_MARKER_DETECTOR_APRIL_TAG_INFO_ML) \
    _avail(XrMarkerDetectorCustomProfileInfoML, XR_TYPE_MARKER_DETECTOR_CUSTOM_PROFILE_INFO_ML) \
    _avail(XrMarkerDetectorSnapshotInfoML, XR_TYPE_MARKER_DETECTOR_SNAPSHOT_INFO_ML) \
    _avail(XrMarkerDetectorStateML, XR_TYPE_MARKER_DETECTOR_STATE_ML) \
    _avail(XrMarkerSpaceCreateInfoML, XR_TYPE_MARKER_SPACE_CREATE_INFO_ML) \
    _avail(XrLocalizationMapML, XR_TYPE_LOCALIZATION_MAP_ML) \
    _avail(XrEventDataLocalizationChangedML, XR_TYPE_EVENT_DATA_LOCALIZATION_CHANGED_ML) \
    _avail(XrMapLocalizationRequestInfoML, XR_TYPE_MAP_LOCALIZATION_REQUEST_INFO_ML) \
    _avail(XrLocalizationMapImportInfoML, XR_TYPE_LOCALIZATION_MAP_IMPORT_INFO_ML) \
    _avail(XrLocalizationEnableEventsInfoML, XR_TYPE_LOCALIZATION_ENABLE_EVENTS_INFO_ML) \
    _avail(XrSpatialAnchorPersistenceInfoMSFT, XR_TYPE_SPATIAL_ANCHOR_PERSISTENCE_INFO_MSFT) \
    _avail(XrSpatialAnchorFromPersistedAnchorCreateInfoMSFT, XR_TYPE_SPATIAL_ANCHOR_FROM_PERSISTED_ANCHOR_CREATE_INFO_MSFT) \
    _avail(XrSceneMarkersMSFT, XR_TYPE_SCENE_MARKERS_MSFT) \
    _avail(XrSceneMarkerTypeFilterMSFT, XR_TYPE_SCENE_MARKER_TYPE_FILTER_MSFT) \
    _avail(XrSceneMarkerQRCodesMSFT, XR_TYPE_SCENE_MARKER_QR_CODES_MSFT) \
    _avail(XrSpaceQueryInfoFB, XR_TYPE_SPACE_QUERY_INFO_FB) \
    _avail(XrSpaceStorageLocationFilterInfoFB, XR_TYPE_SPACE_STORAGE_LOCATION_FILTER_INFO_FB) \
    _avail(XrSpaceUuidFilterInfoFB, XR_TYPE_SPACE_UUID_FILTER_INFO_FB) \
    _avail(XrSpaceComponentFilterInfoFB, XR_TYPE_SPACE_COMPONENT_FILTER_INFO_FB) \
    _avail(XrSpaceQueryResultsFB, XR_TYPE_SPACE_QUERY_RESULTS_FB) \
    _avail(XrEventDataSpaceQueryResultsAvailableFB, XR_TYPE_EVENT_DATA_SPACE_QUERY_RESULTS_AVAILABLE_FB) \
    _avail(XrEventDataSpaceQueryCompleteFB, XR_TYPE_EVENT_DATA_SPACE_QUERY_COMPLETE_FB) \
    _avail(XrSpaceSaveInfoFB, XR_TYPE_SPACE_SAVE_INFO_FB) \
    _avail(XrSpaceEraseInfoFB, XR_TYPE_SPACE_ERASE_INFO_FB) \
    _avail(XrEventDataSpaceSaveCompleteFB, XR_TYPE_EVENT_DATA_SPACE_SAVE_COMPLETE_FB) \
    _avail(XrEventDataSpaceEraseCompleteFB, XR_TYPE_EVENT_DATA_SPACE_ERASE_COMPLETE_FB) \
    _avail(XrSpaceShareInfoFB, XR_TYPE_SPACE_SHARE_INFO_FB) \
    _avail(XrEventDataSpaceShareCompleteFB, XR_TYPE_EVENT_DATA_SPACE_SHARE_COMPLETE_FB) \
    _avail(XrCompositionLayerSpaceWarpInfoFB, XR_TYPE_COMPOSITION_LAYER_SPACE_WARP_INFO_FB) \
    _avail(XrSystemSpaceWarpPropertiesFB, XR_TYPE_SYSTEM_SPACE_WARP_PROPERTIES_FB) \
    _avail(XrHapticAmplitudeEnvelopeVibrationFB, XR_TYPE_HAPTIC_AMPLITUDE_ENVELOPE_VIBRATION_FB) \
    _avail(XrSemanticLabelsFB, XR_TYPE_SEMANTIC_LABELS_FB) \
    _avail(XrRoomLayoutFB, XR_TYPE_ROOM_LAYOUT_FB) \
    _avail(XrBoundary2DFB, XR_TYPE_BOUNDARY_2D_FB) \
    _avail(XrSemanticLabelsSupportInfoFB, XR_TYPE_SEMANTIC_LABELS_SUPPORT_INFO_FB) \
    _avail(XrDigitalLensControlALMALENCE, XR_TYPE_DIGITAL_LENS_CONTROL_ALMALENCE) \
    _avail(XrEventDataSceneCaptureCompleteFB, XR_TYPE_EVENT_DATA_SCENE_CAPTURE_COMPLETE_FB) \
    _avail(XrSceneCaptureRequestInfoFB, XR_TYPE_SCENE_CAPTURE_REQUEST_INFO_FB) \
    _avail(XrSpaceContainerFB, XR_TYPE_SPACE_CONTAINER_FB) \
    _avail(XrFoveationEyeTrackedProfileCreateInfoMETA, XR_TYPE_FOVEATION_EYE_TRACKED_PROFILE_CREATE_INFO_META) \
    _avail(XrFoveationEyeTrackedStateMETA, XR_TYPE_FOVEATION_EYE_TRACKED_STATE_META) \
    _avail(XrSystemFoveationEyeTrackedPropertiesMETA, XR_TYPE_SYSTEM_FOVEATION_EYE_TRACKED_PROPERTIES_META) \
    _avail(XrSystemFaceTrackingPropertiesFB, XR_TYPE_SYSTEM_FACE_TRACKING_PROPERTIES_FB) \
    _avail(XrFaceTrackerCreateInfoFB, XR_TYPE_FACE_TRACKER_CREATE_INFO_FB) \
    _avail(XrFaceExpressionInfoFB, XR_TYPE_FACE_EXPRESSION_INFO_FB) \
    _avail(XrFaceExpressionWeightsFB, XR_TYPE_FACE_EXPRESSION_WEIGHTS_FB) \
    _avail(XrEyeTrackerCreateInfoFB, XR_TYPE_EYE_TRACKER_CREATE_INFO_FB) \
    _avail(XrEyeGazesInfoFB, XR_TYPE_EYE_GAZES_INFO_FB) \
    _avail(XrSystemEyeTrackingPropertiesFB, XR_TYPE_SYSTEM_EYE_TRACKING_PROPERTIES_FB) \
    _avail(XrEyeGazesFB, XR_TYPE_EYE_GAZES_FB) \
    _avail(XrPassthroughKeyboardHandsIntensityFB, XR_TYPE_PASSTHROUGH_KEYBOARD_HANDS_INTENSITY_FB) \
    _avail(XrCompositionLayerSettingsFB, XR_TYPE_COMPOSITION_LAYER_SETTINGS_FB) \
    _avail(XrHapticPcmVibrationFB, XR_TYPE_HAPTIC_PCM_VIBRATION_FB) \
    _avail(XrDevicePcmSampleRateStateFB, XR_TYPE_DEVICE_PCM_SAMPLE_RATE_STATE_FB) \
    _avail(XrCompositionLayerDepthTestFB, XR_TYPE_COMPOSITION_LAYER_DEPTH_TEST_FB) \
    _avail(XrLocalDimmingFrameEndInfoMETA, XR_TYPE_LOCAL_DIMMING_FRAME_END_INFO_META) \
    _avail(XrPassthroughPreferencesMETA, XR_TYPE_PASSTHROUGH_PREFERENCES_META) \
    _avail(XrSystemVirtualKeyboardPropertiesMETA, XR_TYPE_SYSTEM_VIRTUAL_KEYBOARD_PROPERTIES_META) \
    _avail(XrVirtualKeyboardCreateInfoMETA, XR_TYPE_VIRTUAL_KEYBOARD_CREATE_INFO_META) \
    _avail(XrVirtualKeyboardSpaceCreateInfoMETA, XR_TYPE_VIRTUAL_KEYBOARD_SPACE_CREATE_INFO_META) \
    _avail(XrVirtualKeyboardLocationInfoMETA, XR_TYPE_VIRTUAL_KEYBOARD_LOCATION_INFO_META) \
    _avail(XrVirtualKeyboardModelVisibilitySetInfoMETA, XR_TYPE_VIRTUAL_KEYBOARD_MODEL_VISIBILITY_SET_INFO_META) \
    _avail(XrVirtualKeyboardAnimationStateMETA, XR_TYPE_VIRTUAL_KEYBOARD_ANIMATION_STATE_META) \
    _avail(XrVirtualKeyboardModelAnimationStatesMETA, XR_TYPE_VIRTUAL_KEYBOARD_MODEL_ANIMATION_STATES_META) \
    _avail(XrVirtualKeyboardTextureDataMETA, XR_TYPE_VIRTUAL_KEYBOARD_TEXTURE_DATA_META) \
    _avail(XrVirtualKeyboardInputInfoMETA, XR_TYPE_VIRTUAL_KEYBOARD_INPUT_INFO_META) \
    _avail(XrVirtualKeyboardTextContextChangeInfoMETA, XR_TYPE_VIRTUAL_KEYBOARD_TEXT_CONTEXT_CHANGE_INFO_META) \
    _avail(XrEventDataVirtualKeyboardCommitTextMETA, XR_TYPE_EVENT_DATA_VIRTUAL_KEYBOARD_COMMIT_TEXT_META) \
    _avail(XrEventDataVirtualKeyboardBackspaceMETA, XR_TYPE_EVENT_DATA_VIRTUAL_KEYBOARD_BACKSPACE_META) \
    _avail(XrEventDataVirtualKeyboardEnterMETA, XR_TYPE_EVENT_DATA_VIRTUAL_KEYBOARD_ENTER_META) \
    _avail(XrEventDataVirtualKeyboardShownMETA, XR_TYPE_EVENT_DATA_VIRTUAL_KEYBOARD_SHOWN_META) \
    _avail(XrEventDataVirtualKeyboardHiddenMETA, XR_TYPE_EVENT_DATA_VIRTUAL_KEYBOARD_HIDDEN_META) \
    _avail(XrExternalCameraOCULUS, XR_TYPE_EXTERNAL_CAMERA_OCULUS) \
    _avail(XrPerformanceMetricsStateMETA, XR_TYPE_PERFORMANCE_METRICS_STATE_META) \
    _avail(XrPerformanceMetricsCounterMETA, XR_TYPE_PERFORMANCE_METRICS_COUNTER_META) \
    _avail(XrSpaceListSaveInfoFB, XR_TYPE_SPACE_LIST_SAVE_INFO_FB) \
    _avail(XrEventDataSpaceListSaveCompleteFB, XR_TYPE_EVENT_DATA_SPACE_LIST_SAVE_COMPLETE_FB) \
    _avail(XrSpaceUserCreateInfoFB, XR_TYPE_SPACE_USER_CREATE_INFO_FB) \
    _avail(XrSystemHeadsetIdPropertiesMETA, XR_TYPE_SYSTEM_HEADSET_ID_PROPERTIES_META) \
    _avail(XrRecommendedLayerResolutionMETA, XR_TYPE_RECOMMENDED_LAYER_RESOLUTION_META) \
    _avail(XrRecommendedLayerResolutionGetInfoMETA, XR_TYPE_RECOMMENDED_LAYER_RESOLUTION_GET_INFO_META) \
    _avail(XrPassthroughColorLutCreateInfoMETA, XR_TYPE_PASSTHROUGH_COLOR_LUT_CREATE_INFO_META) \
    _avail(XrPassthroughColorLutUpdateInfoMETA, XR_TYPE_PASSTHROUGH_COLOR_LUT_UPDATE_INFO_META) \
    _avail(XrPassthroughColorMapLutMETA, XR_TYPE_PASSTHROUGH_COLOR_MAP_LUT_META) \
    _avail(XrPassthroughColorMapInterpolatedLutMETA, XR_TYPE_PASSTHROUGH_COLOR_MAP_INTERPOLATED_LUT_META) \
    _avail(XrSystemPassthroughColorLutPropertiesMETA, XR_TYPE_SYSTEM_PASSTHROUGH_COLOR_LUT_PROPERTIES_META) \
    _avail(XrSpaceTriangleMeshGetInfoMETA, XR_TYPE_SPACE_TRIANGLE_MESH_GET_INFO_META) \
    _avail(XrSpaceTriangleMeshMETA, XR_TYPE_SPACE_TRIANGLE_MESH_META) \
    _avail(XrSystemFaceTrackingProperties2FB, XR_TYPE_SYSTEM_FACE_TRACKING_PROPERTIES2_FB) \
    _avail(XrFaceTrackerCreateInfo2FB, XR_TYPE_FACE_TRACKER_CREATE_INFO2_FB) \
    _avail(XrFaceExpressionInfo2FB, XR_TYPE_FACE_EXPRESSION_INFO2_FB) \
    _avail(XrFaceExpressionWeights2FB, XR_TYPE_FACE_EXPRESSION_WEIGHTS2_FB) \
    _avail(XrEnvironmentDepthProviderCreateInfoMETA, XR_TYPE_ENVIRONMENT_DEPTH_PROVIDER_CREATE_INFO_META) \
    _avail(XrEnvironmentDepthSwapchainCreateInfoMETA, XR_TYPE_ENVIRONMENT_DEPTH_SWAPCHAIN_CREATE_INFO_META) \
    _avail(XrEnvironmentDepthSwapchainStateMETA, XR_TYPE_ENVIRONMENT_DEPTH_SWAPCHAIN_STATE_META) \
    _avail(XrEnvironmentDepthImageAcquireInfoMETA, XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_ACQUIRE_INFO_META) \
    _avail(XrEnvironmentDepthImageViewMETA, XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_VIEW_META) \
    _avail(XrEnvironmentDepthImageMETA, XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_META) \
    _avail(XrEnvironmentDepthHandRemovalSetInfoMETA, XR_TYPE_ENVIRONMENT_DEPTH_HAND_REMOVAL_SET_INFO_META) \
    _avail(XrSystemEnvironmentDepthPropertiesMETA, XR_TYPE_SYSTEM_ENVIRONMENT_DEPTH_PROPERTIES_META) \
    _avail(XrPassthroughCreateInfoHTC, XR_TYPE_PASSTHROUGH_CREATE_INFO_HTC) \
    _avail(XrPassthroughColorHTC, XR_TYPE_PASSTHROUGH_COLOR_HTC) \
    _avail(XrPassthroughMeshTransformInfoHTC, XR_TYPE_PASSTHROUGH_MESH_TRANSFORM_INFO_HTC) \
    _avail(XrCompositionLayerPassthroughHTC, XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_HTC) \
    _avail(XrFoveationApplyInfoHTC, XR_TYPE_FOVEATION_APPLY_INFO_HTC) \
    _avail(XrFoveationDynamicModeInfoHTC, XR_TYPE_FOVEATION_DYNAMIC_MODE_INFO_HTC) \
    _avail(XrFoveationCustomModeInfoHTC, XR_TYPE_FOVEATION_CUSTOM_MODE_INFO_HTC) \
    _avail(XrSystemAnchorPropertiesHTC, XR_TYPE_SYSTEM_ANCHOR_PROPERTIES_HTC) \
    _avail(XrSpatialAnchorCreateInfoHTC, XR_TYPE_SPATIAL_ANCHOR_CREATE_INFO_HTC) \
    _avail(XrActiveActionSetPrioritiesEXT, XR_TYPE_ACTIVE_ACTION_SET_PRIORITIES_EXT) \
    _avail(XrSystemForceFeedbackCurlPropertiesMNDX, XR_TYPE_SYSTEM_FORCE_FEEDBACK_CURL_PROPERTIES_MNDX) \
    _avail(XrForceFeedbackCurlApplyLocationsMNDX, XR_TYPE_FORCE_FEEDBACK_CURL_APPLY_LOCATIONS_MNDX) \
    _avail(XrHandTrackingDataSourceInfoEXT, XR_TYPE_HAND_TRACKING_DATA_SOURCE_INFO_EXT) \
    _avail(XrHandTrackingDataSourceStateEXT, XR_TYPE_HAND_TRACKING_DATA_SOURCE_STATE_EXT) \
    _avail(XrSystemPlaneDetectionPropertiesEXT, XR_TYPE_SYSTEM_PLANE_DETECTION_PROPERTIES_EXT) \
    _avail(XrPlaneDetectorCreateInfoEXT, XR_TYPE_PLANE_DETECTOR_CREATE_INFO_EXT) \
    _avail(XrPlaneDetectorBeginInfoEXT, XR_TYPE_PLANE_DETECTOR_BEGIN_INFO_EXT) \
    _avail(XrPlaneDetectorGetInfoEXT, XR_TYPE_PLANE_DETECTOR_GET_INFO_EXT) \
    _avail(XrPlaneDetectorLocationEXT, XR_TYPE_PLANE_DETECTOR_LOCATION_EXT) \
    _avail(XrPlaneDetectorLocationsEXT, XR_TYPE_PLANE_DETECTOR_LOCATIONS_EXT) \
    _avail(XrPlaneDetectorPolygonBufferEXT, XR_TYPE_PLANE_DETECTOR_POLYGON_BUFFER_EXT) \
    _avail(XrTrackableTrackerCreateInfoANDROID, XR_TYPE_TRACKABLE_TRACKER_CREATE_INFO_ANDROID) \
    _avail(XrTrackableGetInfoANDROID, XR_TYPE_TRACKABLE_GET_INFO_ANDROID) \
    _avail(XrTrackablePlaneANDROID, XR_TYPE_TRACKABLE_PLANE_ANDROID) \
    _avail(XrAnchorSpaceCreateInfoANDROID, XR_TYPE_ANCHOR_SPACE_CREATE_INFO_ANDROID) \
    _avail(XrSystemTrackablesPropertiesANDROID, XR_TYPE_SYSTEM_TRACKABLES_PROPERTIES_ANDROID) \
    _avail(XrSystemAvatarEyesPropertiesANDROID, XR_TYPE_SYSTEM_AVATAR_EYES_PROPERTIES_ANDROID) \
    _avail(XrEyesANDROID, XR_TYPE_EYES_ANDROID) \
    _avail(XrEyesGetInfoANDROID, XR_TYPE_EYES_GET_INFO_ANDROID) \
    _avail(XrEyeTrackerCreateInfoANDROID, XR_TYPE_EYE_TRACKER_CREATE_INFO_ANDROID) \
    _avail(XrDeviceAnchorPersistenceCreateInfoANDROID, XR_TYPE_DEVICE_ANCHOR_PERSISTENCE_CREATE_INFO_ANDROID) \
    _avail(XrPersistedAnchorSpaceCreateInfoANDROID, XR_TYPE_PERSISTED_ANCHOR_SPACE_CREATE_INFO_ANDROID) \
    _avail(XrPersistedAnchorSpaceInfoANDROID, XR_TYPE_PERSISTED_ANCHOR_SPACE_INFO_ANDROID) \
    _avail(XrSystemDeviceAnchorPersistencePropertiesANDROID, XR_TYPE_SYSTEM_DEVICE_ANCHOR_PERSISTENCE_PROPERTIES_ANDROID) \
    _avail(XrFaceTrackerCreateInfoANDROID, XR_TYPE_FACE_TRACKER_CREATE_INFO_ANDROID) \
    _avail(XrFaceStateGetInfoANDROID, XR_TYPE_FACE_STATE_GET_INFO_ANDROID) \
    _avail(XrFaceStateANDROID, XR_TYPE_FACE_STATE_ANDROID) \
    _avail(XrSystemFaceTrackingPropertiesANDROID, XR_TYPE_SYSTEM_FACE_TRACKING_PROPERTIES_ANDROID) \
    _avail(XrSystemPassthroughCameraStatePropertiesANDROID, XR_TYPE_SYSTEM_PASSTHROUGH_CAMERA_STATE_PROPERTIES_ANDROID) \
    _avail(XrPassthroughCameraStateGetInfoANDROID, XR_TYPE_PASSTHROUGH_CAMERA_STATE_GET_INFO_ANDROID) \
    _avail(XrEventDataRecommendedResolutionChangedANDROID, XR_TYPE_EVENT_DATA_RECOMMENDED_RESOLUTION_CHANGED_ANDROID) \
    _avail(XrPassthroughLayerCreateInfoANDROID, XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_ANDROID) \
    _avail(XrPassthroughLayerMeshANDROID, XR_TYPE_PASSTHROUGH_LAYER_MESH_ANDROID) \
    _avail(XrCompositionLayerPassthroughANDROID, XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_ANDROID) \
    _avail(XrSystemPassthroughLayerPropertiesANDROID, XR_TYPE_SYSTEM_PASSTHROUGH_LAYER_PROPERTIES_ANDROID) \
    _avail(XrRaycastInfoANDROID, XR_TYPE_RAYCAST_INFO_ANDROID) \
    _avail(XrRaycastHitResultsANDROID, XR_TYPE_RAYCAST_HIT_RESULTS_ANDROID) \
    _avail(XrPerformanceMetricsStateANDROID, XR_TYPE_PERFORMANCE_METRICS_STATE_ANDROID) \
    _avail(XrPerformanceMetricsCounterANDROID, XR_TYPE_PERFORMANCE_METRICS_COUNTER_ANDROID) \
    _avail(XrTrackableObjectANDROID, XR_TYPE_TRACKABLE_OBJECT_ANDROID) \
    _avail(XrTrackableObjectConfigurationANDROID, XR_TYPE_TRACKABLE_OBJECT_CONFIGURATION_ANDROID) \
    _avail(XrFutureCancelInfoEXT, XR_TYPE_FUTURE_CANCEL_INFO_EXT) \
    _avail(XrFuturePollInfoEXT, XR_TYPE_FUTURE_POLL_INFO_EXT) \
    _avail(XrFutureCompletionEXT, XR_TYPE_FUTURE_COMPLETION_EXT) \
    _avail(XrFuturePollResultEXT, XR_TYPE_FUTURE_POLL_RESULT_EXT) \
    _avail(XrEventDataUserPresenceChangedEXT, XR_TYPE_EVENT_DATA_USER_PRESENCE_CHANGED_EXT) \
    _avail(XrSystemUserPresencePropertiesEXT, XR_TYPE_SYSTEM_USER_PRESENCE_PROPERTIES_EXT) \
    _avail(XrEventDataHeadsetFitChangedML, XR_TYPE_EVENT_DATA_HEADSET_FIT_CHANGED_ML) \
    _avail(XrEventDataEyeCalibrationChangedML, XR_TYPE_EVENT_DATA_EYE_CALIBRATION_CHANGED_ML) \
    _avail(XrUserCalibrationEnableEventsInfoML, XR_TYPE_USER_CALIBRATION_ENABLE_EVENTS_INFO_ML) \
    _avail(XrSystemLightEstimationPropertiesANDROID, XR_TYPE_SYSTEM_LIGHT_ESTIMATION_PROPERTIES_ANDROID) \
    _avail(XrLightEstimatorCreateInfoANDROID, XR_TYPE_LIGHT_ESTIMATOR_CREATE_INFO_ANDROID) \
    _avail(XrLightEstimateGetInfoANDROID, XR_TYPE_LIGHT_ESTIMATE_GET_INFO_ANDROID) \
    _avail(XrLightEstimateANDROID, XR_TYPE_LIGHT_ESTIMATE_ANDROID) \
    _avail(XrDirectionalLightANDROID, XR_TYPE_DIRECTIONAL_LIGHT_ANDROID) \
    _avail(XrAmbientLightANDROID, XR_TYPE_AMBIENT_LIGHT_ANDROID) \
    _avail(XrSphericalHarmonicsANDROID, XR_TYPE_SPHERICAL_HARMONICS_ANDROID) \
    _avail(XrDepthSwapchainCreateInfoANDROID, XR_TYPE_DEPTH_SWAPCHAIN_CREATE_INFO_ANDROID) \
    _avail(XrDepthSwapchainImageANDROID, XR_TYPE_DEPTH_SWAPCHAIN_IMAGE_ANDROID) \
    _avail(XrDepthAcquireInfoANDROID, XR_TYPE_DEPTH_ACQUIRE_INFO_ANDROID) \
    _avail(XrDepthViewANDROID, XR_TYPE_DEPTH_VIEW_ANDROID) \
    _avail(XrDepthAcquireResultANDROID, XR_TYPE_DEPTH_ACQUIRE_RESULT_ANDROID) \
    _avail(XrSystemDepthTrackingPropertiesANDROID, XR_TYPE_SYSTEM_DEPTH_TRACKING_PROPERTIES_ANDROID) \
    _avail(XrSystemHandMeshTrackingPropertiesANDROID, XR_TYPE_SYSTEM_HAND_MESH_TRACKING_PROPERTIES_ANDROID) \
    _avail(XrHandMeshTrackerCreateInfoANDROID, XR_TYPE_HAND_MESH_TRACKER_CREATE_INFO_ANDROID) \
    _avail(XrHandMeshGetInfoANDROID, XR_TYPE_HAND_MESH_GET_INFO_ANDROID) \
    _avail(XrHandTrackingMeshesANDROID, XR_TYPE_HAND_TRACKING_MESHES_ANDROID) \
    _avail(XrOccupancyGridANDROIDX, XR_TYPE_OCCUPANCY_GRID_ANDROIDX) \
    _avail(XrPlaneTrackableTrackerCreateInfoANDROIDX, XR_TYPE_PLANE_TRACKABLE_TRACKER_CREATE_INFO_ANDROIDX) \
    _avail(XrAvatarSkeletonJointANDROIDX, XR_TYPE_AVATAR_SKELETON_JOINT_ANDROIDX) \
    _avail(XrAvatarSkeletonANDROIDX, XR_TYPE_AVATAR_SKELETON_ANDROIDX) \
    _avail(XrBodyTrackerCreateInfoANDROIDX, XR_TYPE_BODY_TRACKER_CREATE_INFO_ANDROIDX) \
    _avail(XrBodyTrackerGetInfoANDROIDX, XR_TYPE_BODY_TRACKER_GET_INFO_ANDROIDX) \
    _avail(XrBodyTrackerAvatarProportionsANDROIDX, XR_TYPE_BODY_TRACKER_AVATAR_PROPORTIONS_ANDROIDX) \
    _avail(XrBodyTrackerCalibrationANDROIDX, XR_TYPE_BODY_TRACKER_CALIBRATION_ANDROIDX) \
    _avail(XrSystemMarkerTrackingPropertiesANDROIDX, XR_TYPE_SYSTEM_MARKER_TRACKING_PROPERTIES_ANDROIDX) \
    _avail(XrTrackableMarkerConfigurationANDROIDX, XR_TYPE_TRACKABLE_MARKER_CONFIGURATION_ANDROIDX) \
    _avail(XrTrackableMarkerANDROIDX, XR_TYPE_TRACKABLE_MARKER_ANDROIDX) \
    _avail(XrSystemQrCodeTrackingPropertiesANDROIDX, XR_TYPE_SYSTEM_QR_CODE_TRACKING_PROPERTIES_ANDROIDX) \
    _avail(XrTrackableQrCodeConfigurationANDROIDX, XR_TYPE_TRACKABLE_QR_CODE_CONFIGURATION_ANDROIDX) \
    _avail(XrTrackableQrCodeANDROIDX, XR_TYPE_TRACKABLE_QR_CODE_ANDROIDX) \
    _avail(XrTrackableShoeboxANDROIDSYS, XR_TYPE_TRACKABLE_SHOEBOX_ANDROIDSYS) \
    _avail(XrPcaFaceTrackerCreateInfoANDROIDSYS, XR_TYPE_PCA_FACE_TRACKER_CREATE_INFO_ANDROIDSYS) \
    _avail(XrFaceJointANDROIDSYS, XR_TYPE_FACE_JOINT_ANDROIDSYS) \
    _avail(XrPcaFaceStateANDROIDSYS, XR_TYPE_PCA_FACE_STATE_ANDROIDSYS) \
    _avail(XrEyeTrackerCalibrationStatusANDROIDSYS, XR_TYPE_EYE_TRACKER_CALIBRATION_STATUS_ANDROIDSYS) \
    _avail(XrEyeTrackerCalibrationStatusGetInfoANDROIDSYS, XR_TYPE_EYE_TRACKER_CALIBRATION_STATUS_GET_INFO_ANDROIDSYS) \
    _avail(XrEyesGetInfoANDROIDSYS, XR_TYPE_EYES_GET_INFO_ANDROIDSYS) \
    _avail(XrEyeTrackerCreateInfoANDROIDSYS, XR_TYPE_EYE_TRACKER_CREATE_INFO_ANDROIDSYS) \
    _avail(XrCalibrationCreateInfoANDROIDSYS, XR_TYPE_CALIBRATION_CREATE_INFO_ANDROIDSYS) \
    _avail(XrEyesANDROIDSYS, XR_TYPE_EYES_ANDROIDSYS) \
    _avail(XrAndroidSurfaceSwapchainCreateInfoANDROIDX, XR_TYPE_ANDROID_SURFACE_SWAPCHAIN_CREATE_INFO_ANDROIDX) \
    _avail(XrCompositionLayerAxisAlignedDistortionANDROIDX, XR_TYPE_COMPOSITION_LAYER_AXIS_ALIGNED_DISTORTION_ANDROIDX) \
    _avail(XrSpatialCapabilityComponentsEXTX1, XR_TYPE_SPATIAL_CAPABILITY_COMPONENTS_EXTX1) \
    _avail(XrSpatialContextCreateInfoEXTX1, XR_TYPE_SPATIAL_CONTEXT_CREATE_INFO_EXTX1) \
    _avail(XrCreateSpatialContextCompletionEXTX1, XR_TYPE_CREATE_SPATIAL_CONTEXT_COMPLETION_EXTX1) \
    _avail(XrSpatialDiscoverySnapshotCreateInfoEXTX1, XR_TYPE_SPATIAL_DISCOVERY_SNAPSHOT_CREATE_INFO_EXTX1) \
    _avail(XrCreateSpatialDiscoverySnapshotCompletionInfoEXTX1, XR_TYPE_CREATE_SPATIAL_DISCOVERY_SNAPSHOT_COMPLETION_INFO_EXTX1) \
    _avail(XrCreateSpatialDiscoverySnapshotCompletionEXTX1, XR_TYPE_CREATE_SPATIAL_DISCOVERY_SNAPSHOT_COMPLETION_EXTX1) \
    _avail(XrSpatialComponentDataQueryConditionEXTX1, XR_TYPE_SPATIAL_COMPONENT_DATA_QUERY_CONDITION_EXTX1) \
    _avail(XrSpatialComponentDataQueryResultEXTX1, XR_TYPE_SPATIAL_COMPONENT_DATA_QUERY_RESULT_EXTX1) \
    _avail(XrSpatialBufferGetInfoEXTX1, XR_TYPE_SPATIAL_BUFFER_GET_INFO_EXTX1) \
    _avail(XrSpatialComponentBounded2DListEXTX1, XR_TYPE_SPATIAL_COMPONENT_BOUNDED_2D_LIST_EXTX1) \
    _avail(XrSpatialComponentBounded3DListEXTX1, XR_TYPE_SPATIAL_COMPONENT_BOUNDED_3D_LIST_EXTX1) \
    _avail(XrSpatialComponentParentListEXTX1, XR_TYPE_SPATIAL_COMPONENT_PARENT_LIST_EXTX1) \
    _avail(XrSpatialComponentMesh3DListEXTX1, XR_TYPE_SPATIAL_COMPONENT_MESH_3D_LIST_EXTX1) \
    _avail(XrSpatialEntityFromIdCreateInfoEXTX1, XR_TYPE_SPATIAL_ENTITY_FROM_ID_CREATE_INFO_EXTX1) \
    _avail(XrSpatialUpdateSnapshotCreateInfoEXTX1, XR_TYPE_SPATIAL_UPDATE_SNAPSHOT_CREATE_INFO_EXTX1) \
    _avail(XrEventDataSpatialDiscoveryRecommendedEXTX1, XR_TYPE_EVENT_DATA_SPATIAL_DISCOVERY_RECOMMENDED_EXTX1) \
    _avail(XrSpatialFilterTrackingStateEXTX1, XR_TYPE_SPATIAL_FILTER_TRACKING_STATE_EXTX1) \
    _avail(XrSpatialCapabilityConfigurationPlaneTrackingEXTX1, XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_PLANE_TRACKING_EXTX1) \
    _avail(XrSpatialComponentPlaneAlignmentListEXTX1, XR_TYPE_SPATIAL_COMPONENT_PLANE_ALIGNMENT_LIST_EXTX1) \
    _avail(XrSpatialComponentMesh2DListEXTX1, XR_TYPE_SPATIAL_COMPONENT_MESH_2D_LIST_EXTX1) \
    _avail(XrSpatialComponentPolygon2DListEXTX1, XR_TYPE_SPATIAL_COMPONENT_POLYGON_2D_LIST_EXTX1) \
    _avail(XrSpatialComponentPlaneSemanticLabelListEXTX1, XR_TYPE_SPATIAL_COMPONENT_PLANE_SEMANTIC_LABEL_LIST_EXTX1) \
    _avail(XrSpatialCapabilityConfigurationQrCodeEXTX1, XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_QR_CODE_EXTX1) \
    _avail(XrSpatialCapabilityConfigurationMicroQrCodeEXTX1, XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_MICRO_QR_CODE_EXTX1) \
    _avail(XrSpatialCapabilityConfigurationArucoMarkerEXTX1, XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_ARUCO_MARKER_EXTX1) \
    _avail(XrSpatialCapabilityConfigurationAprilTagEXTX1, XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_APRIL_TAG_EXTX1) \
    _avail(XrSpatialMarkerSizeEXTX1, XR_TYPE_SPATIAL_MARKER_SIZE_EXTX1) \
    _avail(XrSpatialMarkerStaticOptimizationEXTX1, XR_TYPE_SPATIAL_MARKER_STATIC_OPTIMIZATION_EXTX1) \
    _avail(XrSpatialMarkerComponentListEXTX1, XR_TYPE_SPATIAL_MARKER_COMPONENT_LIST_EXTX1) \
    _avail(XrSpatialBoundsRaycastEXTX1, XR_TYPE_SPATIAL_BOUNDS_RAYCAST_EXTX1) \
    _avail(XrSpatialComponentHitPoseListEXTX1, XR_TYPE_SPATIAL_COMPONENT_HIT_POSE_LIST_EXTX1) \
    _avail(XrSpatialBoundsSpherefEXTX1, XR_TYPE_SPATIAL_BOUNDS_SPHEREF_EXTX1) \
    _avail(XrSpatialBoundsBoxfEXTX1, XR_TYPE_SPATIAL_BOUNDS_BOXF_EXTX1) \
    _avail(XrSpatialBoundsFrustumfEXTX1, XR_TYPE_SPATIAL_BOUNDS_FRUSTUMF_EXTX1) \
    _avail(XrSpatialCapabilityConfigurationAnchorEXTX1, XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_ANCHOR_EXTX1) \
    _avail(XrSpatialComponentAnchorListEXTX1, XR_TYPE_SPATIAL_COMPONENT_ANCHOR_LIST_EXTX1) \
    _avail(XrSpatialAnchorCreateInfoEXTX1, XR_TYPE_SPATIAL_ANCHOR_CREATE_INFO_EXTX1) \
    _avail(XrSpatialPersistenceContextCreateInfoEXTX1, XR_TYPE_SPATIAL_PERSISTENCE_CONTEXT_CREATE_INFO_EXTX1) \
    _avail(XrCreateSpatialPersistenceContextCompletionEXTX1, XR_TYPE_CREATE_SPATIAL_PERSISTENCE_CONTEXT_COMPLETION_EXTX1) \
    _avail(XrSpatialEntityPersistInfoEXTX1, XR_TYPE_SPATIAL_ENTITY_PERSIST_INFO_EXTX1) \
    _avail(XrPersistSpatialEntityCompletionEXTX1, XR_TYPE_PERSIST_SPATIAL_ENTITY_COMPLETION_EXTX1) \
    _avail(XrSpatialEntityUnpersistInfoEXTX1, XR_TYPE_SPATIAL_ENTITY_UNPERSIST_INFO_EXTX1) \
    _avail(XrUnpersistSpatialEntityCompletionEXTX1, XR_TYPE_UNPERSIST_SPATIAL_ENTITY_COMPLETION_EXTX1) \
    _avail(XrSpatialContextPersistenceConfigEXTX1, XR_TYPE_SPATIAL_CONTEXT_PERSISTENCE_CONFIG_EXTX1) \
    _avail(XrSpatialDiscoveryPersistenceFilterEXTX1, XR_TYPE_SPATIAL_DISCOVERY_PERSISTENCE_FILTER_EXTX1) \
    _avail(XrSpatialComponentPersistenceListEXTX1, XR_TYPE_SPATIAL_COMPONENT_PERSISTENCE_LIST_EXTX1) \


#if defined(XR_USE_GRAPHICS_API_D3D11)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_D3D11(_avail, _unavail) \
    _avail(XrGraphicsBindingD3D11KHR, XR_TYPE_GRAPHICS_BINDING_D3D11_KHR) \
    _avail(XrSwapchainImageD3D11KHR, XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR) \
    _avail(XrGraphicsRequirementsD3D11KHR, XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_D3D11(_avail, _unavail) \
    _unavail(XrGraphicsBindingD3D11KHR, XR_TYPE_GRAPHICS_BINDING_D3D11_KHR) \
    _unavail(XrSwapchainImageD3D11KHR, XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR) \
    _unavail(XrGraphicsRequirementsD3D11KHR, XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR) \

#endif

#if defined(XR_USE_GRAPHICS_API_D3D12)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_D3D12(_avail, _unavail) \
    _avail(XrGraphicsBindingD3D12KHR, XR_TYPE_GRAPHICS_BINDING_D3D12_KHR) \
    _avail(XrSwapchainImageD3D12KHR, XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR) \
    _avail(XrGraphicsRequirementsD3D12KHR, XR_TYPE_GRAPHICS_REQUIREMENTS_D3D12_KHR) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_D3D12(_avail, _unavail) \
    _unavail(XrGraphicsBindingD3D12KHR, XR_TYPE_GRAPHICS_BINDING_D3D12_KHR) \
    _unavail(XrSwapchainImageD3D12KHR, XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR) \
    _unavail(XrGraphicsRequirementsD3D12KHR, XR_TYPE_GRAPHICS_REQUIREMENTS_D3D12_KHR) \

#endif

#if defined(XR_USE_GRAPHICS_API_METAL)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_METAL(_avail, _unavail) \
    _avail(XrGraphicsBindingMetalKHR, XR_TYPE_GRAPHICS_BINDING_METAL_KHR) \
    _avail(XrSwapchainImageMetalKHR, XR_TYPE_SWAPCHAIN_IMAGE_METAL_KHR) \
    _avail(XrGraphicsRequirementsMetalKHR, XR_TYPE_GRAPHICS_REQUIREMENTS_METAL_KHR) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_METAL(_avail, _unavail) \
    _unavail(XrGraphicsBindingMetalKHR, XR_TYPE_GRAPHICS_BINDING_METAL_KHR) \
    _unavail(XrSwapchainImageMetalKHR, XR_TYPE_SWAPCHAIN_IMAGE_METAL_KHR) \
    _unavail(XrGraphicsRequirementsMetalKHR, XR_TYPE_GRAPHICS_REQUIREMENTS_METAL_KHR) \

#endif

#if defined(XR_USE_GRAPHICS_API_OPENGL)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL(_avail, _unavail) \
    _avail(XrSwapchainImageOpenGLKHR, XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR) \
    _avail(XrGraphicsRequirementsOpenGLKHR, XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL(_avail, _unavail) \
    _unavail(XrSwapchainImageOpenGLKHR, XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR) \
    _unavail(XrGraphicsRequirementsOpenGLKHR, XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR) \

#endif

#if defined(XR_USE_GRAPHICS_API_OPENGL) && defined(XR_USE_PLATFORM_WAYLAND)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_WAYLAND(_avail, _unavail) \
    _avail(XrGraphicsBindingOpenGLWaylandKHR, XR_TYPE_GRAPHICS_BINDING_OPENGL_WAYLAND_KHR) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_WAYLAND(_avail, _unavail) \
    _unavail(XrGraphicsBindingOpenGLWaylandKHR, XR_TYPE_GRAPHICS_BINDING_OPENGL_WAYLAND_KHR) \

#endif

#if defined(XR_USE_GRAPHICS_API_OPENGL) && defined(XR_USE_PLATFORM_WIN32)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_WIN32(_avail, _unavail) \
    _avail(XrGraphicsBindingOpenGLWin32KHR, XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_WIN32(_avail, _unavail) \
    _unavail(XrGraphicsBindingOpenGLWin32KHR, XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR) \

#endif

#if defined(XR_USE_GRAPHICS_API_OPENGL) && defined(XR_USE_PLATFORM_XCB)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_XCB(_avail, _unavail) \
    _avail(XrGraphicsBindingOpenGLXcbKHR, XR_TYPE_GRAPHICS_BINDING_OPENGL_XCB_KHR) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_XCB(_avail, _unavail) \
    _unavail(XrGraphicsBindingOpenGLXcbKHR, XR_TYPE_GRAPHICS_BINDING_OPENGL_XCB_KHR) \

#endif

#if defined(XR_USE_GRAPHICS_API_OPENGL) && defined(XR_USE_PLATFORM_XLIB)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_XLIB(_avail, _unavail) \
    _avail(XrGraphicsBindingOpenGLXlibKHR, XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_XR_USE_PLATFORM_XLIB(_avail, _unavail) \
    _unavail(XrGraphicsBindingOpenGLXlibKHR, XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR) \

#endif

#if defined(XR_USE_GRAPHICS_API_OPENGL_ES)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_ES(_avail, _unavail) \
    _avail(XrSwapchainImageOpenGLESKHR, XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR) \
    _avail(XrGraphicsRequirementsOpenGLESKHR, XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR) \
    _avail(XrSwapchainStateSamplerOpenGLESFB, XR_TYPE_SWAPCHAIN_STATE_SAMPLER_OPENGL_ES_FB) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_ES(_avail, _unavail) \
    _unavail(XrSwapchainImageOpenGLESKHR, XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR) \
    _unavail(XrGraphicsRequirementsOpenGLESKHR, XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR) \
    _unavail(XrSwapchainStateSamplerOpenGLESFB, XR_TYPE_SWAPCHAIN_STATE_SAMPLER_OPENGL_ES_FB) \

#endif

#if defined(XR_USE_GRAPHICS_API_OPENGL_ES) && defined(XR_USE_PLATFORM_ANDROID)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_ES_XR_USE_PLATFORM_ANDROID(_avail, _unavail) \
    _avail(XrGraphicsBindingOpenGLESAndroidKHR, XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_OPENGL_ES_XR_USE_PLATFORM_ANDROID(_avail, _unavail) \
    _unavail(XrGraphicsBindingOpenGLESAndroidKHR, XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR) \

#endif

#if defined(XR_USE_GRAPHICS_API_VULKAN)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_VULKAN(_avail, _unavail) \
    _avail(XrVulkanSwapchainFormatListCreateInfoKHR, XR_TYPE_VULKAN_SWAPCHAIN_FORMAT_LIST_CREATE_INFO_KHR) \
    _avail(XrGraphicsBindingVulkanKHR, XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR) \
    _avail(XrSwapchainImageVulkanKHR, XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR) \
    _avail(XrGraphicsRequirementsVulkanKHR, XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN_KHR) \
    _avail(XrVulkanInstanceCreateInfoKHR, XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR) \
    _avail(XrVulkanDeviceCreateInfoKHR, XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR) \
    _avail(XrVulkanGraphicsDeviceGetInfoKHR, XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR) \
    _avail(XrSwapchainImageFoveationVulkanFB, XR_TYPE_SWAPCHAIN_IMAGE_FOVEATION_VULKAN_FB) \
    _avail(XrSwapchainStateSamplerVulkanFB, XR_TYPE_SWAPCHAIN_STATE_SAMPLER_VULKAN_FB) \
    _avail(XrVulkanSwapchainCreateInfoMETA, XR_TYPE_VULKAN_SWAPCHAIN_CREATE_INFO_META) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_GRAPHICS_API_VULKAN(_avail, _unavail) \
    _unavail(XrVulkanSwapchainFormatListCreateInfoKHR, XR_TYPE_VULKAN_SWAPCHAIN_FORMAT_LIST_CREATE_INFO_KHR) \
    _unavail(XrGraphicsBindingVulkanKHR, XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR) \
    _unavail(XrSwapchainImageVulkanKHR, XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR) \
    _unavail(XrGraphicsRequirementsVulkanKHR, XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN_KHR) \
    _unavail(XrVulkanInstanceCreateInfoKHR, XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR) \
    _unavail(XrVulkanDeviceCreateInfoKHR, XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR) \
    _unavail(XrVulkanGraphicsDeviceGetInfoKHR, XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR) \
    _unavail(XrSwapchainImageFoveationVulkanFB, XR_TYPE_SWAPCHAIN_IMAGE_FOVEATION_VULKAN_FB) \
    _unavail(XrSwapchainStateSamplerVulkanFB, XR_TYPE_SWAPCHAIN_STATE_SAMPLER_VULKAN_FB) \
    _unavail(XrVulkanSwapchainCreateInfoMETA, XR_TYPE_VULKAN_SWAPCHAIN_CREATE_INFO_META) \

#endif

#if defined(XR_USE_PLATFORM_ANDROID)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_ANDROID(_avail, _unavail) \
    _avail(XrInstanceCreateInfoAndroidKHR, XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR) \
    _avail(XrLoaderInitInfoAndroidKHR, XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR) \
    _avail(XrAndroidSurfaceSwapchainCreateInfoFB, XR_TYPE_ANDROID_SURFACE_SWAPCHAIN_CREATE_INFO_FB) \
    _avail(XrSwapchainStateAndroidSurfaceDimensionsFB, XR_TYPE_SWAPCHAIN_STATE_ANDROID_SURFACE_DIMENSIONS_FB) \
    _avail(XrAnchorSharingInfoANDROID, XR_TYPE_ANCHOR_SHARING_INFO_ANDROID) \
    _avail(XrAnchorSharingTokenANDROID, XR_TYPE_ANCHOR_SHARING_TOKEN_ANDROID) \
    _avail(XrSystemAnchorSharingExportPropertiesANDROID, XR_TYPE_SYSTEM_ANCHOR_SHARING_EXPORT_PROPERTIES_ANDROID) \
    _avail(XrSystemSceneMeshingPropertiesANDROID, XR_TYPE_SYSTEM_SCENE_MESHING_PROPERTIES_ANDROID) \
    _avail(XrSceneMeshingTrackerCreateInfoANDROID, XR_TYPE_SCENE_MESHING_TRACKER_CREATE_INFO_ANDROID) \
    _avail(XrSceneMeshAcquireInfoANDROID, XR_TYPE_SCENE_MESH_ACQUIRE_INFO_ANDROID) \
    _avail(XrSceneMeshReleaseInfoANDROID, XR_TYPE_SCENE_MESH_RELEASE_INFO_ANDROID) \
    _avail(XrSceneSubmeshANDROID, XR_TYPE_SCENE_SUBMESH_ANDROID) \
    _avail(XrSceneMeshANDROID, XR_TYPE_SCENE_MESH_ANDROID) \
    _avail(XrInstanceCreateInfoBackgroundTrackingANDROIDSYS, XR_TYPE_INSTANCE_CREATE_INFO_BACKGROUND_TRACKING_ANDROIDSYS) \
    _avail(XrSharedAnchorSpaceCreateInfoANDROIDSYS, XR_TYPE_SHARED_ANCHOR_SPACE_CREATE_INFO_ANDROIDSYS) \
    _avail(XrInputTracingDataANDROIDSYS, XR_TYPE_INPUT_TRACING_DATA_ANDROIDSYS) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_ANDROID(_avail, _unavail) \
    _unavail(XrInstanceCreateInfoAndroidKHR, XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR) \
    _unavail(XrLoaderInitInfoAndroidKHR, XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR) \
    _unavail(XrAndroidSurfaceSwapchainCreateInfoFB, XR_TYPE_ANDROID_SURFACE_SWAPCHAIN_CREATE_INFO_FB) \
    _unavail(XrSwapchainStateAndroidSurfaceDimensionsFB, XR_TYPE_SWAPCHAIN_STATE_ANDROID_SURFACE_DIMENSIONS_FB) \
    _unavail(XrAnchorSharingInfoANDROID, XR_TYPE_ANCHOR_SHARING_INFO_ANDROID) \
    _unavail(XrAnchorSharingTokenANDROID, XR_TYPE_ANCHOR_SHARING_TOKEN_ANDROID) \
    _unavail(XrSystemAnchorSharingExportPropertiesANDROID, XR_TYPE_SYSTEM_ANCHOR_SHARING_EXPORT_PROPERTIES_ANDROID) \
    _unavail(XrSystemSceneMeshingPropertiesANDROID, XR_TYPE_SYSTEM_SCENE_MESHING_PROPERTIES_ANDROID) \
    _unavail(XrSceneMeshingTrackerCreateInfoANDROID, XR_TYPE_SCENE_MESHING_TRACKER_CREATE_INFO_ANDROID) \
    _unavail(XrSceneMeshAcquireInfoANDROID, XR_TYPE_SCENE_MESH_ACQUIRE_INFO_ANDROID) \
    _unavail(XrSceneMeshReleaseInfoANDROID, XR_TYPE_SCENE_MESH_RELEASE_INFO_ANDROID) \
    _unavail(XrSceneSubmeshANDROID, XR_TYPE_SCENE_SUBMESH_ANDROID) \
    _unavail(XrSceneMeshANDROID, XR_TYPE_SCENE_MESH_ANDROID) \
    _unavail(XrInstanceCreateInfoBackgroundTrackingANDROIDSYS, XR_TYPE_INSTANCE_CREATE_INFO_BACKGROUND_TRACKING_ANDROIDSYS) \
    _unavail(XrSharedAnchorSpaceCreateInfoANDROIDSYS, XR_TYPE_SHARED_ANCHOR_SPACE_CREATE_INFO_ANDROIDSYS) \
    _unavail(XrInputTracingDataANDROIDSYS, XR_TYPE_INPUT_TRACING_DATA_ANDROIDSYS) \

#endif

#if defined(XR_USE_PLATFORM_EGL)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_EGL(_avail, _unavail) \
    _avail(XrGraphicsBindingEGLMNDX, XR_TYPE_GRAPHICS_BINDING_EGL_MNDX) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_EGL(_avail, _unavail) \
    _unavail(XrGraphicsBindingEGLMNDX, XR_TYPE_GRAPHICS_BINDING_EGL_MNDX) \

#endif

#if defined(XR_USE_PLATFORM_ML)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_ML(_avail, _unavail) \
    _avail(XrCoordinateSpaceCreateInfoML, XR_TYPE_COORDINATE_SPACE_CREATE_INFO_ML) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_ML(_avail, _unavail) \
    _unavail(XrCoordinateSpaceCreateInfoML, XR_TYPE_COORDINATE_SPACE_CREATE_INFO_ML) \

#endif

#if defined(XR_USE_PLATFORM_WIN32)
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_WIN32(_avail, _unavail) \
    _avail(XrHolographicWindowAttachmentMSFT, XR_TYPE_HOLOGRAPHIC_WINDOW_ATTACHMENT_MSFT) \

#else
#define _impl_XR_LIST_ALL_STRUCTURE_TYPES_XR_USE_PLATFORM_WIN32(_avail, _unavail) \
    _unavail(XrHolographicWindowAttachmentMSFT, XR_TYPE_HOLOGRAPHIC_WINDOW_ATTACHMENT_MSFT) \

#endif




#endif

