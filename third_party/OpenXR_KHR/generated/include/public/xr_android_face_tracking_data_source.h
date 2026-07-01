#ifndef XR_ANDROID_FACE_TRACKING_DATA_SOURCE_H_
#define XR_ANDROID_FACE_TRACKING_DATA_SOURCE_H_ 1

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


#ifndef XR_ANDROID_face_tracking_data_source

// XR_ANDROID_face_tracking_data_source is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_face_tracking_data_source 1
#define XR_TYPE_FACE_TRACKING_DATA_SOURCE_INFO_ANDROID ((XrStructureType) 1000706000U)
#define XR_TYPE_FACE_TRACKING_DATA_SOURCE_STATE_ANDROID ((XrStructureType) 1000706001U)

#define XR_ANDROID_face_tracking_data_source_SPEC_VERSION 1
#define XR_ANDROID_FACE_TRACKING_DATA_SOURCE_EXTENSION_NAME "XR_ANDROID_face_tracking_data_source"

typedef enum XrFaceTrackingDataSourceANDROID {
    // Indicates that this config uses image data
    XR_FACE_TRACKING_DATA_SOURCE_IMAGE_ANDROID = 1,
    // Indicates that this config uses audio data
    XR_FACE_TRACKING_DATA_SOURCE_AUDIO_ANDROID = 2,
    // Indicates that this config uses image and audio data
    XR_FACE_TRACKING_DATA_SOURCE_MULTIMODAL_ANDROID = 3,
    XR_FACE_TRACKING_DATA_SOURCE_MAX_ENUM_ANDROID = 0x7FFFFFFF
} XrFaceTrackingDataSourceANDROID;
// XrFaceTrackingDataSourceInfoANDROID extends XrFaceTrackerCreateInfoANDROID
typedef struct XrFaceTrackingDataSourceInfoANDROID {
    XrStructureType                           type;
    const void* XR_MAY_ALIAS                  next;
    uint32_t                                  requestedDataSourceCount;
    const XrFaceTrackingDataSourceANDROID*    requestedDataSources;
} XrFaceTrackingDataSourceInfoANDROID;

// XrFaceTrackingDataSourceStateANDROID extends XrFaceStateANDROID
typedef struct XrFaceTrackingDataSourceStateANDROID {
    XrStructureType                    type;
    void* XR_MAY_ALIAS                 next;
    XrFaceTrackingDataSourceANDROID    dataSource;
} XrFaceTrackingDataSourceStateANDROID;

typedef XrResult                                                                                 (XRAPI_PTR *PFN_xrEnumerateFaceTrackingDataSourcesANDROID)(XrInstance                                                                               instance, XrSystemId                                                                               systemId, uint32_t                                                                 supportedDataSourcesInputCapacity, uint32_t*                                                                                supportedDataSourcesOutputCount, XrFaceTrackingDataSourceANDROID* supportedDataSources);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult                                                                                 XRAPI_CALL xrEnumerateFaceTrackingDataSourcesANDROID(
    XrInstance                                  instance,
    XrSystemId                                  systemId,
    uint32_t                                    supportedDataSourcesInputCapacity,
    uint32_t*                                   supportedDataSourcesOutputCount,
    XrFaceTrackingDataSourceANDROID*            supportedDataSources);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */

// Reflection macros
#define XR_LIST_ENUM_XrFaceTrackingDataSourceANDROID(_) \
    _(XR_FACE_TRACKING_DATA_SOURCE_IMAGE_ANDROID, 1) \
    _(XR_FACE_TRACKING_DATA_SOURCE_AUDIO_ANDROID, 2) \
    _(XR_FACE_TRACKING_DATA_SOURCE_MULTIMODAL_ANDROID, 3) \
    _(XR_FACE_TRACKING_DATA_SOURCE_MAX_ENUM_ANDROID, 0x7FFFFFFF)

#define XR_LIST_STRUCT_XrFaceTrackingDataSourceInfoANDROID(_) \
    _(type) \
    _(next) \
    _(requestedDataSourceCount) \
    _(requestedDataSources)

#define XR_LIST_STRUCT_XrFaceTrackingDataSourceStateANDROID(_) \
    _(type) \
    _(next) \
    _(dataSource)

#define XR_LIST_FUNCTIONS_XR_ANDROID_face_tracking_data_source(_) \
    _(EnumerateFaceTrackingDataSourcesANDROID, ANDROID_face_tracking_data_source)

#endif /* XR_ANDROID_face_tracking_data_source */

#ifdef __cplusplus
}
#endif

#endif
