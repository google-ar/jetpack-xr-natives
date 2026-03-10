// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <jni.h>  // IWYU pragma: keep
#include <openxr/openxr_platform.h>
#include <openxr/openxr_platform_defines.h>
#include <openxr/public/all_extensions.h>
#include <openxr/public/xr_androidx2_geospatial_streetscape.h>

#include <cstdint>
#include <vector>

namespace {
const XrInstance kInstance = XrInstance(1111);
}  // namespace

extern "C" {

const std::vector<XrExtensionProperties> kExtensions = {
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_ANCHOR_SHARING_EXPORT_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_DEPTH_TEXTURE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_DEVICE_ANCHOR_PERSISTENCE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_EYE_TRACKING_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_FACE_TRACKING_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr, XR_ANDROID_RAYCAST_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_TRACKABLES_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_TRACKABLES_OBJECT_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr, XR_EXT_FUTURE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_EXT_HAND_TRACKING_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr, XR_MND_HEADLESS_EXTENSION_NAME},
    // Geospatial extensions
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_GEOSPATIAL_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_GEOSPATIAL_ANCHOR_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROIDX2_GEOSPATIAL_STREETSCAPE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_SPATIAL_ANCHOR_SPACE_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME},
    {XR_TYPE_EXTENSION_PROPERTIES, nullptr,
     XR_ANDROID_GOOGLE_CLOUD_AUTH_EXTENSION_NAME},
};

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateInstanceExtensionProperties(
    const char* layerName, uint32_t propertyCapacityInput,
    uint32_t* propertyCountOutput, XrExtensionProperties* properties) {
  *propertyCountOutput = kExtensions.size();
  if (propertyCapacityInput == 0) {
    return XR_SUCCESS;
  }
  if (propertyCapacityInput < kExtensions.size()) {
    return XR_ERROR_SIZE_INSUFFICIENT;
  }

  for (int i = 0; i < kExtensions.size(); ++i) {
    properties[i] = kExtensions[i];
  }
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL
xrCreateInstance(const XrInstanceCreateInfo* createInfo, XrInstance* instance) {
  *instance = kInstance;
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyInstance(XrInstance instance) {
  return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetInstanceProcAddr(
    XrInstance instance, const char* name, PFN_xrVoidFunction* function) {
  *function = nullptr;
  return XR_SUCCESS;
}

}  // extern "C"
