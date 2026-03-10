#include "openxr_runtime/openxr_instance_manager.h"

#include <openxr/openxr_platform.h>
#include <openxr/openxr_reflection.h>
#include <openxr/public/all_extensions.h>
#include <openxr/public/xr_androidx2_geospatial_streetscape.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "common/openxr_util.h"

namespace androidx::xr::openxr {
namespace {
// TODO: (broken link) - Inject the application name for the OpenXR session.
constexpr char kApplicationName[] = "JetpackXrCore";

struct OpenXrExtension {
  const char* name;
  std::vector<const char*> dependencies;
};

// TODO: (broken link) - Change this from a global list to something more
// flexible. Also split up between "required" and "optional" extensions, and
// check against xrEnumerateInstanceExtensionProperties()
const std::array<std::string, 13> kRequiredExtensions = {
    // (broken link) start
    XR_ANDROID_ANCHOR_SHARING_EXPORT_EXTENSION_NAME,
    XR_ANDROID_DEPTH_TEXTURE_EXTENSION_NAME,
    XR_ANDROID_DEVICE_ANCHOR_PERSISTENCE_EXTENSION_NAME,
    XR_ANDROID_EYE_TRACKING_EXTENSION_NAME,
    XR_ANDROID_FACE_TRACKING_EXTENSION_NAME,
    XR_ANDROID_RAYCAST_EXTENSION_NAME,
    XR_ANDROID_TRACKABLES_EXTENSION_NAME,
    XR_ANDROID_TRACKABLES_OBJECT_EXTENSION_NAME,
    XR_ANDROID_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME,
    XR_EXT_FUTURE_EXTENSION_NAME,
    XR_EXT_HAND_TRACKING_EXTENSION_NAME,
    XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME,
    XR_MND_HEADLESS_EXTENSION_NAME,
    // (broken link) end
};

// Extensions must be listed after their dependencies.
const std::array<OpenXrExtension, 7> kOptionalExtensions = {{
    {XR_ANDROID_GEOSPATIAL_EXTENSION_NAME, {XR_EXT_FUTURE_EXTENSION_NAME}},
    {XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME, {XR_EXT_FUTURE_EXTENSION_NAME}},
    {XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME,
     {XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME}},
    {XR_ANDROID_GEOSPATIAL_ANCHOR_EXTENSION_NAME,
     {XR_ANDROID_GEOSPATIAL_EXTENSION_NAME, XR_EXT_FUTURE_EXTENSION_NAME,
      XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME,
      XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME}},
    {XR_ANDROIDX2_GEOSPATIAL_STREETSCAPE_EXTENSION_NAME,
     {XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME}},
    {XR_ANDROID_SPATIAL_ANCHOR_SPACE_EXTENSION_NAME,
     {XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME}},
    {XR_ANDROID_GOOGLE_CLOUD_AUTH_EXTENSION_NAME,
     {XR_EXT_FUTURE_EXTENSION_NAME}},
}};

const std::array<std::string, 7> kGeospatialExtensions = {
    XR_ANDROID_GEOSPATIAL_EXTENSION_NAME,
    XR_ANDROID_GEOSPATIAL_ANCHOR_EXTENSION_NAME,
    XR_ANDROIDX2_GEOSPATIAL_STREETSCAPE_EXTENSION_NAME,
    XR_ANDROID_SPATIAL_ANCHOR_SPACE_EXTENSION_NAME,
    XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME,
    XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME,
    XR_ANDROID_GOOGLE_CLOUD_AUTH_EXTENSION_NAME,
};
}  // namespace


OpenXrInstanceManager::OpenXrInstanceManager() : instance_(XR_NULL_HANDLE) {}

OpenXrInstanceManager::~OpenXrInstanceManager() { DestroyInstance(); }

XrInstance OpenXrInstanceManager::GetInstance() {
  absl::MutexLock lock(mutex_);
  if (instance_ == XR_NULL_HANDLE) {
    if (!CreateInstance()) {
      return XR_NULL_HANDLE;
    }
  }
  return instance_;
}

bool OpenXrInstanceManager::GetEnabledExtensions(
    std::vector<std::string>& enabled_exts) {
  std::vector<XrExtensionProperties> available_ext_props;
  uint32_t property_count;
  XR_RETURN_IF_FAILED(xrEnumerateInstanceExtensionProperties(
      /*layerName=*/nullptr, /*propertyCapacityInput=*/0, &property_count,
      /*properties=*/nullptr));
  available_ext_props.resize(property_count);
  for (auto& prop : available_ext_props) {
    prop.type = XR_TYPE_EXTENSION_PROPERTIES;
  }
  XR_RETURN_IF_FAILED(xrEnumerateInstanceExtensionProperties(
      /*layerName=*/nullptr, property_count, &property_count,
      available_ext_props.data()));

  std::unordered_set<std::string_view> available_exts;
  for (const auto& prop : available_ext_props) {
    available_exts.insert(prop.extensionName);
  }

  enabled_exts.clear();
  for (const auto& required_ext : kRequiredExtensions) {
    enabled_exts.push_back(required_ext);
  }

  for (const auto& optional_ext : kOptionalExtensions) {
    if (available_exts.count(optional_ext.name)) {
      bool dependencies_met = true;
      for (const auto& dependency : optional_ext.dependencies) {
        if (std::find(enabled_exts.begin(), enabled_exts.end(), dependency) ==
            enabled_exts.end()) {
          dependencies_met = false;
          break;
        }
      }
      if (dependencies_met) {
        enabled_exts.push_back(optional_ext.name);
      }
    }
  }
  return true;
}

bool OpenXrInstanceManager::CreateInstance() {
  std::vector<std::string> enabled_exts_str;
  if (!GetEnabledExtensions(enabled_exts_str)) {
    return false;
  }

  std::vector<const char*> enabled_exts;
  enabled_exts.reserve(enabled_exts_str.size());
  for (const auto& ext : enabled_exts_str) {
    enabled_exts.push_back(ext.c_str());
  }

  XrInstanceCreateInfo create_info = {
      .type = XR_TYPE_INSTANCE_CREATE_INFO,
      .applicationInfo =
          {
              .apiVersion = XR_API_VERSION_1_0,
          },
      .enabledApiLayerCount = 0,
      .enabledApiLayerNames = nullptr,
      .enabledExtensionCount = static_cast<uint32_t>(enabled_exts.size()),
      .enabledExtensionNames = enabled_exts.data(),
  };
  strncpy(create_info.applicationInfo.applicationName, kApplicationName,
          XR_MAX_APPLICATION_NAME_SIZE);

  // Create an OpenXR instance.
  XR_RETURN_IF_FAILED(xrCreateInstance(&create_info, &instance_));

  return true;
}

void OpenXrInstanceManager::DestroyInstance() {
  absl::MutexLock lock(mutex_);
  if (instance_ != XR_NULL_HANDLE) {
    xrDestroyInstance(instance_);
    instance_ = XR_NULL_HANDLE;
  }
}

}  // namespace androidx::xr::openxr
