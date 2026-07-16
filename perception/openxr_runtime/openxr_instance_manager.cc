#include "openxr_runtime/openxr_instance_manager.h"

#include <jni.h>
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

#include "openxr/openxr.h"
#include "absl/log/log.h"
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
// LINT.IfChange
const std::array<OpenXrExtension, 8> kOptionalExtensions = {{
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
    {XR_ANDROID_TRACKABLES_IMAGE_EXTENSION_NAME,
     {XR_EXT_FUTURE_EXTENSION_NAME}},
}};
// LINT.ThenChange(//depot/google3/third_party/jetpack_xr_natives/openxr/openxr_manager.cc)

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


OpenXrInstanceManager::~OpenXrInstanceManager() { DestroyInstance(); }

XrInstance OpenXrInstanceManager::GetInstance(JNIEnv* env, jobject context) {
  absl::MutexLock lock(mutex_);
  if (instance_ == XR_NULL_HANDLE) {
    if (!CreateInstance(env, context)) {
      return XR_NULL_HANDLE;
    }
  }
  return instance_;
}

bool OpenXrInstanceManager::LoadOpenXr(jobject context) {
  PFN_xrInitializeLoaderKHR initialize_loader = nullptr;

  // Gets a function pointer to the OpenXR loader.
  XR_RETURN_IF_FAILED(
      xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                            (PFN_xrVoidFunction*)(&initialize_loader)));
  if (initialize_loader == nullptr) {
    return false;
  }
  XrLoaderInitInfoAndroidKHR loader_init_info_android;
  {
    loader_init_info_android = {
        .type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR,
        .applicationVM = app_vm_,
        .applicationContext = context,
    };
  }

  // Call the loader function obtained above to load OpenXR.
  XR_RETURN_IF_FAILED(
      initialize_loader(reinterpret_cast<const XrLoaderInitInfoBaseHeaderKHR*>(
          &loader_init_info_android)));
  return true;
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

bool OpenXrInstanceManager::IsHandTrackingSupported() {
  absl::MutexLock lock(mutex_);
  if (!GetXrSystem()) {
    LOG(ERROR) << "Unable to retrieve system ID.";
    return false;
  }

  XrSystemHandTrackingPropertiesEXT handTrackingProperties = {
    .type = XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT,
  };

  XrSystemProperties systemProperties = {
      .type = XR_TYPE_SYSTEM_PROPERTIES,
      .next = &handTrackingProperties,
  };

  XR_RETURN_IF_FAILED(
      xrGetSystemProperties(instance_, system_id_, &systemProperties));
  return handTrackingProperties.supportsHandTracking;
}

bool OpenXrInstanceManager::IsEyeTrackingSupported() {
  absl::MutexLock lock(mutex_);
  if (!GetXrSystem()) {
    LOG(ERROR) << "Unable to retrieve system ID.";
    return false;
  }

  XrSystemEyeTrackingPropertiesANDROID eyeTrackingProperties = {
    .type = XR_TYPE_SYSTEM_EYE_TRACKING_PROPERTIES_ANDROID,
  };

  XrSystemProperties systemProperties = {
      .type = XR_TYPE_SYSTEM_PROPERTIES,
      .next = &eyeTrackingProperties,
  };

  XR_RETURN_IF_FAILED(
      xrGetSystemProperties(instance_, system_id_, &systemProperties));
  return eyeTrackingProperties.supportsEyeTracking;
}

bool OpenXrInstanceManager::IsDepthTrackingSupported() {
  absl::MutexLock lock(mutex_);
  if (!GetXrSystem()) {
    LOG(ERROR) << "Unable to retrieve system ID.";
    return false;
  }

  XrSystemDepthTrackingPropertiesANDROID depthTrackingProperties = {
    .type = XR_TYPE_SYSTEM_DEPTH_TRACKING_PROPERTIES_ANDROID,
  };

  XrSystemProperties systemProperties = {
      .type = XR_TYPE_SYSTEM_PROPERTIES,
      .next = &depthTrackingProperties,
  };

  XR_RETURN_IF_FAILED(
      xrGetSystemProperties(instance_, system_id_, &systemProperties));
  return depthTrackingProperties.supportsDepthTracking;
}

bool OpenXrInstanceManager::IsGeospatialSupported() {
  absl::MutexLock lock(mutex_);
  if (!GetXrSystem()) {
    LOG(ERROR) << "Unable to retrieve system ID.";
    return false;
  }

  XrSystemGeospatialPropertiesANDROID geospatialProperties = {
    .type = XR_TYPE_SYSTEM_GEOSPATIAL_PROPERTIES_ANDROID,
  };

  XrSystemProperties systemProperties = {
      .type = XR_TYPE_SYSTEM_PROPERTIES,
      .next = &geospatialProperties,
  };

  XR_RETURN_IF_FAILED(
      xrGetSystemProperties(instance_, system_id_, &systemProperties));
  return geospatialProperties.supportsGeospatial;
}

bool OpenXrInstanceManager::IsRenderingModeSupported(RenderingMode mode) {
  absl::MutexLock lock(mutex_);
  if (!GetXrSystem()) {
    LOG(ERROR) << "Unable to retrieve system ID.";
    return false;
  }

  XrViewConfigurationType requestedType = XR_VIEW_CONFIGURATION_TYPE_MAX_ENUM;
  switch (mode) {
    case kMono:
      requestedType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO;
      break;
    case kStereo:
      requestedType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
      break;
    default:
      LOG(ERROR) << "Unknown rendering mode.";
      return false;
  }

  std::vector<XrViewConfigurationType> view_configuration_types;
  uint32_t view_configuration_count;
  XR_RETURN_IF_FAILED(xrEnumerateViewConfigurations(
      instance_, system_id_, /*viewConfigurationTypeCapacityInput=*/0,
      &view_configuration_count, /*viewConfigurationTypes=*/nullptr));
  view_configuration_types.resize(view_configuration_count);
  XR_RETURN_IF_FAILED(xrEnumerateViewConfigurations(
      instance_, system_id_, view_configuration_count,
      &view_configuration_count, view_configuration_types.data()));
  for (const auto& view_configuration_type : view_configuration_types) {
    if (view_configuration_type == requestedType) {
      return true;
    }
  }
  return false;
}

bool OpenXrInstanceManager::CreateInstance(JNIEnv* env, jobject context) {
  java_env_ = env;
  java_env_->GetJavaVM(&app_vm_);

  if (!LoadOpenXr(context)) {
    return false;
  }

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

  // Retrieve the system ID.
  if (!GetXrSystem()) {
    return false;
  }

  return true;
}

bool OpenXrInstanceManager::GetXrSystem() {
  if (system_id_ != XR_NULL_SYSTEM_ID) {
    return true;
  }

  // TODO: (broken link) - Update this to dynamically evaluate the form factor
  // once we support multiple form factors.
  XrSystemGetInfo system_info = {
      .type = XR_TYPE_SYSTEM_GET_INFO,
      .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY,
  };

  XR_RETURN_IF_FAILED(xrGetSystem(instance_, &system_info, &system_id_));
  if (system_id_ == XR_NULL_SYSTEM_ID) {
    return false;
  }

  return true;
}

void OpenXrInstanceManager::DestroyInstance() {
  absl::MutexLock lock(mutex_);
  if (instance_ != XR_NULL_HANDLE) {
    xrDestroyInstance(instance_);
    instance_ = XR_NULL_HANDLE;
    system_id_ = XR_NULL_SYSTEM_ID;
  }
}

PFN_xrGetInstanceProcAddr OpenXrInstanceManager::GetGetInstanceProcAddr()
    const {
  return xrGetInstanceProcAddr;
}

std::vector<XrEnvironmentBlendMode>
OpenXrInstanceManager::GetEnvironmentBlendModes(XrInstance instance) {
  absl::MutexLock lock(mutex_);
  if (instance_ == XR_NULL_HANDLE || !GetXrSystem()) {
    return {};
  }

  uint32_t blend_mode_count;
  if (XR_FAILED(xrEnumerateEnvironmentBlendModes(
          instance_, system_id_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0,
          &blend_mode_count, nullptr))) {
    return {};
  }

  std::vector<XrEnvironmentBlendMode> blend_modes(blend_mode_count);
  if (XR_FAILED(xrEnumerateEnvironmentBlendModes(
          instance_, system_id_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
          blend_mode_count, &blend_mode_count, blend_modes.data()))) {
    return {};
  }

  return blend_modes;
}

}  // namespace androidx::xr::openxr
