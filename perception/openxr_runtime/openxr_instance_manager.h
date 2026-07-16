#ifndef THIRD_PARTY_JETPACK_XR_NATIVES_OPENXR_RUNTIME_OPENXR_INSTANCE_MANAGER_H_
#define THIRD_PARTY_JETPACK_XR_NATIVES_OPENXR_RUNTIME_OPENXR_INSTANCE_MANAGER_H_
#include <jni.h>
#include <openxr/openxr_platform.h>
#include <openxr/public/all_extensions.h>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
namespace androidx::xr::openxr {

class OpenXrInstanceManager {
 public:
  OpenXrInstanceManager() = default;
  ~OpenXrInstanceManager();

  enum RenderingMode {
    kMono = 0,
    kStereo = 1,
  };

  // Returns OpenXR instance, creates it if it doesn't exist.
  XrInstance GetInstance(JNIEnv* env, jobject context)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Destroys OpenXR instance.
  void DestroyInstance() ABSL_LOCKS_EXCLUDED(mutex_);

  // Returns the address of the global xrGetInstanceProcAddr symbol.
  PFN_xrGetInstanceProcAddr GetGetInstanceProcAddr() const;

  // Returns the supported environment blend modes.
  std::vector<XrEnvironmentBlendMode> GetEnvironmentBlendModes(
      XrInstance instance) ABSL_LOCKS_EXCLUDED(mutex_);

  // Retrieves device support for hand tracking.
  bool IsHandTrackingSupported() ABSL_LOCKS_EXCLUDED(mutex_);

  // Retrieves device support for eye tracking.
  bool IsEyeTrackingSupported() ABSL_LOCKS_EXCLUDED(mutex_);

  // Retrieves device support for depth tracking.
  bool IsDepthTrackingSupported() ABSL_LOCKS_EXCLUDED(mutex_);

  // Retrieves device support for geospatial.
  bool IsGeospatialSupported() ABSL_LOCKS_EXCLUDED(mutex_);

  // Retrieves device support for the provided rendering mode.
  bool IsRenderingModeSupported(RenderingMode mode) ABSL_LOCKS_EXCLUDED(mutex_);

 private:
  // Gets the extensions to be loaded from the required and optional extensions.
  bool GetEnabledExtensions(std::vector<std::string>& enabled_exts);

  // Creates an OpenXR instance. The OpenXR runtime must first be loaded by
  // calling LoadOpenXr.
  bool CreateInstance(JNIEnv* env, jobject context)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Retrieves the XR system ID.
  bool GetXrSystem() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Loads the OpenXR runtime.
  bool LoadOpenXr(jobject context) ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  absl::Mutex mutex_;
  XrInstance instance_ ABSL_GUARDED_BY(mutex_) = XR_NULL_HANDLE;
  XrSystemId system_id_ ABSL_GUARDED_BY(mutex_) = XR_NULL_SYSTEM_ID;

  JNIEnv* java_env_ = nullptr;
  JavaVM* app_vm_ = nullptr;
};
}  // namespace androidx::xr::openxr

#endif  // THIRD_PARTY_JETPACK_XR_NATIVES_OPENXR_RUNTIME_OPENXR_INSTANCE_MANAGER_H_
