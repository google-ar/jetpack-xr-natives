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
  OpenXrInstanceManager();
  ~OpenXrInstanceManager();

  // Returns OpenXR instance, creates it if it doesn't exist.
  XrInstance GetInstance() ABSL_LOCKS_EXCLUDED(mutex_);

  // Destroys OpenXR instance.
  void DestroyInstance() ABSL_LOCKS_EXCLUDED(mutex_);

 private:
  // Gets the extensions to be loaded from the required and optional extensions.
  bool GetEnabledExtensions(std::vector<std::string>& enabled_exts);

  // Creates an OpenXR instance. The OpenXR runtime must first be loaded by
  // calling LoadOpenXr.
  bool CreateInstance() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);


  absl::Mutex mutex_;
  XrInstance instance_ ABSL_GUARDED_BY(mutex_);
};
}  // namespace androidx::xr::openxr

#endif  // THIRD_PARTY_JETPACK_XR_NATIVES_OPENXR_RUNTIME_OPENXR_INSTANCE_MANAGER_H_
