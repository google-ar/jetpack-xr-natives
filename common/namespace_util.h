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

#ifndef JETPACK_XR_NATIVES_COMMON_NAMESPACE_UTIL_H_
#define JETPACK_XR_NATIVES_COMMON_NAMESPACE_UTIL_H_

#include <jni.h>  // IWYU pragma: keep

#include <string>

namespace androidx::xr::common {

enum Package {
  PACKAGE_OPENXR,
  PACKAGE_RUNTIME,
  PACKAGE_MATH,
  PACKAGE_PERCEPTION,
};

inline bool IsLegacyNamespace(JNIEnv* env) {
  static bool* is_legacy_namespace = nullptr;
  if (is_legacy_namespace == nullptr) {
    is_legacy_namespace =
        new bool(env->FindClass("androidx/xr/runtime/math/Pose") == nullptr);
    // Clear the exception so it is not propagated to the JVM.
    env->ExceptionClear();
  }
  return *is_legacy_namespace;
}

inline std::string GetJxrFullClassName(JNIEnv* env, Package package,
                                       std::string class_name) {
  std::string package_name;
  switch (package) {
    case PACKAGE_OPENXR:
      package_name = IsLegacyNamespace(env) ? "androidx/xr/openxr/"
                                            : "androidx/xr/runtime/openxr/";
      break;
    case PACKAGE_RUNTIME:
      package_name = IsLegacyNamespace(env) ? "androidx/xr/runtime/"
                                            : "androidx/xr/runtime/internal/";
      break;
    case PACKAGE_MATH:
      package_name = IsLegacyNamespace(env) ? "androidx/xr/math/"
                                            : "androidx/xr/runtime/math/";
      break;
    case PACKAGE_PERCEPTION:
      package_name =
          IsLegacyNamespace(env)
              ? "com/google/vr/realitycore/runtime/androidxr/perception/"
              : "androidx/xr/scenecore/impl/perception/";
      break;
  }
  return package_name + class_name;
}

inline jclass GetJxrClass(JNIEnv* env, Package package,
                          std::string class_name) {
  return env->FindClass(GetJxrFullClassName(env, package, class_name).c_str());
}

}  // namespace androidx::xr::common

#endif  // JETPACK_XR_NATIVES_COMMON_NAMESPACE_UTIL_H_
