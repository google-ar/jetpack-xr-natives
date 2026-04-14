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
  PACKAGE_ARCORE,
  PACKAGE_ARCORE_OPENXR,
  PACKAGE_ARCORE_RUNTIME,
  PACKAGE_MATH,
  PACKAGE_PERCEPTION,
  PACKAGE_CORE,
  PACKAGE_CORE_INTERFACES,
};

inline std::string GetJxrFullClassName(JNIEnv* env, Package package,
                                       std::string class_name) {
  std::string package_name;
  switch (package) {
    case PACKAGE_ARCORE_OPENXR:
      package_name = "androidx/xr/arcore/openxr/";
      break;
    case PACKAGE_ARCORE:
      package_name = "androidx/xr/arcore/";
      break;
    case PACKAGE_ARCORE_RUNTIME:
      package_name = "androidx/xr/arcore/runtime/";
      break;
    case PACKAGE_MATH:
      package_name = "androidx/xr/runtime/math/";
      break;
    case PACKAGE_PERCEPTION:
      package_name = "androidx/xr/scenecore/impl/perception/";
      break;
    case PACKAGE_CORE:
      package_name = "androidx/xr/runtime/";
      break;
    case PACKAGE_CORE_INTERFACES:
      package_name = "androidx/xr/runtime/interfaces/";
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
