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

#ifndef JETPACK_XR_NATIVES_COMMON_POINTER_UTIL_H_
#define JETPACK_XR_NATIVES_COMMON_POINTER_UTIL_H_

#include <jni.h>  // IWYU pragma: keep
#include <cstdint>

namespace androidx::xr::common {

inline jlong PointerToJLong(void* ptr) {
  return static_cast<jlong>(reinterpret_cast<intptr_t>(ptr));
}

// Converts from jlong back to pointer type. Default to void* for backward
// compatibility.
template <class T = void>
inline T* PointerFromJLong(jlong ptr) {
  return reinterpret_cast<T*>(static_cast<intptr_t>(ptr));
}

}  // namespace androidx::xr::common

#endif  // JETPACK_XR_NATIVES_COMMON_POINTER_UTIL_H_
