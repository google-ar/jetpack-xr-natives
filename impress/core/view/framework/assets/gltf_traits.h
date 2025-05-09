/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_TRAITS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_TRAITS_H_

#include <type_traits>

#include "core/ncsb/component_handle.h"

namespace imp {
namespace gltf_internal {

template <typename Extension, typename Arg,
          std::enable_if_t<std::is_same_v<bool, decltype(Extension::IsValidFor(
                                                    std::declval<Arg>()))>,
                           int> = 0>
static constexpr bool HasValidForFunc(int) {
  return true;
}

template <typename Extension, typename Arg>
static constexpr bool HasValidForFunc(...) {
  return false;
}

}  // namespace gltf_internal

template <typename Extension, typename Component>
constexpr bool kHasValidForFunc =
    gltf_internal::HasValidForFunc<Extension, ComponentHandle<Component>>(0);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_TRAITS_H_
