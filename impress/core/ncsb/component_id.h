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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_ID_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_ID_H_

#include <array>

#include "absl/base/attributes.h"
#include "core/common/hash.h"
#include "core/ncsb/update_id.h"

namespace imp {

// Used to refer to a component type at runtime.
using ComponentId = HashValue;

// Used to get a Component's id so that it can be used at runtime.
template <typename T>
constexpr ComponentId kComponentId = type_traits::kTypeHash<T>;

template <typename... Components>
using ComponentIds ABSL_DEPRECATED("Use imp::UpdateIds<T> instead.") =
    UpdateIds<Components...>;

// Helper for representing a list of component ids at compile time to specify
// the order components are cleaned up in.
//
// Ex:
//   using CleanupDependencies = CleanupIds<Foo>;
//
// For more details:
//   third_party/impress/core/ncsb/component.h
//   third_party/impress/core/ncsb/component_test.cc
template <typename... Args>
struct CleanupIds {
  static constexpr std::array<ComponentId, sizeof...(Args)> kIds{
      kComponentId<Args>...};
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_ID_H_
