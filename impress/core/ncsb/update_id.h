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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_UPDATE_ID_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_UPDATE_ID_H_

#include <array>

#include "core/common/hash.h"
#include "core/common/type_traits.h"
#include "core/ncsb/update_phase.h"
#include "core/ncsb/updater_traits.h"

namespace imp {

using UpdateId = HashValue;

template <typename T>
constexpr UpdateId kUpdateId = type_traits::kTypeHash<T>;

// Helper for representing a list of updater ids at compile time.
//
// Used to specify the order the Update methods are called in.
//
// Ex:
//   using UpdateDependencies = UpdateIds<Foo>;
//
// For more details:
//   third_party/impress/core/ncsb/component.h
//   third_party/impress/core/ncsb/component_test.cc
template <typename... Args>
struct UpdateIds {
  static constexpr std::array<UpdateId, sizeof...(Args)> kIds{
      kUpdateId<Args>...};
  // A list of all the UpdatePhase associated with each args. Used to validate
  // that they are all are within the same phase.
  static constexpr std::array<UpdatePhase, sizeof...(Args)> kPhases{
      updater_traits::kUpdatePhaseOrDefault<Args>...};
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_UPDATE_ID_H_
