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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_UPDATER_TRAITS_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_UPDATER_TRAITS_H_

#include <type_traits>

#include "core/ncsb/update_phase.h"

namespace imp {

namespace updater_traits {

namespace internal {

// Uses SFINAE to detect if T has UpdateDependencies type defined.
template <typename T,
          std::enable_if_t<
              !std::is_same_v<void, typename T::UpdateDependencies>, int> = 0>
static constexpr bool AreUpdateDependenciesDefined(int) {
  return true;
}

template <typename T>
static constexpr bool AreUpdateDependenciesDefined(...) {
  return false;
}

// Uses SFINAE to detect if T has UpdateDependents type defined.
template <typename T,
          std::enable_if_t<!std::is_same_v<void, typename T::UpdateDependents>,
                           int> = 0>
static constexpr bool AreUpdateDependentsDefined(int) {
  return true;
}

template <typename T>
static constexpr bool AreUpdateDependentsDefined(...) {
  return false;
}

template <typename T, typename = void>
struct UpdatePhaseOrDefault {
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kDefault;
};

template <typename T>
struct UpdatePhaseOrDefault<T, decltype((void)T::kUpdatePhase, void())> {
  static constexpr UpdatePhase kUpdatePhase = T::kUpdatePhase;
};

}  // namespace internal

// Compile-time check to see if T has a nested type named
// UpdateDependencies defined.
template <typename T>
static constexpr bool kAreUpdateDependenciesDefined =
    internal::AreUpdateDependenciesDefined<T>(0);

// Compile-time check to see if T has a nested type named
// UpdateDependents defined.
template <typename T>
static constexpr bool kAreUpdateDependentsDefined =
    internal::AreUpdateDependentsDefined<T>(0);

// Compile-time check to get the kUpdatePhase field from T if it exists, and if
// it doesn't return the default update phase.
template <typename T>
static constexpr UpdatePhase kUpdatePhaseOrDefault =
    internal::UpdatePhaseOrDefault<T>::kUpdatePhase;

}  // namespace updater_traits

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_UPDATER_TRAITS_H_
