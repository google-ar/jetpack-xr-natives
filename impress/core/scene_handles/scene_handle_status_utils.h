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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_SCENE_HANDLE_STATUS_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_SCENE_HANDLE_STATUS_UTILS_H_

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "core/common/type_traits.h"

namespace imp {

// Helper function for checking that a SceneHandle within a component's State
// proto has been assigned and returnin an error if it hasn't.
//
// If the SceneHandle is assigned, returns an Ok status.
// Otherwise, returns an error including the type of scene handle and the name
// of the field.
//
// Example Usage:
//  MP_RETURN_IF_ERROR(IsHandleValid<&FooState::bar>(state_));
template <auto scene_handle_field, typename StateT>
absl::Status IsHandleValid(const StateT& state) {
  if (state.*scene_handle_field) {
    return absl::OkStatus();
  }

  using SceneHandleT = decltype(state.*scene_handle_field);
  return absl::FailedPreconditionError(absl::StrFormat(
      "%s %s is unassigned.", type_traits::kTypeName<SceneHandleT>,
      type_traits::kFieldName<scene_handle_field>));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_SCENE_HANDLE_STATUS_UTILS_H_
