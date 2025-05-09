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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_DETAILS_ANIMATION_RESOURCES_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_DETAILS_ANIMATION_RESOURCES_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/animation/gltf_animation.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"

namespace imp::loader::details {

OptionalError VerifyAnimations(const schemas::LoadedModel* loaded_model);

std::vector<absl::string_view> GetAnimationNames(
    const schemas::LoadedModel* loaded_model);

absl::StatusOr<std::unique_ptr<animation::GltfAnimation>>
CreateAnimationResources(const schemas::LoadedModel* loaded_model,
                         size_t animation_index);

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_DETAILS_ANIMATION_RESOURCES_H_
