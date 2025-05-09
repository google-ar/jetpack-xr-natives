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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_ANIMATION_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_ANIMATION_H_

#include "core/common/optional_error.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/math/math.h"
#include "core/model/joint_data.h"

namespace imp::loader::details::provider_gltf {

/// Populates |out_anim| with animation |data| that refers to the TinyGLTF
/// |animation| and |model|. This include allocating splines, populating them
/// with curve data, and creating the matrix operations that drive the
/// animation.

OptionalError GetInverseBindPoses(const imp::gltf::Gltf& gltf,
                                  const imp::gltf::Skin& skin,
                                  model::SampledJointLookup<mat4f>* out_poses);

}  // namespace imp::loader::details::provider_gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_ANIMATION_H_
