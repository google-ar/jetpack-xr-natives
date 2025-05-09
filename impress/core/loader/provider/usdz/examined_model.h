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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_EXAMINED_MODEL_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_EXAMINED_MODEL_H_

#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/model/entity_data.h"
#include "third_party/tinyusdz/src/prim-types.hh"
#include "third_party/tinyusdz/src/stage.hh"

namespace imp::loader::details::provider_usdz {

// Analogous to GltfLookup; provides a structured view into the relevant
// contents of a USDZ file to simplify the structure of processing.
struct ExaminedModel {
  template <typename T>
  using BoneLookup = LoadedModelBuilder::BoneLookup<T>;
  template <typename T>
  using EntityLookup = LoadedModelBuilder::EntityLookup<T>;
  template <typename T>
  using MaterialLookup = model::MaterialLookup<T>;

  using BoneId = LoadedModelBuilder::BoneId;
  using WeakBoneId = LoadedModelBuilder::WeakBoneId;
  using WeakEntityId = LoadedModelBuilder::WeakEntityId;
  using EntityId = LoadedModelBuilder::EntityId;

  BoneLookup<const tinyusdz::Prim *> bone_prims;
  BoneLookup<uint32_t> bone_child_counts;
  EntityLookup<BoneId> entity_bones;
  EntityLookup<const tinyusdz::Prim *> entity_prims;
  EntityLookup<uint32_t> entity_child_counts;
  MaterialLookup<const tinyusdz::Prim *> material_prims;

  static absl::StatusOr<ExaminedModel> FromStage(const tinyusdz::Stage &stage);

  // Private constructed and move-only
  ExaminedModel(const ExaminedModel &) = delete;
  ExaminedModel &operator=(const ExaminedModel &rhs) = delete;
  ExaminedModel(ExaminedModel &&rhs) = default;
  ExaminedModel &operator=(ExaminedModel &&rhs) = default;

 private:
  ExaminedModel() = default;

  static absl::Status ExaminePrim(const tinyusdz::Prim &prim,
                                  ExaminedModel &model,
                                  WeakBoneId parent_bone = {},
                                  WeakEntityId parent_entity = {});
};

}  // namespace imp::loader::details::provider_usdz

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_EXAMINED_MODEL_H_
