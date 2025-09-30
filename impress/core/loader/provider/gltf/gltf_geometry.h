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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_GEOMETRY_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_GEOMETRY_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/inlined_vector.h"
#include "core/common/filament_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/robin_set.h"
#include "core/geometry/shapes/box.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/loader/provider/gltf/gltf_lookup.h"
#include "core/math/mat.h"
#include "core/model/entity_data.h"

namespace imp::loader::details::provider_gltf {

struct ProcessedPrimitive {
  ProcessedPrimitive(
      std::vector<LoadedModelBuilder::VertexBlock>&& in_vertex_blocks,
      size_t in_vertex_count, LoadedModelBuilder::IndexBufferId in_index_buffer,
      Box in_root_bounds,
      LoadedModelBuilder::SkinningBufferId in_skinning_buffer,
      uint32_t in_morph_target_offset, uint32_t in_morph_target_count,
      std::vector<gltf::imp_proto::Primitive::FeatureIdTexture>
          in_feature_id_textures,
      RobinSet<int> in_required_materials)
      : vertex_blocks(std::move(in_vertex_blocks)),
        vertex_count(in_vertex_count),
        index_buffer(in_index_buffer),
        root_bounds(in_root_bounds),
        skinning_buffer(in_skinning_buffer),
        morph_target_offset(in_morph_target_offset),
        morph_target_count(in_morph_target_count),
        feature_id_textures(std::move(in_feature_id_textures)),
        required_materials(std::move(in_required_materials)) {}

  std::vector<LoadedModelBuilder::VertexBlock> vertex_blocks;
  size_t vertex_count;
  LoadedModelBuilder::IndexBufferId index_buffer;
  Box root_bounds;
  LoadedModelBuilder::SkinningBufferId skinning_buffer;
  uint32_t morph_target_offset;
  uint32_t morph_target_count;
  std::vector<gltf::imp_proto::Primitive::FeatureIdTexture> feature_id_textures;

  // Represents the materials that may be required to render this primitive
  // by their index within the glTF file. This is used later on to know
  // which materials need to be processed. This consists of the default
  // material for the primitive as well as any materials associated with
  // this primitive via the KHR_materials_variants extension.
  RobinSet<int> required_materials;

  // Move only
  ProcessedPrimitive(const ProcessedPrimitive&) = delete;
  ProcessedPrimitive& operator=(const ProcessedPrimitive& rhs) = delete;
  ProcessedPrimitive(ProcessedPrimitive&& rhs) = default;
  ProcessedPrimitive& operator=(ProcessedPrimitive&& rhs) = default;
};

// Compute vertex/index/morph target buffer Id's, per-material bounds (in root
// space), and mesh bounds (in mesh space).
OptionalError ProcessPrimitives(
    const gltf::imp_proto::Gltf& gltf,
    const std::vector<imp::gltf::imp_proto::Primitive>& primitives,
    const mat4& transform, uint16_t sampled_joint_count,
    LoadedModelBuilder* out_model_builder,
    GltfPrimitiveVector<ProcessedPrimitive>* out_processed_primitives,
    Box* out_bounds, model::MorphTargetBufferId* out_morph_target_buffer_id);

}  // namespace imp::loader::details::provider_gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_GEOMETRY_H_
