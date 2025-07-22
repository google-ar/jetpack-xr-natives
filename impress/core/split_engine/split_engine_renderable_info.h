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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERABLE_INFO_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERABLE_INFO_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/ncsb/component.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_filament_resource_ptrs.h"

namespace imp::split_engine {
// Component for storing additional information about the renderable added to a
// node.
//
// Used to track things like mesh data to do per-triangle collision detection.
class SplitEngineRenderableInfo : public Component {
 public:
  absl::Status Setup(BridgeId bridge_id);

  absl::Status SetPrimitiveMeshData(size_t primitive_index,
                                    MeshVertexData* vertex_data,
                                    MeshIndexData* index_data);

  absl::Status SetMaterialInstance(size_t primitive_index,
                                   BorrowedMaterialPtr material_instance);

  absl::Span<const MeshVertexAndIndexData> GetAllMeshData();

  void UpdateBones(const std::vector<mat4f>& bone_transforms);

  void SetMorphTargetBuffer(BorrowedMorphTargetBufferPtr morph_target_buffer);

  void SetVertexBuffer(BorrowedVertexBufferPtr vertex_buffer);

  void SetIndexBuffer(BorrowedIndexBufferPtr index_buffer);

  void SetSkinningBoneCount(uint32_t skinning_bone_count) noexcept;
  uint32_t GetSkinningBoneCount() const noexcept;

  BridgeId GetBridgeId() const noexcept;

  template <typename Fn>
  void ForEachTexture(
      Fn fn, SmallSourceLocation loc = SmallSourceLocation::Current()) {
    for (auto& primitive : primitives_) {
      if (primitive.material_instance) {
        primitive.material_instance->ForEachTexture(fn, loc);
      }
    }
  }

 private:
  struct PrimitiveInfo {
    // Unowned pointers to the unskinned mesh data for this primitive.
    MeshVertexAndIndexData unskinned_mesh_data;
    // Copy of the vertex data for this primitive with skinning applied.
    MeshVertexDataPtr skinned_vertex_data;
    // The material instance id for this primitive.
    BorrowedMaterialPtr material_instance;
  };

  std::vector<PrimitiveInfo> primitives_;

  // Stores the final available mesh data for all primitives so that it's easily
  // accessible for collision detection.
  std::optional<std::vector<MeshVertexAndIndexData>> all_mesh_data_;

  // Tracks the morph target buffer used by this renderable.
  BorrowedMorphTargetBufferPtr morph_target_buffer_;

  // Tracks the vertex buffer used by this renderable.
  BorrowedVertexBufferPtr vertex_buffer_;

  // Tracks the index buffer used by this renderable.
  BorrowedIndexBufferPtr index_buffer_;

  uint32_t skinning_bone_count_ = 0;

  // This should always be set, so setting a default to garbage is a hint that
  // something bad is happening.
  BridgeId bridge_id_ = UINT64_MAX - 1337;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERABLE_INFO_H_
