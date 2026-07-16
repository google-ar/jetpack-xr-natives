// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/split_engine/split_engine_renderable_info.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "core/config.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"
#include "core/ncsb/node.h"
#include "core/render/base_renderable_manager.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/skinning_helpers.h"
#include "core/split_engine/split_engine_filament_resource_ptrs.h"

#if IMP_RUNTIME(DEV)
#include <cfloat>
#include <cstdio>

#include "dear_imgui/imgui.h"
#include "core/common/small_source_location.h"
#include "core/render/texture.h"
#endif  // IMP_RUNTIME(DEV)

namespace imp::split_engine {

namespace {

using RenderableInstance = filament::RenderableManager::Instance;

}  // namespace

absl::Status SplitEngineRenderableInfo::Setup(BridgeId bridge_id) {
  BaseRenderableManager& rm = GetView().GetRenderableManager();
  RenderableInstance renderable = rm.GetInstance(GetEntity());
  if (!renderable) {
    return absl::FailedPreconditionError(absl::StrFormat(
        "Cannot add SplitEngineRenderableInfo to a node without a "
        "renderable: %d",
        GetNode()->GetEntity().getId()));
  }

  size_t primitive_count =
      GetView().GetRenderableManager().GetPrimitiveCount(renderable);
  primitives_.resize(primitive_count);

  bridge_id_ = bridge_id;

  return absl::OkStatus();
}

absl::Status SplitEngineRenderableInfo::SetPrimitiveMeshData(
    size_t primitive_index, MeshVertexData* vertex_data,
    MeshIndexData* index_data) {
  if (primitive_index >= primitives_.size()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Primitive index %d is out of bounds.", primitive_index));
  }

  primitives_[primitive_index].unskinned_mesh_data =
      MeshVertexAndIndexData{vertex_data, index_data};

  // If there is vertex data and it has bone indices, create a skinned vertex
  // data object so that we can store the skinned data.
  if (vertex_data) {
    const VertexFormat kVertexFormat =
        vertex_data->GetDescription().vertex_format;
    const std::optional<size_t> bone_indices_attr_id =
        kVertexFormat.GetIndexForAttribute(
            VertexFormat::VertexAttribute::BONE_INDICES);
    if (bone_indices_attr_id) {
      primitives_[primitive_index].skinned_vertex_data =
          std::make_unique<MeshVertexData>(vertex_data->GetDescription());
    }
  } else {
    // Reset the skinned vertex data if there is no vertex data.
    primitives_[primitive_index].skinned_vertex_data.reset();
  }

  // Reset this so that it can be re-created on the next call to GetAllMeshData.
  all_mesh_data_.reset();

  return absl::OkStatus();
}

void SplitEngineRenderableInfo::UpdateBones(
    const std::vector<mat4f>& bone_transforms) {
  for (PrimitiveInfo& primitive : primitives_) {
    if (!primitive.unskinned_mesh_data.vertex_data ||
        !primitive.unskinned_mesh_data.index_data ||
        !primitive.skinned_vertex_data) {
      continue;
    }

    UpdateSkinning(primitive.unskinned_mesh_data,
                   *primitive.skinned_vertex_data, bone_transforms);
  }
}

void SplitEngineRenderableInfo::SetMorphTargetBuffer(
    BorrowedMorphTargetBufferPtr morph_target_buffer) {
  morph_target_buffer_ = morph_target_buffer;
}

void SplitEngineRenderableInfo::SetVertexBuffer(
    BorrowedVertexBufferPtr vertex_buffer) {
  vertex_buffer_ = vertex_buffer;
}

void SplitEngineRenderableInfo::SetIndexBuffer(
    BorrowedIndexBufferPtr index_buffer) {
  index_buffer_ = index_buffer;
}

absl::Span<const MeshVertexAndIndexData>
SplitEngineRenderableInfo::GetAllMeshData() {
  if (all_mesh_data_) {
    return *all_mesh_data_;
  }

  std::vector<MeshVertexAndIndexData>& mesh_data = all_mesh_data_.emplace();
  mesh_data.reserve(primitives_.size());

  // Gather together all the mesh data for each primitive if it's available.
  // It may not be available if the data wasn't saved outside of the GPU or
  // a primitive has no geometry assigned.
  for (const PrimitiveInfo& primitive : primitives_) {
    if (!primitive.unskinned_mesh_data.vertex_data ||
        !primitive.unskinned_mesh_data.index_data) {
      continue;
    }

    // Use the skinned vertex data if it exists, otherwise use the unskinned
    // vertex data.
    mesh_data.push_back({primitive.skinned_vertex_data
                             ? primitive.skinned_vertex_data.get()
                             : primitive.unskinned_mesh_data.vertex_data,
                         primitive.unskinned_mesh_data.index_data});
  }

  return mesh_data;
}

void SplitEngineRenderableInfo::SetSkinningBoneCount(
    uint32_t skinning_bone_count) noexcept {
  skinning_bone_count_ = skinning_bone_count;
}

void SplitEngineRenderableInfo::SetRenderableBounds(
    const imp::Box& renderable_bounds) {
  renderable_bounds_ = renderable_bounds;
}

uint32_t SplitEngineRenderableInfo::GetSkinningBoneCount() const noexcept {
  return skinning_bone_count_;
}

void SplitEngineRenderableInfo::SetChannel(uint8_t channel) {
  channel_ = channel;
}

std::optional<uint8_t> SplitEngineRenderableInfo::GetChannel() const {
  return channel_;
}

const imp::Box& SplitEngineRenderableInfo::GetRenderableBounds() const {
  return renderable_bounds_;
}

BridgeId SplitEngineRenderableInfo::GetBridgeId() const noexcept {
  return bridge_id_;
}

absl::Status SplitEngineRenderableInfo::SetMaterialInstance(
    size_t primitive_index, BorrowedMaterialPtr material_instance) {
  if (primitive_index >= primitives_.size()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Primitive index %u is out of bounds.", primitive_index));
  }

  primitives_[primitive_index].material_instance = material_instance;

  return absl::OkStatus();
}

#if IMP_RUNTIME(DEV)
void SplitEngineRenderableInfo::DrawEditorUi() {
  // Simple properties with label-value alignment.
  if (ImGui::BeginTable(
          "##RenderableInfoTable", 2,
          ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings)) {
    ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed, 140.0f);
    ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);

    auto draw_row = [](const char* label, const char* id, void* data,
                       ImGuiDataType type, const char* format) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(label);
      ImGui::TableNextColumn();
      ImGui::SetNextItemWidth(-FLT_MIN);
      ImGui::InputScalar(id, type, data, nullptr, nullptr, format,
                         ImGuiInputTextFlags_ReadOnly);
    };

    uint64_t bridge_id = bridge_id_;
    draw_row("Bridge ID", "##BridgeID", &bridge_id, ImGuiDataType_U64, "%llu");

    if (channel_.has_value()) {
      uint8_t channel = *channel_;
      draw_row("Channel", "##Channel", &channel, ImGuiDataType_U8, "%u");
    }

    uint32_t skinning_bone_count = skinning_bone_count_;
    draw_row("Skinning Bones", "##SkinningBones", &skinning_bone_count,
             ImGuiDataType_U32, "%u");

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("Bounds");
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("Center");
    ImGui::TableNextColumn();
    ImGui::SetNextItemWidth(-FLT_MIN);
    float center[3] = {renderable_bounds_.center.x, renderable_bounds_.center.y,
                       renderable_bounds_.center.z};
    ImGui::InputFloat3("##Center", center, "%.3f",
                       ImGuiInputTextFlags_ReadOnly);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("Half-Extent");
    ImGui::TableNextColumn();
    ImGui::SetNextItemWidth(-FLT_MIN);
    float half_extent[3] = {renderable_bounds_.halfExtent.x,
                            renderable_bounds_.halfExtent.y,
                            renderable_bounds_.halfExtent.z};
    ImGui::InputFloat3("##HalfExtent", half_extent, "%.3f",
                       ImGuiInputTextFlags_ReadOnly);

    auto draw_status = [](const char* label, const char* id, bool present) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(label);
      ImGui::TableNextColumn();
      ImGui::SetNextItemWidth(-FLT_MIN);
      char buf[16];
      snprintf(buf, sizeof(buf), "%s", present ? "Present" : "None");
      ImGui::InputText(id, buf, sizeof(buf), ImGuiInputTextFlags_ReadOnly);
    };

    draw_status("Vertex Buffer", "##VertexBuffer", vertex_buffer_ != nullptr);
    draw_status("Index Buffer", "##IndexBuffer", index_buffer_ != nullptr);
    draw_status("Morph Target Buffer", "##MorphBuffer",
                morph_target_buffer_ != nullptr);
    draw_status("Cached Mesh Data", "##CachedMeshData",
                all_mesh_data_.has_value());

    ImGui::EndTable();
  }

  ImGui::Separator();

  if (primitives_.empty()) {
    ImGui::TextUnformatted("No primitives");
    return;
  }

  if (ImGui::TreeNode("primitives", "Primitives (%zu)", primitives_.size())) {
    for (size_t i = 0; i < primitives_.size(); ++i) {
      const auto& prim = primitives_[i];
      ImGui::PushID(static_cast<int>(i));
      if (ImGui::TreeNode("Material", "Primitive %zu Material", i)) {
        const char* mat_name = prim.material_instance
                                   ? prim.material_instance->GetName().c_str()
                                   : "No Material";
        if (ImGui::BeginTable("##PrimTable", 2,
                              ImGuiTableFlags_Resizable |
                                  ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_SizingStretchProp)) {
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted("Material Name");
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(mat_name);

          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted("Textures");
          ImGui::TableNextColumn();

          if (prim.material_instance) {
            if (ImGui::BeginTable("##TextureTable", 2,
                                  ImGuiTableFlags_SizingFixedFit)) {
              prim.material_instance->ForEachTexture(
                  [&](BorrowedTexturePtr texture) {
                    ImGui::TableNextColumn();
                    if (texture->GetName().empty()) {
                      ImGui::TextUnformatted("<unnamed>");
                    } else {
                      ImGui::TextUnformatted(texture->GetName().data());
                    }
                  },
                  SmallSourceLocation::Current());
              ImGui::EndTable();
            }
          } else {
            ImGui::TextUnformatted("None");
          }

          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted("Unskinned Data");
          ImGui::TableNextColumn();
          if (prim.unskinned_mesh_data.vertex_data) {
            size_t v_count =
                prim.unskinned_mesh_data.vertex_data->GetDescription()
                    .vertex_count;
            size_t i_count =
                prim.unskinned_mesh_data.index_data
                    ? prim.unskinned_mesh_data.index_data->GetDescription()
                          .index_count
                    : 0;
            ImGui::Text("%zu verts, %zu ind", v_count, i_count);
          } else {
            ImGui::TextUnformatted("None");
          }

          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted("Skinned Data");
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(prim.skinned_vertex_data ? "Present" : "None");

          ImGui::EndTable();
        }
        ImGui::TreePop();
      }
      ImGui::PopID();
    }
    ImGui::TreePop();
  }
}
#endif  // IMP_RUNTIME(DEV)

}  // namespace imp::split_engine
