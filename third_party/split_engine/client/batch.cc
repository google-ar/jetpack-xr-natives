// Copyright 2026 Google LLC
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

#include "split_engine/client/batch.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/base.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "split_engine/flatbuffer_size_calculator.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_data_size_calculator_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_ipc_size_calculator_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_material_size_calculator_generated.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"
#include "split_engine/schemas/split_engine_render_passes_size_calculator_generated.h"
#include "split_engine/schemas/split_engine_schema_version.h"

namespace split_engine {

namespace {

static constexpr absl::string_view kTag = "[SplitEngineBatch]: ";
static constexpr absl::string_view kIndent = "  ";

}  // namespace

using flatbuffers::FlatBufferBuilder;
using flatbuffers::Offset;
using flatbuffers::Vector;

using android_xr::schemas::AddNode;
using android_xr::schemas::AddOrUpdateCollider;
using android_xr::schemas::AddRenderable;
using android_xr::schemas::AddTexturePipelineRenderer;
using android_xr::schemas::AssignUserIdToNode;
using android_xr::schemas::BuiltInMaterialInstanceParameters;
using android_xr::schemas::Command;
using android_xr::schemas::CommandTypes;
using android_xr::schemas::CommandTypesTraits;
using android_xr::schemas::CreateAddMaterialInstances;
using android_xr::schemas::CreateAddMaterials;
using android_xr::schemas::CreateAddMeshData;
using android_xr::schemas::CreateAddMorphTargetBuffers;
using android_xr::schemas::CreateAddNode;
using android_xr::schemas::CreateAddNodes;
using android_xr::schemas::CreateAddOrUpdateColliders;
using android_xr::schemas::CreateAddRenderables;
using android_xr::schemas::CreateAddTextures;
using android_xr::schemas::CreateAssignUserIdToNode;
using android_xr::schemas::CreateIndexBuffer;
using android_xr::schemas::CreateIndexBufferInfo;
using android_xr::schemas::CreateIndexBufferUpdate;
using android_xr::schemas::CreateMaterialInstance;
using android_xr::schemas::CreateMorphTargetAttributeInfo;
using android_xr::schemas::CreateMorphTargetBuffer;
using android_xr::schemas::CreateMorphTargetBufferInfo;
using android_xr::schemas::CreatePixelBuffer;
using android_xr::schemas::CreateRegisterNamedTextures;
using android_xr::schemas::CreateRemoveColliders;
using android_xr::schemas::CreateRemoveImageBasedLightingAssets;
using android_xr::schemas::CreateRemoveMaterialInstances;
using android_xr::schemas::CreateRemoveMaterials;
using android_xr::schemas::CreateRemoveMorphTargetBuffers;
using android_xr::schemas::CreateRemoveNode;
using android_xr::schemas::CreateRemoveNodes;
using android_xr::schemas::CreateRemoveRenderable;
using android_xr::schemas::CreateRemoveRenderables;
using android_xr::schemas::CreateRemoveTexturePipelineRenderer;
using android_xr::schemas::CreateRemoveTextures;
using android_xr::schemas::CreateSetBuiltInMaterialParameters;
using android_xr::schemas::CreateSetMaterialParameters;
using android_xr::schemas::CreateTexture;
using android_xr::schemas::CreateUnregisterNamedTexture;
using android_xr::schemas::CreateUpdateMeshData;
using android_xr::schemas::CreateUpdateNodes;
using android_xr::schemas::CreateUpdateRenderables;
using android_xr::schemas::CreateUpdateTexturePipelineRenderers;
using android_xr::schemas::CreateVertexBlockInfo;
using android_xr::schemas::CreateVertexBlockUpdate;
using android_xr::schemas::CreateVertexBuffer;
using android_xr::schemas::CreateVertexBufferInfo;
using android_xr::schemas::CreateVertexBufferUpdate;
using android_xr::schemas::DuplicateMaterialInstance;
using android_xr::schemas::IndexBufferUpdate;
using android_xr::schemas::Material;
using android_xr::schemas::MaterialInstance;
using android_xr::schemas::MaterialParameters;
using android_xr::schemas::RegisterNamedTexture;
using android_xr::schemas::RemoveCollider;
using android_xr::schemas::RemoveImageBasedLightingAssets;
using android_xr::schemas::RemoveMaterialInstances;
using android_xr::schemas::RemoveMaterials;
using android_xr::schemas::RemoveMeshData;
using android_xr::schemas::RemoveMorphTargetBuffers;
using android_xr::schemas::RemoveNode;
using android_xr::schemas::RemoveNodes;
using android_xr::schemas::RemoveRenderable;
using android_xr::schemas::RemoveTexturePipelineRenderer;
using android_xr::schemas::RemoveTextures;
using android_xr::schemas::SetMaterialParameters;
using android_xr::schemas::SetPreferredEnvironmentIblAsset;
using android_xr::schemas::UnregisterNamedTexture;
using android_xr::schemas::UnregisterNamedTextures;
using android_xr::schemas::UpdateMeshData;
using android_xr::schemas::UpdateNode;
using android_xr::schemas::UpdateRenderable;
using android_xr::schemas::UpdateTexturePipelineRenderer;
using android_xr::schemas::UpdateTexturePipelineRenderers;
using android_xr::schemas::VertexBlockUpdate;
using android_xr::schemas::VertexBufferUpdate;

using android_xr::schemas::AddMaterialInstances;
using android_xr::schemas::AddMaterials;
using android_xr::schemas::AddNodes;
using android_xr::schemas::AddOrUpdateColliders;
using android_xr::schemas::AddRenderables;
using android_xr::schemas::AddTexturePipelineRenderers;
using android_xr::schemas::AssignUserIdToNodes;
using android_xr::schemas::DuplicateMaterialInstances;
using android_xr::schemas::EstimateAddMeshDataSize;
using android_xr::schemas::EstimateAddMorphTargetBuffersSize;
using android_xr::schemas::EstimateAddNodeSize;
using android_xr::schemas::EstimateAddTexturesSize;
using android_xr::schemas::EstimateAssignUserIdToNodeSize;
using android_xr::schemas::EstimateIndexBufferInfoSize;
using android_xr::schemas::EstimateIndexBufferSize;
using android_xr::schemas::EstimateIndexBufferUpdateSize;
using android_xr::schemas::EstimateMaterialInstanceSize;
using android_xr::schemas::EstimateMorphTargetBufferInfoSize;
using android_xr::schemas::EstimateMorphTargetBufferSize;
using android_xr::schemas::EstimateRemoveImageBasedLightingAssetsSize;
using android_xr::schemas::EstimateRemoveMaterialInstancesSize;
using android_xr::schemas::EstimateRemoveMaterialsSize;
using android_xr::schemas::EstimateRemoveMorphTargetBuffersSize;
using android_xr::schemas::EstimateRemoveNodeSize;
using android_xr::schemas::EstimateRemoveRenderableSize;
using android_xr::schemas::EstimateRemoveTexturePipelineRendererSize;
using android_xr::schemas::EstimateRemoveTexturesSize;
using android_xr::schemas::EstimateTextureSize;
using android_xr::schemas::EstimateUnregisterNamedTextureSize;
using android_xr::schemas::EstimateUpdateMeshDataSize;
using android_xr::schemas::EstimateVertexBlockInfoSize;
using android_xr::schemas::EstimateVertexBlockUpdateSize;
using android_xr::schemas::EstimateVertexBufferInfoSize;
using android_xr::schemas::EstimateVertexBufferSize;
using android_xr::schemas::EstimateVertexBufferUpdateSize;
using android_xr::schemas::RegisterNamedTextures;
using android_xr::schemas::RemoveColliders;
using android_xr::schemas::RemoveRenderables;
using android_xr::schemas::RemoveTexturePipelineRenderers;
using android_xr::schemas::SetBuiltInMaterialParameters;
using android_xr::schemas::UpdateNodes;
using android_xr::schemas::UpdateRenderables;

template <typename T>
Offset<Command> CreateCommand(FlatBufferBuilder& fbb,
                              Offset<T> command_offset) {
  return CreateCommand(fbb, CommandTypesTraits<T>::enum_value,
                       command_offset.Union());
}

template <typename T>
void FinishCommand(split_engine::FlatbufferSizeCalculator& calc,
                   flatbuffers::Offset<T> command_offset) {
  auto root = android_xr::schemas::EstimateCommandSize(
      calc, android_xr::schemas::CommandTypesTraits<T>::enum_value,
      command_offset.Union());
  calc.Finish(root);
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::SetMaterialParameters>::Serialize(FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  std::vector<Offset<MaterialParameters>> params;
  params.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << "update material params for instance: " << entry.first;
    params.push_back(MaterialParameters::Pack(fbb, &entry.second));
  }
  return CreateCommand(
      fbb, CreateSetMaterialParameters(fbb, fbb.CreateVector(params)));
}

template <>
size_t Batch<CommandTypes::SetMaterialParameters>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& [material_instance_id, material_parameters] : data) {
    EstimateOffset(calc, &material_parameters);
  }
  Offset<SetMaterialParameters> root_offset = EstimateSetMaterialParametersSize(
      calc, calc.CreateOffsetVector<MaterialParameters>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::SetBuiltInMaterialParameters>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  std::vector<Offset<BuiltInMaterialInstanceParameters>> params;
  params.reserve(data.size());
  for (const auto& entry : data) {
    params.push_back(BuiltInMaterialInstanceParameters::Pack(fbb, &entry));
  }

  return CreateCommand(
      fbb, CreateSetBuiltInMaterialParameters(fbb, fbb.CreateVector(params)));
}

template <>
size_t Batch<CommandTypes::SetBuiltInMaterialParameters>::GetSerializedSize()
    const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& entry : data) {
    EstimateOffset(calc, &entry);
  }
  Offset<SetBuiltInMaterialParameters> root_offset =
      EstimateSetBuiltInMaterialParametersSize(
          calc, calc.CreateOffsetVector<BuiltInMaterialInstanceParameters>(
                    data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::DuplicateMaterialInstances>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  std::vector<Offset<DuplicateMaterialInstance>> duplicates;
  duplicates.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << "duplicate material instance: " << entry.original_id
               << " -> " << entry.duplicate_id;
    duplicates.push_back(DuplicateMaterialInstance::Pack(fbb, &entry));
  }
  return CreateCommand(
      fbb, CreateDuplicateMaterialInstances(fbb, fbb.CreateVector(duplicates)));
}

template <>
size_t Batch<CommandTypes::DuplicateMaterialInstances>::GetSerializedSize()
    const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& entry : data) {
    EstimateOffset(calc, &entry);
  }
  Offset<DuplicateMaterialInstances> root_offset =
      EstimateDuplicateMaterialInstancesSize(
          calc,
          calc.CreateOffsetVector<DuplicateMaterialInstance>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::AddOrUpdateColliders>::Serialize(FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  std::vector<Offset<AddOrUpdateCollider>> add_or_updates;
  add_or_updates.reserve(data.size());
  for (const auto& entry : data) {
    add_or_updates.push_back(AddOrUpdateCollider::Pack(fbb, &entry.second));
  }
  return CreateCommand(
      fbb, CreateAddOrUpdateColliders(fbb, fbb.CreateVector(add_or_updates)));
}

template <>
size_t Batch<CommandTypes::AddOrUpdateColliders>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& [entity_id, collider] : data) {
    EstimateOffset(calc, &collider);
  }
  Offset<AddOrUpdateColliders> root_offset = EstimateAddOrUpdateCollidersSize(
      calc, calc.CreateOffsetVector<AddOrUpdateCollider>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>> Batch<CommandTypes::RemoveColliders>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  std::vector<Offset<RemoveCollider>> removals;
  removals.reserve(data.size());
  for (const auto& entry : data) {
    removals.push_back(CreateRemoveCollider(fbb, entry.first, entry.second));
  }
  return CreateCommand(fbb,
                       CreateRemoveColliders(fbb, fbb.CreateVector(removals)));
}

template <>
size_t Batch<CommandTypes::RemoveColliders>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& [entity_id, collider_type] : data) {
    EstimateRemoveColliderSize(calc, entity_id, collider_type);
  }
  Offset<RemoveColliders> root_offset = EstimateRemoveCollidersSize(
      calc, calc.CreateOffsetVector<RemoveCollider>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::RemoveMorphTargetBuffers>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  return CreateCommand(fbb,
                       CreateRemoveMorphTargetBuffers(
                           fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
size_t Batch<CommandTypes::RemoveMorphTargetBuffers>::GetSerializedSize()
    const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  Offset<RemoveMorphTargetBuffers> root_offset =
      EstimateRemoveMorphTargetBuffersSize(
          calc, calc.CreateVector<uint64_t>(data.size(), sizeof(uint64_t),
                                            alignof(uint64_t)));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>> Batch<CommandTypes::RemoveMeshData>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.index_buffer_ids.empty() && data.vertex_buffer_ids.empty())
    return {};
  return CreateCommand(fbb, RemoveMeshData::Pack(fbb, &data));
}

template <>
size_t Batch<CommandTypes::RemoveMeshData>::GetSerializedSize() const {
  if (data.index_buffer_ids.empty() && data.vertex_buffer_ids.empty()) return 0;
  FlatbufferSizeCalculator calc;
  Offset<RemoveMeshData> root_offset = EstimateOffset(calc, &data);
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>> Batch<CommandTypes::UpdateMeshData>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.vertex_buffers.empty() && data.index_buffers.empty()) return {};

  std::vector<Offset<VertexBufferUpdate>> vertex_buffer_updates;
  vertex_buffer_updates.reserve(data.vertex_buffers.size());
  for (const auto& vertex_buffer : data.vertex_buffers) {
    std::vector<Offset<VertexBlockUpdate>> block_updates;
    block_updates.reserve(vertex_buffer.block_updates.size());
    for (const auto& block : vertex_buffer.block_updates) {
      block_updates.push_back(CreateVertexBlockUpdate(
          fbb, block.block_index, block.offset,
          fbb.CreateVector(block.buffer->data(), block.buffer->size())));
    }
    vertex_buffer_updates.push_back(CreateVertexBufferUpdate(
        fbb, vertex_buffer.id, fbb.CreateVector(block_updates)));
  }

  std::vector<Offset<IndexBufferUpdate>> index_buffer_updates;
  index_buffer_updates.reserve(data.index_buffers.size());
  for (const auto& index_buffer : data.index_buffers) {
    index_buffer_updates.push_back(
        CreateIndexBufferUpdate(fbb, index_buffer.id, index_buffer.offset,
                                fbb.CreateVector(index_buffer.buffer->data(),
                                                 index_buffer.buffer->size())));
  }

  return CreateCommand(
      fbb, CreateUpdateMeshData(fbb, fbb.CreateVector(vertex_buffer_updates),
                                fbb.CreateVector(index_buffer_updates)));
}

template <>
size_t Batch<CommandTypes::UpdateMeshData>::GetSerializedSize() const {
  if (data.vertex_buffers.empty() && data.index_buffers.empty()) return 0;
  FlatbufferSizeCalculator calc;

  for (const auto& vb : data.vertex_buffers) {
    for (const auto& block : vb.block_updates) {
      EstimateVertexBlockUpdateSize(
          calc, block.block_index, block.offset,
          calc.CreateVector<uint8_t>(block.buffer->size(), sizeof(uint8_t),
                                     alignof(uint8_t)));
    }
    EstimateVertexBufferUpdateSize(
        calc, vb.id,
        calc.CreateOffsetVector<VertexBlockUpdate>(vb.block_updates.size()));
  }

  for (const auto& ib : data.index_buffers) {
    EstimateIndexBufferUpdateSize(
        calc, ib.id, ib.offset,
        calc.CreateVector<uint8_t>(ib.buffer->size(), sizeof(uint8_t),
                                   alignof(uint8_t)));
  }

  Offset<UpdateMeshData> root_offset = EstimateUpdateMeshDataSize(
      calc,
      calc.CreateOffsetVector<VertexBufferUpdate>(data.vertex_buffers.size()),
      calc.CreateOffsetVector<IndexBufferUpdate>(data.index_buffers.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::RemoveImageBasedLightingAssets>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "remove IBLs: count: " << data.size();

  return CreateCommand(
      fbb, CreateRemoveImageBasedLightingAssets(fbb, fbb.CreateVector(data)));
}

template <>
size_t Batch<CommandTypes::RemoveImageBasedLightingAssets>::GetSerializedSize()
    const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  Offset<RemoveImageBasedLightingAssets> root_offset =
      EstimateRemoveImageBasedLightingAssetsSize(
          calc, calc.CreateVector<uint64_t>(data.size(), sizeof(uint64_t),
                                            alignof(uint64_t)));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::SetPreferredEnvironmentIblAsset>::Serialize(
    FlatBufferBuilder& fbb) {
  IMP_LOG(imp::INFO) << kTag
             << "set preferred IBL: " << data.image_based_lighting_asset_id;

  return CreateCommand(fbb, SetPreferredEnvironmentIblAsset::Pack(fbb, &data));
}

template <>
size_t Batch<CommandTypes::SetPreferredEnvironmentIblAsset>::GetSerializedSize()
    const {
  FlatbufferSizeCalculator calc;
  auto root_offset = EstimateOffset(calc, &data);
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>> Batch<CommandTypes::AddMaterials>::Serialize(
    FlatBufferBuilder& fbb) {
  IMP_LOG(imp::INFO) << kTag << "materials: " << data.size() << " new materials";
  std::vector<Offset<Material>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    offsets.push_back(Material::Pack(fbb, &entry));
  }
  return CreateCommand(fbb, CreateAddMaterials(fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::AddMaterials>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& entry : data) {
    EstimateOffset(calc, &entry);
  }
  Offset<AddMaterials> root_offset = EstimateAddMaterialsSize(
      calc, calc.CreateOffsetVector<Material>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>> Batch<CommandTypes::RemoveMaterials>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "destroy materials: ";
  for (auto id : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }
  return CreateCommand(
      fbb,
      CreateRemoveMaterials(fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
size_t Batch<CommandTypes::RemoveMaterials>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  Offset<RemoveMaterials> root_offset = EstimateRemoveMaterialsSize(
      calc, calc.CreateVector<uint64_t>(data.size(), sizeof(uint64_t),
                                        alignof(uint64_t)));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::AddMaterialInstances>::Serialize(FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "material instances: ";
  std::vector<Offset<MaterialInstance>> instances;
  instances.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.first << " -> " << entry.second;
    instances.push_back(CreateMaterialInstance(fbb, entry.first, entry.second));
  };
  return CreateCommand(
      fbb, CreateAddMaterialInstances(fbb, fbb.CreateVector(instances)));
}

template <>
size_t Batch<CommandTypes::AddMaterialInstances>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& [instance_id, material_id] : data) {
    EstimateMaterialInstanceSize(calc, instance_id, material_id);
  }
  Offset<AddMaterialInstances> root_offset = EstimateAddMaterialInstancesSize(
      calc, calc.CreateOffsetVector<MaterialInstance>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::RemoveMaterialInstances>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "destroy material instances: ";
  for (auto id : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }
  return CreateCommand(fbb,
                       CreateRemoveMaterialInstances(
                           fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
size_t Batch<CommandTypes::RemoveMaterialInstances>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  Offset<RemoveMaterialInstances> root_offset =
      EstimateRemoveMaterialInstancesSize(
          calc, calc.CreateVector<uint64_t>(data.size(), sizeof(uint64_t),
                                            alignof(uint64_t)));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>> Batch<CommandTypes::RemoveTextures>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "destroy textures: ";
  for (const auto id : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }

  return CreateCommand(
      fbb,
      CreateRemoveTextures(fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
size_t Batch<CommandTypes::RemoveTextures>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  Offset<RemoveTextures> root_offset = EstimateRemoveTexturesSize(
      calc, calc.CreateVector<uint64_t>(data.size(), sizeof(uint64_t),
                                        alignof(uint64_t)));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>> Batch<CommandTypes::AddNodes>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "adding nodes:";
  std::vector<Offset<AddNode>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry;
    offsets.push_back(CreateAddNode(fbb, entry));
  }
  return CreateCommand(fbb, CreateAddNodes(fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::AddNodes>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& entry : data) {
    EstimateAddNodeSize(calc, entry);
  }
  Offset<AddNodes> root_offset =
      EstimateAddNodesSize(calc, calc.CreateOffsetVector<AddNode>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>> Batch<CommandTypes::RemoveNodes>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "removing nodes:";
  std::vector<Offset<RemoveNode>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry;
    offsets.push_back(CreateRemoveNode(fbb, entry));
  }
  return CreateCommand(fbb, CreateRemoveNodes(fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::RemoveNodes>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& entry : data) {
    EstimateRemoveNodeSize(calc, entry);
  }
  Offset<RemoveNodes> root_offset = EstimateRemoveNodesSize(
      calc, calc.CreateOffsetVector<RemoveNode>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::AssignUserIdToNodes>::Serialize(FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "assigning user ids:";
  std::vector<Offset<AssignUserIdToNode>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.first << " -> " << entry.second;
    offsets.push_back(CreateAssignUserIdToNode(fbb, entry.first, entry.second));
  }
  return CreateCommand(
      fbb, CreateAssignUserIdToNodes(fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::AssignUserIdToNodes>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& [entity_id, user_id] : data) {
    EstimateAssignUserIdToNodeSize(calc, entity_id, user_id);
  }
  Offset<AssignUserIdToNodes> root_offset = EstimateAssignUserIdToNodesSize(
      calc, calc.CreateOffsetVector<AssignUserIdToNode>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>> Batch<CommandTypes::UpdateNodes>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "updating nodes:";
  std::vector<Offset<UpdateNode>> node_updates;
  node_updates.reserve(data.size());
  for (const auto& [node_id, update_node] : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << node_id;
    node_updates.push_back(UpdateNode::Pack(fbb, &update_node));
  }
  return CreateCommand(fbb,
                       CreateUpdateNodes(fbb, fbb.CreateVector(node_updates)));
}

template <>
size_t Batch<CommandTypes::UpdateNodes>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& [node_id, update_node] : data) {
    EstimateOffset(calc, &update_node);
  }
  Offset<UpdateNodes> root_offset = EstimateUpdateNodesSize(
      calc, calc.CreateOffsetVector<UpdateNode>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>> Batch<CommandTypes::AddRenderables>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  std::vector<Offset<AddRenderable>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << "add renderable " << entry.entity;
    offsets.push_back(AddRenderable::Pack(fbb, &entry));
  }
  return CreateCommand(fbb,
                       CreateAddRenderables(fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::AddRenderables>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& entry : data) {
    EstimateOffset(calc, &entry);
  }
  Offset<AddRenderables> root_offset = EstimateAddRenderablesSize(
      calc, calc.CreateOffsetVector<AddRenderable>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::RemoveRenderables>::Serialize(FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  std::vector<Offset<RemoveRenderable>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << "remove renderable: " << entry;
    offsets.push_back(CreateRemoveRenderable(fbb, entry));
  }
  return CreateCommand(fbb,
                       CreateRemoveRenderables(fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::RemoveRenderables>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& entry : data) {
    EstimateRemoveRenderableSize(calc, entry);
  }
  Offset<RemoveRenderables> root_offset = EstimateRemoveRenderablesSize(
      calc, calc.CreateOffsetVector<RemoveRenderable>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::UpdateRenderables>::Serialize(FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  std::vector<Offset<UpdateRenderable>> offsets;
  offsets.reserve(data.size());
  for (const auto& [entity, update_renderable] : data) {
    IMP_LOG(imp::INFO) << kTag << "update renderable: " << entity;
    offsets.push_back(UpdateRenderable::Pack(fbb, &update_renderable));
  }
  return CreateCommand(fbb,
                       CreateUpdateRenderables(fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::UpdateRenderables>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& [entity, update_renderable] : data) {
    EstimateOffset(calc, &update_renderable);
  }
  Offset<UpdateRenderables> root_offset = EstimateUpdateRenderablesSize(
      calc, calc.CreateOffsetVector<UpdateRenderable>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::AddTexturePipelineRenderers>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "add or update texture pipeline renderer: count: "
             << data.size();
  std::vector<Offset<AddTexturePipelineRenderer>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << "add texture pipeline renderer: " << entry.first;
    offsets.push_back(AddTexturePipelineRenderer::Pack(fbb, &entry.second));
  };
  return CreateCommand(
      fbb, CreateAddTexturePipelineRenderers(fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::AddTexturePipelineRenderers>::GetSerializedSize()
    const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& [entity, add_renderer] : data) {
    EstimateOffset(calc, &add_renderer);
  }
  Offset<AddTexturePipelineRenderers> root_offset =
      EstimateAddTexturePipelineRenderersSize(
          calc,
          calc.CreateOffsetVector<AddTexturePipelineRenderer>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::RemoveTexturePipelineRenderers>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag
             << "remove texture pipeline renderer: count: " << data.size();
  std::vector<Offset<RemoveTexturePipelineRenderer>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << "remove texture pipeline renderer: " << entry;
    offsets.push_back(CreateRemoveTexturePipelineRenderer(fbb, entry));
  }
  return CreateCommand(fbb, CreateRemoveTexturePipelineRenderers(
                                fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::RemoveTexturePipelineRenderers>::GetSerializedSize()
    const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& entry : data) {
    EstimateRemoveTexturePipelineRendererSize(calc, entry);
  }
  Offset<RemoveTexturePipelineRenderers> root_offset =
      EstimateRemoveTexturePipelineRenderersSize(
          calc,
          calc.CreateOffsetVector<RemoveTexturePipelineRenderer>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::RegisterNamedTextures>::Serialize(FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "register named textures: count: " << data.size();
  std::vector<Offset<RegisterNamedTexture>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << "register named texture: " << entry.first << " -> "
               << entry.second.name;
    offsets.push_back(RegisterNamedTexture::Pack(fbb, &entry.second));
  }
  return CreateCommand(
      fbb, CreateRegisterNamedTextures(fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::RegisterNamedTextures>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& [entity, register_texture] : data) {
    EstimateOffset(calc, &register_texture);
  }
  Offset<RegisterNamedTextures> root_offset = EstimateRegisterNamedTexturesSize(
      calc, calc.CreateOffsetVector<RegisterNamedTexture>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::UnregisterNamedTextures>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "register named textures: count: " << data.size();
  std::vector<Offset<UnregisterNamedTexture>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << "unregister named texture: " << entry;
    offsets.push_back(CreateUnregisterNamedTexture(fbb, entry));
  }
  return CreateCommand(
      fbb, CreateUnregisterNamedTextures(fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::UnregisterNamedTextures>::GetSerializedSize() const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& entry : data) {
    EstimateUnregisterNamedTextureSize(calc, entry);
  }
  Offset<UnregisterNamedTextures> root_offset =
      EstimateUnregisterNamedTexturesSize(
          calc, calc.CreateOffsetVector<UnregisterNamedTexture>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

template <>
std::optional<Offset<Command>>
Batch<CommandTypes::UpdateTexturePipelineRenderers>::Serialize(
    FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag
             << "update texture pipeline renderer: count: " << data.size();
  std::vector<Offset<UpdateTexturePipelineRenderer>> offsets;
  offsets.reserve(data.size());
  for (const auto& entry : data) {
    IMP_LOG(imp::INFO) << kTag << "update texture pipeline renderer: " << entry.first;
    offsets.push_back(UpdateTexturePipelineRenderer::Pack(fbb, &entry.second));
  }
  return CreateCommand(fbb, CreateUpdateTexturePipelineRenderers(
                                fbb, fbb.CreateVector(offsets)));
}

template <>
size_t Batch<CommandTypes::UpdateTexturePipelineRenderers>::GetSerializedSize()
    const {
  if (data.empty()) return 0;
  FlatbufferSizeCalculator calc;
  for (const auto& [entity, update_renderer] : data) {
    EstimateOffset(calc, &update_renderer);
  }
  Offset<UpdateTexturePipelineRenderers> root_offset =
      EstimateUpdateTexturePipelineRenderersSize(
          calc,
          calc.CreateOffsetVector<UpdateTexturePipelineRenderer>(data.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

static constexpr size_t kReferenceSize = 4;

size_t GetSerializedSize(const AddMeshDataInfo& info) {
  if (info.vertex_buffers.empty() && info.index_buffers.empty()) return 0;
  FlatbufferSizeCalculator calc;

  for (const auto& vb : info.vertex_buffers) {
    for (const auto& block : vb.buffer.blocks) {
      EstimateVertexBlockInfoSize(
          calc,
          calc.CreateVector<android_xr::schemas::VertexAttributeInfo>(
              block.attributes.size(),
              sizeof(android_xr::schemas::VertexAttributeInfo),
              alignof(android_xr::schemas::VertexAttributeInfo)),
          calc.CreateVector<uint8_t>(block.buffer->size(), sizeof(uint8_t),
                                     alignof(uint8_t)),
          block.stride);
    }
    EstimateVertexBufferInfoSize(
        calc,
        calc.CreateOffsetVector<android_xr::schemas::VertexBlockInfo>(
            vb.buffer.blocks.size()),
        vb.buffer.vertex_count, vb.buffer.advanced_skinning);
    EstimateVertexBufferSize(calc, vb.id, kReferenceSize,
                             vb.vertex_access_flags);
  }

  for (const auto& ib : info.index_buffers) {
    EstimateIndexBufferInfoSize(
        calc, ib.buffer.type,
        calc.CreateVector<uint8_t>(ib.buffer.buffer->size(), sizeof(uint8_t),
                                   alignof(uint8_t)));
    EstimateIndexBufferSize(calc, ib.id, kReferenceSize, ib.store_index_data);
  }

  auto root_offset = EstimateAddMeshDataSize(
      calc,
      calc.CreateOffsetVector<android_xr::schemas::VertexBuffer>(
          info.vertex_buffers.size()),
      calc.CreateOffsetVector<android_xr::schemas::IndexBuffer>(
          info.index_buffers.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

flatbuffers::Offset<android_xr::schemas::AddMeshData> Pack(
    flatbuffers::FlatBufferBuilder& fbb, const AddMeshDataInfo& info) {
  std::vector<flatbuffers::Offset<android_xr::schemas::VertexBuffer>>
      vb_offsets;
  vb_offsets.reserve(info.vertex_buffers.size());
  for (const auto& vb : info.vertex_buffers) {
    std::vector<flatbuffers::Offset<android_xr::schemas::VertexBlockInfo>>
        block_offsets;
    block_offsets.reserve(vb.buffer.blocks.size());
    for (const auto& block : vb.buffer.blocks) {
      block_offsets.push_back(CreateVertexBlockInfo(
          fbb, fbb.CreateVectorOfStructs(block.attributes),
          fbb.CreateVector(block.buffer->data(), block.buffer->size()),
          block.stride));
    }
    auto vb_info_offset = CreateVertexBufferInfo(
        fbb, fbb.CreateVector(block_offsets), vb.buffer.vertex_count,
        vb.buffer.advanced_skinning);
    vb_offsets.push_back(
        CreateVertexBuffer(fbb, vb.id, vb_info_offset, vb.vertex_access_flags));
  }

  std::vector<flatbuffers::Offset<android_xr::schemas::IndexBuffer>> ib_offsets;
  ib_offsets.reserve(info.index_buffers.size());
  for (const auto& ib : info.index_buffers) {
    auto ib_info_offset = CreateIndexBufferInfo(
        fbb, ib.buffer.type,
        fbb.CreateVector(ib.buffer.buffer->data(), ib.buffer.buffer->size()));
    ib_offsets.push_back(
        CreateIndexBuffer(fbb, ib.id, ib_info_offset, ib.store_index_data));
  }

  return CreateAddMeshData(fbb, fbb.CreateVector(vb_offsets),
                           fbb.CreateVector(ib_offsets));
}

size_t GetSerializedSize(const AddMorphTargetBuffersInfo& info) {
  if (info.morph_target_buffers.empty()) return 0;
  FlatbufferSizeCalculator calc;

  for (const auto& buf : info.morph_target_buffers) {
    for (const auto& target : buf.buffer.targets) {
      calc.TrackMinAlign(4);  // MorphTargetAttributeInfo vector
      calc.Pad(4);
      calc.CreateVector<uint8_t>(target.positions->size(), sizeof(uint8_t),
                                 alignof(uint8_t));
      calc.CreateVector<uint8_t>(target.tangents->size(), sizeof(uint8_t),
                                 alignof(uint8_t));
    }
    EstimateMorphTargetBufferInfoSize(
        calc,
        calc.CreateOffsetVector<android_xr::schemas::MorphTargetAttributeInfo>(
            buf.buffer.targets.size()),
        buf.buffer.vertex_count, buf.buffer.enable_custom_morphing);
    EstimateMorphTargetBufferSize(calc, buf.id, kReferenceSize);
  }

  auto root_offset = EstimateAddMorphTargetBuffersSize(
      calc, calc.CreateOffsetVector<android_xr::schemas::MorphTargetBuffer>(
                info.morph_target_buffers.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

flatbuffers::Offset<android_xr::schemas::AddMorphTargetBuffers> Pack(
    flatbuffers::FlatBufferBuilder& fbb,
    const AddMorphTargetBuffersInfo& info) {
  std::vector<flatbuffers::Offset<android_xr::schemas::MorphTargetBuffer>>
      buf_offsets;
  buf_offsets.reserve(info.morph_target_buffers.size());
  for (const auto& buf : info.morph_target_buffers) {
    std::vector<
        flatbuffers::Offset<android_xr::schemas::MorphTargetAttributeInfo>>
        target_offsets;
    target_offsets.reserve(buf.buffer.targets.size());
    for (const auto& target : buf.buffer.targets) {
      target_offsets.push_back(CreateMorphTargetAttributeInfo(
          fbb,
          fbb.CreateVector(target.positions->data(), target.positions->size()),
          fbb.CreateVector(target.tangents->data(), target.tangents->size())));
    }
    auto info_offset = CreateMorphTargetBufferInfo(
        fbb, fbb.CreateVector(target_offsets), buf.buffer.vertex_count,
        buf.buffer.enable_custom_morphing);
    buf_offsets.push_back(CreateMorphTargetBuffer(fbb, buf.id, info_offset));
  }
  return CreateAddMorphTargetBuffers(fbb, fbb.CreateVector(buf_offsets));
}

size_t GetSerializedSize(const AddTexturesInfo& info, int32_t api_level) {
  if (info.textures.empty()) return 0;
  FlatbufferSizeCalculator calc;

  for (const auto& tex : info.textures) {
    for (const auto& pb : tex.pixel_buffers) {
      calc.TrackMinAlign(4);  // PixelBuffer vector
      calc.Pad(4);
      calc.CreateVector<uint8_t>(pb.buffer->size(), sizeof(uint8_t),
                                 alignof(uint8_t));
    }
    for (const auto& p : tex.image_params) {
      android_xr::schemas::EstimateImageParamsSize(calc, p.level, p.format,
                                                   p.type, p.alignment, p.left,
                                                   p.top, p.stride);
    }

    flatbuffers::Offset<flatbuffers::String> name_offset = 0;
    if (api_level >= 4) {
      name_offset = calc.CreateString(tex.name);
    }

    EstimateTextureSize(
        calc, tex.id, tex.width, tex.height, tex.format, tex.levels,
        tex.sampler, tex.mips,
        calc.CreateOffsetVector<android_xr::schemas::ImageParams>(
            tex.image_params.size()),
        calc.CreateOffsetVector<android_xr::schemas::PixelBuffer>(
            tex.pixel_buffers.size()),
        name_offset);
  }

  auto root_offset = EstimateAddTexturesSize(
      calc, calc.CreateOffsetVector<android_xr::schemas::Texture>(
                info.textures.size()));
  FinishCommand(calc, root_offset);
  return calc.ComputeSize();
}

flatbuffers::Offset<android_xr::schemas::AddTextures> Pack(
    flatbuffers::FlatBufferBuilder& fbb, const AddTexturesInfo& info,
    int32_t api_level) {
  std::vector<flatbuffers::Offset<android_xr::schemas::Texture>> tex_offsets;
  tex_offsets.reserve(info.textures.size());
  for (const auto& tex : info.textures) {
    std::vector<flatbuffers::Offset<android_xr::schemas::PixelBuffer>>
        pb_offsets;
    pb_offsets.reserve(tex.pixel_buffers.size());
    for (const auto& pb : tex.pixel_buffers) {
      pb_offsets.push_back(CreatePixelBuffer(
          fbb, fbb.CreateVector(pb.buffer->data(), pb.buffer->size())));
    }
    std::vector<flatbuffers::Offset<android_xr::schemas::ImageParams>>
        param_offsets;
    param_offsets.reserve(tex.image_params.size());
    for (const auto& p : tex.image_params) {
      param_offsets.push_back(android_xr::schemas::CreateImageParams(
          fbb, p.level, p.format, p.type, p.alignment, p.left, p.top,
          p.stride));
    }

    flatbuffers::Offset<flatbuffers::String> name_offset = 0;
    if (api_level >= 4) {
      name_offset = fbb.CreateString(tex.name);
    }

    tex_offsets.push_back(CreateTexture(
        fbb, tex.id, tex.width, tex.height, tex.format, tex.levels, tex.sampler,
        tex.mips, fbb.CreateVector(param_offsets), fbb.CreateVector(pb_offsets),
        name_offset));
  }
  return CreateAddTextures(fbb, fbb.CreateVector(tex_offsets));
}

}  // namespace split_engine
