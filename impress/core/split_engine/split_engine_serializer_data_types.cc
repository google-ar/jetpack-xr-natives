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

#include "core/split_engine/split_engine_serializer_data_types.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"
#include "core/geometry/shapes/box.h"
#include "core/geometry/shapes/capsule.h"
#include "core/geometry/shapes/sphere.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/texture_pipeline_schema_conversion.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"

namespace imp::split_engine {

namespace {

static constexpr absl::string_view kTag = "[SplitEngineSerializer]: ";
static constexpr absl::string_view kIndent = "  ";

}  // namespace

using android_xr::schemas::ColliderData;
using android_xr::schemas::CommandTypes;

template <typename T>
flatbuffers::Offset<android_xr::schemas::Command> CreateCommand(
    flatbuffers::FlatBufferBuilder& fbb,
    flatbuffers::Offset<T> command_offset) {
  return android_xr::schemas::CreateCommand(
      fbb, android_xr::schemas::CommandTypesTraits<T>::enum_value,
      command_offset.Union());
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::SetMaterialParameters>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  VectorOffset<android_xr::schemas::MaterialParameters> params(data.size());
  absl::c_transform(data, params.data(), [&fbb](const auto& entry) {
    IMP_LOG(imp::INFO) << kTag << "update material params for instance: " << entry.first;
    return android_xr::schemas::CreateMaterialParameters(
        fbb, entry.first,
        fbb.CreateVector(entry.second.params.data(),
                         entry.second.params.size()),
        fbb.CreateVector(entry.second.texture_params.data(),
                         entry.second.texture_params.size()));
  });
  return CreateCommand(fbb, android_xr::schemas::CreateSetMaterialParameters(
                                fbb, fbb.CreateVector(params)));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::SetBuiltInMaterialParameters>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  return CreateCommand(fbb,
                       android_xr::schemas::CreateSetBuiltInMaterialParameters(
                           fbb, fbb.CreateVector(data)));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::DuplicateMaterialInstances>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  std::vector<
      flatbuffers::Offset<android_xr::schemas::DuplicateMaterialInstance>>
      duplicates(data.size());
  absl::c_transform(data, duplicates.data(), [&fbb](const auto& entry) {
    IMP_LOG(imp::INFO) << kTag << "duplicate material instance: " << entry.instance_id
               << " -> " << entry.copy_id;
    return android_xr::schemas::CreateDuplicateMaterialInstance(
        fbb, entry.copy_id, entry.instance_id);
  });
  return CreateCommand(fbb,
                       android_xr::schemas::CreateDuplicateMaterialInstances(
                           fbb, fbb.CreateVector(duplicates)));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::AddOrUpdateColliders>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  VectorOffset<android_xr::schemas::AddOrUpdateCollider> add_or_updates(
      data.size());
  absl::c_transform(data, add_or_updates.data(), [&fbb](const auto& entry) {
    const AddOrUpdateColliderInfo& collider_update = entry.second;

    struct Visitor {
      flatbuffers::FlatBufferBuilder& fbb;
      const uint32_t entity_id;
      const android_xr::schemas::Bool* enabled;
      flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider> operator()(
          const Box& value) {
        const android_xr::schemas::Float3 center(value.center.x, value.center.y,
                                                 value.center.z);
        const android_xr::schemas::Float3 half_extent(
            value.halfExtent.x, value.halfExtent.y, value.halfExtent.z);
        return android_xr::schemas::CreateAddOrUpdateCollider(
            fbb, entity_id, ColliderData::BoxCollider,
            android_xr::schemas::CreateBoxCollider(fbb, &center, &half_extent)
                .Union(),
            enabled);
      }
      flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider> operator()(
          const MeshCollider& value) {
        return android_xr::schemas::CreateAddOrUpdateCollider(
            fbb, entity_id, ColliderData::MeshCollider,
            android_xr::schemas::CreateMeshCollider(fbb).Union(), enabled);
      }
      flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider> operator()(
          const Sphere& value) {
        const android_xr::schemas::Float3 center(value.center.x, value.center.y,
                                                 value.center.z);
        const android_xr::schemas::Float radius(value.radius);
        return android_xr::schemas::CreateAddOrUpdateCollider(
            fbb, entity_id, ColliderData::SphereCollider,
            android_xr::schemas::CreateSphereCollider(fbb, &center, &radius)
                .Union(),
            enabled);
      }
      flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider> operator()(
          const Capsule& value) {
        const android_xr::schemas::Float3 center(value.center.x, value.center.y,
                                                 value.center.z);
        const android_xr::schemas::Float height(value.height);
        const android_xr::schemas::Float radius(value.radius);
        return android_xr::schemas::CreateAddOrUpdateCollider(
            fbb, entity_id, ColliderData::CapsuleCollider,
            android_xr::schemas::CreateCapsuleCollider(fbb, &center, &height,
                                                       &radius)
                .Union(),
            enabled);
      }
    };
    return std::visit(Visitor{fbb, entry.first.getId(),
                              collider_update.enabled.has_value()
                                  ? &collider_update.enabled.value()
                                  : nullptr},
                      collider_update.collider);
  });
  return CreateCommand(fbb, android_xr::schemas::CreateAddOrUpdateColliders(
                                fbb, fbb.CreateVector(add_or_updates)));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RemoveColliders>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  VectorOffset<android_xr::schemas::RemoveCollider> removals(data.size());
  absl::c_transform(data, removals.data(), [&fbb](const auto& entry) {
    return android_xr::schemas::CreateRemoveCollider(fbb, entry.first.getId(),
                                                     entry.second);
  });

  return CreateCommand(fbb, android_xr::schemas::CreateRemoveColliders(
                                fbb, fbb.CreateVector(removals)));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RemoveMorphTargetBuffers>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  return CreateCommand(fbb,
                       android_xr::schemas::CreateRemoveMorphTargetBuffers(
                           fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RemoveMeshData>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.vertex_buffers.empty() && data.index_buffers.empty()) return {};

  return CreateCommand(fbb, android_xr::schemas::CreateRemoveMeshData(
                                fbb,
                                fbb.CreateVector(data.vertex_buffers.data(),
                                                 data.vertex_buffers.size()),
                                fbb.CreateVector(data.index_buffers.data(),
                                                 data.index_buffers.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RemoveImageBasedLightingAssets>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "remove IBLs: count: " << data.size();

  return CreateCommand(
      fbb, android_xr::schemas::CreateRemoveImageBasedLightingAssets(
               fbb, fbb.CreateVector(data)));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::SetPreferredEnvironmentIblAsset>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (!data.has_value()) return {};

  IMP_LOG(imp::INFO) << kTag
             << "set preferred IBL: " << data->image_based_lighting_asset_id;
  const android_xr::schemas::Float3 tint = Pack(data->tint);
  return CreateCommand(
      fbb,
      android_xr::schemas::CreateSetPreferredEnvironmentIblAsset(
          fbb, data->image_based_lighting_asset_id, data->intensity, &tint));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::AddMaterials>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  IMP_LOG(imp::INFO) << kTag << "materials: " << data.size() << " new materials";
  return CreateCommand(fbb, android_xr::schemas::CreateAddMaterials(
                                fbb, fbb.CreateVector(data)));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RemoveMaterials>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "destroy materials: ";
  for (auto id : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }
  return CreateCommand(fbb,
                       android_xr::schemas::CreateRemoveMaterials(
                           fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::AddMaterialInstances>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "material instances: ";
  VectorOffset<android_xr::schemas::MaterialInstance> instances(data.size());
  absl::c_transform(data, instances.data(), [&fbb](const auto& entry) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.first << " -> " << entry.second;
    return android_xr::schemas::CreateMaterialInstance(fbb, entry.first,
                                                       entry.second);
  });
  return CreateCommand(
      fbb, android_xr::schemas::CreateAddMaterialInstances(
               fbb, fbb.CreateVector(instances.data(), instances.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RemoveMaterialInstances>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "destroy material instances: ";
  for (auto id : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }
  return CreateCommand(fbb,
                       android_xr::schemas::CreateRemoveMaterialInstances(
                           fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RemoveTextures>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "destroy textures: ";
  for (const auto id : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }

  return CreateCommand(fbb,
                       android_xr::schemas::CreateRemoveTextures(
                           fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::AddNodes>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "adding nodes:";
  VectorOffset<android_xr::schemas::AddNode> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const utils::Entity& entry) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.getId();
    return android_xr::schemas::CreateAddNode(fbb, entry.getId());
  });
  return CreateCommand(
      fbb, android_xr::schemas::CreateAddNodes(
               fbb, fbb.CreateVector(offset.data(), offset.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RemoveNodes>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "removing nodes:";
  VectorOffset<android_xr::schemas::RemoveNode> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const utils::Entity& entry) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.getId();
    return android_xr::schemas::CreateRemoveNode(fbb, entry.getId());
  });
  return CreateCommand(
      fbb, android_xr::schemas::CreateRemoveNodes(
               fbb, fbb.CreateVector(offset.data(), offset.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::AssignUserIdToNodes>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "assigning user ids:";
  VectorOffset<android_xr::schemas::AssignUserIdToNode> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const auto& entry) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.first.getId() << " -> "
               << entry.second;
    return android_xr::schemas::CreateAssignUserIdToNode(
        fbb, entry.first.getId(), entry.second);
  });
  return CreateCommand(
      fbb, android_xr::schemas::CreateAssignUserIdToNodes(
               fbb, fbb.CreateVector(offset.data(), offset.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::UpdateNodes>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "updating nodes:";
  VectorOffset<android_xr::schemas::UpdateNode> node_updates(data.size());
  absl::c_transform(data, node_updates.data(), [&fbb](const auto& entry) {
    const UpdateNodeInfo& update = entry.second;
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.first.getId();
    // Process the name of the node.
    flatbuffers::Offset<flatbuffers::String> name;
    if (update.name.has_value()) {
      name = fbb.CreateString(*update.name);
    }

    flatbuffers::Offset<android_xr::schemas::Transform> transform;
    if (update.transform.has_value()) {
      struct Visitor {
        flatbuffers::FlatBufferBuilder& fbb;
        flatbuffers::Offset<android_xr::schemas::Transform> operator()(
            const mat4f& value) {
          return android_xr::schemas::CreateTransform(
              fbb, android_xr::schemas::TransformData::Mat4f,
              fbb.CreateStruct(flatbuffers::Pack(value)).Union());
        }
        flatbuffers::Offset<android_xr::schemas::Transform> operator()(
            const mat4& value) {
          return android_xr::schemas::CreateTransform(
              fbb, android_xr::schemas::TransformData::Mat4,
              fbb.CreateStruct(flatbuffers::Pack(value)).Union());
        }
      };
      transform = std::visit(Visitor{fbb}, *update.transform);
    }

    flatbuffers::Offset<android_xr::schemas::Parent> parent;
    if (update.parent.has_value()) {
      parent = android_xr::schemas::CreateParent(fbb, update.parent->getId());
    }

    flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
        groups;
    if (update.groups.has_value()) {
      groups = fbb.CreateVectorOfStrings(*update.groups);
    }

    NodeHandle node = NodeHandle(entry.first);

    IMP_LOG(imp::INFO) << kTag << kIndent << ToString(node);
    return android_xr::schemas::CreateUpdateNode(
        fbb, entry.first.getId(), name, PointerFromOptional(update.enabled),
        transform, parent, groups);
  });
  return CreateCommand(fbb, android_xr::schemas::CreateUpdateNodes(
                                fbb, fbb.CreateVector(node_updates.data(),
                                                      node_updates.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::AddRenderables>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  VectorOffset<android_xr::schemas::AddRenderable> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const auto& entry) {
    const AddRenderableInfo& add = entry.second;
    IMP_LOG(imp::INFO) << kTag << "add renderable " << entry.first.getId() << ":";

    flatbuffers::Offset<android_xr::schemas::MorphTargetData> morph_target_data;
    if (add.morph_target_data.has_value()) {
      VectorOffset<android_xr::schemas::MorphTargetInfo> morph_target_info(
          add.morph_target_data->morph_target_info.size());
      absl::c_transform(add.morph_target_data->morph_target_info,
                        morph_target_info.data(),
                        [&fbb](const MorphTargetInfo& morph_target) {
                          return android_xr::schemas::CreateMorphTargetInfo(
                              fbb, morph_target.morph_target_buffer_offset,
                              morph_target.morph_target_buffer_count);
                        });
      IMP_LOG(imp::INFO) << kTag << kIndent << "morph target size: "
                 << add.morph_target_data->morph_target_info.size();
      morph_target_data = android_xr::schemas::CreateMorphTargetData(
          fbb,
          PointerFromOptional(add.morph_target_data->morph_target_buffer_id),
          fbb.CreateVector(morph_target_info));
    }
    flatbuffers::Offset<android_xr::schemas::RenderableFlags> renderable_flags;
    if (add.renderable_flags.has_value()) {
      renderable_flags = android_xr::schemas::CreateRenderableFlags(
          fbb, PointerFromOptional(add.renderable_flags->culling_enabled));
      IMP_LOG(imp::INFO) << kTag << kIndent << "culling enabled: "
                 << add.renderable_flags->culling_enabled->value();
    }

    if (add.skinning_bone_count.has_value()) {
      IMP_LOG(imp::INFO) << kTag << kIndent
                 << "skinning bone count: " << add.skinning_bone_count->value();
    }

    return android_xr::schemas::CreateAddRenderable(
        fbb, entry.first.getId(), add.primitive_count,
        PointerFromOptional(add.skinning_bone_count), morph_target_data,
        renderable_flags);
  });
  return CreateCommand(
      fbb, android_xr::schemas::CreateAddRenderables(
               fbb, fbb.CreateVector(offset.data(), offset.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RemoveRenderables>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  VectorOffset<android_xr::schemas::RemoveRenderable> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const utils::Entity& entry) {
    IMP_LOG(imp::INFO) << kTag << "remove renderable: " << entry.getId();
    return android_xr::schemas::CreateRemoveRenderable(fbb, entry.getId());
  });
  return CreateCommand(
      fbb, android_xr::schemas::CreateRemoveRenderables(
               fbb, fbb.CreateVector(offset.data(), offset.size())));
}

// Copied from loaded_model_builder.cc
flatbuffers::Offset<android_xr::schemas::BoundsInfo> CreateBoundsInfo(
    flatbuffers::FlatBufferBuilder& fbb, const Box& bounds) {
  const float3 center = bounds.center;
  const float3 half_extent = bounds.halfExtent;
  android_xr::schemas::Box box(Pack(center), Pack(half_extent));

  return android_xr::schemas::CreateBoundsInfo(fbb, &box);
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::UpdateRenderables>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  VectorOffset<android_xr::schemas::UpdateRenderable> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const auto& entry) {
    const UpdateRenderableInfo& update = entry.second;
    IMP_LOG(imp::INFO) << kTag << "update renderable: " << entry.first.getId() << ":";

    VectorOffset<android_xr::schemas::PrimitiveUpdate> primitives(
        update.primitives.size());
    absl::c_transform(
        update.primitives, primitives.data(), [&fbb](const auto& entry) {
          IMP_LOG(imp::INFO) << kTag << kIndent << "primitive: " << entry.first;
          const PrimitiveUpdateInfo& primitive = entry.second;

          flatbuffers::Offset<android_xr::schemas::GeometryUpdate> geometry;
          if (primitive.geometry.has_value()) {
            IMP_LOG(imp::INFO) << kTag << kIndent << kIndent
                       << "geometry: " << primitive.geometry->vertex_buffer_id
                       << " " << primitive.geometry->index_buffer_id;
            geometry = android_xr::schemas::CreateGeometryUpdate(
                fbb, primitive.geometry->vertex_buffer_id,
                primitive.geometry->index_buffer_id, primitive.geometry->offset,
                primitive.geometry->count, primitive.geometry->primitive_type);
          }

          if (primitive.material_instance_id) {
            IMP_LOG(imp::INFO) << kTag << kIndent << kIndent << "material: "
                       << primitive.material_instance_id->value();
          }

          if (primitive.blend_order) {
            IMP_LOG(imp::INFO) << kTag << kIndent << kIndent
                       << "blend_order: " << primitive.blend_order->value();
          }

          if (primitive.global_blend_order_enabled) {
            IMP_LOG(imp::INFO) << kTag << kIndent << kIndent
                       << "global_blend_order_enabled: "
                       << primitive.global_blend_order_enabled->value();
          }

          return android_xr::schemas::CreatePrimitiveUpdate(
              fbb, entry.first, geometry,
              PointerFromOptional(primitive.material_instance_id),
              PointerFromOptional(primitive.blend_order),
              PointerFromOptional(primitive.global_blend_order_enabled));
        });

    flatbuffers::Offset<android_xr::schemas::BoundsInfo> bounds_info;
    if (update.bounds) {
      IMP_LOG(imp::INFO) << kTag << kIndent << "bounds: " << update.bounds->center
                 << ", " << update.bounds->halfExtent;
      bounds_info = CreateBoundsInfo(fbb, *update.bounds);
    }

    flatbuffers::Offset<android_xr::schemas::LayerMask> layer_mask;
    if (update.layer_mask) {
      IMP_LOG(imp::INFO) << kTag << kIndent
                 << absl::StrFormat("layer_mask: {select: 0x%x, values: 0x%x}",
                                    update.layer_mask->select,
                                    update.layer_mask->values);
      layer_mask = android_xr::schemas::CreateLayerMask(
          fbb, update.layer_mask->select, update.layer_mask->values);
    }

    if (update.priority) {
      IMP_LOG(imp::INFO) << kTag << kIndent
                 << "priority: " << static_cast<int>(update.priority->value());
    }

    return android_xr::schemas::CreateUpdateRenderable(
        fbb, entry.first.getId(), fbb.CreateVector(primitives), bounds_info,
        layer_mask, update.bones, update.morph_weights,
        PointerFromOptional(update.priority));
  });
  return CreateCommand(
      fbb, android_xr::schemas::CreateUpdateRenderables(
               fbb, fbb.CreateVector(offset.data(), offset.size())));
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::AddTexturePipelineRenderers>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "add or update texture pipeline renderer: count: "
             << data.size();
  VectorOffset<android_xr::schemas::AddTexturePipelineRenderer> offset(
      data.size());
  absl::c_transform(data, offset.data(), [&fbb](const auto& entry) {
    const AddTexturePipelineRendererInfo& info = entry.second;
    auto state_offset = TexturePipelineRendererSchemaFromState(fbb, info.state);
    if (!state_offset.ok()) {
      IMP_LOG(imp::FATAL) << kTag << "Failed to serialize texture pipeline renderer: "
                 << state_offset.status();
    }
    return android_xr::schemas::CreateAddTexturePipelineRenderer(
        fbb, entry.first.getId(), *state_offset);
  });

  auto command = android_xr::schemas::CreateAddTexturePipelineRenderers(
      fbb, fbb.CreateVector(offset.data(), offset.size()));
  return CreateCommand(fbb, command);
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RemoveTexturePipelineRenderers>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag
             << "remove texture pipeline renderer: count: " << data.size();
  VectorOffset<android_xr::schemas::RemoveTexturePipelineRenderer> offset(
      data.size());
  absl::c_transform(data, offset.data(), [&fbb](const utils::Entity& entity) {
    return android_xr::schemas::CreateRemoveTexturePipelineRenderer(
        fbb, entity.getId());
  });

  auto command = android_xr::schemas::CreateRemoveTexturePipelineRenderers(
      fbb, fbb.CreateVector(offset.data(), offset.size()));
  return CreateCommand(fbb, command);
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::RegisterNamedTextures>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "register named textures: count: " << data.size();
  VectorOffset<android_xr::schemas::RegisterNamedTexture> offset(data.size());
  size_t i = 0;
  for (const auto& [id, info] : data) {
    auto name_offset = fbb.CreateString(info.name);
    offset[i++] =
        android_xr::schemas::CreateRegisterNamedTexture(fbb, id, name_offset);
  }

  auto command = android_xr::schemas::CreateRegisterNamedTextures(
      fbb, fbb.CreateVector(offset.data(), offset.size()));
  return CreateCommand(fbb, command);
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::UnregisterNamedTextures>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag << "register named textures: count: " << data.size();
  VectorOffset<android_xr::schemas::UnregisterNamedTexture> offset(data.size());
  size_t i = 0;
  for (uint64_t id : data) {
    offset[i++] = android_xr::schemas::CreateUnregisterNamedTexture(fbb, id);
  }

  auto command = android_xr::schemas::CreateUnregisterNamedTextures(
      fbb, fbb.CreateVector(offset.data(), offset.size()));
  return CreateCommand(fbb, command);
}

template <>
std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
SerializerDataTypes::Batch<CommandTypes::UpdateTexturePipelineRenderers>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return {};

  IMP_LOG(imp::INFO) << kTag
             << "update texture pipeline renderer: count: " << data.size();
  VectorOffset<android_xr::schemas::UpdateTexturePipelineRenderer> offset(
      data.size());
  absl::c_transform(data, offset.data(), [&fbb](const auto& entry) {
    const UpdateTexturePipelineRendererInfo& info = entry.second;

    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> enabled_passes_offset;
    if (info.enabled_passes) {
      enabled_passes_offset = fbb.CreateVector(*info.enabled_passes);
    }

    flatbuffers::Offset<android_xr::schemas::ProjectionQuad> quad_offset = 0;
    if (info.projection_quad) {
      if (info.projection_quad->has_value()) {
        const auto& q = **info.projection_quad;
        auto size = android_xr::schemas::Float2(q.size.x, q.size.y);
        auto center =
            android_xr::schemas::Float3(q.center.x, q.center.y, q.center.z);
        auto rotation = android_xr::schemas::Quatf(q.rotation.x, q.rotation.y,
                                                   q.rotation.z, q.rotation.w);
        quad_offset = android_xr::schemas::CreateProjectionQuad(
            fbb, &size, &center, &rotation);
      } else {
        // If projection_quad is set but contains nullopt, it means clear it.
        // We'll represent a cleared quad by leaving projection_quad null in
        // the flatbuffer.
      }
    }

    return android_xr::schemas::CreateUpdateTexturePipelineRenderer(
        fbb, entry.first.getId(), enabled_passes_offset, quad_offset);
  });

  auto command = android_xr::schemas::CreateUpdateTexturePipelineRenderers(
      fbb, fbb.CreateVector(offset.data(), offset.size()));
  return CreateCommand(fbb, command);
}

}  // namespace imp::split_engine
