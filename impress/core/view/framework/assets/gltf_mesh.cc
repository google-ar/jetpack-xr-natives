// Copyright 2025 Google LLC
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

#include "core/view/framework/assets/gltf_mesh.h"

#include <assert.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/InstanceBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "core/collision/mesh_collision_accelerator.h"
#include "core/common/filament_helpers.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/paired_vector.h"
#include "core/common/typed_id.h"
#include "core/config.h"
#include "core/material_library/generic_material.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"
#include "core/model/model_data.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/render/base_renderable_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/utils/frame_time.h"
#if IMP_RUNTIME(DEV)
#include "core/common/log.h"
#endif

namespace imp {

using model::ModelData;
using RenderFlags = ModelData::RenderFlags;
using SampledJointId = model::ModelData::SampledJointId;

namespace {

static constexpr uint8_t kMinPriority = 0;
static constexpr uint8_t kMaxPriority = 7;

static constexpr uint8_t kMinChannel = 0;
static constexpr uint8_t kMaxChannel = 3;
}  // namespace

GltfMesh::GltfMesh() : owner_(), self_() {}

void GltfMesh::Setup(ComponentHandle<GltfRenderer> owner,
                     GltfRenderer::ItemId self,
                     std::optional<GltfRenderer::InstanceInfo> instance_info) {
  owner_ = owner;
  self_ = self;
  if (instance_info.has_value()) {
    instance_transforms_.reserve(instance_info->instance_transforms.size());
  }

  const ModelData& model_data = owner_->GetGltfAsset()->GetModelData();
  const ModelData::EntityData::Proxy entity_data = model_data.Entities()[self_];
  const std::vector<ModelData::PartData>& entity_parts = entity_data.parts;
  primitive_count_ = entity_parts.size();
  const model::ModelData::MeshVertexDataLookup& all_vertex_data =
      model_data.GetStoredVertexData();
  const model::ModelData::MeshIndexDataLookup& all_index_data =
      model_data.GetStoredIndexData();
  bool mesh_data_loaded = false;
  if (!all_index_data.empty() && !all_vertex_data.empty()) {
    mesh_data_loaded = true;
  }

  if (primitive_count_) {
    auto* engine = BaseView::GetSharedEngine();
    assert(!GetRenderableManager().HasComponent(GetEntity()));
    std::unique_ptr<BaseRenderableManager::Builder> builder =
        GetRenderableManager().NewBuilder(primitive_count_);
    const absl::optional<filament::Box>& local_bounds =
        entity_data.local_bounds;

    Box bounds = *local_bounds;
    if (instance_info.has_value() &&
        !instance_info->instance_transforms.empty()) {
      filament::Aabb base_bounds{.min = bounds.getMin(),
                                 .max = bounds.getMax()};
      filament::Aabb combined_bounds = base_bounds;
      for (auto instance_transform : instance_info->instance_transforms) {
        mat4f& transform =
            instance_transforms_.emplace_back(instance_transform);
        // The transform applied by the glTF tree (from the root node down to
        // this specific node).
        mat4f gltf_transform{inverse(owner->GetNode()->GetWorldTrsPrecise()) *
                             GetNode()->GetWorldTrsPrecise()};
        // The original instance transform is expected to be applied at the glTF
        // root node, but Filament applies the transform at the specific node's
        // primitive in model space. This maneuver solves for the transform to
        // use at the specific node rather given the transform at the root node.
        transform = inverse(gltf_transform) * (transform * gltf_transform);
        filament::Aabb current_bounds = base_bounds.transform(transform);
        combined_bounds.min = min(combined_bounds.min, current_bounds.min);
        combined_bounds.max = max(combined_bounds.max, current_bounds.max);
      }
      bounds = Box{combined_bounds.center(), combined_bounds.extent()};
    }

    const absl::optional<ModelData::RuntimeData>& runtime_or =
        entity_data.runtime;
    const ModelData::RuntimeData& runtime = *runtime_or;
    const ModelData::SkinId skin = entity_data.skin;
    const ModelData::MorphTargetBufferId morph_target_buffer =
        entity_data.morph_target_buffer;

    builder->BoundingBox(bounds);
    builder->Priority(runtime.priority);
    builder->Culling(!(runtime.flags & RenderFlags::DisableFrustumCulling));

    size_t sampled_transform_count =
        skin ? model_data.Skins()[skin].sampled_joints.size() : 0;

    if (morph_target_buffer) {
      builder->Morphing(model_data.MorphTargetBuffers()[morph_target_buffer]);
    }

    for (auto part_index = 0; part_index < primitive_count_; part_index++) {
      const ModelData::PartData& part = entity_parts[part_index];
      if (mesh_data_loaded) {
        MeshVertexData* vertex_data = all_vertex_data[part.vertex_buffer].get();
        MeshIndexData* index_data = all_index_data[part.index_buffer].get();
        if (!vertex_data->GetDescription()
                 .vertex_format
                 .GetIndexForAttribute(VertexFormat::VertexAttribute::POSITION)
                 .has_value()) {
          mesh_data_loaded = false;
        } else {
          runtime_meshes_.push_back(
              RuntimeMesh{vertex_data, index_data, nullptr});
        }
      }

      if (part.morph_target_buffer_count > 0) {
        builder->Morphing(0, part_index, part.morph_target_buffer_offset,
                          part.morph_target_buffer_count);
      }
      const GenericMaterialPtr& generic_material =
          owner->GetMaterialsInternal()[part.material];
      if (!generic_material) {
        IMP_LOG(imp::FATAL) << "No material found for primitive " << part_index;
      }
      BorrowedMaterialPtr material = generic_material->GetMaterial();
      builder->Material(part_index, material->GetFilamentMaterialInstance());
      builder->Geometry(part_index, part.primitive_type,
                        model_data.VertexBuffers()[part.vertex_buffer],
                        model_data.IndexBuffers()[part.index_buffer],
                        part.index_offset, part.index_count);

      if (part.skinning_buffer) {
        auto buffer = model_data.SkinningBuffers()[part.skinning_buffer]
                          .bone_indices_and_weights;
        auto vertex_count =
            model_data.VertexBuffers()[part.vertex_buffer]->getVertexCount();
        auto bones_per_vertex = buffer.size() / vertex_count;

        builder->BoneIndicesAndWeights(part_index, buffer.data(), buffer.size(),
                                       bones_per_vertex);

#if IMP_RUNTIME(DEV)
        IMP_LOG(imp::INFO) << "The loaded model uses advanced skinning.";
        IMP_LOG(imp::INFO) << "  There are a total of "
                  << model_data.SkinningBuffers().size()
                  << "skinning buffers in this model.";
        IMP_LOG(imp::INFO) << "  Primitive #" << part_index << " (" << part.name
                  << ") uses the buffer at index "
                  << static_cast<uint16_t>(part.skinning_buffer) << ".";
        IMP_LOG(imp::INFO) << "  The skinning buffer size is " << buffer.size()
                  << " float2s, which IS"
                  << (bones_per_vertex * vertex_count == buffer.size() ? ""
                                                                       : " NOT")
                  << "divisible by the vertex count (" << vertex_count << ").";
        IMP_LOG(imp::INFO) << "  " << buffer.size() << "/" << vertex_count << " = "
                  << bones_per_vertex << " bones per vertex.";
        IMP_LOG(imp::INFO) << "  The head of the skinning buffer is as follows:";
        float2* ptr = buffer.data();
        for (auto vertex_index = 0u; vertex_index < 3; ++vertex_index) {
          for (auto bone_index = 0u; bone_index < bones_per_vertex;
               ++bone_index, ++ptr) {
            IMP_LOG(imp::INFO) << "    v" << vertex_index << "/b" << bone_index << ": "
                      << ptr->x << " " << ptr->y;
          }
        }
#endif
      }
    }

    // If some of the primitive in the mesh doesn't have position data loaded,
    // mesh collider is not feasible for this mesh, remove all primitive mesh
    // data, so that the mesh can have AABB collider.
    if (!mesh_data_loaded) {
      runtime_meshes_.clear();
    }
    builder->Skinning(sampled_transform_count);
    builder->CastShadows(!(runtime.flags & RenderFlags::DoNotCastShadows));
    builder->ReceiveShadows(
        !(runtime.flags & RenderFlags::DoNotReceiveShadows));

    if (instance_info.has_value() && !instance_transforms_.empty()) {
      filament::InstanceBuffer::Builder buffer_builder(
          instance_transforms_.size());
      buffer_builder.localTransforms(instance_transforms_.data());
      instance_buffer_ = buffer_builder.build(*BaseView::GetSharedEngine());
      builder->Instances(instance_info->instance_count, instance_buffer_);
    }

    builder->Build(*engine, GetEntity());

    const std::vector<float>& morph_target_weights =
        entity_data.morph_target_weights;
    if (!morph_target_weights.empty()) {
      SetMorphTargetWeights(morph_target_weights);
    } else {
      filament::RenderableManager::Instance instance = GetInstance();
      size_t num_morph_targets =
          GetRenderableManager().GetMorphTargetCount(instance);
      if (num_morph_targets > 0) {
        std::vector<float> weights(num_morph_targets, 0.0f);
        SetMorphTargetWeights(weights);
      }
    }

    // TODO: Remove split engine check once the bug is fixed.
    if (!IsSkinned() || GetView().GetSplitEngineSerializer()) {
      for (RuntimeMesh& runtime_mesh : runtime_meshes_) {
        primitive_mesh_data_.push_back(
            {runtime_mesh.mesh_vertex_data, runtime_mesh.mesh_index_data});
      }
    } else {
      for (RuntimeMesh& runtime_mesh : runtime_meshes_) {
        runtime_mesh.skinned_mesh_vertex_data =
            std::make_unique<MeshVertexData>(
                runtime_mesh.mesh_vertex_data->GetDescription());
        primitive_mesh_data_.push_back(
            {runtime_mesh.skinned_mesh_vertex_data.get(),
             runtime_mesh.mesh_index_data});
      }
    }
  }
}

void GltfMesh::Update(const FrameTime& frame_time) {
  skinned_mesh_data_updated_ = false;
}

void GltfMesh::Cleanup() {
  if (primitive_count_) {
    GetRenderableManager().Destroy(GetEntity());
  }
  if (instance_buffer_) {
    BaseView::GetSharedEngine()->destroy(instance_buffer_);
  }
}

size_t GltfMesh::GetPrimitiveCount() const { return primitive_count_; }

Material* GltfMesh::GetMaterial(size_t primitive_index) const {
  assert(primitive_index < primitive_count_);
  return owner_->GetMaterial(self_.CastTo<GltfRenderer::EntityId>(),
                             primitive_index);
}

Material* GltfMesh::GetMaterialOverride(size_t primitive_index) const {
  return owner_->GetMaterialOverride(self_.CastTo<GltfRenderer::EntityId>(),
                                     primitive_index);
}

void GltfMesh::SetMaterialOverride(Material* new_material,
                                   size_t primitive_index) {
  // Sets or unsets the material override.
  owner_->SetMaterialOverride(
      new_material, self_.CastTo<GltfRenderer::EntityId>(), primitive_index);
}

void GltfMesh::SetMaterialOverride(OwnedMaterialPtr new_material,
                                   size_t primitive_index) {
  // Sets or unsets the material override.
  owner_->SetMaterialOverride(
      OwnedOrBorrowedPtr<Material>(std::move(new_material)),
      self_.CastTo<GltfRenderer::EntityId>(), primitive_index);
}

void GltfMesh::SetMaterialOverride(BorrowedMaterialPtr new_material,
                                   size_t primitive_index) {
  // Sets or unsets the material override.
  owner_->SetMaterialOverride(OwnedOrBorrowedPtr<Material>(new_material),
                              self_.CastTo<GltfRenderer::EntityId>(),
                              primitive_index);
}

size_t GltfMesh::GetMorphTargetCount() const {
  return owner_->GetMorphTargetCount(self_.CastTo<GltfRenderer::EntityId>());
}

void GltfMesh::SetMorphTargetWeights(const std::vector<float>& weights) {
  owner_->SetMorphTargetWeights(weights,
                                self_.CastTo<GltfRenderer::EntityId>());
}

void GltfMesh::SetBlendOrder(uint16_t blend_order, BlendOrderMode mode,
                             size_t primitive) {
  filament::RenderableManager::Instance instance = GetInstance();
  // Filament also clamps the order to 15 bits.
  blend_order = std::clamp(blend_order, kMinBlendOrder, kMaxBlendOrder);
  GetRenderableManager().SetBlendOrderAt(instance, primitive, blend_order);
  GetRenderableManager().SetGlobalBlendOrderEnabledAt(
      instance, primitive, mode == BlendOrderMode::kLocal ? false : true);
}

void GltfMesh::SetPriority(uint8_t priority) {
  // Filament also clamps the priority between 0 and 7, but filament has no
  // getter for the priority so we need to track it ourselves.
  priority_ = std::clamp(priority, kMinPriority, kMaxPriority);
  GetRenderableManager().SetPriority(GetInstance(), priority_);
}

uint8_t GltfMesh::GetPriority() const { return priority_; }

void GltfMesh::SetChannel(uint8_t channel) {
  // Filament also clamps the channel between 0 and 3, but filament has no
  // getter for the channel so we need to track it ourselves.
  channel_ = std::clamp(channel, kMinChannel, kMaxChannel);
  GetRenderableManager().SetChannel(GetInstance(), channel_);
}

uint8_t GltfMesh::GetChannel() const { return channel_; }

const std::string& GltfMesh::GetName() const {
  const ModelData& model_data = owner_->GetGltfAsset()->GetModelData();
  const ModelData::EntityData::Proxy entity_data = model_data.Entities()[self_];
  return entity_data.name;
}

GltfMesh::PrimitiveType GltfMesh::GetPrimitiveType(
    size_t primitive_index) const {
  assert(primitive_index < primitive_count_);
  const ModelData& model_data = owner_->GetGltfAsset()->GetModelData();
  const ModelData::EntityData::Proxy entity_data = model_data.Entities()[self_];
  const std::vector<ModelData::PartData>& parts = entity_data.parts;
  const ModelData::PartData& part = parts[primitive_index];
  return part.primitive_type;
}

BaseRenderableManager& GltfMesh::GetRenderableManager() const {
  return GetView().GetRenderableManager();
}

filament::RenderableManager::Instance GltfMesh::GetInstance() const {
  return GetRenderableManager().GetInstance(GetEntity());
}

void GltfMesh::SetLocalBounds(const Box& local_bounds) {
  GetRenderableManager().SetAxisAlignedBoundingBox(GetInstance(), local_bounds);
}

const Box& GltfMesh::GetLocalBounds() const {
  return GetRenderableManager().GetAxisAlignedBoundingBox(GetInstance());
}

Box GltfMesh::GetWorldBounds() const {
  return TransformBounds(GetLocalBounds(), GetNode()->GetWorldTrs());
}

void GltfMesh::SetShadowCastingMode(ShadowMode shadow_mode) {
  // This class owns this enum, so use an exhaustive switch ((broken link)).
  switch (shadow_mode) {
    case ShadowMode::kHardShadows:
      GetRenderableManager().SetCastShadows(GetInstance(), true);
      break;
    case ShadowMode::kNone:
      GetRenderableManager().SetCastShadows(GetInstance(), false);
      break;
  }
}

GltfMesh::ShadowMode GltfMesh::GetShadowCastingMode() const {
  bool is_shadow_caster = GetRenderableManager().IsShadowCaster(GetInstance());
  return is_shadow_caster ? ShadowMode::kHardShadows : ShadowMode::kNone;
}

void GltfMesh::SetShadowReceivingMode(ShadowMode shadow_mode) {
  // This class owns this enum, so use an exhaustive switch ((broken link)).
  switch (shadow_mode) {
    case ShadowMode::kHardShadows:
      GetRenderableManager().SetReceiveShadows(GetInstance(), true);
      break;
    case ShadowMode::kNone:
      GetRenderableManager().SetReceiveShadows(GetInstance(), false);
      break;
  }
}

GltfMesh::ShadowMode GltfMesh::GetShadowReceivingMode() const {
  bool is_shadow_receiver =
      GetRenderableManager().IsShadowReceiver(GetInstance());
  return is_shadow_receiver ? ShadowMode::kHardShadows : ShadowMode::kNone;
}

void GltfMesh::SetFogEnabled(bool enable) {
  GetRenderableManager().SetFogEnabled(GetInstance(), enable);
}

bool GltfMesh::GetFogEnabled() const {
  return GetRenderableManager().GetFogEnabled(GetInstance());
}

absl::Span<const MeshVertexAndIndexData> GltfMesh::GetMeshData() const {
  return absl::MakeSpan(primitive_mesh_data_);
}

const MeshCollisionAccelerator* GltfMesh::GetMeshCollisionAccelerator() const {
  const ModelData::MorphTargetBufferId morph_target_buffer_id =
      owner_->GetGltfAsset()
          ->GetModelData()
          .Entities()[self_]
          .morph_target_buffer;
  if (IsSkinned() || morph_target_buffer_id) {
    return nullptr;
  }
  return owner_->GetGltfAsset()->GetMeshCollisionAccelerator(self_);
}

// TODO: This method should use skinning_helpers.h so that we can
// deduplicate the code.
void GltfMesh::UpdateSkinnedMesh() {
  // TODO: Make skinning update on the split engine app side
  // configurable. If this is a split engine app, the skinning is done in the
  // split engine renderer side. Skipping this as it is not needed. NOTE: When
  // skipping this, don't try to access the skinning data for the app side, as
  // it will be incorrect.
  if (GetView().GetSplitEngineSerializer()) {
    return;
  }

  if (!IsSkinned()) {
    return;
  }

  if (skinned_mesh_data_updated_) {
    return;
  }
  skinned_mesh_data_updated_ = true;

  model::ModelData::SkinId skin_id =
      owner_->GetGltfAsset()->GetModelData().Entities()[self_].skin;
  model::ModelData::EntityId entity_id = self_;
  const PairedVector<mat4f, model::ModelData::SampledJointData>&
      sampled_xforms = owner_->GetSampledTransforms(skin_id, entity_id);

  for (auto& runtime_mesh : runtime_meshes_) {
    MeshVertexData* vertices = runtime_mesh.mesh_vertex_data;
    MeshVertexData* vertices_on_skin =
        runtime_mesh.skinned_mesh_vertex_data.get();

    const VertexFormat& vertex_format =
        vertices->GetDescription().vertex_format;
    absl::optional<size_t> bone_indices_attr_id =
        vertex_format.GetIndexForAttribute(
            VertexFormat::VertexAttribute::BONE_INDICES);
    // In case, a primitive has no bone indices/weights.
    if (!bone_indices_attr_id.has_value()) {
      vertices_on_skin = vertices;
      continue;
    }

    size_t bone_indicies_attribute_offset = vertex_format.GetAttributeOffsetAt(
        vertex_format
            .GetIndexForAttribute(VertexFormat::VertexAttribute::BONE_INDICES)
            .value());
    size_t bone_weights_attribute_offset = vertex_format.GetAttributeOffsetAt(
        vertex_format
            .GetIndexForAttribute(VertexFormat::VertexAttribute::BONE_WEIGHTS)
            .value());
    size_t position_attribute_offset = vertex_format.GetAttributeOffsetAt(
        vertex_format
            .GetIndexForAttribute(VertexFormat::VertexAttribute::POSITION)
            .value());

    for (size_t i = 0; i < vertices->GetDescription().vertex_count; i++) {
      ushort4 bone_indices;
      if (vertex_format.GetAttributeAt(bone_indices_attr_id.value()).type ==
          VertexFormat::AttributeType::USHORT4) {
        bone_indices = vertices->VertexAttributeAt<ushort4>(
            i, bone_indicies_attribute_offset);
      } else {
        bone_indices = static_cast<ushort4>(vertices->VertexAttributeAt<ubyte4>(
            i, bone_indicies_attribute_offset));
      }
      float4& bone_weight =
          vertices->VertexAttributeAt<float4>(i, bone_weights_attribute_offset);

      SampledJointId i1 = SampledJointId::At(bone_indices.x);
      SampledJointId i2 = SampledJointId::At(bone_indices.y);
      SampledJointId i3 = SampledJointId::At(bone_indices.z);
      SampledJointId i4 = SampledJointId::At(bone_indices.w);

      if (sampled_xforms.IsValid(i1) && sampled_xforms.IsValid(i2) &&
          sampled_xforms.IsValid(i3) && sampled_xforms.IsValid(i4)) {
        mat4f transform = bone_weight.x * sampled_xforms[i1] +
                          bone_weight.y * sampled_xforms[i2] +
                          bone_weight.z * sampled_xforms[i3] +
                          bone_weight.w * sampled_xforms[i4];
        float3& posi =
            vertices->VertexAttributeAt<float3>(i, position_attribute_offset);
        float4 new_posi = transform * float4(posi, 1.0f);
        vertices_on_skin->VertexAttributeAt<float3>(
            i, position_attribute_offset) = new_posi.xyz;
      }
    }
  }
}

bool GltfMesh::IsSkinned() const {
  const ModelData& model_data = owner_->GetGltfAsset()->GetModelData();
  model::ModelData::SkinId skin_id = model_data.Entities()[self_].skin;
  return !model_data.Skins().empty() && skin_id;
}

uint64_t GltfMesh::GetOriginalGltfIndex() const {
  const ModelData& model_data = owner_->GetGltfAsset()->GetModelData();
  return model_data.Entities()[self_].original_index;
}

const std::vector<mat4f>& GltfMesh::GetInstanceTransforms() const {
  return instance_transforms_;
}

ComponentHandle<GltfRenderer> GltfMesh::GetGltfRenderer() const {
  return owner_;
}

}  // namespace imp
