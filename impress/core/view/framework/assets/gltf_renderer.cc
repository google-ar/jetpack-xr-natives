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

#include "core/view/framework/assets/gltf_renderer.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/libs/math/include/math/scalar.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/bit_vector.h"
#include "core/common/filament_helpers.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/paired_vector.h"
#include "core/common/trace.h"
#include "core/common/typed_id.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_vector.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/generic_material.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/render/base_renderable_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_extension.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/assets/gltf_state.proto.imp.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/lighting/light_component.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/frame_time.h"

namespace imp {
using ::filament::TransformManager;
using ModelData = model::ModelData;
using SkinnedEntityData = ModelData::SkinnedEntityData;
using SampledJoint = ModelData::SampledJointData;
using Instance = TransformManager::Instance;

namespace {
ComponentHandle<LightComponent> AddLightForNode(
    NodeHandle node, const ModelData::LightPunctualData& light) {
  auto intensity = light.intensity;

  LightComponent::Type type;
  switch (light.type) {
    case schemas::LightPunctualType::DIRECTIONAL:
      type = LightComponent::Type::DIRECTIONAL;
      break;
    case schemas::LightPunctualType::POINT:
      type = LightComponent::Type::POINT;
      break;
    case schemas::LightPunctualType::SPOT:
      type = LightComponent::Type::SPOT;
      break;
  }
  if (type == LightComponent::Type::POINT ||
      type == LightComponent::Type::SPOT) {
    // For point and spot lights, gltf intensity is specified in
    // candelas, but filament accepts lumens.
    intensity *= 4 * filament::math::F_PI;
  }
  auto light_component = node->AddComponent<imp::LightComponent>(type);
  light_component->SetShadowCastingDisabled(true);
  light_component->SetColor(light.color);
  light_component->SetIntensity(intensity);
  light_component->SetFalloff(light.range);

  if (type == LightComponent::Type::SPOT && light.spot_cone_angles) {
    light_component->SetSpotCone(*light.spot_cone_angles);
  }

  return light_component;
}
}  // namespace

void GltfRenderer::Cleanup() {
  // If this is being Cleanedup() with the general Node hierarchy, then other
  // components or our children may be destroyed already, so check before
  // calling any functions.
  if (auto scene = GetNode()->GetComponent<GltfScene>()) {
    scene->ForAllNodes([this](NodeHandle node) {
      if (node) {
        node->RemoveComponent<GltfMesh>();
        if (GetColliderMode() == GltfState::ColliderMode::
                                     GLTF_COLLIDER_BOUNDS_PER_MESH_DEFAULT ||
            GetColliderMode() ==
                GltfState::ColliderMode::GLTF_COLLIDER_TRIANGLES_PER_MESH ||
            GetColliderMode() == GltfState::ColliderMode::
                                     GLTF_COLLIDER_MESH_COLLISION_ACCELERATOR) {
          node->RemoveComponent<GltfCollider>();
        }
      }
    });
    if (GetColliderMode() == GltfState::ColliderMode::BOX_COLLIDER) {
      GetNode()->RemoveComponent<BoxCollider>();
    }
  }

  duplicated_materials_.clear();
  raw_material_index_overrides_.clear();
  owned_or_borrowed_material_index_overrides_.clear();
}

void GltfRenderer::Setup(AssetPtr<GltfAsset> gltf_asset,
                         std::optional<GltfAsset::LoadOptions> options) {
  if (!gltf_asset) {
    IMP_LOG(imp::FATAL) << "Cannot Setup GltfRenderer with invalid GltfAsset.";
  }

  if (options) {
    load_options_ = *options;
  } else {
    load_options_ = GetView().GetAssetManager().GetDefaultLoadOptions();
  }

  gltf_asset_ = gltf_asset;
  const auto& data = gltf_asset->GetModelData();

  // Ideally we should just call resize on the per-index material override array
  // so that each index will contain a nullptr. However, that would result in
  // the compiler not being happy about the deleted copy constructor so we're
  // just allocating the memory first and then manually assign them one by one.
  raw_material_index_overrides_.reserve(data.Materials().size());
  owned_or_borrowed_material_index_overrides_.reserve(data.Materials().size());
  for (int i = 0; i < data.Materials().size(); ++i) {
    raw_material_index_overrides_.emplace_back(nullptr);
    owned_or_borrowed_material_index_overrides_.emplace_back(
        OwnedOrBorrowedPtr<Material>{});
  }

  if (GetMaterialSharingMode() ==
      GltfState::MaterialSharingMode::DUPLICATED_DEFAULT) {
    // Duplicate all the materials so that this model can set parameters
    // independently of other models.
    duplicated_materials_.reserve(data.Materials().size());
    for (auto& material : data.Materials()) {
      duplicated_materials_.emplace_back(material->Duplicate());
    }
  }

  ComponentHandle<GltfScene> scene =
      GetNode()->AddComponent<GltfScene>(gltf_asset);

  bool instanced_rendering =
      options.has_value() && !options->instance_transforms.empty();
  if (instanced_rendering) {
    instance_info_.instance_count = options->instance_transforms.size();
    instance_info_.instance_transforms =
        std::move(options->instance_transforms);
  }

  // Cache the node for each part in the model.
  node_entities_.resize(data.Entities().size());
  const TypedSetVector<EntityData>& entities = data.Entities();
  for (auto self : entities.Ids<ModelData::EntityId>()) {
    const auto entity_data = entities[self];
    NodeHandle node = scene->GetOrCreateNodeFromBone(entity_data.bone);
    const std::vector<model::ModelData::PartData>& parts = entity_data.parts;
    model::ModelData::LightPunctualId light_punctual =
        entity_data.light_punctual;
    node_entities_[self] = node->GetEntity();

    if (!parts.empty()) {
      // TODO Add collision support for instanced gltfs
      if (instanced_rendering) {
        node->AddComponent<GltfMesh>(GetHandle(this), self.CastTo<ItemId>(),
                                     instance_info_);
      } else {
        auto gltf_mesh = node->AddComponent<GltfMesh>(GetHandle(this),
                                                      self.CastTo<ItemId>());

        if (!GetColliderMode() ||
            GetColliderMode() == GltfState::ColliderMode::
                                     GLTF_COLLIDER_BOUNDS_PER_MESH_DEFAULT) {
          node->AddComponent<GltfCollider>(gltf_mesh);
        } else if (GetColliderMode() == GltfState::ColliderMode::
                                            GLTF_COLLIDER_TRIANGLES_PER_MESH ||
                   GetColliderMode() ==
                       GltfState::ColliderMode::
                           GLTF_COLLIDER_MESH_COLLISION_ACCELERATOR) {
          node->AddComponent<GltfCollider>(
              gltf_mesh, GltfCollider::CollisionMode::kTriangles);
        }
      }
    }

    if (data.LightsPunctual().IsValid(light_punctual)) {
      const ModelData::LightPunctualData& light =
          data.LightsPunctual()[light_punctual];
      light_punctual_lookup_[light_punctual] = AddLightForNode(node, light);
    }

    std::optional<model::ModelData::NodeVisibility> node_visibility =
        entity_data.node_visibility;
    if (node_visibility) {
      node->SetEnabled(node_visibility->visible.value_or(true));
    }
  }

  // TODO Add collision support for instanced gltfs
  if (!instanced_rendering) {
    if (GetColliderMode() == GltfState::ColliderMode::BOX_COLLIDER) {
      Box bounds = GetLocalBounds();
      if (auto box_collider = GetNode()->GetComponent<BoxCollider>()) {
        box_collider->SetBox(bounds);
      } else {
        GetNode()->AddComponent<BoxCollider>(bounds);
      }
    }
  }

  if (state_.material_variant) {
    SetMaterialsVariant(
        model::ModelData::MaterialsVariantsId::At(*state_.material_variant));
  }

  InitializeSkinning();
  UpdateSkinning();

  GltfRenderer::System& system =
      GetView().GetComponentManager().GetComponentSystem<GltfRenderer>();

  extension_setup_future_ = system.SetupExtensionsForRenderer(GetHandle(this));
}

Future<absl::Status> GltfRenderer::Setup() {
  // This version of Setup should only be called as part of an isf load.
  assert(!state_.asset.empty());
  absl::optional<GltfAsset::LoadOptions> options =
      GetView().GetAssetManager().GetDefaultLoadOptions();
  if (state_.collider_mode.has_value()) {
    options->collider_mode = *state_.collider_mode;
  }

  if (state_.contents.has_value()) {
    return Setup(absl::Cord(*state_.contents), state_.asset,
                 std::move(options));
  }
  return Setup(state_.asset, std::move(options));
}

Future<absl::Status> GltfRenderer::Setup(
    const AssetDefinition& asset_definition,
    std::optional<GltfAsset::LoadOptions> options) {
  return Setup(asset_definition.GetUrl(), std::move(options));
}

Future<absl::Status> GltfRenderer::Setup(
    absl::string_view asset_url,
    std::optional<GltfAsset::LoadOptions> options) {
  IMP_TRACE();
  if (state_.asset != asset_url) {
    state_.asset = std::string{asset_url};
  }

  return GetView()
      .GetAssetManager()
      .LoadGltfAsset(asset_url, options)
      .Then([this, options](const AssetPtr<GltfAsset>& asset) mutable
                -> Future<absl::Status> {
        IMP_TRACE_BLOCK("Then");
        Setup(asset, options);

        return extension_setup_future_;
      });
}

Future<absl::Status> GltfRenderer::Setup(
    absl::Cord contents, absl::string_view asset_url,
    std::optional<GltfAsset::LoadOptions> options) {
  IMP_TRACE();
  state_.asset = std::string(asset_url);

  contents.Flatten();
  return GetView()
      .GetAssetManager()
      .LoadGltfAsset(std::move(contents), asset_url, options)
      .Then([this, options](const AssetPtr<GltfAsset>& asset) mutable
                -> Future<absl::Status> {
        IMP_TRACE_BLOCK("Then");
        Setup(asset, options);

        return extension_setup_future_;
      });
}

void GltfRenderer::OnActiveStatusChanged(bool is_active) {
  IMP_TRACE();
  BaseRenderableManager& renderable_manager = GetView().GetRenderableManager();

  const TypedSetVector<EntityData>& entities =
      gltf_asset_->GetModelData().Entities();
  for (auto entity_id : entities.Ids<ModelData::EntityId>()) {
    utils::Entity node_id = node_entities_[entity_id];
    if (!renderable_manager.HasComponent(node_id)) {
      continue;
    }
    // Turns off or on the rendering of the model by setting the layer mask to 0
    // or 1. If the layer mask is used elsewhere in the future, this method of
    // setting visibility may need to be revisited.
    if (!is_active) {
      renderable_manager.SetLayerMask(renderable_manager.GetInstance(node_id),
                                      0xff, 0);
    } else {
      renderable_manager.SetLayerMask(renderable_manager.GetInstance(node_id),
                                      0xff, 1);
    }
  }
}

void GltfRenderer::Update(const FrameTime& frame_time) { UpdateSkinning(); }

AssetPtr<GltfAsset> GltfRenderer::GetGltfAsset() const { return gltf_asset_; }

NodeHandle GltfRenderer::GetModelRoot() const {
  return GetNode()->GetComponent<GltfScene>()->GetRoot();
}

const GenericMaterialListing& GltfRenderer::GetMaterialsInternal() const {
  if (GetMaterialSharingMode() == GltfState::MaterialSharingMode::SHARED) {
    return gltf_asset_->GetSharedMaterials();
  }
  return duplicated_materials_;
}

std::vector<Material*> GltfRenderer::GetMaterials() const {
  std::vector<Material*> return_value;

  const GenericMaterialListing& materials = GetMaterialsInternal();

  return_value.reserve(materials.size());

  for (const auto& material : materials) {
    if (material) {
      // TODO: (broken link) - Use the BorrowedMaterialPtr instead of the
      // Material* once the Material API is updated.
      return_value.push_back(&(*material->GetMaterial()));
    }
  }

  auto overrides(GetMaterialOverrides());
  return_value.insert(return_value.end(), overrides.begin(), overrides.end());
  return return_value;
}

Material* GltfRenderer::GetMaterialByIndex(uint16_t material_index) const {
  const GenericMaterialListing& materials = GetMaterialsInternal();
  MaterialId material_id =
      GetGltfAsset()->GetModelData().GetMaterialId(material_index);
  // Check if the material id is valid.
  if (!material_id) {
    return nullptr;
  }
  // TODO: (broken link) - Use the BorrowedMaterialPtr instead of the
  // Material* once the Material API is updated.
  return &(*materials[material_id]->GetMaterial());
}

absl::StatusOr<GenericMaterial*> GltfRenderer::GetGenericMaterialByIndex(
    uint16_t material_index) const {
  if (GetMaterialSharingMode() == GltfState::MaterialSharingMode::SHARED) {
    return absl::UnavailableError(
        "Cannot return non-const material for models using shared materials.");
  }
  MaterialId material_id =
      GetGltfAsset()->GetModelData().GetMaterialId(material_index);
  // Check if the material id is valid.
  if (!material_id) {
    return absl::NotFoundError("Material index invalid");
  }
  if (!duplicated_materials_.IsValid(material_id)) {
    return absl::NotFoundError("Material not found for given index");
  }
  return duplicated_materials_[material_id].get();
}

ComponentHandle<LightComponent> GltfRenderer::GetLightComponentById(
    LightPunctualId id) {
  return light_punctual_lookup_[id];
}

std::vector<Material*> GltfRenderer::GetMaterialOverrides() const {
  std::vector<Material*> return_value;
  return_value.reserve(raw_material_primitive_overrides_.size() +
                       owned_or_borrowed_material_primitive_overrides_.size());

  for (const auto& pair : raw_material_primitive_overrides_) {
    if (Material* material = pair.second; material) {
      return_value.push_back(material);
    }
  }

  for (const auto& material_override :
       owned_or_borrowed_material_primitive_overrides_) {
    if (Material* material = material_override.second.operator->(); material) {
      return_value.push_back(material);
    }
  }

  for (Material* raw_material_override : raw_material_index_overrides_) {
    if (raw_material_override) {
      return_value.push_back(raw_material_override);
    }
  }

  for (const OwnedOrBorrowedPtr<Material>& material_override :
       owned_or_borrowed_material_index_overrides_) {
    if (Material* material = material_override.operator->(); material) {
      return_value.push_back(material);
    }
  }

  return return_value;
}

void GltfRenderer::SetMaterialOverride(Material* raw_new_material,
                                       EntityId entity_id,
                                       size_t primitive_index) {
  GltfRenderer::GltfPrimitive primitive_id{.entity_id = entity_id,
                                           .primitive_index = primitive_index};

  if (raw_new_material == nullptr) {
    raw_material_primitive_overrides_.erase(primitive_id);
  } else {
    raw_material_primitive_overrides_.insert_or_assign(primitive_id,
                                                       raw_new_material);
  }
  owned_or_borrowed_material_primitive_overrides_.erase(primitive_id);

  // Change the material filament is using to new material or the original
  // material.
  // use GetMaterial to re-get the material which has been moved and correctly
  // handle the case where the override has been set to null.
  BaseRenderableManager& renderable_manager = GetView().GetRenderableManager();
  utils::Entity node_id = node_entities_[entity_id];
  renderable_manager.SetMaterialInstanceAt(
      renderable_manager.GetInstance(node_id), primitive_index,
      GetMaterial(entity_id, primitive_index)->GetFilamentMaterialInstance());
}

void GltfRenderer::SetMaterialOverride(
    OwnedOrBorrowedPtr<Material> new_material, EntityId entity_id,
    size_t primitive_index) {
  GltfRenderer::GltfPrimitive primitive_id{.entity_id = entity_id,
                                           .primitive_index = primitive_index};

  if (!new_material) {
    owned_or_borrowed_material_primitive_overrides_.erase(primitive_id);
  } else {
    owned_or_borrowed_material_primitive_overrides_.insert_or_assign(
        primitive_id, std::move(new_material));
  }
  raw_material_primitive_overrides_.erase(primitive_id);

  // Change the material filament is using to new material or the original
  // material.
  // use GetMaterial to re-get the material which has been moved and correctly
  // handle the case where the override has been set to null.
  BaseRenderableManager& renderable_manager = GetView().GetRenderableManager();
  utils::Entity node_id = node_entities_[entity_id];
  renderable_manager.SetMaterialInstanceAt(
      renderable_manager.GetInstance(node_id), primitive_index,
      GetMaterial(entity_id, primitive_index)->GetFilamentMaterialInstance());
}

void GltfRenderer::SetMaterialOverrideByIndex(Material* new_material,
                                              size_t material_index) {
  SetMaterialOverrideByIndexInternal(new_material, material_index);
}

void GltfRenderer::SetMaterialOverrideByIndex(OwnedMaterialPtr new_material,
                                              size_t material_index) {
  SetMaterialOverrideByIndexInternal(
      OwnedOrBorrowedPtr<Material>(std::move(new_material)), material_index);
}

void GltfRenderer::SetMaterialOverrideByIndex(BorrowedMaterialPtr new_material,
                                              size_t material_index) {
  SetMaterialOverrideByIndexInternal(OwnedOrBorrowedPtr<Material>(new_material),
                                     material_index);
}

void GltfRenderer::SetMaterialOverrideByIndexInternal(
    Material* raw_new_material, size_t material_index) {
  const GenericMaterialListing& materials = GetMaterialsInternal();

  if (material_index >= materials.size()) {
    IMP_LOG(imp::ERROR) << "Failed to set material override by index: " << material_index
               << " is not within the range of [0, " << materials.size() << ")";
  }

  const MaterialId material_id = MaterialId::At(material_index);

  owned_or_borrowed_material_index_overrides_[material_id] =
      OwnedOrBorrowedPtr<Material>();
  raw_material_index_overrides_[material_id] = raw_new_material;

  BaseRenderableManager& renderable_manager = GetView().GetRenderableManager();

  const TypedSetVector<EntityData>& entities =
      gltf_asset_->GetModelData().Entities();

  // Similar to SetMaterialOverride, we need to update the materials
  for (auto entity_id : entities.Ids<ModelData::EntityId>()) {
    const auto entity_data = entities[entity_id];
    const std::vector<model::ModelData::PartData>& parts = entity_data.parts;
    utils::Entity node_id = node_entities_[entity_id];

    for (size_t primitive_index = 0; primitive_index < parts.size();
         primitive_index++) {
      if (parts[primitive_index].material != material_id) {
        continue;
      }

      renderable_manager.SetMaterialInstanceAt(
          renderable_manager.GetInstance(node_id), primitive_index,
          GetMaterial(entity_id, primitive_index)
              ->GetFilamentMaterialInstance());
    }
  }
}

void GltfRenderer::SetMaterialOverrideByIndexInternal(
    OwnedOrBorrowedPtr<Material> new_material, size_t material_index) {
  const GenericMaterialListing& materials = GetMaterialsInternal();

  if (material_index >= materials.size()) {
    IMP_LOG(imp::ERROR) << "Failed to set material override by index: " << material_index
               << " is not within the range of [0, " << materials.size() << ")";
  }

  const MaterialId material_id = MaterialId::At(material_index);

  owned_or_borrowed_material_index_overrides_[material_id] =
      std::move(new_material);
  raw_material_index_overrides_[material_id] = nullptr;

  BaseRenderableManager& renderable_manager = GetView().GetRenderableManager();

  const TypedSetVector<EntityData>& entities =
      gltf_asset_->GetModelData().Entities();

  // Similar to SetMaterialOverride, we need to update the materials
  for (auto entity_id : entities.Ids<ModelData::EntityId>()) {
    const auto entity_data = entities[entity_id];
    const std::vector<model::ModelData::PartData>& parts = entity_data.parts;
    utils::Entity node_id = node_entities_[entity_id];

    for (size_t primitive_index = 0; primitive_index < parts.size();
         primitive_index++) {
      if (parts[primitive_index].material != material_id) {
        continue;
      }

      renderable_manager.SetMaterialInstanceAt(
          renderable_manager.GetInstance(node_id), primitive_index,
          GetMaterial(entity_id, primitive_index)
              ->GetFilamentMaterialInstance());
    }
  }
}

Material* GltfRenderer::GetMaterialOverrideByIndex(
    size_t material_index) const {
  const MaterialId material_id = MaterialId::At(material_index);
  if (raw_material_index_overrides_.IsValid(material_id)) {
    if (Material* material = raw_material_index_overrides_[material_id];
        material) {
      return material;
    }
  }
  if (owned_or_borrowed_material_index_overrides_.IsValid(material_id)) {
    if (Material* material =
            owned_or_borrowed_material_index_overrides_[material_id]
                .operator->();
        material) {
      return material;
    }
  }

  return nullptr;
}

Material* GltfRenderer::GetMaterial(EntityId entity_id,
                                    size_t primitive_index) const {
  Material* material_override = GetMaterialOverride(entity_id, primitive_index);
  if (material_override) {
    return material_override;
  }

  const ModelData& model_data = GetGltfAsset()->GetModelData();
  const ModelData::EntityData::Proxy entity_data =
      model_data.Entities()[entity_id];
  const std::vector<ModelData::PartData> entity_parts = entity_data.parts;
  const model::ModelData::PartData& part = entity_parts[primitive_index];

  const GenericMaterialListing& materials = GetMaterialsInternal();

  // If there is an active material variant, see if this part has a special
  // material for this variant.
  if (active_materials_variant_id_.has_value()) {
    model::ModelData::MaterialId optional_material_id =
        part.materials_variants_mappings[active_materials_variant_id_.value()];
    if (optional_material_id) {
      // TODO: (broken link) - Use the BorrowedMaterialPtr instead of the
      // Material* once the Material API is updated.
      return &(*materials[optional_material_id]->GetMaterial());
    }
  }

  // No override or active material variant, return the default material for
  // this part.
  // TODO: (broken link) - Use the BorrowedMaterialPtr instead of the
  // Material* once the Material API is updated.
  return &(*materials[part.material]->GetMaterial());
}

Material* GltfRenderer::GetMaterialOverride(EntityId entity_id,
                                            size_t primitive_index) const {
  GltfRenderer::GltfPrimitive primitive_id{.entity_id = entity_id,
                                           .primitive_index = primitive_index};

  // First, check for per-primitive override
  auto map_iter = raw_material_primitive_overrides_.find(primitive_id);
  if (map_iter != raw_material_primitive_overrides_.end()) {
    return map_iter.value();
  }
  auto owned_or_borrowed_map_iter =
      owned_or_borrowed_material_primitive_overrides_.find(primitive_id);
  if (owned_or_borrowed_map_iter !=
      owned_or_borrowed_material_primitive_overrides_.end()) {
    return owned_or_borrowed_map_iter.value().operator->();
  }

  // Then check for per-index override
  const ModelData& model_data = GetGltfAsset()->GetModelData();
  const ModelData::EntityData::Proxy entity_data =
      model_data.Entities()[entity_id];
  const std::vector<ModelData::PartData> entity_parts = entity_data.parts;
  const model::ModelData::PartData& part = entity_parts[primitive_index];

  const MaterialId material_id = part.material;

  if (Material* material = raw_material_index_overrides_[material_id];
      material) {
    return material;
  }
  return owned_or_borrowed_material_index_overrides_[material_id].operator->();
}

Box GltfRenderer::GetLocalBounds() const {
  Box result = NilBounds();
  auto local_from_world = inverse(GetNode()->GetWorldTrs());

  for (const auto& e : node_entities_) {
    NodeHandle child_node(e);
    if (!child_node) {
      continue;
    }
    auto gltf_mesh = child_node->GetComponent<GltfMesh>();
    if (!gltf_mesh) {
      continue;
    }

    mat4f local_from_entity = local_from_world * child_node->GetWorldTrs();
    Box local_bounds =
        imp::TransformBounds(gltf_mesh->GetLocalBounds(), local_from_entity);
    result.unionSelf(local_bounds);
  }

  return result;
}

Box GltfRenderer::GetWorldBounds() const {
  return TransformBounds(GetLocalBounds(), GetNode()->GetWorldTrs());
}

Box GltfRenderer::GetLocalFullBounds() const {
  Box result = GetLocalBounds();
  result.unionSelf(GetNode()->GetComponent<GltfScene>()->GetLocalBoneBounds());
  return result;
}

Box GltfRenderer::GetWorldFullBounds() const {
  Box result = GetWorldBounds();
  result.unionSelf(GetNode()->GetComponent<GltfScene>()->GetWorldBoneBounds());
  return result;
}

void GltfRenderer::SetRenderBounds(const Box& local_bounds) {
  auto* engine = BaseView::GetSharedEngine();
  filament::TransformManager& tm = engine->getTransformManager();

  mat4f world_from_node =
      tm.getWorldTransform(tm.getInstance(GetModelRoot()->GetEntity()));
  const auto& entity_datas = gltf_asset_->GetModelData().Entities();
  for (auto id : entity_datas.Ids<ModelData::EntityId>()) {
    const auto entity_data = entity_datas[id];
    if (const std::vector<model::ModelData::PartData>& parts =
            entity_data.parts;
        parts.empty()) {
      continue;
    }
    auto entity = node_entities_[id];
    auto world_from_entity = tm.getWorldTransform(tm.getInstance(entity));
    auto entity_from_node = inverse(world_from_entity) * world_from_node;
    NodeHandle(entity)->GetComponent<GltfMesh>()->SetLocalBounds(
        TransformBounds(local_bounds, entity_from_node));
  }
}

void GltfRenderer::InitializeSkinning() {
  assert(gltf_asset_);

  runtime_skins_.clear();
  const auto& data = gltf_asset_->GetModelData();
  const auto& skins = data.Skins();
  const size_t bone_count = data.Skeleton().bones.size();
  root_xforms_.resize(bone_count);
  root_xforms_updated_.Resize(bone_count);
  for (auto skin : skins.Ids<ModelData::SkinId>()) {
    const auto& skin_data = skins[skin];
    RuntimeSkin skin_runtime;
    skin_runtime.skin = skin;

    for (auto skinned_entity :
         skin_data.skinned_entities.Ids<ModelData::SkinnedEntityId>()) {
      PairedVector<mat4f, SampledJoint> sampled_xforms;
      sampled_xforms.resize(skin_data.sampled_joints.size());
      skin_runtime.skinned_entities.push_back(
          RuntimeSkinnedEntity{.skinned_entity = skinned_entity,
                               .sampled_xforms = std::move(sampled_xforms)});
    }
    runtime_skins_.push_back(std::move(skin_runtime));
  }
}

NodeHandle GltfRenderer::GetOrCreateNode(absl::string_view name) {
  return GetNode()->GetComponent<GltfScene>()->GetOrCreateNode(name);
}

void GltfRenderer::ScheduleSkinningUpdate() { skinning_scheduled_ = true; }

void GltfRenderer::UpdateSkinning() {
  IMP_TRACE();

  GltfRenderer::System& system =
      GetView().GetComponentManager().GetComponentSystem<GltfRenderer>();

  if (system.GetSkinningSystemOverride() ==
      SkinningSystemOverride::kSkinningSystemDisabled) {
    return;
  }

  // Since we can load async, Setup() may not be called yet.
  auto scene = GetNode()->GetComponent<GltfScene>();
  if (!gltf_asset_ || !scene || !IsActive() ||
      (!skinning_scheduled_ && !scene->GetLocalTrsUpdated())) {
    return;
  }

  skinning_scheduled_ = false;
  auto* engine = BaseView::GetSharedEngine();
  filament::TransformManager& tm = engine->getTransformManager();

  if (!runtime_skins_.empty()) {
    BaseRenderableManager& renderable_manager =
        GetView().GetRenderableManager();
    const ModelData& data = gltf_asset_->GetModelData();
    size_t bone_count = data.Skeleton().bones.size();

    if (root_xforms_.size() < bone_count) {
      root_xforms_.resize(bone_count);
    }
    if (root_xforms_updated_.size() < bone_count) {
      root_xforms_updated_.Resize(bone_count);
    }
    root_xforms_updated_.SetAll(false);

    for (RuntimeSkin& runtime_skin : runtime_skins_) {
      const ModelData::SkinData& skin_data = data.Skins()[runtime_skin.skin];

      // Walk the Joint tree composing local transforms into world transforms.
      for (JointId self : skin_data.joints.Ids<JointId>()) {
        BoneId source = skin_data.joints[self].source;
        if (root_xforms_updated_.Get(source)) continue;
        const mat4f& parent_from_joint =
            scene->GetLocalTransformMatFromBone(source);
        if (model::ModelData::JointParentId parent =
                skin_data.joints[self].parent) {
          BoneId parent_source = skin_data.joints[parent].source;
          const mat4f& root_from_parent = root_xforms_[parent_source];
          root_xforms_[source] = root_from_parent * parent_from_joint;
        } else {
          root_xforms_[source] = parent_from_joint;
        }
        root_xforms_updated_.Set(source);
      }

      utils::Entity root_entity = GetModelRoot()->GetEntity();
      for (RuntimeSkinnedEntity& runtime_skinned_entity :
           runtime_skin.skinned_entities) {
        PairedVector<mat4f, SampledJoint>& sampled_xforms =
            runtime_skinned_entity.sampled_xforms;
        const SkinnedEntityData::Proxy skinned_entity =
            skin_data.skinned_entities[runtime_skinned_entity.skinned_entity];
        EntityId target = skinned_entity.target;
        const ModelData::SampledJointLookup<filament::Aabb>&
            sampled_joint_bounds = skinned_entity.sampled_joint_bounds;
        const PairedBitVector<ModelData::SampledJointData>&
            sampled_joint_in_use = skinned_entity.sampled_joint_in_use;
        auto combined_bounds = filament::Aabb{};

        const utils::Entity target_entity = node_entities_[target];
        mat4f target_from_root;
        if (tm.isAccurateTranslationsEnabled()) {
          mat4 world_from_target =
              tm.getWorldTransformAccurate(tm.getInstance(target_entity));
          mat4 world_from_root =
              tm.getWorldTransformAccurate(tm.getInstance(root_entity));
          target_from_root =
              inverse(mat4f(inverse(world_from_root) * world_from_target));
        } else {
          mat4f world_from_target =
              tm.getWorldTransform(tm.getInstance(target_entity));
          mat4f world_from_root =
              tm.getWorldTransform(tm.getInstance(root_entity));
          target_from_root =
              inverse(inverse(world_from_root) * world_from_target);
        }

        for (auto sampled_joint :
             skin_data.sampled_joints.Ids<SampledJointId>()) {
          if (!sampled_joint_in_use.Get(sampled_joint)) continue;
          JointId joint = skin_data.sampled_joints[sampled_joint].joint;
          BoneId bone = skin_data.joints[joint].source;
          const mat4f& bind_bone_from_bind_pose =
              skin_data.inverse_bind_poses[sampled_joint];
          const mat4f& root_from_joint = root_xforms_[bone];
          const mat4f target_from_bind_pose =
              target_from_root * root_from_joint * bind_bone_from_bind_pose;
          sampled_xforms[sampled_joint] = target_from_bind_pose;

          filament::Aabb joint_bounds =
              sampled_joint_bounds[sampled_joint].transform(
                  target_from_bind_pose);
          if (!joint_bounds.isEmpty()) {
            combined_bounds.min = min(combined_bounds.min, joint_bounds.min);
            combined_bounds.max = max(combined_bounds.max, joint_bounds.max);
          }
        }

        filament::RenderableManager::Instance target_instance =
            renderable_manager.GetInstance(target_entity);

        // TODO: Should be able to accumulate these settings and
        // use as "deltas" so if this gets called multiple times per frame, we
        // update the new data. Then, when we go to call View::Render, we
        // serialize all the data. Could be a good use-case for an Updater:
        // google3/third_party/impress/core/view/utils/render_state_validator.h
        renderable_manager.SetBones(target_instance, sampled_xforms.data(),
                                    sampled_xforms.size());

        if (combined_bounds.isEmpty()) {
          // Don't convert default-value 'Box' into 'Aabb'; it doesn't work
          // with -ffast-math.
          combined_bounds.min = imp::kZero3;
          combined_bounds.max = imp::kZero3;
        }

        if (!instance_info_.instance_transforms.empty()) {
          filament::Aabb base_bounds = combined_bounds;
          for (auto instance_transform : NodeHandle(target_entity)
                                             ->GetComponent<GltfMesh>()
                                             ->GetInstanceTransforms()) {
            filament::Aabb bounds = base_bounds.transform(instance_transform);
            combined_bounds.min = min(combined_bounds.min, bounds.min);
            combined_bounds.max = max(combined_bounds.max, bounds.max);
          }
        }

        renderable_manager.SetAxisAlignedBoundingBox(
            target_instance, {.center = combined_bounds.center(),
                              .halfExtent = combined_bounds.extent()});
      }
    }
  }

  if (gltf_asset_->GetModelData().GetStoredVertexData().empty()) return;
}

const PairedVector<mat4f, SampledJoint>& GltfRenderer::GetSampledTransforms(
    model::ModelData::SkinId skin_id,
    model::ModelData::EntityId entity_id) const {
  const auto& data = gltf_asset_->GetModelData();
  const auto& all_skinned_entities = data.Skins()[skin_id].skinned_entities;

  for (const RuntimeSkin& runtime_skin : runtime_skins_) {
    if (runtime_skin.skin != skin_id) continue;
    for (const RuntimeSkinnedEntity& runtime_skinned_entity :
         runtime_skin.skinned_entities) {
      if (all_skinned_entities[runtime_skinned_entity.skinned_entity].target !=
          entity_id)
        continue;
      return runtime_skinned_entity.sampled_xforms;
      break;
    }
    break;
  }

  IMP_LOG(imp::FATAL) << "Skin or Skinned Entity not found.";
  return runtime_skins_.front().skinned_entities.front().sampled_xforms;
}

float3 GltfRenderer::GetPivot() const {
  // TODO Use BoxCollider instead.
  Box root_relative_bounds = gltf_asset_->GetModelData().GetAxisAlignedBounds();
  auto bounds_min = root_relative_bounds.getMin();
  auto bounds_max = root_relative_bounds.getMax();
  float3 root_relative_offset = -GetModelRoot()->GetLocalPosition();

  return (root_relative_offset - bounds_min) / (bounds_max - bounds_min);
}

void GltfRenderer::SetPivot(float3 pivot) {
  // TODO Use BoxCollider instead.
  Box root_relative_bounds = gltf_asset_->GetModelData().GetAxisAlignedBounds();
  auto bounds_min = root_relative_bounds.getMin();
  auto bounds_max = root_relative_bounds.getMax();
  float3 root_relative_offset = bounds_min + pivot * (bounds_max - bounds_min);

  GetModelRoot()->SetLocalPosition(-root_relative_offset);

  Box node_relative_bounds;
  node_relative_bounds.set(bounds_min - root_relative_offset,
                           bounds_max - root_relative_offset);
  if (auto box_collider = GetNode()->GetComponent<BoxCollider>()) {
    box_collider->SetBox(node_relative_bounds);
  }
}

void GltfRenderer::SetMaterialsVariant(
    std::optional<model::ModelData::MaterialsVariantsId> materials_variant_id) {
  if (materials_variant_id.has_value() &&
      !gltf_asset_->GetModelData().MaterialsVariants().IsValid(
          materials_variant_id.value())) {
    IMP_LOG(imp::WARNING) << "Cannot set MaterialsVariant. Id "
                 << ToString(materials_variant_id.value()) << " doesn't exist.";
    return;
  }

  active_materials_variant_id_ = materials_variant_id;

  BaseRenderableManager& renderable_manager = GetView().GetRenderableManager();
  const TypedSetVector<EntityData>& entities =
      gltf_asset_->GetModelData().Entities();
  for (auto entity_id : entities.Ids<ModelData::EntityId>()) {
    const auto entity_data = entities[entity_id];
    const std::vector<model::ModelData::PartData>& parts = entity_data.parts;
    utils::Entity node_id = node_entities_[entity_id];

    for (size_t part_index = 0; part_index < parts.size(); part_index++) {
      renderable_manager.SetMaterialInstanceAt(
          renderable_manager.GetInstance(node_id), part_index,
          GetMaterial(entity_id, part_index)->GetFilamentMaterialInstance());
    }
  }
}

void GltfRenderer::SetMorphTargetWeights(const std::vector<float>& weights,
                                         EntityId entity_id) {
  BaseRenderableManager& renderable_manager = GetView().GetRenderableManager();
  utils::Entity node_id = node_entities_[entity_id];
  renderable_manager.SetMorphWeights(renderable_manager.GetInstance(node_id),
                                     weights.data(), weights.size(), 0);
}

size_t GltfRenderer::GetMorphTargetCount(EntityId entity_id) const {
  BaseRenderableManager& renderable_manager = GetView().GetRenderableManager();
  utils::Entity node_id = node_entities_[entity_id];
  return renderable_manager.GetMorphTargetCount(
      renderable_manager.GetInstance(node_id));
}

GltfState::ColliderMode GltfRenderer::GetColliderMode() const {
  return load_options_.collider_mode;
}

GltfState::MaterialSharingMode GltfRenderer::GetMaterialSharingMode() const {
  return load_options_.material_sharing_mode;
}

Future<absl::Status> GltfRenderer::System::SetupExtensionsForRenderer(
    ComponentHandle<GltfRenderer> renderer) {
  return extension_dependency_graph_.ParallelTraverse(
      [this, renderer](ComponentId component) {
        const ExtensionInfo& extension_info = extension_info_map_[component];
        if (extension_info.is_valid(renderer)) {
          return extension_info.creation_function(renderer);
        } else {
          return Future<absl::Status>(absl::OkStatus());
        }
      });
}

size_t GltfRenderer::System::GetExtensionCount() const {
  return extension_info_map_.size();
}

void GltfRenderer::System::AfterComponentAdded(GltfRenderer& renderer) {
  renderer.extension_setup_future_
      .Then([&renderer] {
        // Start all extensions once they're fully set up.
        for (ComponentHandle<GltfExtension> extension : renderer.extensions_) {
          absl::Status status = extension->Start();
          if (!status.ok()) {
            IMP_LOG(imp::ERROR) << "Failed to start GltfExtension: " << status.message();
          }
        }
      })
      .KeptBy(&renderer);
}

}  // namespace imp
