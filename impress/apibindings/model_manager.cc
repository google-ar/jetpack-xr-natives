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

#include "apibindings/model_manager.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "apibindings/asset_animator.h"
#include "apibindings/asset_loader.h"
#include "apibindings/asset_ptr_map.h"
#include "apibindings/bindings_material.h"
#include "apibindings/impress_api_view.h"
#include "core/assets/asset_ptr.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/box.h"
#include "core/materials/material.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/animation/gltf_animator.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_state.proto.imp.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

ModelManager::ModelManager(ImpressApiView& view) : view_(view) {}

void ModelManager::LoadGltfAsset(absl::string_view path,
                                 std::unique_ptr<AssetLoader> asset_loader) {
  view_.GetAssetPtrMap().LoadGltfAsset(path, std::move(asset_loader));
}

void ModelManager::LoadGltfAsset(absl::Cord data, absl::string_view key,
                                 std::unique_ptr<AssetLoader> asset_loader) {
  view_.GetAssetPtrMap().LoadGltfAsset(data, key, std::move(asset_loader));
}

absl::Status ModelManager::ReleaseGltfAsset(std::intptr_t gltf_token) {
  return view_.GetAssetPtrMap().ReleaseGltfAsset(gltf_token);
}

absl::StatusOr<int32_t> ModelManager::InstanceGltfModel(
    std::intptr_t gltf_token, bool enable_collider) {
  absl::StatusOr<AssetPtr<GltfAsset>> gltf_asset_ptr =
      view_.GetAssetPtrMap().GetStoredGltfAsset(gltf_token);

  if (!gltf_asset_ptr.ok()) {
    return absl::InvalidArgumentError("Gltf asset is not cached.");
  }

  NodeHandle node = view_.CreateNode();

  if (!enable_collider) {
    auto render_options = GltfAsset::LoadOptions(
        {.collider_mode = GltfState::ColliderMode::NONE});
    node->AddComponent<GltfRenderer>(gltf_asset_ptr.value(), render_options);
  } else {
    node->AddComponent<GltfRenderer>(gltf_asset_ptr.value());
  }

  return node.GetEntity().getId();
}

absl::Status ModelManager::SetGltfModelColliderEnabled(int32_t node,
                                                       bool enable_collider) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  std::vector<ComponentHandle<GltfMesh>> gltf_meshes =
      view_.GetPathManager().GetComponentsInDescendantsOrSelf<GltfMesh>(
          node_handle);

  for (ComponentHandle<GltfMesh> gltf_mesh : gltf_meshes) {
    if (enable_collider) {
      gltf_mesh->GetNode()->AddComponent<GltfCollider>(gltf_mesh);
    } else {
      gltf_mesh->GetNode()->RemoveComponent<GltfCollider>();
    }
  }

  return absl::OkStatus();
}

void ModelManager::AnimateGltfModel(
    int32_t node, absl::string_view animation_name, bool loop,
    std::unique_ptr<AssetAnimator> asset_animator) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    asset_animator->OnFailure("Node is not valid.");
    return;
  }
  ComponentHandle<GltfAnimator> gltf_animator =
      node_handle->GetOrAddComponent<GltfAnimator>();

  GltfAnimator::PlayCommand gltf_animation;
  // If no animation_name was supplied, default to first available animation.
  if (!animation_name.empty()) {
    gltf_animation.animation = std::string(animation_name);
  }
  gltf_animation.options.looping = loop;

  absl::Status can_play = gltf_animator->CanPlay(gltf_animation);
  if (!can_play.ok()) {
    IMP_LOG(imp::ERROR) << "Cannot play animation: " << can_play.message();
    asset_animator->OnFailure("Cannot play animation.");
    return;
  }
  gltf_animator->Play(gltf_animation);
  node_to_anim_ctx_[node] = std::make_tuple(
      gltf_animator,
      std::optional<std::unique_ptr<AssetAnimator>>(std::move(asset_animator)));
}

absl::Status ModelManager::StopGltfModelAnimation(int32_t node) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }
  auto it = node_to_anim_ctx_.find(node);
  if (it != node_to_anim_ctx_.end()) {
    auto& [animator, callback] = node_to_anim_ctx_[node];
    if (animator) {
      // We technically can avoid checking validity here because we don't
      // support attaching and detaching the animation component from the
      // application side, but keeping for correctness.
      animator->Stop();
    }
    // Optionally we could call the callback here, but this method implies
    // that the animation has been "cancelled,"  rather than completing.
    node_to_anim_ctx_.erase(it);
    return absl::OkStatus();
  }
  return absl::NotFoundError("Animation is not playing.");
}

absl::StatusOr<imp::Box> ModelManager::GetGltfModelLocalBounds(int32_t node) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  ComponentHandle<GltfRenderer> gltf_renderer =
      node_handle->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InvalidArgumentError("Node does not have a GltfRenderer.");
  }

  return gltf_renderer->GetLocalBounds();
}

absl::Status ModelManager::SetMaterialOverride(int32_t node_id,
                                               std::intptr_t material,
                                               absl::string_view node_name,
                                               size_t primitive_index) {
  BindingsMaterial* bindings_material =
      view_.FromJava<BindingsMaterial>(material);
  if (!bindings_material) {
    return absl::InvalidArgumentError("Provided material handle is not valid.");
  }

  MP_ASSIGN_OR_RETURN(ComponentHandle<GltfMesh> mesh,
                   FindGltfMeshByNodeName(node_id, node_name));

  mesh->SetMaterialOverride(
      bindings_material->GetMaterial(SmallSourceLocation::Current()),
      primitive_index);
  return absl::OkStatus();
}

absl::Status ModelManager::ClearMaterialOverride(int32_t node_id,
                                                 absl::string_view node_name,
                                                 size_t primitive_index) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<GltfMesh> mesh,
                   FindGltfMeshByNodeName(node_id, node_name));
  // Clears the material override for that mesh.
  mesh->SetMaterialOverride(OwnedMaterialPtr{}, primitive_index);
  return absl::OkStatus();
}

void ModelManager::Update(const FrameTime& frame_time) {
  // TODO: (broken link) - Hook into the Impress Animation Event system and drive
  //                     the callback dispatch from there, instead of polling on
  //                     Update.
  for (auto it = node_to_anim_ctx_.begin(); it != node_to_anim_ctx_.end();) {
    auto& [animator, callback] = it->second;
    if (animator && !animator->IsPlaying() && callback.has_value()) {
      callback.value()->OnComplete();
      node_to_anim_ctx_.erase(it++);
    } else {
      ++it;
    }
  }
}

void ModelManager::DisposeGltfAssetsAndInstances() {
  // Clear animation contexts first.
  node_to_anim_ctx_.clear();
  // Now destroy all glTF assets and their instances.
  view_.GetAssetPtrMap().DestroyGltfAssetsAndInstances();
}

absl::StatusOr<ComponentHandle<GltfMesh>> ModelManager::FindGltfMeshByNodeName(
    int32_t node_id, absl::string_view node_name) {
  NodeHandle model_node(utils::Entity::import(node_id));
  if (!model_node) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  NodeHandle mesh_node = model_node->FindByName(node_name);
  if (!mesh_node) {
    return absl::InvalidArgumentError(
        absl::StrFormat("No Gltf child node named %s.", node_name));
  }

  ComponentHandle<GltfMesh> mesh = mesh_node->GetComponent<GltfMesh>();
  if (!mesh) {
    return absl::InvalidArgumentError("Child doesn't have a mesh.");
  }
  return mesh;
}

}  // namespace imp
