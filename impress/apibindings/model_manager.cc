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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "apibindings/asset_ptr_map.h"
#include "apibindings/base_asset_animator.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/bindings_material.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/model_interaction_ux/scene_viewer_component.h"
#include "core/animation/gltf_animation.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/gltf/gltf_asset.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/box.h"
#include "core/materials/material.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/animation/gltf_animator.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_state.proto.imp.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// The default speed multiplier for the initial playback.
constexpr float kInitialSpeedMultiplier = 1.0f;
// The default channel id for old animation APIs.
constexpr int32_t kDefaultChannelId = 0;

namespace {

class ModelManagerImpl : public ModelManager {
 public:
  explicit ModelManagerImpl(ImpressApiView& view);
  ~ModelManagerImpl() override = default;

  void LoadGltfAsset(absl::string_view path,
                     std::unique_ptr<BaseAssetLoader> asset_loader) override;
  void LoadGltfAsset(absl::Cord data, absl::string_view key,
                     std::unique_ptr<BaseAssetLoader> asset_loader) override;
  absl::Status ReleaseGltfAsset(std::intptr_t gltf_token) override;
  absl::StatusOr<int32_t> InstanceGltfModel(std::intptr_t gltf_token) override;
  absl::Status SetGltfModelColliderEnabled(int32_t node,
                                           bool enable_collider) override;
  absl::Status SetGltfReformAffordanceEnabled(int32_t impress_node,
                                              bool enable_affordance,
                                              bool system_movable) override;
  void AnimateGltfModel(
      int32_t node, absl::string_view animation_name, bool loop, float speed,
      float start_time, int32_t channel_id,
      std::unique_ptr<BaseAssetAnimator> asset_animator) override;
  absl::Status StopGltfModelAnimation(int32_t node,
                                      int32_t channel_id) override;
  absl::Status ToggleGltfModelAnimation(int32_t node, bool toggle,
                                        int32_t channel_id) override;
  absl::Status SetGltfModelAnimationSpeed(int32_t node, float speed,
                                          int32_t channel_id) override;
  absl::Status SetGltfModelAnimationPlaybackTime(int32_t node,
                                                 float playback_time,
                                                 int32_t channel_id) override;
  absl::StatusOr<int32_t> GetGltfModelAnimationCount(int32_t node) override;
  absl::StatusOr<std::string> GetGltfModelAnimationName(int32_t node,
                                                        int32_t index) override;
  absl::StatusOr<float> GetGltfModelAnimationDurationSeconds(
      int32_t node, int32_t index) override;
  absl::StatusOr<imp::Box> GetGltfModelLocalBounds(int32_t node) override;
  absl::Status SetGltfModelNodeMaterialOverride(
      int32_t node_id, std::intptr_t material, size_t primitive_index) override;
  absl::Status ClearGltfModelNodeMaterialOverride(
      int32_t node_id, size_t primitive_index) override;
  absl::Status ScheduleReskinning(int32_t node_id) override;
  void Update(const FrameTime& frame_time) override;
  void ResetAnimationContexts() override;

 private:
  // Returns a glTF mesh component for a subnode of an instance of a glTF asset
  // in the Jepack XR scene.
  absl::StatusOr<ComponentHandle<GltfMesh>> FindGltfMeshByNodeName(
      int32_t node_id, absl::string_view node_name);

  ImpressApiView& view_;
  // Map from node entity ID to a tuple of GltfAnimator component and a map
  // of channel ID to asset animator callbacks. This is used to play multiple
  // animations on the same node at the same time on different channels.
  absl::flat_hash_map<
      int32_t, std::tuple<ComponentHandle<GltfAnimator>,
                          absl::flat_hash_map<
                              int32_t, std::unique_ptr<BaseAssetAnimator>>>>
      node_to_anim_ctx_;
};

}  // namespace

ModelManagerImpl::ModelManagerImpl(ImpressApiView& view) : view_(view) {}

void ModelManagerImpl::LoadGltfAsset(
    absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader) {
  view_.GetAssetPtrMap().LoadGltfAsset(path, std::move(asset_loader));
}

void ModelManagerImpl::LoadGltfAsset(
    absl::Cord data, absl::string_view key,
    std::unique_ptr<BaseAssetLoader> asset_loader) {
  view_.GetAssetPtrMap().LoadGltfAsset(data, key, std::move(asset_loader));
}

absl::Status ModelManagerImpl::ReleaseGltfAsset(std::intptr_t gltf_token) {
  return view_.GetAssetPtrMap().ReleaseGltfAsset(gltf_token);
}

absl::StatusOr<int32_t> ModelManagerImpl::InstanceGltfModel(
    std::intptr_t gltf_token) {
  absl::StatusOr<AssetPtr<GltfAsset>> gltf_asset_ptr =
      view_.GetAssetPtrMap().GetStoredGltfAsset(gltf_token);

  if (!gltf_asset_ptr.ok()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Gltf asset is not cached: %s.", gltf_asset_ptr.status().message()));
  }
  NodeHandle node = view_.CreateNode();
  auto load_options =
      GltfAsset::LoadOptions({.collider_mode = GltfState::ColliderMode::NONE});
  node->AddComponent<GltfRenderer>(gltf_asset_ptr.value(), load_options);
  return node.GetEntity().getId();
}

absl::Status ModelManagerImpl::SetGltfModelColliderEnabled(
    int32_t node, bool enable_collider) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  std::vector<ComponentHandle<GltfMesh>> gltf_meshes =
      view_.GetPathManager().GetComponentsInDescendantsOrSelf<GltfMesh>(
          node_handle);

  for (ComponentHandle<GltfMesh> gltf_mesh : gltf_meshes) {
    if (enable_collider) {
      gltf_mesh->GetNode()->AddComponent<GltfCollider>(
          gltf_mesh, imp::GltfCollider::CollisionMode::kTriangles);
    } else {
      gltf_mesh->GetNode()->RemoveComponent<GltfCollider>();
    }
  }

  return absl::OkStatus();
}

absl::Status ModelManagerImpl::SetGltfReformAffordanceEnabled(
    int32_t impress_node, bool enable_affordance, bool system_movable) {
  NodeHandle node_handle(utils::Entity::import(impress_node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  if (enable_affordance) {
    return node_handle
        ->AddComponent<SceneViewerComponent>(node_handle, system_movable)
        .status();
  } else {
    node_handle->RemoveComponent<SceneViewerComponent>();
    return absl::OkStatus();
  }
}

void ModelManagerImpl::AnimateGltfModel(
    int32_t node, absl::string_view animation_name, bool loop, float speed,
    float start_time, int32_t channel_id,
    std::unique_ptr<BaseAssetAnimator> asset_animator) {
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
  gltf_animation.options.start_time_seconds = start_time;
  gltf_animation.options.playback_channel = {.id = channel_id};
  // Use 1.0 as the default speed for the initial playback.
  // The speed will be updated once playback begins, as Impress requires
  // the animation to be in the playing state before speed is set.
  gltf_animation.options.speed_multiplier = kInitialSpeedMultiplier;

  absl::Status can_play = gltf_animator->CanPlay(gltf_animation);
  if (!can_play.ok()) {
    IMP_LOG(imp::ERROR) << "Cannot play animation: " << can_play.message();
    asset_animator->OnFailure(
        absl::StrFormat("Cannot play animation: %s", can_play.message()));
    return;
  }

  gltf_animator->Play(gltf_animation);
  gltf_animator->SetSpeedMultiplier(speed,
                                    gltf_animation.options.playback_channel);
  auto& [animator, channel_map] = node_to_anim_ctx_[node];
  animator = gltf_animator;
  channel_map[channel_id] = std::move(asset_animator);
}

absl::Status ModelManagerImpl::StopGltfModelAnimation(int32_t node,
                                                      int32_t channel_id) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  ComponentHandle<GltfAnimator> animator =
      node_handle->GetComponent<GltfAnimator>();
  if (!animator) {
    return absl::NotFoundError("Animation is not playing.");
  }

  animator->Stop({.id = channel_id});

  // Optionally we could call the callback here, but this method implies
  // that the animation has been "cancelled,"  rather than completing.
  auto it = node_to_anim_ctx_.find(node);
  if (it != node_to_anim_ctx_.end()) {
    auto& [animator_handle, channel_map] = it->second;
    channel_map.erase(channel_id);
    if (channel_map.empty()) {
      node_to_anim_ctx_.erase(it);
    }
  }

  return absl::OkStatus();
}

absl::Status ModelManagerImpl::ToggleGltfModelAnimation(int32_t node,
                                                        bool toggle,
                                                        int32_t channel_id) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  ComponentHandle<GltfAnimator> animator =
      node_handle->GetComponent<GltfAnimator>();
  if (!animator) {
    return absl::NotFoundError("Animation is not playing.");
  }

  animator->SetPaused(!toggle, {.id = channel_id});
  return absl::OkStatus();
}

absl::Status ModelManagerImpl::SetGltfModelAnimationSpeed(int32_t node,
                                                          float speed,
                                                          int32_t channel_id) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  ComponentHandle<GltfAnimator> animator =
      node_handle->GetComponent<GltfAnimator>();
  if (!animator) {
    return absl::NotFoundError(
        "Node does not have an active GltfAnimator component.");
  }

  animator->SetSpeedMultiplier(speed, {.id = channel_id});
  return absl::OkStatus();
}

absl::Status ModelManagerImpl::SetGltfModelAnimationPlaybackTime(
    int32_t node, float playback_time, int32_t channel_id) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  ComponentHandle<GltfAnimator> animator =
      node_handle->GetComponent<GltfAnimator>();
  if (!animator) {
    return absl::NotFoundError(
        "Node does not have an active GltfAnimator component.");
  }

  animator->SetPlaybackTime(absl::Seconds(playback_time), {.id = channel_id});
  return absl::OkStatus();
}

absl::StatusOr<int32_t> ModelManagerImpl::GetGltfModelAnimationCount(
    int32_t node) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  ComponentHandle<GltfRenderer> gltf_renderer =
      node_handle->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InvalidArgumentError("Node does not have a GltfRenderer.");
  }

  return gltf_renderer->GetGltfAsset()->AnimationCount();
}

absl::StatusOr<std::string> ModelManagerImpl::GetGltfModelAnimationName(
    int32_t node, int32_t index) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  ComponentHandle<GltfRenderer> gltf_renderer =
      node_handle->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InvalidArgumentError("Node does not have a GltfRenderer.");
  }

  auto anim_names = gltf_renderer->GetGltfAsset()->GetAnimNames();
  if (index < 0 || index >= anim_names.size()) {
    return absl::OutOfRangeError("Animation index is out of range.");
  }

  return anim_names[index];
}

absl::StatusOr<float> ModelManagerImpl::GetGltfModelAnimationDurationSeconds(
    int32_t node, int32_t index) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  ComponentHandle<GltfRenderer> gltf_renderer =
      node_handle->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InvalidArgumentError("Node does not have a GltfRenderer.");
  }

  AssetPtr<GltfAsset> asset = gltf_renderer->GetGltfAsset();
  if (index < 0 || index >= asset->AnimationCount()) {
    return absl::OutOfRangeError("Animation index is out of range.");
  }

  const animation::GltfAnimation* anim =
      asset->GetGltfAnimData(GltfAsset::AnimId::At(index));
  if (anim == nullptr) {
    return absl::NotFoundError("Animation data not found.");
  }

  return static_cast<float>(absl::ToDoubleSeconds(anim->Duration()));
}

absl::StatusOr<imp::Box> ModelManagerImpl::GetGltfModelLocalBounds(
    int32_t node) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  ComponentHandle<GltfRenderer> gltf_renderer =
      node_handle->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InvalidArgumentError("Node does not have a GltfRenderer.");
  }

  // TODO: (broken link) - A policy to handle the NaN or negative
  // center / halfExtents of a glTF model.
  imp::Box bounds = gltf_renderer->GetLocalBounds();
  if (std::isnan(bounds.center.x) || std::isnan(bounds.center.y) ||
      std::isnan(bounds.center.z)) {
    bounds.center = {0.0f, 0.0f, 0.0f};
  }
  if (std::isnan(bounds.halfExtent.x) || bounds.halfExtent.x < 0.0f ||
      std::isnan(bounds.halfExtent.y) || bounds.halfExtent.y < 0.0f ||
      std::isnan(bounds.halfExtent.z) || bounds.halfExtent.z < 0.0f) {
    bounds.halfExtent = {0.0f, 0.0f, 0.0f};
  }
  return bounds;
}

absl::Status ModelManagerImpl::SetGltfModelNodeMaterialOverride(
    int32_t node_id, std::intptr_t material, size_t primitive_index) {
  BindingsMaterial* bindings_material =
      view_.FromJava<BindingsMaterial>(material);
  if (!bindings_material) {
    return absl::InvalidArgumentError("Provided material handle is not valid.");
  }
  NodeHandle node_handle(utils::Entity::import(node_id));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }
  ComponentHandle<GltfMesh> mesh = node_handle->GetComponent<GltfMesh>();
  if (!mesh) {
    return absl::InvalidArgumentError(
        "The targeted node does not have a GltfMesh component.");
  }
  mesh->SetMaterialOverride(
      bindings_material->GetMaterial(SmallSourceLocation::Current()),
      primitive_index);
  return absl::OkStatus();
}

absl::Status ModelManagerImpl::ClearGltfModelNodeMaterialOverride(
    int32_t node_id, size_t primitive_index) {
  NodeHandle node_handle(utils::Entity::import(node_id));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }
  ComponentHandle<GltfMesh> mesh = node_handle->GetComponent<GltfMesh>();
  if (!mesh) {
    return absl::InvalidArgumentError(
        "The targeted node does not have a GltfMesh component.");
  }
  mesh->SetMaterialOverride(OwnedMaterialPtr{}, primitive_index);
  return absl::OkStatus();
}

absl::Status ModelManagerImpl::ScheduleReskinning(int32_t node_id) {
  NodeHandle node_handle(utils::Entity::import(node_id));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }
  ComponentHandle<GltfRenderer> gltf_renderer =
      node_handle->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::FailedPreconditionError("Node does not have a GltfRenderer.");
  }
  gltf_renderer->ScheduleSkinningUpdate();
  return absl::OkStatus();
}

void ModelManagerImpl::Update(const FrameTime& frame_time) {
  // TODO: (broken link) - Hook into the Impress Animation Event system and drive
  //                     the callback dispatch from there, instead of polling on
  //                     Update.
  for (auto it = node_to_anim_ctx_.begin(); it != node_to_anim_ctx_.end();) {
    auto& [animator, channel_map] = it->second;
    if (!animator) {
      node_to_anim_ctx_.erase(it++);
      continue;
    }

    for (auto channel_it = channel_map.begin();
         channel_it != channel_map.end();) {
      GltfAnimator::PlaybackChannelId channel_id;
      channel_id.id = channel_it->first;
      if (!animator->IsPlaying(channel_id)) {
        channel_it->second->OnComplete();
        if (!animator) {
          channel_map.clear();
          break;
        }
        channel_map.erase(channel_it++);
      } else {
        ++channel_it;
      }
    }

    if (channel_map.empty()) {
      node_to_anim_ctx_.erase(it++);
    } else {
      ++it;
    }
  }
}

void ModelManagerImpl::ResetAnimationContexts() {
  // Clear animation contexts first.
  node_to_anim_ctx_.clear();
}

std::unique_ptr<ModelManager> CreateModelManager(ImpressApiView& view) {
  return std::make_unique<ModelManagerImpl>(view);
}

}  // namespace imp
