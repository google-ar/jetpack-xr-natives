/*
 * Copyright 2026 Google LLC
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

#include "core/particle/utils/instanced_particle_emitter.h"

#include <algorithm>
#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/gltf/gltf_asset.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/particle/custom_particle_behavior.h"
#include "core/particle/particle_behavior_result.h"
#include "core/particle/particle_emitter_info.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/particle_instance.h"
#include "core/particle/utils/particle_behavior.h"
#include "core/particle/utils/particle_emitter.h"
#include "core/particle/utils/particle_pool.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/utils/frame_time.h"

namespace imp::imp_particle {

constexpr absl::string_view kParticlesRootNodeName = "ParticlesRootNode";
constexpr absl::string_view kParticleBlockNodeName = "ParticleInstancesNode";

Future<OwnedParticleEmitterPtr> InstancedParticleEmitter::Create(
    NodeHandle emitter_node, const ParticleEmitterState& emitter_state,
    std::unique_ptr<CustomParticleBehavior> custom_particle_behavior) {
  // Validate the emitter state before creating the emitter.
  std::string invalid_reason = ValidateEmitterState(emitter_state);
  if (!invalid_reason.empty()) {
    return Future<OwnedParticleEmitterPtr>(
        absl::InvalidArgumentError(invalid_reason));
  }

  int32_t max_particles = emitter_state.emitter_config->max_particles.Value();
  GltfAsset::LoadOptions options =
      emitter_node->GetView().GetAssetManager().GetDefaultLoadOptions();
  options.instance_transforms.resize(max_particles, kIdentityMat4f);

  Future<AssetPtr<GltfAsset>> gltf_future =
      emitter_node->GetView().GetAssetManager().LoadGltfAsset(
          emitter_state.particle_config->gltf_asset.Value(), options);
  Future<OwnedMaterialPtr> material_future =
      ParticleEmitter::GetMaterialFuture(emitter_state, emitter_node);

  return gltf_future.Merge(material_future)
      .Then([&emitter_state, emitter_node,
             custom_particle_behavior = std::move(custom_particle_behavior)](
                std::tuple<AssetPtr<GltfAsset>, OwnedMaterialPtr>
                    results) mutable -> OwnedParticleEmitterPtr {
        auto [gltf_asset, material_instance] = std::move(results);
        return OwnedParticleEmitterPtr(new InstancedParticleEmitter(
            emitter_node, emitter_state, std::move(custom_particle_behavior),
            gltf_asset, std::move(material_instance)));
      });
}

InstancedParticleEmitter::InstancedParticleEmitter(
    NodeHandle emitter_node, const ParticleEmitterState& emitter_state,
    std::unique_ptr<CustomParticleBehavior> custom_particle_behavior,
    AssetPtr<GltfAsset> gltf_asset, OwnedMaterialPtr material_instance)
    : ParticleEmitter(emitter_node, emitter_state,
                      std::move(custom_particle_behavior), gltf_asset,
                      std::move(material_instance)) {
  // Create the root node for all particles.
  particle_root_node_ = emitter_node->GetView().CreateNode();
  particle_root_node_->SetName(kParticlesRootNodeName);
  // TODO: (broken link) - Add test when this affects behavior.
  particle_root_node_->SetParent(emitter_node_);

  // Determine the engine limit for hybrid instancing.
  int32_t max_automatic_instances =
      BaseView::GetSharedEngine()->getMaxAutomaticInstances();

  // Resize the instance transforms buffer to the maximum number of particles.
  instance_transforms_.resize(max_automatic_instances);

  int32_t max_particles = emitter_state.emitter_config->max_particles.Value();

  // Divide particles into blocks that fit within the engine's limit.
  for (int32_t i = 0; i < max_particles; i += max_automatic_instances) {
    int32_t block_size = std::min(max_automatic_instances, max_particles - i);

    NodeHandle block_node = emitter_node->GetView().CreateNode();
    block_node->SetName(kParticleBlockNodeName);
    block_node->SetParent(particle_root_node_);

    // Configure the GltfRenderer for this instance.
    GltfAsset::LoadOptions options =
        emitter_node->GetView().GetAssetManager().GetDefaultLoadOptions();
    options.instance_transforms.resize(block_size, kIdentityMat4f);

    auto renderer = block_node->AddComponent<GltfRenderer>(gltf_asset, options);
    if (material_instance_ != nullptr) {
      renderer->SetMaterialOverrideByIndex(material_instance_.Borrow(), 0);
    }
    block_nodes_.push_back(block_node);
    block_sizes_.push_back(block_size);
  }
}

void InstancedParticleEmitter::UpdateParticleSystem(
    const FrameTime& frame_time) {
  // Prepare the emitter info.
  imp::ParticleEmitterInfo emitter_info = GetParticleEmitterInfo();

  // Update behaviors specific to the emitter.
  UpdateEmitterBehavior(frame_time);

  // Process active particles.
  auto it = active_particles_.begin();
  while (it != active_particles_.end()) {
    // Get the particle instance.
    ParticleInstance particle_instance =
        particle_pool_.GetParticleInstance(*it);

    // Update particle behaviors.
    ParticleBehaviorResult result = particle_behavior_.UpdateParticle(
        emitter_info, frame_time.GetDeltaSeconds(), particle_instance);
    if (result == ParticleBehaviorResult::kExpired) {
      particle_pool_.DestroyParticle(*it);
      it = active_particles_.erase(it);
    } else {
      ++it;
    }
  }

  // Create new particles.
  particle_delay_ -= frame_time.GetDeltaSeconds();
  while (CanEmitParticles()) {
    // Determine when the next particle will be emitted.
    particle_delay_ += 1.0f / particles_per_second_;

    // Get an available particle instance record.
    int32_t particle_index = particle_pool_.CreateParticle();
    if (particle_index != kInvalidParticleIndex) {
      // Set the default state for the particle.
      ParticleInstance particle_instance =
          particle_pool_.GetParticleInstance(particle_index);
      particle_behavior_.SetDefaultValues(particle_instance);

      // Add to the list of active particles.
      active_particles_.push_back(particle_index);
    }
  }

  // Prepare the transform buffer for rendering.
  PrepareRendering();

  next_debug_time_ -= frame_time.GetDeltaSeconds();
  if (next_debug_time_ < 0.0f) {
    IMP_LOG(imp::INFO) << "Max active particles: " << max_active_particles_;
    next_debug_time_ = 1.0f;
    max_active_particles_ = 0;
  }

  if (active_particles_.size() > max_active_particles_) {
    max_active_particles_ = active_particles_.size();
  }
}

void InstancedParticleEmitter::PrepareRendering() {
  if (!particle_root_node_.IsValid()) return;

  // Determine the engine limit for automatic instancing.
  int32_t max_automatic_instances = 64;  // Default fallback.
  if (auto* engine = BaseView::GetSharedEngine()) {
    max_automatic_instances = engine->getMaxAutomaticInstances();
  }

  // Use the parent's world TRs if particle_root_node_ is new, as it might be
  // stale. Since particle_root_node_ is a child with identity local TRs, its
  // world TRs should theoretically match the parent's TRs after update.
  mat4f root_world_trs = particle_root_node_->GetWorldTrs();
  if (root_world_trs == kIdentityMat4f && emitter_node_.IsValid()) {
    root_world_trs = emitter_node_->GetWorldTrs();
  }

  // If still identity, we might be in the very first frame.
  // Use identity for now, but particles will be offset.
  mat4f inv_root_transform = inverse(root_world_trs);

  auto it = active_particles_.begin();

  mat4f zero_scale_mat = mat4f::scaling(0.0f);

  for (int32_t block_idx = 0; block_idx < block_nodes_.size(); ++block_idx) {
    NodeHandle block_node = block_nodes_[block_idx];
    ComponentHandle<GltfRenderer> renderer =
        block_node->GetComponent<GltfRenderer>();
    if (!renderer.IsValid()) continue;

    // Get the size of this specific instance.
    int32_t block_size = block_sizes_[block_idx];
    instance_transforms_.resize(block_size);

    int32_t i = 0;
    for (; i < block_size && it != active_particles_.end(); ++i, ++it) {
      ParticleInstance particle_instance =
          particle_pool_.GetParticleInstance(*it);

      float3 pos = particle_instance.GetPosition();
      quatf rot = particle_instance.HasRotation()
                      ? particle_instance.GetRotation()
                      : kIdentityQuatf;
      float3 scale =
          particle_instance.HasScale() ? particle_instance.GetScale() : kOne3;

      mat4f world_transform = Transform<float>(pos, rot, scale).AsMat4();
      instance_transforms_[i] = inv_root_transform * world_transform;
    }

    // Zero out all other transform sin the block.
    for (; i < block_size; ++i) {
      instance_transforms_[i] = zero_scale_mat;
    }

    renderer->UpdateInstanceTransforms(instance_transforms_);
  }
}

InstancedParticleEmitter::~InstancedParticleEmitter() {
  // Destroy all block nodes.
  for (NodeHandle block_node : block_nodes_) {
    if (block_node.IsValid()) {
      block_node->GetView().DestroyNode(block_node);
    }
  }
  block_nodes_.clear();

  // Destroy the particle root node.
  if (particle_root_node_.IsValid()) {
    particle_root_node_->GetView().DestroyNode(particle_root_node_);
  }
  active_particles_.clear();
}

std::string InstancedParticleEmitter::ValidateEmitterState(
    const ParticleEmitterState& emitter_state) {
  return ParticleEmitter::ValidateEmitterState(emitter_state);
}

}  // namespace imp::imp_particle
