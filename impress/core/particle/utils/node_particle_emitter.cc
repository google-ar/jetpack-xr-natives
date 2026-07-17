/*
 * Copyright 2025 Google LLC
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

#include "core/particle/utils/node_particle_emitter.h"

#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/ncsb/component.h"
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

Future<OwnedParticleEmitterPtr> NodeParticleEmitter::Create(
    NodeHandle emitter_node, const ParticleEmitterState& emitter_state,
    std::unique_ptr<CustomParticleBehavior> custom_particle_behavior) {
  // Validate the emitter state before creating the emitter.
  std::string invalid_reason = ValidateEmitterState(emitter_state);
  if (!invalid_reason.empty()) {
    return Future<OwnedParticleEmitterPtr>(
        absl::InvalidArgumentError(invalid_reason));
  }

  // Preload the gltf asset to use when creating particles.
  return emitter_node->GetView()
      .GetAssetManager()
      .LoadGltfAsset(emitter_state.particle_config->gltf_asset.Value())
      .Then([&emitter_state, emitter_node,
             custom_particle_behavior = std::move(custom_particle_behavior)](
                AssetPtr<GltfAsset> gltf_asset) mutable
                -> OwnedParticleEmitterPtr {
        return OwnedParticleEmitterPtr(new NodeParticleEmitter(
            emitter_node, emitter_state, std::move(custom_particle_behavior),
            gltf_asset));
      });
}

NodeParticleEmitter::NodeParticleEmitter(
    NodeHandle emitter_node, const ParticleEmitterState& emitter_state,
    std::unique_ptr<CustomParticleBehavior> custom_particle_behavior,
    AssetPtr<GltfAsset> gltf_asset)
    : ParticleEmitter(emitter_node, emitter_state,
                      std::move(custom_particle_behavior)),
      gltf_asset_(gltf_asset) {}

NodeParticleEmitter::~NodeParticleEmitter() {
  // Destroy all active particles.
  for (const NodeParticle& particle : active_particles_) {
    if (particle.node.IsValid()) {
      particle.node->GetView().DestroyNode(particle.node);
    }
  }
  active_particles_.clear();
}

void NodeParticleEmitter::UpdateParticleSystem(const FrameTime& frame_time) {
  // Prepare the emitter info.
  imp::ParticleEmitterInfo emitter_info = GetParticleEmitterInfo();

  // Update behaviors specific to the emitter.
  UpdateEmitterBehavior(frame_time);

  // Process active particles.
  auto it = active_particles_.begin();
  while (it != active_particles_.end()) {
    // Get the particle instance.
    ParticleInstance particle_instance =
        particle_pool_.GetParticleInstance(it->particle_index);

    // Update particle behaviors.
    ParticleBehaviorResult result = particle_behavior_.UpdateParticle(
        emitter_info, frame_time.GetDeltaSeconds(), particle_instance);
    if (result == ParticleBehaviorResult::kExpired) {
      // Removes the node from the active particles list.
      it->node->GetView().DestroyNode(it->node);
      particle_pool_.DestroyParticle(it->particle_index);
      it = active_particles_.erase(it);
    } else {
      // Synchronize the node and advance the iterator.
      SyncNode(particle_instance, it->node);
      ++it;
    }
  }

  // Create new particles.
  // TODO: (broken link) - Revisit this while loop to prevent performance issues.
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

      // Create a new scene node.
      NodeHandle particle_node = emitter_node_->GetView().CreateNode();
      // TODO: (broken link) - Add test when this affects behavior.
      particle_node->SetParent(emitter_node_);
      particle_node->AddComponent<GltfRenderer>(gltf_asset_);

      // Add to the list of active particles.
      active_particles_.push_back(
          {.node = particle_node, .particle_index = particle_index});

      // Perform an initial sync for the new particle.
      SyncNode(particle_instance, particle_node);
    }
  }
}

void NodeParticleEmitter::SyncNode(const ParticleInstance& particle_instance,
                                   NodeHandle node) {
  // All particles have a position in the scene.
  node->SetWorldPosition(particle_instance.GetPosition());

  // Update the scale if defined.
  if (particle_instance.HasScale()) {
    node->SetWorldScale(particle_instance.GetScale());
  }

  // Update the rotation if defined.
  if (particle_instance.HasRotation()) {
    node->SetWorldRotation(particle_instance.GetRotation());
  }
}

std::string NodeParticleEmitter::ValidateEmitterState(
    const ParticleEmitterState& emitter_state) {
  // A particle config is required.
  if (!emitter_state.particle_config) {
    return "NodeParticleEmitter - particle_config not set!";
  }

  // A gltf asset reference is required.
  if (!emitter_state.particle_config->gltf_asset) {
    return "NodeParticleEmitter - gltf_asset not set!";
  }

  // Check the base class state also.
  return ParticleEmitter::ValidateEmitterState(emitter_state);
}

}  // namespace imp::imp_particle
