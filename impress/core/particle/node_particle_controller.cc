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

#include "core/particle/node_particle_controller.h"

#include <cstdint>
#include <list>
#include <string>

#include "absl/status/status.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/ncsb/component.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/particle/particle_behavior.h"
#include "core/particle/particle_controller.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/particle_instance.h"
#include "core/particle/particle_service.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/utils/frame_time.h"

namespace imp {

Future<OwnedParticleControllerPtr> NodeParticleController::Create(
    NodeHandle emitter_node, const ParticleEmitterState& emitter_state) {
  // Validate the emitter state before creating the controller.
  std::string invalid_reason = ValidateEmitterState(emitter_state);
  if (!invalid_reason.empty()) {
    return Future<OwnedParticleControllerPtr>(
        absl::InvalidArgumentError(invalid_reason));
  }

  // Preload the gltf asset to use when creating particles.
  return emitter_node->GetView()
      .GetAssetManager()
      .LoadGltfAsset(emitter_state.particle_config.gltf_asset.value())
      .Then([&emitter_state, emitter_node](
                AssetPtr<GltfAsset> gltf_asset) -> OwnedParticleControllerPtr {
        return OwnedParticleControllerPtr(new NodeParticleController(
            emitter_node, gltf_asset, emitter_state));
      });
}

NodeParticleController::NodeParticleController(
    NodeHandle emitter_node, AssetPtr<GltfAsset> gltf_asset,
    const ParticleEmitterState& emitter_state)
    : particle_service_(emitter_state.particle_config,
                        emitter_state.max_particles),
      particle_behavior_(emitter_node, emitter_state.particle_config),
      emitter_node_(emitter_node),
      gltf_asset_(gltf_asset) {
  particles_per_second_ = emitter_state.particles_per_second;
  particle_delay_ = 0.0f;
}

void NodeParticleController::UpdateParticleSystem(const FrameTime& frame_time) {
  // Process active particles.
  auto it = active_particles_.begin();
  while (it != active_particles_.end()) {
    // Get the particle instance.
    ParticleInstance particle_instance =
        particle_service_.GetParticleInstance(it->particle_index);

    // Update particle behaviors.
    ParticleBehavior::UpdateResult result = particle_behavior_.UpdateParticle(
        frame_time.GetDeltaSeconds(), particle_instance);
    if (result == ParticleBehavior::UpdateResult::kExpired) {
      // Removes the node from the active particles list.
      it->node->GetView().DestroyNode(it->node);
      particle_service_.DestroyParticle(it->particle_index);
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
  while (particle_delay_ <= 0.0f &&
         active_particles_.size() < particle_service_.GetMaxParticles()) {
    // Determine when the next particle will be emitted.
    particle_delay_ += 1.0f / particles_per_second_;

    // Get an available particle instance record.
    int32_t particle_index = particle_service_.CreateParticle();
    if (particle_index != kInvalidParticleIndex) {
      // Set the default state for the particle.
      ParticleInstance particle_instance =
          particle_service_.GetParticleInstance(particle_index);
      particle_behavior_.SetDefaultValues(particle_instance);

      // Create a new scene node.
      NodeHandle particle_node = emitter_node_->GetView().CreateNode();
      particle_node->AddComponent<GltfRenderer>(gltf_asset_);

      // Add to the list of active particles.
      active_particles_.push_back(
          {.node = particle_node, .particle_index = particle_index});

      // Perform an initial sync for the new particle.
      SyncNode(particle_instance, particle_node);
    }
  }
}

void NodeParticleController::SyncNode(const ParticleInstance& particle_instance,
                                      NodeHandle node) {
  // All particles have a position in the scene.
  node->SetWorldPosition(particle_instance.GetPosition());

  // Update the scale if defined.
  if (particle_instance.HasScale()) {
    node->SetWorldScale(particle_instance.GetScale());
  }
}

std::string NodeParticleController::ValidateEmitterState(
    const ParticleEmitterState& emitter_state) {
  // A gltf asset reference is required.
  if (!emitter_state.particle_config.gltf_asset.has_value()) {
    return "NodeParticleController - gltf_asset not set!";
  }

  // Particles must be emitted.
  if (emitter_state.particles_per_second <= 0.0f) {
    return "NodeParticleController - particles per second must be > 0!";
  }

  // There must be an allowance for the maximum number of particles.
  if (emitter_state.max_particles <= 0) {
    return "NodeParticleController - max particles must be > 0!";
  }

  return "";
}

}  // namespace imp
