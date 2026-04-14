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
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_manager.h"
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
        return OwnedParticleEmitterPtr(
            new NodeParticleEmitter(emitter_node, gltf_asset, emitter_state,
                                    std::move(custom_particle_behavior)));
      });
}

NodeParticleEmitter::NodeParticleEmitter(
    NodeHandle emitter_node, AssetPtr<GltfAsset> gltf_asset,
    const ParticleEmitterState& emitter_state,
    std::unique_ptr<CustomParticleBehavior> custom_particle_behavior)
    : particle_pool_(emitter_state.particle_config.Value(),
                     emitter_state.emitter_config->max_particles.Value(),
                     emitter_node),
      particle_behavior_(emitter_node, emitter_state.particle_config.Value(),
                         std::move(custom_particle_behavior)),
      emitter_node_(emitter_node),
      gltf_asset_(gltf_asset),
      emitter_duration_finite_(
          emitter_state.emitter_config->duration_in_seconds.Value() > 0.0f),
      remaining_emitter_duration_(
          emitter_state.emitter_config->duration_in_seconds.Value()),
      emitter_duration_(
          emitter_state.emitter_config->duration_in_seconds.Value()),
      looping_(emitter_state.emitter_config->loop.Value()),
      particles_per_second_(
          emitter_state.emitter_config->particles_per_second.Value()),
      particle_delay_(0.0f) {}

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

void NodeParticleEmitter::UpdateEmitterBehavior(const FrameTime& frame_time) {
  // Update the remaining duration.
  if (emitter_duration_finite_) {
    // Reduce the emitter lifetime.
    if (remaining_emitter_duration_ > 0.0f) {
      remaining_emitter_duration_ -= frame_time.GetDeltaSeconds();
    }

    // If the emitter has expired, but is looping, reset its duration.
    if (looping_ && remaining_emitter_duration_ <= 0.0f) {
      remaining_emitter_duration_ = emitter_duration_;
    }
  }
}

bool NodeParticleEmitter::CanEmitParticles() {
  // Waiting for the next time to emit a particle?
  if (particle_delay_ > 0.0f) return false;

  // Are there too many active particles?
  if (active_particles_.size() >= particle_pool_.GetMaxParticles())
    return false;

  // Is the emitter actively emitting particles?
  if (emitter_duration_finite_ && remaining_emitter_duration_ <= 0.0f)
    return false;

  return true;
}

std::string NodeParticleEmitter::ValidateEmitterState(
    const ParticleEmitterState& emitter_state) {
  // A gltf asset reference is required.
  if (!emitter_state.particle_config->gltf_asset) {
    return "NodeParticleEmitter - gltf_asset not set!";
  }

  // Particles must be emitted.
  if (emitter_state.emitter_config->particles_per_second.Value() <= 0.0f) {
    return "NodeParticleEmitter - particles per second must be > 0!";
  }

  // There must be an allowance for the maximum number of particles.
  if (emitter_state.emitter_config->max_particles.Value() <= 0) {
    return "NodeParticleEmitter - max particles must be > 0!";
  }

  // The duration must be non-negative.
  if (emitter_state.emitter_config->duration_in_seconds.Value() < 0.0f) {
    return "NodeParticleEmitter - duration in seconds must be >= 0!";
  }

  return "";
}

imp::ParticleEmitterInfo NodeParticleEmitter::GetParticleEmitterInfo() const {
  // Get the current camera position.
  float3 camera_position = kZero3;
  if (emitter_node_.IsValid()) {
    ComponentHandle<CameraComponent> camera =
        emitter_node_->GetView().GetCameraManager().GetCamera();
    if (camera.IsValid()) {
      camera_position = camera->GetNode()->GetWorldPosition();
    }
  }

  // Assemble the emitter info.
  return imp::ParticleEmitterInfo(camera_position);
}

}  // namespace imp::imp_particle
