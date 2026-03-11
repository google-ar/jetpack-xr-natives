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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_NODE_PARTICLE_CONTROLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_NODE_PARTICLE_CONTROLLER_H_

#include <cstdint>
#include <list>
#include <string>

#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/ncsb/node_handle.h"
#include "core/particle/particle_behavior.h"
#include "core/particle/particle_controller.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/particle_instance.h"
#include "core/particle/particle_service.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// A particle controller that manages node-based particles. These are particles
// that are each represented by a Node in the scene.
class NodeParticleController : public ParticleController {
 public:
  // Creates a new NodeParticleController.
  static Future<OwnedParticleControllerPtr> Create(
      NodeHandle emitter_node, const ParticleEmitterState& emitter_state);

  // Updates all active particles in the system, creates and destroys particles
  // as defined by the ParticleEmitterConfig.
  void UpdateParticleSystem(const FrameTime& frame_time) override;

 protected:
  // Verifies elements of the emitter state to ensure the particle system can
  // be properly initialized.
  static std::string ValidateEmitterState(
      const ParticleEmitterState& emitter_state);

 private:
  // Initializes the controller using the configuration provided. The emitter
  // node is stored here so particles may be emitted from it using its position
  // and orientation when configured to emit into world space.
  NodeParticleController(NodeHandle emitter_node,
                         AssetPtr<GltfAsset> gltf_asset,
                         const ParticleEmitterState& emitter_state);

  // Synchronizes the scene node with the current state of the particle.
  void SyncNode(const ParticleInstance& particle_instance, NodeHandle node);

  // Node particles include a scene node and a particle index, which refers to
  // the particle state stored in the ParticleService, and updated by a
  // ParticleBehavior.
  struct NodeParticle {
    NodeHandle node;
    int32_t particle_index;
  };

  // Service to hold particle state.
  ParticleService particle_service_;

  // Behaviors to perform behaviors on particles.
  ParticleBehavior particle_behavior_;

  // List of active particles.
  std::list<NodeParticle> active_particles_;

  // Reference to the emitter node.
  NodeHandle emitter_node_;

  // Particle asset, by holding this when the controller is created, we ensure
  // that the asset is loaded and available when instantiating particles.
  AssetPtr<GltfAsset> gltf_asset_;

  // Particle emission control.
  float particles_per_second_;
  float particle_delay_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_NODE_PARTICLE_CONTROLLER_H_
