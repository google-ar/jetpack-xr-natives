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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_INSTANCED_PARTICLE_EMITTER_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_INSTANCED_PARTICLE_EMITTER_H_

#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "core/assets/asset_ptr.h"
#include "core/assets/gltf/gltf_asset.h"
#include "core/async/future.h"
#include "core/math/mat.h"
#include "core/ncsb/node_handle.h"
#include "core/particle/custom_particle_behavior.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/utils/particle_emitter.h"
#include "core/view/utils/frame_time.h"

namespace imp::imp_particle {

// A particle emitter that manages instanced-based particles. These are
// particles that are managed individually but rendered using Filament's
// instanced rendering API.
class InstancedParticleEmitter : public ParticleEmitter {
 public:
  // Creates a new InstancedParticleEmitter.
  static Future<OwnedParticleEmitterPtr> Create(
      NodeHandle emitter_node, const ParticleEmitterState& emitter_state,
      std::unique_ptr<CustomParticleBehavior> custom_particle_behavior =
          std::unique_ptr<CustomParticleBehavior>());

  // Updates all active particles in the system, creates and destroys particles
  // as defined by the ParticleEmitterConfig.
  void UpdateParticleSystem(const FrameTime& frame_time) override;

 protected:
  // Destroys the emitter and all nodes that were created for it.
  ~InstancedParticleEmitter() override;

  // Verifies elements of the emitter state to ensure the particle system can
  // be properly initialized.
  static std::string ValidateEmitterState(
      const ParticleEmitterState& emitter_state);

  // Synchronizes the transforms of all active particles with the instances.
  void PrepareRendering();

 private:
  // Initializes the emitter using the configuration provided. The emitter
  // node is stored here so particles may be emitted from it using its position
  // and orientation when configured to emit into world space.
  InstancedParticleEmitter(
      NodeHandle emitter_node, const ParticleEmitterState& emitter_state,
      std::unique_ptr<CustomParticleBehavior> custom_particle_behavior,
      AssetPtr<GltfAsset> gltf_asset, OwnedMaterialPtr material_instance);

 protected:
  // These manage the blocks of nodes uses for instancing. There is a maximum
  // for the number of instances that can be rendered with one Node (Filament's
  // entity,) so we must create multiple nodes to render more particles.
  NodeHandle particle_root_node_;
  std::vector<NodeHandle> block_nodes_;
  std::vector<int32_t> block_sizes_;
  std::vector<mat4f> instance_transforms_;  // Reusable for transform update.

  // List of active particles indices.
  std::list<int32_t> active_particles_;

  int32_t max_active_particles_ = 0;
  float next_debug_time_ = 0.0f;
};

}  // namespace imp::imp_particle

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_INSTANCED_PARTICLE_EMITTER_H_
