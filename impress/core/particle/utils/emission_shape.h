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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_EMITTER_SHAPE_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_EMITTER_SHAPE_H_

#include <cmath>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"

namespace imp::imp_particle {

// Emission shapes are used to define where particles are initially created
// relative to the emitter node. Derived classes will implement different
// shape geometries, but the default behavior is the `PointEmissionShape`, which
// emits from a point at the emitter node's current position.
class EmissionShape {
 public:
  // Initializes the shape. Particle positions are provided in world space, so
  // the emitter node is used to determine where the shape is in the scene.
  EmissionShape(NodeHandle emitter_node) : emitter_node_(emitter_node) {
    // TODO: (broken link) - Use a more robust method for randomization.
    rand_seed_ = static_cast<unsigned int>(absl::ToUnixMicros(absl::Now()));
  }

  // Virtual destructor for the emitter shape.
  virtual ~EmissionShape() = default;

  // Returns the next randomized position of the particle to be emitted.
  virtual float3 GetNextParticlePosition() = 0;

  // Helper to return a random float value in the range [min, max].
  float RandRangef(float min, float max) {
    // TODO: (broken link) - Use a more robust method for randomization.
    return min + (rand_r(&rand_seed_) % 1000) / 1000.0f * (max - min);
  }

 protected:
  NodeHandle emitter_node_;
  unsigned int rand_seed_ = 42;
};

class PointEmissionShape : public EmissionShape {
 public:
  // Initializes the shape. Particle positions are provided in world space, so
  // the emitter node is used to determine where the shape is in the scene.
  PointEmissionShape(NodeHandle emitter_node) : EmissionShape(emitter_node) {}

  // Returns the next randomized position of the particle to be emitted.
  float3 GetNextParticlePosition() override {
    return emitter_node_->GetWorldPosition();
  }
};

class PlaneEmissionShape : public EmissionShape {
 public:
  // Initializes the shape. Particle positions are provided in world space, so
  // the emitter node is used to determine where the shape is in the scene.
  PlaneEmissionShape(NodeHandle emitter_node, float2 extents,
                     float3 plane_normal)
      : EmissionShape(emitter_node) {
    // Construct the u and v vectors to be aligned with the x and z axes in the
    // plane. Scale them to the provided extents.
    const float3 normal = normalize(plane_normal);
    float3 u_dir;
    if (std::abs(normal.x) < 0.99f) {
      u_dir = imp::kRight - normal * normal.x;
    } else {
      u_dir = imp::kBack - normal * normal.z;
    }
    const float3 u = normalize(u_dir);
    const float3 v = cross(u, normal);
    local_extents_u_ = extents.x * u;
    local_extents_v_ = extents.y * v;
  }

  // Returns the next randomized position of the particle to be emitted.
  float3 GetNextParticlePosition() override {
    const auto& world_rotation = emitter_node_->GetWorldRotation();
    float3 rotated_extents_u = world_rotation * local_extents_u_;
    float3 rotated_extents_v = world_rotation * local_extents_v_;
    return emitter_node_->GetWorldPosition() +
           RandRangef(-1.0f, 1.0f) * rotated_extents_u +
           RandRangef(-1.0f, 1.0f) * rotated_extents_v;
  }

 private:
  float3 local_extents_u_ = kZero3;
  float3 local_extents_v_ = kZero3;
};

}  // namespace imp::imp_particle

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_EMITTER_SHAPE_H_
