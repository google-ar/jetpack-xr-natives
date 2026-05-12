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

#include "core/particle/data_layout.h"

#include "absl/types/optional.h"
#include "core/particle/particle_emitter_state.proto.imp.h"

namespace imp {
namespace imp_particle {

enum class AttributeType {
  // Time the particle will remain active. (float, seconds)
  kLifetime = 0,

  // Alpha value for the particle. (float, 0.0f -> 1.0f)
  kAlpha,

  // Scale value for the particle. (float3, sx, sy, sz, >= 0.0f)
  kScale,

  // Position. (float3, x, y, z)
  kPosition,

  // Velocity. (float3, dx, dy, dz)
  kVelocity,

  // Acceleration. (float3, ddx, ddy, ddz)
  kAcceleration,

  // Rotation. (float4, w, x, y, z)
  kRotation,

  // This must be the last field.
  kNumFields,
};

DataLayout::DataLayout(const ParticleConfig& config) {
  // Initialize the offsets.
  offsets_.resize(static_cast<int>(AttributeType::kNumFields));
  for (int i = 0; i < static_cast<int>(AttributeType::kNumFields); ++i) {
    offsets_[i] = kInvalidParticleDataOffset;
  }

  // Infer used fields from the particle config.
  particle_data_size_ = 0;

  if (config.lifetime_in_seconds.has_value()) {
    offsets_[static_cast<int>(AttributeType::kLifetime)] = particle_data_size_;
    particle_data_size_++;
  }

  if (config.alpha.has_value()) {
    offsets_[static_cast<int>(AttributeType::kAlpha)] = particle_data_size_;
    particle_data_size_++;
  }

  if (config.scale.has_value()) {
    offsets_[static_cast<int>(AttributeType::kScale)] = particle_data_size_;
    particle_data_size_ += 3;
  }

  if (config.billboard.value_or(false)) {
    offsets_[static_cast<int>(AttributeType::kRotation)] = particle_data_size_;
    particle_data_size_ += 4;
  }

  // All particles have a position, it is not configurable from the proto.
  offsets_[static_cast<int>(AttributeType::kPosition)] = particle_data_size_;
  particle_data_size_ += 3;

  if (config.velocity.has_value()) {
    offsets_[static_cast<int>(AttributeType::kVelocity)] = particle_data_size_;
    particle_data_size_ += 3;
  }

  if (config.acceleration.has_value()) {
    offsets_[static_cast<int>(AttributeType::kAcceleration)] =
        particle_data_size_;
    particle_data_size_ += 3;
  }
}

int DataLayout::GetSize() const { return particle_data_size_; }

int DataLayout::GetLifetime() const {
  return offsets_[static_cast<int>(AttributeType::kLifetime)];
}

int DataLayout::GetAlpha() const {
  return offsets_[static_cast<int>(AttributeType::kAlpha)];
}

int DataLayout::GetScale() const {
  return offsets_[static_cast<int>(AttributeType::kScale)];
}

int DataLayout::GetPosition() const {
  return offsets_[static_cast<int>(AttributeType::kPosition)];
}

int DataLayout::GetVelocity() const {
  return offsets_[static_cast<int>(AttributeType::kVelocity)];
}

int DataLayout::GetAcceleration() const {
  return offsets_[static_cast<int>(AttributeType::kAcceleration)];
}

int DataLayout::GetRotation() const {
  return offsets_[static_cast<int>(AttributeType::kRotation)];
}

}  // namespace imp_particle
}  // namespace imp
