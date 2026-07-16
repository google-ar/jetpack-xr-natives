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

#include "core/particle/utils/particle_format.h"

#include "core/particle/particle_emitter_state.proto.imp.h"

namespace imp::imp_particle {

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

ParticleFormat::ParticleFormat(const ParticleConfig& config) {
  // Initialize the offsets.
  offsets_.resize(static_cast<int>(AttributeType::kNumFields));
  for (int i = 0; i < static_cast<int>(AttributeType::kNumFields); ++i) {
    offsets_[i] = kInvalidParticleDataOffset;
  }

  // Infer used fields from the particle config.
  particle_data_size_ = 0;

  // All particles have a position, it is not configurable from the proto.
  offsets_[static_cast<int>(AttributeType::kPosition)] = particle_data_size_;
  particle_data_size_ += 3;

  if (config.initial_rotation || config.billboard) {
    offsets_[static_cast<int>(AttributeType::kRotation)] = particle_data_size_;
    particle_data_size_ += 4;
  }

  if (config.initial_scale) {
    offsets_[static_cast<int>(AttributeType::kScale)] = particle_data_size_;
    particle_data_size_ += 3;
  }

  if (config.lifetime_in_seconds) {
    offsets_[static_cast<int>(AttributeType::kLifetime)] = particle_data_size_;
    particle_data_size_++;
  }

  if (config.initial_alpha) {
    offsets_[static_cast<int>(AttributeType::kAlpha)] = particle_data_size_;
    particle_data_size_++;
  }

  if (config.initial_velocity) {
    offsets_[static_cast<int>(AttributeType::kVelocity)] = particle_data_size_;
    particle_data_size_ += 3;
  }

  if (config.acceleration) {
    offsets_[static_cast<int>(AttributeType::kAcceleration)] =
        particle_data_size_;
    particle_data_size_ += 3;
  }
}

int ParticleFormat::GetSize() const { return particle_data_size_; }

int ParticleFormat::GetLifetime() const {
  return offsets_[static_cast<int>(AttributeType::kLifetime)];
}

int ParticleFormat::GetAlpha() const {
  return offsets_[static_cast<int>(AttributeType::kAlpha)];
}

int ParticleFormat::GetScale() const {
  return offsets_[static_cast<int>(AttributeType::kScale)];
}

int ParticleFormat::GetPosition() const {
  return offsets_[static_cast<int>(AttributeType::kPosition)];
}

int ParticleFormat::GetVelocity() const {
  return offsets_[static_cast<int>(AttributeType::kVelocity)];
}

int ParticleFormat::GetAcceleration() const {
  return offsets_[static_cast<int>(AttributeType::kAcceleration)];
}

int ParticleFormat::GetRotation() const {
  return offsets_[static_cast<int>(AttributeType::kRotation)];
}

}  // namespace imp::imp_particle
