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

#include "core/particle/particle_instance.h"

#include <cstdint>

#include "core/common/log.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/particle/data_layout.h"
#include "core/particle/particle_data_provider.h"

namespace imp {

using imp_particle::DataLayout;
using imp_particle::kInvalidParticleDataOffset;

ParticleInstance::ParticleInstance(ParticleDataProvider& data_provider,
                                   const DataLayout& data_layout,
                                   int32_t particle_index,
                                   NodeHandle emitter_node)
    : data_provider_(data_provider),
      data_layout_(data_layout),
      particle_index_(particle_index),
      float_offset_(particle_index * data_layout.GetSize()),
      emitter_node_(emitter_node) {
  // Verify the particle index is valid.
  if (particle_index < 0 ||
      particle_index >= data_provider.GetNumFloats() / data_layout.GetSize()) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, particle access out of range!";
  }
}

int32_t ParticleInstance::GetParticleIndex() const { return particle_index_; }

NodeHandle ParticleInstance::GetEmitterNode() const { return emitter_node_; }

bool ParticleInstance::HasRemainingLifetimeSeconds() const {
  return data_layout_.GetLifetime() != kInvalidParticleDataOffset;
}

float ParticleInstance::GetRemainingLifetimeSeconds() const {
  if (data_layout_.GetLifetime() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid lifetime access.";
  }

  return data_provider_.GetFloat(float_offset_ + data_layout_.GetLifetime());
}

void ParticleInstance::SetRemainingLifetimeSeconds(float seconds) {
  if (data_layout_.GetLifetime() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid lifetime access.";
  }

  data_provider_.SetFloat(float_offset_ + data_layout_.GetLifetime(), seconds);
}

bool ParticleInstance::HasAlpha() const {
  return data_layout_.GetAlpha() != kInvalidParticleDataOffset;
}

float ParticleInstance::GetAlpha() const {
  if (data_layout_.GetAlpha() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid alpha access.";
  }

  return data_provider_.GetFloat(float_offset_ + data_layout_.GetAlpha());
}

void ParticleInstance::SetAlpha(float alpha) {
  if (data_layout_.GetAlpha() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid alpha access.";
  }

  data_provider_.SetFloat(float_offset_ + data_layout_.GetAlpha(), alpha);
}

bool ParticleInstance::HasScale() const {
  return data_layout_.GetScale() != kInvalidParticleDataOffset;
}

float3 ParticleInstance::GetScale() const {
  if (data_layout_.GetScale() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid scale access.";
  }

  return data_provider_.GetVector3f(float_offset_ + data_layout_.GetScale());
}

void ParticleInstance::SetScale(float3 scale) {
  if (data_layout_.GetScale() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid scale access.";
  }

  data_provider_.SetVector3f(float_offset_ + data_layout_.GetScale(), scale);
}

float3 ParticleInstance::GetPosition() const {
  if (data_layout_.GetPosition() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid position access.";
  }

  return data_provider_.GetVector3f(float_offset_ + data_layout_.GetPosition());
}

void ParticleInstance::SetPosition(float3 position) {
  if (data_layout_.GetPosition() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid position access.";
  }

  data_provider_.SetVector3f(float_offset_ + data_layout_.GetPosition(),
                             position);
}

bool ParticleInstance::HasVelocity() const {
  return data_layout_.GetVelocity() != kInvalidParticleDataOffset;
}

float3 ParticleInstance::GetVelocity() const {
  if (data_layout_.GetVelocity() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid velocity access.";
  }

  return data_provider_.GetVector3f(float_offset_ + data_layout_.GetVelocity());
}

void ParticleInstance::SetVelocity(float3 velocity) {
  if (data_layout_.GetVelocity() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid velocity access.";
  }

  data_provider_.SetVector3f(float_offset_ + data_layout_.GetVelocity(),
                             velocity);
}

bool ParticleInstance::HasAcceleration() const {
  return data_layout_.GetAcceleration() != kInvalidParticleDataOffset;
}

float3 ParticleInstance::GetAcceleration() const {
  if (data_layout_.GetAcceleration() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid acceleration access.";
  }

  return data_provider_.GetVector3f(float_offset_ +
                                    data_layout_.GetAcceleration());
}

void ParticleInstance::SetAcceleration(float3 acceleration) {
  if (data_layout_.GetAcceleration() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid acceleration access.";
  }

  data_provider_.SetVector3f(float_offset_ + data_layout_.GetAcceleration(),
                             acceleration);
}

bool ParticleInstance::HasRotation() const {
  return data_layout_.GetRotation() != kInvalidParticleDataOffset;
}

quatf ParticleInstance::GetRotation() const {
  if (data_layout_.GetRotation() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid rotation access.";
  }

  return data_provider_.GetQuatf(float_offset_ + data_layout_.GetRotation());
}

void ParticleInstance::SetRotation(quatf rotation) {
  if (data_layout_.GetRotation() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleInstance, invalid rotation access.";
  }

  data_provider_.SetQuatf(float_offset_ + data_layout_.GetRotation(), rotation);
}

}  // namespace imp
