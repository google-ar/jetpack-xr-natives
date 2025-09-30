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

#include "core/particle/particle_data.h"

#include <cstdint>

#include "core/common/log.h"
#include "core/math/vec.h"
#include "core/particle/data_layout.h"
#include "core/particle/particle_data_provider.h"

namespace imp {

using imp_particle::DataLayout;
using imp_particle::kInvalidParticleDataOffset;

ParticleData::ParticleData(ParticleDataProvider& data_provider,
                           const DataLayout& data_layout, int32_t float_offset)
    : data_provider_(data_provider),
      data_layout_(data_layout),
      float_offset_(float_offset) {}

float ParticleData::GetRemainingLifetimeSeconds() const {
  if (data_layout_.GetLifetime() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleData does not have a lifetime value.";
  }

  return data_provider_.GetFloat(float_offset_ + data_layout_.GetLifetime());
}

float ParticleData::GetAlpha() const {
  if (data_layout_.GetAlpha() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleData does not have an alpha value.";
  }

  return data_provider_.GetFloat(float_offset_ + data_layout_.GetAlpha());
}

void ParticleData::SetAlpha(float alpha) {
  if (data_layout_.GetAlpha() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleData does not have an alpha value.";
  }

  data_provider_.SetFloat(float_offset_ + data_layout_.GetAlpha(), alpha);
}

float ParticleData::GetScale() const {
  if (data_layout_.GetScale() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleData does not have a scale value.";
  }

  return data_provider_.GetFloat(float_offset_ + data_layout_.GetScale());
}

void ParticleData::SetScale(float scale) {
  if (data_layout_.GetScale() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleData does not have a scale value.";
  }

  data_provider_.SetFloat(float_offset_ + data_layout_.GetScale(), scale);
}

float3 ParticleData::GetPosition() const {
  if (data_layout_.GetPosition() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleData does not have a position value.";
  }

  int32_t position_offset = float_offset_ + data_layout_.GetPosition();
  return float3(data_provider_.GetFloat(position_offset + 0),
                data_provider_.GetFloat(position_offset + 1),
                data_provider_.GetFloat(position_offset + 2));
}

void ParticleData::SetPosition(float3 position) {
  if (data_layout_.GetPosition() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleData does not have a position value.";
  }

  int32_t position_offset = float_offset_ + data_layout_.GetPosition();
  data_provider_.SetFloat(position_offset + 0, position.x);
  data_provider_.SetFloat(position_offset + 1, position.y);
  data_provider_.SetFloat(position_offset + 2, position.z);
}

float3 ParticleData::GetVelocity() const {
  if (data_layout_.GetVelocity() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleData does not have a velocity value.";
  }

  int32_t velocity_offset = float_offset_ + data_layout_.GetVelocity();
  return float3(data_provider_.GetFloat(velocity_offset + 0),
                data_provider_.GetFloat(velocity_offset + 1),
                data_provider_.GetFloat(velocity_offset + 2));
}

void ParticleData::SetVelocity(float3 velocity) {
  if (data_layout_.GetVelocity() == kInvalidParticleDataOffset) {
    IMP_LOG(imp::FATAL) << "ParticleData does not have a velocity value.";
  }

  int32_t velocity_offset = float_offset_ + data_layout_.GetVelocity();
  data_provider_.SetFloat(velocity_offset + 0, velocity.x);
  data_provider_.SetFloat(velocity_offset + 1, velocity.y);
  data_provider_.SetFloat(velocity_offset + 2, velocity.z);
}

}  // namespace imp
