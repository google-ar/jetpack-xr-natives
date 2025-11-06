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

#include "core/particle/standard_particle_service.h"

#include <cstdint>
#include <utility>

#include "absl/memory/memory.h"
#include "core/async/future.h"
#include "core/common/owned_ptr.h"
#include "core/particle/particle_config.proto.imp.h"
#include "core/particle/particle_data.h"
#include "core/particle/particle_data_provider.h"
#include "core/particle/particle_emitter_info.h"
#include "core/particle/particle_service.h"
#include "core/view/utils/frame_time.h"

namespace imp {

StandardParticleService::StandardParticleService(
    const ParticleConfig& particle_config)
    : data_layout_(particle_config), data_provider_(1) {}

StandardParticleService::~StandardParticleService() {}

Future<OwnedParticleServicePtr> StandardParticleService::Create(
    const ParticleConfig& particle_config) {
  OwnedPtr<StandardParticleService> service =
      absl::WrapUnique(new StandardParticleService(particle_config));

  return Future<OwnedParticleServicePtr>(std::move(service));
}

void StandardParticleService::ProcessActiveParticles(
    const FrameTime& frame_time) {
  // TODO: (broken link) - Implement particle update logic.
}

ParticleData StandardParticleService::GetDataForParticle(
    int32_t particle_index) {
  return ParticleData(data_provider_, data_layout_,
                      particle_index * data_layout_.GetSize());
}

ParticleEmitterInfo StandardParticleService::GetEmitterInfo() const {
  return ParticleEmitterInfo();
}

ParticleDataProvider& StandardParticleService::GetDataProvider() {
  return data_provider_;
}

}  // namespace imp
