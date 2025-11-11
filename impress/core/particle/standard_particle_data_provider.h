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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_STANDARD_PARTICLE_DATA_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_STANDARD_PARTICLE_DATA_PROVIDER_H_

#include <cstdint>
#include <vector>

#include "core/particle/particle_data_provider.h"

namespace imp {

// Implementation of the ParticleDataProvider, this class stores arbitrary
// float data values for use by the particle
class StandardParticleDataProvider : public ParticleDataProvider {
 public:
  // Creates a new ParticleDataProvider backed by a vector of floats. The
  // constructor will fail if the number of floats requested can not be
  // satisfied.
  StandardParticleDataProvider(int32_t num_floats);

  // Returns the number of floats managed by the data provider.
  int32_t GetNumFloats() const override;

  // Returns the float value at the given index.
  float GetFloat(int32_t index) const override;

  // Sets the float value at the given index.
  void SetFloat(int32_t index, float value) override;

 private:
  std::vector<float> float_data_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_STANDARD_PARTICLE_DATA_PROVIDER_H_
