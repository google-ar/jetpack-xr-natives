// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TEXTURE_VALIDATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TEXTURE_VALIDATOR_H_

#include "absl/status/status.h"
#include "filament/filament/include/filament/Engine.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

class TextureValidator {
 public:
  static absl::Status ValidateAddTexturesMessage(
      const android_xr::schemas::AddTextures& command,
      filament::Engine& engine);
};

}  // namespace imp::split_engine
#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TEXTURE_VALIDATOR_H_
