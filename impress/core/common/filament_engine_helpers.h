/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_FILAMENT_ENGINE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_FILAMENT_ENGINE_HELPERS_H_

#include "absl/status/statusor.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Material.h"

namespace imp {

// Block until all pending frames have been processed.
void SynchronizePendingFrames(filament::Engine* engine);

// Block until the GPU has finished processing all submitted commands
void FlushEngineAndWait(filament::Engine* engine);

// Retrieve a .cmat file from the resources packaged with our binary and build a
// filament material.
absl::StatusOr<filament::Material*> LoadPackagedMaterial(
    filament::Engine* engine, absl::string_view path);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_FILAMENT_ENGINE_HELPERS_H_
