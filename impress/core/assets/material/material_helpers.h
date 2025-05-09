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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_MATERIAL_MATERIAL_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_MATERIAL_MATERIAL_HELPERS_H_

#include "absl/status/status.h"
#include "filament/filament/include/filament/Material.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"

namespace imp::material_helpers {

// Precompiles the filament::Material with the specified
// `MaterialPreCompileOptions`. Please note that this does not speed up shader
// compilation and it is only moving the workload up front.
//
// High priority variants will always be compiled before low priority variants.
//
// If `MaterialPreCompileOptions::wait_for_high_priority_variant` is set to be
// true, the returned future will not be ready until all the high priority
// variants are compiled. Otherwise the returned future will be ready right
// away.
Future<absl::Status> PreCompileMaterial(
    filament::Material* material,
    const MaterialPreCompileOptions& pre_compile_options);

}  // namespace imp::material_helpers

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_MATERIAL_MATERIAL_HELPERS_H_
