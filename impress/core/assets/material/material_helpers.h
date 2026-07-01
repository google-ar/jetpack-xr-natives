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
#include "core/view/base_view.h"

namespace imp::material_helpers {

// Contains Filament variant masks at different priorities.
struct FilamentVariantMask {
  filament::UserVariantFilterMask high_priority_mask = 0;
  filament::UserVariantFilterMask low_priority_mask = 0;
};

// Converts the MaterialPreCompileVariants to Filament variant mask.
FilamentVariantMask GetMaterialVariantMask(
    const imp::MaterialPreCompileVariants& material_pre_compile_variants);

// Changes the default setting for high priority mask during precompilation.
//
// If unmodified the default setting is to EXCLUDE all variants.
// Adding bits here will cause variants to be compiled asynchronously for every
// material which doesn't explicitly exclude them.
void SetDefaultHighPriorityVariants(
    filament::UserVariantFilterMask high_priority_mask);

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

// Precompiles the filament::Material using a given View's active features as
// a baseline. `MaterialPreCompileByView` must be specified within
// `MaterialPreCompileOptions` to use this API.
//
// There are two main differences between this and `PreCompileMaterial`:
// 1. Baseline View: Instead of compiling variants in isolation, it inspects the
//    provided `BaseView` for its active view-global features (e.g., fog,
//    dynamic lighting, SSR) to determine the exact baseline permutations
//    required to render the material in that view.
// 2. Explicit Variant Filtering: Rather than providing a raw variant filter
//    mask, the caller explicitly specifies whether the remaining per-renderable
//    variant features (`shadow_receiver` and `skinning`) should be compiled for
//    the enabled permutation, the disabled permutation, or both.
//
// If `MaterialPreCompileByView::priority` is set to `PRIORITY_HIGH`, the
// returned future will not be ready until the high-priority variants have
// finished compiling in the backend. Otherwise, for `PRIORITY_LOW`, the future
// is ready immediately.
// TODO: Support non-main Views as well.
Future<absl::Status> PreCompileMaterialByView(
    filament::Material* material,
    const MaterialPreCompileOptions& pre_compile_options, BaseView* view);

}  // namespace imp::material_helpers

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_MATERIAL_MATERIAL_HELPERS_H_
