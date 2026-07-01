// Copyright 2024 Google LLC
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

#include "core/assets/material/material_helpers.h"

#include <cstdint>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/View.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/utils/include/utils/tribool.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/view/base_view.h"

namespace imp::material_helpers {

namespace {
filament::UserVariantFilterMask default_high_priority_mask = 0;

filament::UserVariantFilterMask GetMaterialVariantMask(
    const imp::MaterialPreCompileVariants& material_pre_compile_variants,
    MaterialPreCompileVariants::VariantPreCompileMode pre_compile_mode) {
  filament::UserVariantFilterMask variant_mask = 0;

  if (material_pre_compile_variants.directional_lighting == pre_compile_mode) {
    variant_mask |= static_cast<uint32_t>(
        filament::UserVariantFilterBit::DIRECTIONAL_LIGHTING);
  }

  if (material_pre_compile_variants.dynamic_lighting == pre_compile_mode) {
    variant_mask |=
        static_cast<uint32_t>(filament::UserVariantFilterBit::DYNAMIC_LIGHTING);
  }

  if (material_pre_compile_variants.shadow_receiver == pre_compile_mode) {
    variant_mask |=
        static_cast<uint32_t>(filament::UserVariantFilterBit::SHADOW_RECEIVER);
  }

  if (material_pre_compile_variants.skinning == pre_compile_mode) {
    variant_mask |=
        static_cast<uint32_t>(filament::UserVariantFilterBit::SKINNING);
  }

  if (material_pre_compile_variants.fog == pre_compile_mode) {
    variant_mask |= static_cast<uint32_t>(filament::UserVariantFilterBit::FOG);
  }

  if (material_pre_compile_variants.ssr == pre_compile_mode) {
    variant_mask |= static_cast<uint32_t>(filament::UserVariantFilterBit::SSR);
  }

  if (material_pre_compile_variants.vsm == pre_compile_mode) {
    variant_mask |= static_cast<uint32_t>(filament::UserVariantFilterBit::VSM);
  }

  if (material_pre_compile_variants.ste == pre_compile_mode) {
    variant_mask |= static_cast<uint32_t>(filament::UserVariantFilterBit::STE);
  }

  return variant_mask;
}

utils::tribool ToTribool(
    MaterialPreCompileOptions::MaterialPreCompileByView::VariantFilterOption
        option) {
  switch (option) {
    case MaterialPreCompileOptions::MaterialPreCompileByView::
        VARIANT_FILTER_OPTION_ENABLED:
      return utils::tribool(true);
    case MaterialPreCompileOptions::MaterialPreCompileByView::
        VARIANT_FILTER_OPTION_DISABLED:
      return utils::tribool(false);
    case imp::MaterialPreCompileOptions::MaterialPreCompileByView::
        VARIANT_FILTER_OPTION_INDETERMINATE:
    case imp::MaterialPreCompileOptions::MaterialPreCompileByView::
        VARIANT_FILTER_OPTION_UNSPECIFIED:
      return utils::tribool(utils::tribool::Indeterminate);
  }
}

}  // namespace

FilamentVariantMask GetMaterialVariantMask(
    const imp::MaterialPreCompileVariants& material_pre_compile_variants) {
  FilamentVariantMask variant_mask;

  // Build default set of high priority variants. The variant must be set to
  // DEFAULT and be in the static default variant.
  filament::UserVariantFilterMask implicitly_high_priority_mask =
      default_high_priority_mask &
      GetMaterialVariantMask(material_pre_compile_variants,
                             MaterialPreCompileVariants::DEFAULT);

  filament::UserVariantFilterMask explicitly_high_priority_mask =
      GetMaterialVariantMask(material_pre_compile_variants,
                             MaterialPreCompileVariants::HIGH_PRIORITY);

  variant_mask.high_priority_mask =
      implicitly_high_priority_mask | explicitly_high_priority_mask;

  variant_mask.low_priority_mask = GetMaterialVariantMask(
      material_pre_compile_variants, MaterialPreCompileVariants::LOW_PRIORITY);

  return variant_mask;
}

void SetDefaultHighPriorityVariants(
    filament::UserVariantFilterMask high_priority_mask) {
  default_high_priority_mask = high_priority_mask;
}

Future<absl::Status> PreCompileMaterial(
    filament::Material* material,
    const MaterialPreCompileOptions& pre_compile_options) {
  Future<absl::Status> compile_status = Future<absl::Status>(absl::OkStatus());
  FilamentVariantMask variant_mask =
      GetMaterialVariantMask(pre_compile_options.variants);

  if (variant_mask.high_priority_mask) {
    // Compiles the high priority variants.
    // Resets `compile_status` so that it wouldn't be ready right away.
    compile_status = Future<absl::Status>();
    // Registers the callback so that `compile_status` becomes ready when the
    // high priority variants' compilation is complete.
    material->compile(filament::backend::CompilerPriorityQueue::HIGH,
                      variant_mask.high_priority_mask, nullptr,
                      [compile_status](filament::Material* material) {
                        compile_status.Return(absl::OkStatus());
                      });

    compile_status = compile_status.Then(
        [](absl::Status status) mutable {
          // Force the chain of futures to wait until the next time the
          // foreground executor is pumped before continuing.
          //
          // This is because the compile callback can occur within a call to
          // filament::Renderer::render which is not a safe time to complete the
          // future. When a material is finished loading it will likely cause
          // material assignments to change, possibly destroying materials,
          // which can cause use-after-free issues when done within a render
          // call.
          return status;
        },
        FutureThenOptions{.executor_mode =
                              FutureExecutorMode::kScheduleAlways});
  }

  if (variant_mask.low_priority_mask) {
    // Compiles the low priority variants.
    material->compile(filament::backend::CompilerPriorityQueue::LOW,
                      variant_mask.low_priority_mask);
  }

  return compile_status;
}

Future<absl::Status> PreCompileMaterialByView(
    filament::Material* material,
    const MaterialPreCompileOptions& pre_compile_options, BaseView* view) {
  
  

  filament::View* filament_view = view->GetHost()->GetView();

  if (pre_compile_options.compile_by_view->priority ==
      MaterialPreCompileOptions::MaterialPreCompileByView::PRIORITY_HIGH) {
    Future<absl::Status> compile_status = Future<absl::Status>();
    view->GetHost()->GetEngine()->compile(
        filament::backend::CompilerPriorityQueue::HIGH, material, filament_view,
        ToTribool(pre_compile_options.compile_by_view->shadow_receiver),
        ToTribool(pre_compile_options.compile_by_view->skinning),
        /* CallbackHandler= */ nullptr,
        [compile_status](filament::Material* material) mutable {
          compile_status.Return(absl::OkStatus());
        });

    compile_status = compile_status.Then(
        [](absl::Status status) mutable {
          // Force the chain of futures to wait until the next time the
          // foreground executor is pumped before continuing.
          //
          // This is because the compile callback can occur within a call to
          // filament::Renderer::render which is not a safe time to complete
          // the future. When a material is finished loading it will likely
          // cause material assignments to change, possibly destroying
          // materials, which can cause use-after-free issues when done
          // within a render call.
          return status;
        },
        FutureThenOptions{.executor_mode =
                              FutureExecutorMode::kScheduleAlways});
    return compile_status;

  } else {
    Future<absl::Status> compile_status =
        Future<absl::Status>(absl::OkStatus());

    view->GetHost()->GetEngine()->compile(
        filament::backend::CompilerPriorityQueue::LOW, material, filament_view,
        ToTribool(pre_compile_options.compile_by_view->shadow_receiver),
        ToTribool(pre_compile_options.compile_by_view->skinning));

    return compile_status;
  }
}

}  // namespace imp::material_helpers
