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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_CUSTOM_MATERIAL_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_CUSTOM_MATERIAL_FACTORY_H_

#include <functional>
#include <memory>
#include <optional>

#include "core/async/future.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/split_engine/split_engine_filament_resource_ptrs.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// TODO : Replace with a statically registered request handler.
// We have a dedicated custom material factory because it needs to links with
// the runtime material compiler. We cannot link it to the built-in material
// factory because it is linked in standalone Impress apps to allow them to use
// built-in materials. This causes an unacceptable increase in binary size.
class SplitEngineCustomMaterialFactory {
 public:
  explicit SplitEngineCustomMaterialFactory(BaseView& view);

  // Handles a request to create a split engine custom material.
  Future<OwnedFilamentMaterialPtr> CreateCustomMaterial(
      BaseView& view,
      const android_xr::schemas::AddCustomMaterialRequest& request);

 private:
  Future<std::reference_wrapper<RuntimeMaterialCompiler>>
  GetOrCreateMaterialCompiler();

  Future<OwnedFilamentMaterialPtr> CreateFilamentMaterial(
      const android_xr::schemas::FilamentMaterialSpec& spec);

  BaseView& view_;
  std::optional<Future<std::reference_wrapper<RuntimeMaterialCompiler>>>
      material_compiler_future_;
  std::unique_ptr<RuntimeMaterialCompiler> material_compiler_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_CUSTOM_MATERIAL_FACTORY_H_
