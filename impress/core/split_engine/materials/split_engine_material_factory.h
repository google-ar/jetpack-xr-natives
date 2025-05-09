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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_MATERIAL_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_MATERIAL_FACTORY_H_

#include <cstdint>
#include <memory>

#include "core/async/future.h"
#include "core/material_library/material_package.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

using MaterialId = uint64_t;

// Creates built-in Split Engine materials on the backend renderer by schema.
// Built-in materials are defined by android_xr::schemas::BuiltInMaterialSpec.
class SplitEngineMaterialFactory {
 public:
  // Handles a request to create a built-in material on the remote renderer.
  // Note: this variant cannot create generic materials, only custom builtins.
  static Future<BuiltInMaterialPtr> HandleCreateRequest(
      BaseView& view, const android_xr::schemas::BuiltInMaterialRequest& spec);

  // Handles a request to create a built-in material on the remote renderer.
  // Note: this variant can also create generic materials and is called by
  // SplitEngineRendererImpl to handle requests from the app side.
  Future<BuiltInMaterialPtr> HandleCreateRequest(
      const android_xr::schemas::BuiltInMaterialRequest& spec);

  // Creates a material factory that can create built-in materials.
  SplitEngineMaterialFactory(BaseView& view);

 private:
  // Handler for creating a material based on schemas::GenericMaterialSpec.
  Future<BuiltInMaterialPtr> CreateBuiltInGenericMaterial(
      const android_xr::schemas::GenericMaterialSpec& schema);

  BaseView& view_;
  std::unique_ptr<MaterialPackage> material_package_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_MATERIAL_FACTORY_H_
