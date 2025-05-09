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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_MATERIAL_PACKAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_MATERIAL_PACKAGE_H_

#include <optional>

#include "absl/container/flat_hash_set.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Material.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/async/future_group.h"
#include "core/common/robin_map.h"
#include "core/material_library/generic_material_spec.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp {

// Manages a package of materials contained inside of a zip file.
//
// The material package will extract the materials from the zip file and load
// them. It will also cache them so they can be reused between multiple glTFs.
//
// The MaterialPackage should outlive all ModelData created with it, since it
// owns the lifetime of the materials.
class MaterialPackage {
 public:
  using MaterialCache =
      ::imp::RobinMap<GenericMaterialSpec, ::filament::Material*>;

  explicit MaterialPackage(Future<resources::Resource> materials_zip,
                           std::optional<MaterialPreCompileOptions>
                               material_pre_compile_options = std::nullopt);
  ~MaterialPackage();

  // Given a set of material params, lazily loads the materials from the zip
  // file, reusing the materials if they've already been loaded in the past.
  Future<MaterialCache> GetOrLoadMaterials(
      BaseView& view, filament::Engine* engine,
      absl::flat_hash_set<GenericMaterialSpec> requested_materials,
      std::optional<FutureGroup> future_group = std::nullopt);

  // Returns the future to the actual zip file. This is used for tracking
  // metrics for how long it takes to download the materials.
  Future<resources::Resource> GetMaterialsZipFuture();

 private:
  // Engine that was used to create the materials.
  filament::Engine* engine_ = nullptr;

  // The zip file contains all the materials.
  Future<resources::Resource> materials_zip_;

  // Contains all the already loaded materials so they can be reused when
  // requested again.
  MaterialCache cached_material_by_params_;

  std::optional<MaterialPreCompileOptions> material_pre_compile_options_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_MATERIAL_PACKAGE_H_
