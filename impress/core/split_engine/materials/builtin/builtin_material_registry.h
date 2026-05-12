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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_MATERIAL_REGISTRY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_MATERIAL_REGISTRY_H_

#include <functional>
#include <optional>

#include "absl/status/statusor.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/common/small_source_location.h"
#include "core/material_library/material_package.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// A registry of built-in material creators.
//
// To use this registry, register a creator for each material spec type. For
// example:
//
//    const bool kRegisterMaterial = BuiltinMaterialRegistry::RegisterOrDie(
//        android_xr::schemas::BuiltInMaterialSpec::GenericMaterialSpec,
//        [](BaseView& view, BridgeId bridge_id,
//           const android_xr::schemas::BuiltInMaterialRequest& request,
//           std::optional<
//               std::reference_wrapper<const MaterialPackage::MaterialCache>>
//               cache) -> Future<BuiltInMaterialPtr> {
//          const android_xr::schemas::GenericMaterialSpec* spec =
//              request.data_as_GenericMaterialSpec();
//          if (spec == nullptr) {
//            return Future<BuiltInMaterialPtr>(absl::InvalidArgumentError(
//                "Failed to get the GenericMaterialSpec from the request."));
//          }
//          absl::StatusOr<GenericMaterialSpec> unpacked_spec = Unpack(*spec);
//          if (!unpacked_spec.ok()) {
//            return Future<BuiltInMaterialPtr>(unpacked_spec.status());
//          }
//          return BuiltInGenericMaterial::Create(view, *unpacked_spec, *cache);
//        });
//
// Make sure to call RegisterOrDie in a .cc file in a global scope to prevent
// the call from being optimized away. The cc file should also have
// `alwayslink = True` in the build rule to prevent the registration from being
// stripped by the linker.
//
// See an example in builtin_generic_material.cc.
class BuiltinMaterialRegistry {
 public:
  using BuiltinMaterialCreator = Invocable<Future<BuiltInMaterialPtr>(
      BaseView& view, BridgeId bridge_id,
      const android_xr::schemas::BuiltInMaterialRequest& request,
      std::optional<
          std::reference_wrapper<const MaterialPackage::MaterialCache>>
          cache)>;

  // Returns a reference to the creator for the given material spec type.
  static absl::StatusOr<BuiltinMaterialCreator&> Get(
      android_xr::schemas::BuiltInMaterialSpec type);

  // Registers a creator for the given material spec type. It will cause a crash
  // if a creator is registered twice for the same material spec type. This
  // must be called at the global scope, for example in a .cc file. Returns a
  // boolean to prevent the compiler from optimizing the call away.
  static bool RegisterOrDie(
      android_xr::schemas::BuiltInMaterialSpec type,
      BuiltinMaterialCreator creator,
      SmallSourceLocation loc = SmallSourceLocation::Current());
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_MATERIAL_REGISTRY_H_
