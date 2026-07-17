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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALCOMPILER_RUNTIME_MATERIAL_COMPILER_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALCOMPILER_RUNTIME_MATERIAL_COMPILER_H_

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Material.h"
#include "core/assets/material/material_asset.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/materials/compiler/cache/material_cache.h"
#include "core/materials/compiler/material_compiler_client.h"
#include "core/materials/compiler/material_compiler_config.h"
#include "core/materials/compiler/schemas/material_compiler_ipc_generated.h"
#include "core/view/base_view.h"

namespace imp {

// Main API for material compilation at runtime. It internally branches based
// on the target platform. (Only supports Android and Desktop)
// Use RuntimeMaterialCompilerCreator to create this.
class RuntimeMaterialCompiler {
 public:
  using Platform = schemas::Platform;
  using TargetApi = schemas::TargetApi;

  RuntimeMaterialCompiler(BaseView& view,
                          std::unique_ptr<MaterialCompilerClient> native_client,
                          std::unique_ptr<MaterialCache> cache)
      : native_client_(std::move(native_client)),
        view_(view),
        cache_(std::move(cache)) {}

  virtual ~RuntimeMaterialCompiler() = default;

  // Compiles the given source material (.mat) and returns a runtime Filament
  // Material representation. Expects the material input to be already inlined.
  Future<filament::Material*> CompileMaterial(
      absl::string_view source_material_string,
      const MaterialCompilerConfig& config,
      const MaterialPreCompileOptions& material_precompile_options =
          MaterialAsset::kDefaultPreCompileOptions);

  // Compiles the given source material (.mat) and returns the raw compiled
  // bytes. Expects the material input to be already inlined.
  Future<std::vector<uint8_t>> CompileMaterialToBytes(
      absl::string_view source_material_string, Platform platform,
      TargetApi target_api);

 protected:
  std::unique_ptr<MaterialCompilerClient> native_client_;

 private:
  Future<std::vector<uint8_t>> CompileMaterialInternal(
      absl::string_view source_material_string,
      const MaterialCompilerConfig& config);

  BaseView& view_;
  std::unique_ptr<MaterialCache> cache_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALCOMPILER_RUNTIME_MATERIAL_COMPILER_H_
