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
#include "core/materials/compiler/runtime_material_compiler.h"

#include <sys/socket.h>
#include <sys/types.h>

#include <cstdint>
#include <string>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Material.h"
#include "flatbuffers/vector.h"
#include "core/assets/material/material_asset.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/materials/compiler/cache/material_cache.h"
#include "core/materials/compiler/material_compiler_client.h"
#include "core/materials/compiler/schemas/material_compiler_ipc_generated.h"
#include "core/view/base_view.h"

namespace imp {

Future<filament::Material*> RuntimeMaterialCompiler::CompileMaterial(
    absl::string_view source_material_string, Platform platform,
    TargetApi target_api,
    const MaterialPreCompileOptions& material_precompile_options) {
  // Check if material exists in the cache.
  MaterialHash hash = cache_->Hash(source_material_string);
  absl::StatusOr<std::vector<uint8_t>> cached_cmat = cache_->Get(hash);
  if (cached_cmat.ok()) {
    return Future<filament::Material*>(MaterialAsset::BuildMaterial(
        *(view_.GetSharedEngine()), cached_cmat->data(), cached_cmat->size(),
        material_precompile_options));
  }

  return Future<FlatBufferAccess<const schemas::CompileResponse>>::Schedule(
             // Copy the source material string to a std::string since the
             // lambda may outlives the memory under the source_material_string
             // parameter.
             [this, source = std::string(source_material_string), platform,
              target_api]() {
               return native_client_->CompileMaterial(source, platform,
                                                      target_api);
             },
             {.executor = Executor::Type::kBackground})
      .Then(
          // Creating a Filament Material (runtime representation) in the
          // foreground, since we need the filament Engine.
          [this, hash, material_precompile_options](
              FlatBufferAccess<const schemas::CompileResponse> response)
              -> absl::StatusOr<filament::Material*> {
            const flatbuffers::Vector<uint8_t>* compiled_material =
                response->compiled_material();

            if (compiled_material->empty()) {
              return absl::InternalError("Compiled material is empty");
            }
            if (compiled_material->data() == nullptr) {
              return absl::InternalError("Compiled material data is null");
            }

            if (absl::Status status = cache_->Store(
                    hash, std::vector<uint8_t>(compiled_material->begin(),
                                               compiled_material->end()));
                !status.ok()) {
              // Do not return an error here, since the material compiled and
              // can be used, even if it wasn't cached.
              IMP_LOG(imp::ERROR) << "Failed to store material in cache: " << status;
            }

            return MaterialAsset::BuildMaterial(
                *(view_.GetSharedEngine()), compiled_material->data(),
                compiled_material->size(), material_precompile_options);
          },
          {.executor = Executor::Type::kForeground});
}

}  // namespace imp
