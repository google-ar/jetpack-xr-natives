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

#include <memory>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/resource_helpers.h"
#include "core/loader/data/embedded_imp_default_gltf_materials.h"
#include "core/loader/loader.h"
#include "core/loader/loader_in_process.h"
#include "core/loader/loader_options.h"
#include "core/material_library/material_package.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader {

absl::StatusOr<std::unique_ptr<Loader>> CreateLoaderInProcess(
    BaseView& view, absl::string_view path, BufferAccess access,
    LoaderOptions options) {
  RegisterPackagedResources(embedded_imp_default_gltf_materials_create());

  absl::Cord materials_zip_bytes;
  MP_ASSIGN_OR_RETURN(
      materials_zip_bytes,
      PackagedFileToCord("compiled_imp_default_gltf_materials.zip"));

  auto material_package =
      std::make_unique<MaterialPackage>(Future<resources::Resource>(
          resources::Resource(std::move(materials_zip_bytes))));

  return CreateLoaderInProcessWithOwnedMaterialPackage(
      view, path, std::move(access), material_package.get(), std::move(options),
      std::move(material_package));
}

}  // namespace imp::loader
