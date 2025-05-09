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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_IN_PROCESS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_IN_PROCESS_H_

#include <memory>

#include "core/common/context.h"
#include "core/loader/loader.h"
#include "core/material_library/material_package.h"
#include "core/view/base_view.h"

namespace imp::loader {

// Create an in-process loader, with a caller-provided material package.
absl::StatusOr<std::unique_ptr<Loader>> CreateLoaderInProcess(
    BaseView& view, absl::string_view path, BufferAccess access,
    MaterialPackage* material_package, LoaderOptions options);

// owned_material_package is for backwards compatibility with the old
// sceneform loader! Normally, the material_package is owned externally so that
// materials can be re-used between glTF models. For backwards compatibility
// with loader_in_process_embedded, we have the ability to have the Loader
// itself own the MaterialPackage.
absl::StatusOr<std::unique_ptr<Loader>>
CreateLoaderInProcessWithOwnedMaterialPackage(
    BaseView& view, absl::string_view path, BufferAccess access,
    MaterialPackage* material_package, LoaderOptions options,
    std::unique_ptr<MaterialPackage> owned_material_package);

}  // namespace imp::loader

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_IN_PROCESS_H_
