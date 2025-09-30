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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_IN_SANDBOX_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_IN_SANDBOX_H_

#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/loader/loader.h"
#include "core/loader/loader_creator.h"
#include "core/loader/loader_options.h"
#include "core/material_library/material_package.h"
#include "core/view/base_view.h"

namespace imp::loader {

// An implementation of a GLTF loader that runs in a sandboxed process.
// This allows for loading GLTFs from insecure (third-party) sources.
// If your project needs to load untrusted GLTFs (on Android), you need to make
// this call:
// #if IMP_PLATFORM(ANDROID)
// imp view->GetAssetManager().SetSandboxedGltfLoader(
//     std::make_unique<LoaderInSandboxCreator>());
// #endif
struct LoaderInSandboxCreator : public LoaderCreator {
  // Creates a sandbox loader assuming the ipc client has already been created.
  // The fd parameter determines which socket to use when interacting with
  // the ipc client.
  absl::StatusOr<std::unique_ptr<Loader>> Create(
      BaseView& view, int fd, absl::string_view path, BufferAccess access,
      MaterialPackage* material_package, LoaderOptions options);

  // owned_material_package is for backwards compatibility with the old
  // sceneform loader! Normally, the material_package is owned externally so
  // that materials can be re-used between glTF models. For backwards
  // compatibility with loader_sandboxed_jni.cc, we have the ability to have
  // the Loader itself own the MaterialPackage.
  absl::StatusOr<std::unique_ptr<Loader>> CreateWithOwnedMaterialPackage(
      BaseView& view, int fd, absl::string_view path, BufferAccess access,
      MaterialPackage* material_package, LoaderOptions options,
      std::unique_ptr<MaterialPackage> owned_material_package);

  // Starts an isolated process asynchronously (if it hasn't already been
  // started) and returns a function to create a sandbox loader within that
  // process. The reason this is split across a future function and a create
  // method is so the creation of the isolated process can be done in parallel
  // with other work (such as loading an asset).
  Future<GetLoaderFn> Create(BaseView& view) override;
};

}  // namespace imp::loader

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_IN_SANDBOX_H_
