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

#include "core/loader/loader_in_sandbox.h"

#include <cassert>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/animation/gltf_animation.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/resource_helpers.h"
#include "core/loader/creator/creator.h"
#include "core/loader/data/embedded_placeholder_textures.h"
#include "core/loader/ipc/java_loader_client_jni.h"
#include "core/loader/ipc/loader_client.h"
#include "core/loader/ipc/loader_client_base.h"
#include "core/loader/loader.h"
#include "core/loader/loader_creator.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/extensions/verification.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/material_package.h"
#include "core/model/model_data.h"
#include "core/view/base_view.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader {

using ::imp::loader::details::Creator;
using ::imp::model::ModelData;
using IsolatedProcessClient = std::unique_ptr<ipc::JavaLoaderClient>;

class LoaderInSandbox : public Loader {
 public:
  // Note: owned_material_package is for backwards compatibility, see
  // explanation below.
  LoaderInSandbox(BaseView &view, absl::string_view path,
                  std::unique_ptr<ipc::LoaderClient> loader_client,
                  MaterialPackage *material_package,
                  std::unique_ptr<ipc::JavaLoaderClient> java_loader_service,
                  std::unique_ptr<MaterialPackage> owned_material_package,
                  LoaderOptions options = LoaderOptions());

  // Add the contents of a requested resource.  'path' is a context-free (i.e.
  // not asset-relative) path returned as the out-param of TryLoad.  Passing in
  // other resources will return an error.
  absl::Status AddMissingResource(absl::string_view path,
                                  BufferAccess &&access) override;

  // Add the contents of a resource via asset-relative path (i.e. "./foo.bin").
  // This method can be called before TryLoad().
  absl::Status AddResource(absl::string_view path,
                           BufferAccess &&access) override;

  // Client check to see if the asset is loaded.
  bool Loaded() const override;
  bool IsFullyLoaded() const override;
  // Block until a loader's resources are fully processed
  absl::Status Flush(filament::Engine *engine) override;
  // Fire-and-forget load mechanism.  Will fail to resolve missing resources in
  // contexts without filesystem access.
  Future<absl::Status> Load(std::function<void()>&& callback) override;
  // Iterative load mechanism.  Will attempt to load given the currently loaded
  // resources; any missing assets which are required to load will have their
  // paths appear in out_missing_resource_paths.
  Future<absl::Status> TryLoad(
      std::vector<std::string>* out_missing_resource_paths,
      std::function<void()>&& callback) override;

  // Instantiation.
  Future<std::unique_ptr<model::ModelData>> CreateModel(
      filament::Engine* engine) override;
  Future<std::unique_ptr<model::ModelData>> CreateModel(
      filament::Engine* engine, std::function<void()>&& callback) override;
  void WhenFullyLoaded(std::function<void()> &&callback) override;
  void RemoveWhenFullyLoadedCallback() override;
  absl::StatusOr<std::unique_ptr<animation::GltfAnimation>> CreateAnimation(
      size_t animation_index) override;
  absl::StatusOr<std::unique_ptr<animation::GltfAnimation>> CreateAnimation(
      absl::string_view name) override;

  // Queries.
  std::vector<absl::string_view> GetAnimationNames() const override;

  absl::string_view GetName() const override;

 private:
  absl::Status CreateCreator();

  BaseView &view_;
  Context context_;
  std::string name_;
  std::unique_ptr<ipc::LoaderClient> loader_client_;
  // A reference for maintaining the lifetime of the java side loader client.
  std::unique_ptr<ipc::JavaLoaderClient> java_loader_service_;
  std::unique_ptr<details::Creator> creator_;
  MaterialPackage *material_package_;

  // This is for backwards compatibility with the old sceneform loader!
  // Normally, the material_package is owned externally so that materials can be
  // re-used between glTF models.
  // For backwards compatibility with loader_sandboxed_jni.cc, we have the
  // ability to have the Loader itself own the MaterialPackage.
  std::unique_ptr<MaterialPackage> owned_material_package_;

  LoaderOptions loader_options_;
};

LoaderInSandbox::LoaderInSandbox(
    BaseView &view, absl::string_view path,
    std::unique_ptr<ipc::LoaderClient> loader_client,
    MaterialPackage *material_package,
    std::unique_ptr<ipc::JavaLoaderClient> java_loader_service,
    std::unique_ptr<MaterialPackage> owned_material_package,
    LoaderOptions options)
    : view_(view),
      context_(view_.GetContext()),
      name_(GetLocalFilenameFromFilename(path)),
      loader_client_(std::move(loader_client)),
      java_loader_service_(std::move(java_loader_service)),
      material_package_(material_package),
      owned_material_package_(std::move(owned_material_package)),
      loader_options_(std::move(options)) {}

Future<std::unique_ptr<model::ModelData>> LoaderInSandbox::CreateModel(
    filament::Engine* engine) {
  if (!Loaded()) {
    return Future<std::unique_ptr<model::ModelData>>(
        absl::FailedPreconditionError("Not Loaded"));
  }

  return creator_->CreateModel(engine, loader_options_, name_);
}

Future<std::unique_ptr<model::ModelData>> LoaderInSandbox::CreateModel(
    filament::Engine* engine, std::function<void()>&& callback) {
  Future<std::unique_ptr<model::ModelData>> create_model_future =
      CreateModel(engine);
  creator_->WhenFullyLoaded(std::move(callback));
  return create_model_future;
}

void LoaderInSandbox::WhenFullyLoaded(std::function<void()> &&callback) {
  creator_->WhenFullyLoaded(std::move(callback));
}

void LoaderInSandbox::RemoveWhenFullyLoadedCallback() {
  if (creator_) {
    creator_->RemoveWhenFullyLoadedCallback();
  }
}

absl::StatusOr<std::unique_ptr<animation::GltfAnimation>>
LoaderInSandbox::CreateAnimation(size_t animation_index) {
  if (!creator_) {
    return absl::InternalError(
        "Tried to create animation without anything loaded");
  }

  return creator_->CreateAnimation(animation_index);
}

absl::StatusOr<std::unique_ptr<animation::GltfAnimation>>
LoaderInSandbox::CreateAnimation(absl::string_view name) {
  if (!creator_) {
    return absl::InternalError(
        "Tried to create animation without anything loaded");
  }

  return creator_->CreateAnimation(name);
}

std::vector<absl::string_view> LoaderInSandbox::GetAnimationNames() const {
  if (creator_) {
    return creator_->GetAnimationNames();
  }
  return std::vector<absl::string_view>{};
}

absl::string_view LoaderInSandbox::GetName() const { return name_; }

absl::Status LoaderInSandbox::AddResource(absl::string_view path,
                                          BufferAccess &&access) {
  if (Loaded()) {
    return absl::InternalError("Already loaded");
  }

  return loader_client_->AddResource(path, std::move(access),
                                     ipc::ResourceType::Default);
}

absl::Status LoaderInSandbox::AddMissingResource(absl::string_view path,
                                                 BufferAccess &&access) {
  if (Loaded()) {
    return absl::InternalError("Already loaded");
  }

  return loader_client_->AddResource(path, std::move(access),
                                     ipc::ResourceType::Missing);
}

bool LoaderInSandbox::Loaded() const { return creator_ != nullptr; }

bool LoaderInSandbox::IsFullyLoaded() const {
  return creator_ != nullptr ? creator_->IsFullyLoaded() : false;
}

absl::Status LoaderInSandbox::Flush(filament::Engine *engine) {
  if (creator_ == nullptr) return absl::InternalError("No creator");
  return creator_->BlockUntilLoaded(engine);
}

Future<absl::Status> LoaderInSandbox::Load(std::function<void()>&& callback) {
  if (Loaded()) {
    return Future<absl::Status>(absl::InternalError("Already loaded"));
  }

  std::vector<std::string> missing_resource_paths;
  bool loaded;
  absl::Status loader_client_status =
      loader_client_->TryLoad(&missing_resource_paths, &loaded);
  if (!loader_client_status.ok()) {
    return Future<absl::Status>(loader_client_status);
  }

  if (!missing_resource_paths.empty()) {
    return Future<absl::Status>(absl::InternalError(absl::StrFormat(
        "Resource missing: '%s'", missing_resource_paths.front().c_str())));
  }

  assert(loaded);
  absl::Status creator_status = CreateCreator();
  if (!creator_status.ok()) {
    return Future<absl::Status>(creator_status);
  }

  return creator_->LoadImages(context_, std::move(callback));
}

Future<absl::Status> LoaderInSandbox::TryLoad(
    std::vector<std::string>* out_missing_resource_paths,
    std::function<void()>&& callback) {
  out_missing_resource_paths->clear();
  if (Loaded()) {
    return Future<absl::Status>(absl::InternalError("Already loaded"));
  }

  bool loaded;
  absl::Status loader_client_status =
      loader_client_->TryLoad(out_missing_resource_paths, &loaded);
  if (!loader_client_status.ok()) {
    return Future<absl::Status>(loader_client_status);
  }

  if (!out_missing_resource_paths->empty()) {
    // TODO: Remove once we can return out_loaded instead of
    // generating an error.
    return Future<absl::Status>(absl::InternalError(
        absl::StrFormat("Resource missing: '%s'",
                        out_missing_resource_paths->front().c_str())));
  }

  if (loaded) {
    absl::Status creator_status = CreateCreator();
    if (!creator_status.ok()) {
      return Future<absl::Status>(creator_status);
    }
    return creator_->LoadImages(context_, std::move(callback));
  }

  return Future<absl::Status>(absl::OkStatus());
}

absl::Status LoaderInSandbox::CreateCreator() {
  FlatBufferAccess<schemas::LoadedModel> model;
  MP_RETURN_IF_ERROR(loader_client_->GetLoadedModel(&model));
  MP_RETURN_IF_ERROR(optional_features::VerifyModelNestedData(model.Root()));

  creator_ =
      std::make_unique<Creator>(view_, material_package_, std::move(model));
  return absl::OkStatus();
}

Future<IsolatedProcessClient> CreateIsolatedProcessClient(
    const Context& context) {
  // Instantiates the java side loader that will launch the Android isolated
  // process.
  auto isolated_process_client =
      std::make_unique<loader::ipc::JavaLoaderClient>(
          context.GetJniEnv(), context.GetActivityContext());

  return Future<IsolatedProcessClient>::Schedule(
      [&context,
       isolated_process_client = std::move(isolated_process_client)]() mutable
          -> absl::StatusOr<IsolatedProcessClient> {
        // Launches the isolated process and connects to it.
        if (!isolated_process_client->ConnectToLoaderService(context)) {
          return absl::InternalError("Unable to connect to loader service.");
        }

        return std::move(isolated_process_client);
      },
      {.executor = Executor::Type::kBackground});
}

absl::StatusOr<std::unique_ptr<Loader>> CreateSandboxLoader(
    BaseView &view, int fd, absl::string_view path, BufferAccess access,
    MaterialPackage *material_package, LoaderOptions options,
    IsolatedProcessClient java_loader_client = nullptr,
    std::unique_ptr<MaterialPackage> owned_material_package = nullptr) {
  RegisterPackagedResources(embedded_placeholder_textures_create());

  if (java_loader_client) {
    // Gets the client socket ID of the client/server relationship.
    fd = java_loader_client->GetClientSocketFileDescriptor();
    if (fd == 0) {
      return absl::InternalError(
          "Error creating and connecting to the isolated loader "
          "process.");
    }
  }

  auto loader_client = std::make_unique<ipc::LoaderClient>(fd);

  if (java_loader_client) {
    java_loader_client->SetNativeHandler(loader_client.get());
  }

  MP_RETURN_IF_ERROR(loader_client->Start(path, std::move(access), options));

  return std::make_unique<LoaderInSandbox>(
      view, path, std::move(loader_client), material_package,
      std::move(java_loader_client), std::move(owned_material_package),
      options);
}

absl::StatusOr<std::unique_ptr<Loader>> LoaderInSandboxCreator::Create(
    BaseView &view, int fd, absl::string_view path, BufferAccess access,
    MaterialPackage *material_package, LoaderOptions options) {
  return CreateSandboxLoader(view, fd, path, std::move(access),
                             material_package, options, nullptr);
}

absl::StatusOr<std::unique_ptr<Loader>>
LoaderInSandboxCreator::CreateWithOwnedMaterialPackage(
    BaseView &view, int fd, absl::string_view path, BufferAccess access,
    MaterialPackage *material_package, LoaderOptions options,
    std::unique_ptr<MaterialPackage> owned_material_package) {
  return CreateSandboxLoader(view, fd, path, std::move(access),
                             material_package, options, nullptr,
                             std::move(owned_material_package));
}

Future<GetLoaderFn> LoaderInSandboxCreator::Create(BaseView& view) {
  return CreateIsolatedProcessClient(view.GetContext())
      .Then(
          [](IsolatedProcessClient java_loader_client) -> GetLoaderFn {
            return [java_loader_client = std::move(java_loader_client)](
                       BaseView &view, absl::string_view path,
                       BufferAccess access, MaterialPackage *material_package,
                       LoaderOptions options) mutable
                   -> absl::StatusOr<std::unique_ptr<Loader>> {
              return CreateSandboxLoader(view, 0, path, std::move(access),
                                         material_package, options,
                                         std::move(java_loader_client));
            };
          });
}

}  // namespace imp::loader
