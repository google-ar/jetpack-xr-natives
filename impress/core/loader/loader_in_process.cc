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

#include "core/loader/loader_in_process.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/animation/gltf_animation.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/resource_helpers.h"
#include "core/loader/creator/creator.h"
#include "core/loader/data/embedded_placeholder_textures.h"
#include "core/loader/loader.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/extensions/verification.h"
#include "core/loader/provider/provider.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/material_package.h"
#include "core/model/model_data.h"
#include "core/view/base_view.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader {

using ::imp::loader::Provider;
using ::imp::loader::details::Creator;
using ::imp::model::ModelData;

// Wrapper class for a loading
class LoaderInProcess : public Loader {
 public:
  // Note: owned_material_package is for backwards compatibility, see
  // explanation below.
  LoaderInProcess(BaseView &view, absl::string_view path,
                  std::unique_ptr<Provider> provider,
                  MaterialPackage *material_package,
                  std::unique_ptr<MaterialPackage> owned_material_package = {},
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
  std::unique_ptr<Provider> provider_;
  std::unique_ptr<Creator> creator_;
  MaterialPackage *material_package_;

  // This is for backwards compatibility with the old sceneform loader!
  // Normally, the material_package is owned externally so that materials can be
  // re-used between glTF models.
  // For backwards compatibility with loader_in_process_embedded, we have the
  // ability to have the Loader itself own the MaterialPackage.
  std::unique_ptr<MaterialPackage> owned_material_package_;

  LoaderOptions loader_options_;
};

absl::StatusOr<std::unique_ptr<Loader>> CreateLoaderInProcess(
    BaseView &view, absl::string_view path, BufferAccess access,
    MaterialPackage *material_package, LoaderOptions options) {
  // Pass in {} as the owned_material_package, which is just for backwards
  // compatibility.
  return CreateLoaderInProcessWithOwnedMaterialPackage(
      view, path, std::move(access), material_package, options, {});
}

absl::StatusOr<std::unique_ptr<Loader>>
CreateLoaderInProcessWithOwnedMaterialPackage(
    BaseView &view, absl::string_view path, BufferAccess access,
    MaterialPackage *material_package, LoaderOptions options,
    std::unique_ptr<MaterialPackage> owned_material_package) {
  RegisterPackagedResources(embedded_placeholder_textures_create());

  MP_ASSIGN_OR_RETURN(std::unique_ptr<Provider> provider,
                   Provider::Create(path.substr(0, path.find('?')),
                                    std::move(access), options));

  return std::make_unique<LoaderInProcess>(
      view, path, std::move(provider), material_package,
      std::move(owned_material_package), std::move(options));
}

LoaderInProcess::LoaderInProcess(
    BaseView &view, absl::string_view path, std::unique_ptr<Provider> provider,
    MaterialPackage *material_package,
    std::unique_ptr<MaterialPackage> owned_material_package,
    LoaderOptions options)
    : view_(view),
      context_(view_.GetContext()),
      name_(GetLocalFilenameFromFilename(path.substr(0, path.find('?')))),
      provider_(std::move(provider)),
      material_package_(material_package),
      owned_material_package_(std::move(owned_material_package)),
      loader_options_(std::move(options)) {}

Future<std::unique_ptr<model::ModelData>> LoaderInProcess::CreateModel(
    filament::Engine* engine) {
  if (!Loaded()) {
    return Future<std::unique_ptr<model::ModelData>>(
        absl::FailedPreconditionError("Not Loaded"));
  }

  return creator_->CreateModel(engine, loader_options_, name_);
}

Future<std::unique_ptr<model::ModelData>> LoaderInProcess::CreateModel(
    filament::Engine* engine, std::function<void()>&& callback) {
  Future<std::unique_ptr<model::ModelData>> create_model_future =
      CreateModel(engine);
  creator_->WhenFullyLoaded(std::move(callback));
  return create_model_future;
}

void LoaderInProcess::WhenFullyLoaded(std::function<void()> &&callback) {
  creator_->WhenFullyLoaded(std::move(callback));
}

void LoaderInProcess::RemoveWhenFullyLoadedCallback() {
  if (creator_) {
    creator_->RemoveWhenFullyLoadedCallback();
  }
}

absl::StatusOr<std::unique_ptr<animation::GltfAnimation>>
LoaderInProcess::CreateAnimation(size_t animation_index) {
  if (!creator_) {
    return absl::InternalError(
        "Tried to create animation without anything loaded");
  }

  return creator_->CreateAnimation(animation_index);
}

absl::StatusOr<std::unique_ptr<animation::GltfAnimation>>
LoaderInProcess::CreateAnimation(absl::string_view name) {
  if (!creator_) {
    return absl::InternalError(
        "Tried to create animation without anything loaded");
  }

  return creator_->CreateAnimation(name);
}

std::vector<absl::string_view> LoaderInProcess::GetAnimationNames() const {
  if (creator_) {
    return creator_->GetAnimationNames();
  }
  return std::vector<absl::string_view>{};
}

absl::string_view LoaderInProcess::GetName() const { return name_; }

absl::Status LoaderInProcess::AddResource(absl::string_view path,
                                          BufferAccess &&access) {
  if (Loaded()) {
    return absl::InternalError("Already loaded");
  }
  provider_->AddResource(path, std::move(access));
  return absl::OkStatus();
}

absl::Status LoaderInProcess::AddMissingResource(absl::string_view path,
                                                 BufferAccess &&access) {
  if (Loaded()) {
    return absl::InternalError("Already loaded");
  }
  return provider_->AddMissingResource(path, std::move(access));
}

bool LoaderInProcess::Loaded() const { return (creator_ != nullptr); }

bool LoaderInProcess::IsFullyLoaded() const {
  return (creator_ != nullptr) ? creator_->IsFullyLoaded() : false;
}

absl::Status LoaderInProcess::Flush(filament::Engine *engine) {
  if (creator_ == nullptr) return Error("No creator");
  return creator_->BlockUntilLoaded(engine);
}

Future<absl::Status> LoaderInProcess::Load(std::function<void()>&& callback) {
  absl::Status provider_status = provider_->Load();
  if (!provider_status.ok()) {
    return Future<absl::Status>(provider_status);
  }
  absl::Status creator_status = CreateCreator();
  if (!creator_status.ok()) {
    return Future<absl::Status>(creator_status);
  }
  return creator_->LoadImages(context_, std::move(callback));
}

Future<absl::Status> LoaderInProcess::TryLoad(
    std::vector<std::string>* out_missing_resource_paths,
    std::function<void()>&& callback) {
  out_missing_resource_paths->clear();
  if (Loaded()) {
    return Future<absl::Status>(absl::InternalError("already loaded"));
  }

  bool complete;
  absl::Status provider_status =
      provider_->TryLoad(out_missing_resource_paths, &complete);
  if (!provider_status.ok()) {
    return Future<absl::Status>(provider_status);
  }

  if (complete) {
    absl::Status creator_status = CreateCreator();
    if (!creator_status.ok()) {
      return Future<absl::Status>(creator_status);
    }
    return creator_->LoadImages(context_, std::move(callback));
  }

  return Future<absl::Status>(absl::OkStatus());
}

absl::Status LoaderInProcess::CreateCreator() {
  FlatBufferAccess<schemas::LoadedModel> model;

  MP_RETURN_IF_ERROR(provider_->GetLoadedModel(&model));
  MP_RETURN_IF_ERROR(optional_features::VerifyModelNestedData(model.Root()));

  creator_ = std::make_unique<Creator>(view_, material_package_,
                                       std::move(model), loader_options_);
  return absl::OkStatus();
}

}  // namespace imp::loader
