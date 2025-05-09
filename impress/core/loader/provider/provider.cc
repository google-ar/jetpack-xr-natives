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

#include "core/loader/provider/provider.h"

#include <iomanip>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/platform_helpers.h"
#include "core/common/zip_helpers.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/details/gltf_provider.h"
#include "core/loader/provider/details/loaded_zip.h"
#include "core/loader/provider/details/provider_constants.h"
#include "core/loader/provider/details/usdz_provider.h"
#include "core/loader/provider/gltf/provider_gltf.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader {
namespace {

// TODO Move/clone this code into SceneViewer.
bool CanLoadExtension(absl::string_view extension) {
  return (extension == details::kGltfJsonExtension ||
          extension == details::kGltfBundleExtension ||
          extension == details::kUsdzExtension);
}

}  // namespace

Provider::~Provider() {}

absl::StatusOr<std::unique_ptr<Provider>> Provider::Create(
    absl::string_view path, BufferAccess &&access, LoaderOptions options,
    std::unique_ptr<details::GltfProvider> gltf_provider,
    std::unique_ptr<details::UsdzProvider> usdz_provider) {
  if (access.Empty()) {
    return Error("no access");
  }

  if (!gltf_provider) gltf_provider = CreateDefaultGltfProvider();

  auto directory = GetDirectoryFromFilename(path);
  auto basename = RemoveDirectoryAndExtensionFromFilename(path);
  auto extension = GetExtensionFromFilename(GetLocalFilenameFromFilename(path));

  if (extension.empty()) {
    IMP_LOG(imp::INFO) << "Loader cannot determine type for path \""
              << std::setw(static_cast<int>(path.size())) << path.data()
              << "\". treating as gltf.";
    extension = details::kGltfJsonExtension;
  }

  if (extension == details::kZipArchiveExtension) {
    std::vector<std::string> filenames;
    MP_RETURN_IF_ERROR(GetFilenamesFromZip(access, &filenames));
    auto it = absl::c_find_if(filenames, [](const std::string &filename) {
      return CanLoadExtension(GetExtensionFromFilename(filename));
    });
    if (it == filenames.end()) {
      return Error("None of the %d files in %.*s%.*s were loadable",
                   filenames.size(), static_cast<int>(basename.size()),
                   basename.data(), static_cast<int>(extension.size()),
                   extension.data());
    }

    const std::string &sub_path = *it;
    BufferAccess sub_access;
    MP_RETURN_IF_ERROR(GetFileFromZip(access, sub_path, &sub_access));
    IMP_LOG(imp::INFO) << "Attempting to load '" << it->c_str() << " ' from archive";
    MP_ASSIGN_OR_RETURN(
        std::unique_ptr<Provider> result,
        Create(sub_path, std::move(sub_access), std::move(options),
               std::move(gltf_provider), std::move(usdz_provider)));
    result->loaded_zip_ = absl::WrapUnique(
        new details::LoadedZip{std::move(access), std::move(filenames)});
    return result;
  } else if (!CanLoadExtension(extension)) {
    return Error("Loader doesn't know how to process a '%.*s' file",
                 static_cast<int>(extension.size()), extension.data());
  }
  BufferAccess primary_resource =
      BufferAccess::Wrap(access.Data(), access.Size());

  // Using `new` to access a non-public constructor.
  std::unique_ptr<Provider> result = absl::WrapUnique(new Provider(
      directory, basename, extension, std::move(primary_resource),
      std::move(options), std::move(gltf_provider), std::move(usdz_provider)));
  // Save off the resource (with it's original path) in our map.
  result->AddResource(path, std::move(access));
  return result;
}

std::unique_ptr<details::GltfProvider> Provider::CreateDefaultGltfProvider() {
  return details::provider_gltf::CreateGltfProvider();
}

std::unique_ptr<details::UsdzProvider> Provider::CreateDefaultUsdzProvider() {
  // TODO: optional implementation landing in subsequent CLs
  return {};
}

void Provider::AddResource(absl::string_view name, BufferAccess &&access) {
  state_->AddResource(name, std::move(access));
}

OptionalError Provider::AddMissingResource(absl::string_view path,
                                           BufferAccess &&access) {
  auto it = state_->missing_resource_name_from_path_.find(std::string{path});
  if (it == state_->missing_resource_name_from_path_.end()) {
    return Error("Unknown resource!");
  }
  AddResource(it->second, std::move(access));
  state_->missing_resource_name_from_path_.erase(it);
  return NoError();
}

bool Provider::Loaded() const { return loaded_model_.has_value(); }

OptionalError Provider::Load() {
  bool complete = false;
  while (!complete) {
    std::vector<std::string> missing_paths;
    auto error = TryLoad(&missing_paths, &complete);
    if (!error.ok() && !missing_paths.empty()) {
      for (auto &missing_path : missing_paths) {
        BufferAccess access;
        MP_RETURN_IF_ERROR(LoadBinary(missing_path, &access))
            << "Resolving missing dependencies";
        MP_RETURN_IF_ERROR(AddMissingResource(missing_path, std::move(access)));
      }
      continue;
    } else if (!error.ok()) {
      return error;
    }
  }
  assert(Loaded());
  return NoError();
}

OptionalError Provider::TryLoad(
    std::vector<std::string> *out_missing_resource_paths, bool *out_completed) {
  out_missing_resource_paths->clear();
  *out_completed = false;
  if (Loaded()) {
    *out_completed = true;
    return Error("already loaded");
  }
  if (state_->extension_ == details::kGltfJsonExtension ||
      state_->extension_ == details::kGltfBundleExtension) {
    OptionalError result = NoError();
    for (;;) {
      state_->missing_resource_name_from_path_.clear();
      if (!gltf_provider_->IsParsed()) {
        result = gltf_provider_->TryParseGltf(state_.get());

        // If we successfully parse but still have pending resources, mark us
        // with an error so we will retry.
        if (result.ok() && gltf_provider_->HasPendingResources(state_.get())) {
          result = Error("Missing resources");
        }
      }
      if (gltf_provider_->IsParsed() && result.ok()) {
        auto load_result = gltf_provider_->TryLoadGltf(state_.get());
        if (!load_result.ok()) {
          result = load_result.status();
        } else {
          loaded_model_.emplace(std::move(load_result.value()));
        }
      }

      // If we're being loaded from an archive, fill in missing resources.
      if (loaded_zip_ && !result.ok() &&
          !state_->missing_resource_name_from_path_.empty()) {
        MP_RETURN_IF_ERROR(
            details::TryAddMissingResources(loaded_zip_.get(), state_.get()));
        result = NoError();
        continue;  // Retry if adding missing resources didn't return errors.
      }
      for (const auto &it : state_->missing_resource_name_from_path_) {
        out_missing_resource_paths->push_back(it.first);
      }
      break;
    }
    MP_RETURN_IF_ERROR(result);
    if (!loaded_model_) return Error("Internal error");
    *out_completed = true;
    return NoError();
  } else if (state_->extension_ == details::kUsdzExtension) {
    if (!usdz_provider_) return Error("Missing USDZ provider");
    MP_RETURN_IF_ERROR(usdz_provider_->TryParseUsdz(state_.get()));
    if (!usdz_provider_->IsParsed()) {
      return Error("Failed to parse");
    }
    auto result = usdz_provider_->TryLoadUsdz(state_.get());
    if (!result.ok()) return result.status();
    loaded_model_.emplace(std::move(result.value()));
    *out_completed = true;
    return NoError();
  } else {
    return Error("Unknown extension '%.*s' - cannot load",
                 static_cast<int>(state_->extension_.size()),
                 state_->extension_.data());
  }
}

OptionalError Provider::GetLoadedModel(
    FlatBufferAccess<schemas::LoadedModel> *out_loaded_model) {
  if (loaded_model_.has_value()) {
    *out_loaded_model = *std::move(loaded_model_);
    return absl::OkStatus();
  }
  return NoError();
}

Provider::Provider(absl::string_view directory, absl::string_view basename,
                   absl::string_view extension, BufferAccess &&primary_resource,
                   LoaderOptions options,
                   std::unique_ptr<details::GltfProvider> gltf_provider,
                   std::unique_ptr<details::UsdzProvider> usdz_provider)
    : state_(std::make_unique<details::LoaderState>(
          directory, basename, extension, std::move(primary_resource),
          std::move(options))),
      gltf_provider_(std::move(gltf_provider)),
      usdz_provider_(std::move(usdz_provider)) {}

const details::provider_gltf::ParsedGltf *Provider::GetParsedGltf() const {
  return gltf_provider_->GetParsedGltf();
}

}  // namespace imp::loader
