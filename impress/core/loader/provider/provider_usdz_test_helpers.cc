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

#include "core/loader/provider/provider_usdz_test_helpers.h"

#include <string>
#include <utility>
#include <vector>

#include "devtools/build/runtime/get_runfiles_dir.h"
#include "gmock/gmock.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/resource_helpers.h"
#include "core/common/test_helpers.h"
#include "core/loader/data/embedded_imp_default_gltf_materials.h"
#include "core/loader/data/embedded_placeholder_textures.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/provider.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/loader/provider/usdz/provider_usdz.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details {

ProviderUsdzTestHelpers::ProviderUsdzTestHelpers() {
  // Resource setup.
  RegisterPackagedResources(embedded_imp_default_gltf_materials_create());
  RegisterPackagedResources(embedded_placeholder_textures_create());
}

std::string ProviderUsdzTestHelpers::GetSampleAssetsDirectory() {
  const auto sample_assets_directory =
      devtools_build::testonly::GetTestSrcdir() +
      "/google3/third_party/arcore/googledata/sceneform/assets/";
  return sample_assets_directory;
}

std::string ProviderUsdzTestHelpers::SampleAsset(
    absl::string_view friendly_path) {
  return absl::StrCat(GetSampleAssetsDirectory(), friendly_path);
}

OptionalError ProviderUsdzTestHelpers::CreateProvider(
    absl::string_view friendly_path) {
  return CreateProvider(friendly_path, {});
}

OptionalError ProviderUsdzTestHelpers::CreateProvider(
    absl::string_view friendly_path, LoaderOptions options) {
  auto path = SampleAsset(friendly_path);
  BufferAccess access;
  MP_RETURN_IF_ERROR(LoadBinary(path, &access));
  MP_ASSIGN_OR_RETURN(
      provider_,
      Provider::Create(path, std::move(access), options,
                       Provider::CreateDefaultGltfProvider(),
                       details::provider_usdz::CreateUsdzProvider()));
  return NoError();
}

const schemas::LoadedModel* ProviderUsdzTestHelpers::GetLoadedModel() {
  if (auto error = provider_->GetLoadedModel(&loaded_model_); !error.ok()) {
    return nullptr;
  }
  return *loaded_model_;
}

void ProviderUsdzTestHelpers::AddMissingResources(
    const std::vector<std::string>& missing_resources, Provider* provider) {
  for (auto& missing_resource : missing_resources) {
    BufferAccess access;
    EXPECT_THAT(LoadBinary(missing_resource, &access), IsNoError());
    EXPECT_THAT(
        provider->AddMissingResource(missing_resource, std::move(access)),
        IsNoError());
  }
}

OptionalError ProviderUsdzTestHelpers::LoadWithResources(
    absl::string_view friendly_path) {
  EXPECT_THAT(CreateProvider(std::string(friendly_path)), IsNoError());
  std::vector<std::string> missing_resources;
  bool complete;
  OptionalError error = provider_->TryLoad(&missing_resources, &complete);
  if (complete) {
    return error;
  }

  // Add the missing resources.
  AddMissingResources(missing_resources, provider_.get());
  return provider_->TryLoad(&missing_resources, &complete);
}

}  // namespace imp::loader::details
