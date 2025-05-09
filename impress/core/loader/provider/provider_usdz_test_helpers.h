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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_PROVIDER_USDZ_TEST_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_PROVIDER_USDZ_TEST_HELPERS_H_

#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/provider.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp::loader::details {

// Test Fixture to wire private bits of Provider out to tests.
class ProviderUsdzTestHelpers {
 public:
  ProviderUsdzTestHelpers();

 protected:
  std::string GetSampleAssetsDirectory();
  std::string SampleAsset(absl::string_view friendly_path);
  OptionalError CreateProvider(absl::string_view friendly_path);
  OptionalError CreateProvider(absl::string_view friendly_path,
                               LoaderOptions options);
  const schemas::LoadedModel* GetLoadedModel();
  void AddMissingResources(const std::vector<std::string>& missing_resources,
                           Provider* provider);
  OptionalError LoadWithResources(absl::string_view friendly_path);

  std::unique_ptr<Provider> provider_;
  FlatBufferAccess<schemas::LoadedModel> loaded_model_;
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_PROVIDER_USDZ_TEST_HELPERS_H_
