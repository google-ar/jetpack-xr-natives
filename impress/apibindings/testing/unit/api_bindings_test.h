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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_UNIT_API_BINDINGS_TEST_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_UNIT_API_BINDINGS_TEST_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <utility>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "apibindings/asset_ptr_map.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/generic_material_manager.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/model_manager.h"
#include "apibindings/skybox_manager.h"
#include "apibindings/stereo_surface_manager.h"
#include "apibindings/testing/unit/mock_asset_loader.h"
#include "apibindings/texture_manager.h"
#include "apibindings/water_material_manager.h"
#include "core/common/invocable.h"
#include "core/view/framework/assets/tests/test_resources.h"
#include "core/view/utils/asset.h"
#include "core/window/default_lighting.h"
#include "testing/view_fixture.h"

namespace imp {

namespace testing {

using ::testing::_;
using ::testing::DoAll;
using ::testing::SaveArg;

// A helper struct to hold the asset loading state for a single load operation.
struct MockLoaderContext {
  // The mock of the AssetLoader.
  imp::testing::MockAssetLoader& mock;

  // The token captured from the load operation.
  std::intptr_t token = 0;

  // Allows to trigger the actual load when the test is ready.
  imp::Invocable<void()> load_fn;
};

// Base class for testing the Impress API bindings.
class ApiBindingsTest
    : public imp::testing::GenericViewFixture<imp::ImpressApiView> {
 protected:
  ApiBindingsTest() {
    imp::testing::GenericViewFixture<imp::ImpressApiView>::SetUp();
    this->view_->SetSplitEngineMaterialLocalMode(true);
    this->view_->SetupImpressApiNative();
  }

  void TearDown() override {
    EXPECT_TRUE(this->view_->DisposeAllResources().ok());
    imp::testing::GenericViewFixture<imp::ImpressApiView>::TearDown();
  }

  // Returns the real ModelManager for testing purposes.
  ModelManager& GetModelManager() { return this->view_->GetModelManager(); }

  // Returns the real SkyboxManager for testing purposes.
  SkyboxManager& GetSkyboxManager() { return this->view_->GetSkyboxManager(); }

  // Returns the real TextureManager for testing purposes.
  TextureManager& GetTextureManager() {
    return this->view_->GetTextureManager();
  }

  // Returns the real WaterMaterialManager for testing purposes.
  WaterMaterialManager& GetWaterMaterialManager() {
    return this->view_->GetWaterMaterialManager();
  }

  // Returns the real GenericMaterialManager for testing purposes.
  GenericMaterialManager& GetGenericMaterialManager() {
    return this->view_->GetGenericMaterialManager();
  }

  // Returns the real StereoSurfaceManager for testing purposes.
  StereoSurfaceManager& GetStereoSurfaceManager() {
    return this->view_->GetStereoSurfaceManager();
  }

  // Returns the real AssetPtrMap for testing purposes.
  AssetPtrMap& GetAssetPtrMap() { return this->view_->GetAssetPtrMap(); }

  // Creates a mock loader for a GLTF asset.
  MockLoaderContext CreateGltfLoader() {
    // We use the asset path directly inside the lambda now.
    return CreateLoaderInternal(
        [this](std::unique_ptr<BaseAssetLoader> loader) {
          GetAssetPtrMap().LoadGltfAsset(test_data::kAnimatedMorphCubeGltf,
                                         std::move(loader));
        });
  }

  // Creates a mock loader for an IBL asset.
  MockLoaderContext CreateIblLoader() {
    return CreateLoaderInternal(
        [this](std::unique_ptr<BaseAssetLoader> loader) {
          GetAssetPtrMap().LoadImageBasedLightingAsset(kDefaultIblZip,
                                                       std::move(loader));
        });
  }

  // Creates a mock loader for a texture asset.
  MockLoaderContext CreateTextureLoader() {
    return CreateLoaderInternal(
        [this](std::unique_ptr<BaseAssetLoader> loader) {
          GetTextureManager().LoadTexture(test_data::kSampleImagePng,
                                          std::move(loader));
        });
  }

  // Creates a mock loader for a water material.
  MockLoaderContext CreateWaterMaterialLoader() {
    return CreateLoaderInternal(
        [this](std::unique_ptr<BaseAssetLoader> loader) {
          GetWaterMaterialManager().CreateWaterMaterial(
              std::move(loader),
              /*is_alpha_map_version=*/false);
        });
  }

 private:
  using LoadFunc = std::function<void(std::unique_ptr<BaseAssetLoader>)>;

  MockLoaderContext CreateLoaderInternal(LoadFunc trigger_fn) {
    auto mock_loader = std::make_unique<imp::testing::MockAssetLoader>();
    imp::testing::MockAssetLoader& mock_ref = *mock_loader;

    auto load_action = [this, trigger_fn,
                        loader = std::move(mock_loader)]() mutable {
      trigger_fn(std::move(loader));
      // Pump the main thread to run the callback.
      this->DrainAllExecutors();
    };

    return MockLoaderContext{
        .mock = mock_ref, .token = 0, .load_fn = std::move(load_action)};
  }
};

}  // namespace testing

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_UNIT_API_BINDINGS_TEST_H_
