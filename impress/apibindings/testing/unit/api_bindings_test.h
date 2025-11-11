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

#include "apibindings/asset_ptr_map.h"
#include "apibindings/generic_material_manager.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/model_manager.h"
#include "apibindings/skybox_manager.h"
#include "apibindings/stereo_surface_manager.h"
#include "apibindings/texture_manager.h"
#include "apibindings/water_material_manager.h"
#include "testing/view_fixture.h"

namespace imp {

namespace testing {

// Base class for testing the Impress API bindings.
class ApiBindingsTest
    : public imp::testing::GenericViewFixture<imp::ImpressApiView> {
 protected:
  ApiBindingsTest() {
    imp::testing::GenericViewFixture<imp::ImpressApiView>::SetUp();
    this->view_->SetupImpressApiNative();
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
};

}  // namespace testing

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_UNIT_API_BINDINGS_TEST_H_
