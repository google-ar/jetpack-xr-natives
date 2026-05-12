// Copyright 2025 Google LLC
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

#include "apibindings/testing/marshalling/test_impress_api_view.h"

#include <memory>

#include "apibindings/asset_ptr_map.h"
#include "apibindings/testing/marshalling/test_generic_material_manager.h"
#include "apibindings/testing/marshalling/test_model_manager.h"
#include "apibindings/testing/marshalling/test_node_manager.h"
#include "apibindings/testing/marshalling/test_skybox_manager.h"
#include "apibindings/testing/marshalling/test_stereo_surface_manager.h"
#include "apibindings/testing/marshalling/test_texture_manager.h"
#include "apibindings/testing/marshalling/test_water_material_manager.h"

namespace imp {

TestImpressApiView::TestImpressApiView() {
  asset_ptr_map_ = std::make_unique<AssetPtrMap>(*this);
  model_manager_ = std::make_unique<TestModelManager>(*this);
  skybox_manager_ = std::make_unique<TestSkyboxManager>(*this);
  stereo_surface_manager_ = std::make_unique<TestStereoSurfaceManager>(*this);
  texture_manager_ = std::make_unique<TestTextureManager>(*this);
  water_material_manager_ = std::make_unique<TestWaterMaterialManager>(*this);
  generic_material_manager_ =
      std::make_unique<TestGenericMaterialManager>(*this);
  node_manager_ = std::make_unique<TestNodeManager>(*this);
}

// Prevents the base class from overwriting our test managers with real ones.
void TestImpressApiView::SetupImpressApiNative() {}

}  // namespace imp
