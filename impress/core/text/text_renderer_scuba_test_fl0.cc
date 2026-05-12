/*
 * Copyright 2026 Google LLC
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

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/text/text_renderer.h"
#include "core/text/text_renderer_assets.h"
#include "core/text/text_renderer_state.proto.imp.h"
#include "testing/base_scuba_view_fixture.h"
#include "testing/fl0_scuba_view_fixture.h"

namespace imp {
namespace {

using ::imp::testing::FeatureLevelZeroTestFixture;

class FL0TextTestFixutre : public FeatureLevelZeroTestFixture {
 public:
  FL0TextTestFixutre()
      : FeatureLevelZeroTestFixture(
            "third_party/impress/core/text/scuba_goldens") {}
};

TEST_F(FL0TextTestFixutre, CanRenderText) {
  imp::NodeHandle text_node = GetView()->CreateNode();
  text_node->SetLocalPosition({-1.0f, 0.0f, -6.0f});

  imp::TextRendererState state;
  state.text = "Hello, World!";
  state.material = imp::text_renderer_assets::kTextMaterialFl0Cmat.GetUrl();

  MP_ASSERT_OK(
      GetFuture(text_node->AddComponentWithState<imp::TextRenderer>(state)));

  EXPECT_THAT(
      RenderAndDiffGolden("TextRendererScubaTest_CanRenderTextInFL0.png"),
      testing::GoldenPassed());
}

}  // namespace
}  // namespace imp
