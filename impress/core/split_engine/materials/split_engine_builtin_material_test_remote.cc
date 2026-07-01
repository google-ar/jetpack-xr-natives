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
#include "core/common/trace.h"
#include "core/split_engine/integration_test/split_engine_integration_test_fixture.h"
#include "core/split_engine/integration_test/split_engine_integration_test_params.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/view/base_view.h"
#include "core/view/framework/view.h"
#include "core/view/utils/frame_time.h"

namespace imp::split_engine {
namespace {

// Test suite to validate behavior of SplitEngineBuiltinMaterial in remote mode
// where SplitEngineSerializer is present.

class RendererView : public imp::View {
 public:
  RendererView() {
    // Disable default load to wait for the serializer to become available.
    GetLightManager().DisableDefaultLoad();
  }
};

using SplitEngineBuiltinMaterialRemoteTest =
    SplitEngineIntegrationTestFixture<RendererView>;

TEST_P(SplitEngineBuiltinMaterialRemoteTest, CreateFlatBufferBuilder) {
  IMP_TRACE_NAME("CreateFlatBufferBuilder");

  SerializerViewUpdate(
      [](BaseView& serializer_view, const imp::FrameTime& frame_time) {
        auto builder = SplitEngineBuiltinMaterial::CreateFlatBufferBuilder(
            serializer_view, 100);
        EXPECT_OK(builder);
        EXPECT_NE(*builder, nullptr);
      });
}

INSTANTIATE_TEST_SUITE_P(
    SplitEngineIntegrationTests, SplitEngineBuiltinMaterialRemoteTest,
    ::testing::ValuesIn<SplitEngineIntegrationTestParams>({
        imp::split_engine::kSingleMachineTransportParams,
    }),

    [](const ::testing::TestParamInfo<
        SplitEngineBuiltinMaterialRemoteTest::ParamType>& info) {
      return info.param.test_name_suffix;
    });

}  // namespace
}  // namespace imp::split_engine
