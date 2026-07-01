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
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "testing/view_fixture.h"

namespace imp::split_engine {
namespace {

// Test suite to validate behavior of SplitEngineBuiltinMaterial in local mode
// where SplitEngineSerializer is not present.

class SplitEngineBuiltinMaterialLocalTest : public testing::ViewFixture {};

TEST_F(SplitEngineBuiltinMaterialLocalTest, CreateFlatBufferBuilder) {
  view_->SetSplitEngineMaterialLocalMode(true);

  MP_ASSERT_OK_AND_ASSIGN(
      auto builder,
      SplitEngineBuiltinMaterial::CreateFlatBufferBuilder(*view_, 100));
  EXPECT_NE(builder, nullptr);
}

}  // namespace

}  // namespace imp::split_engine
