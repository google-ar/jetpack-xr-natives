// Copyright 2026 Google LLC
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

#include "split_engine/materials/gsplat_material_serializer.h"

#include <memory>
#include <optional>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "testing/base/public/test_utils.h"
#include "absl/memory/memory.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/assets/asset_ptr.h"
#include "core/gsplat/gsplat_material_params.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "testing/view_fixture.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace android_xr {

using ::imp::testing::ViewFixture;
using ::testing::IsNull;
using ::testing::NotNull;

class GsplatMaterialSerializerTest : public ViewFixture {
 protected:
  GsplatMaterialSerializerTest() {
    view_->SetSplitEngineMaterialLocalMode(false);
  }

  std::unique_ptr<GsplatMaterialSerializer> CreateSerializer(
      schemas::GsplatMode mode) {
    // Since we are a friend of GsplatMaterialSerializer, we can call the
    // private constructor directly to avoid the complex Create() flow that
    // involves requesting materials and loading assets.
    return absl::WrapUnique(new GsplatMaterialSerializer(
        *view_, /*material=*/{}, /*gsplat_asset=*/{}, mode,
        /*precomputed_data_texture=*/{},
        /*magic_window_offscreen_resolution=*/{}));
  }
};

TEST_F(GsplatMaterialSerializerTest, CheckDefaultsIsNull) {
  std::unique_ptr<GsplatMaterialSerializer> serializer =
      CreateSerializer(schemas::GsplatMode::GSPLAT);
  ASSERT_NE(serializer, nullptr);

  flatbuffers::FlatBufferBuilder fbb;
  imp::split_engine::BuiltInTextureParameterCreator texture_parameter_creator(
      true);
  flatbuffers::Offset<void> offset =
      serializer->SerializeParameters(fbb, texture_parameter_creator);
  fbb.Finish(offset);

  const android_xr::schemas::BuiltInMaterialGsplatParameters* params =
      flatbuffers::GetRoot<schemas::BuiltInMaterialGsplatParameters>(
          fbb.GetBufferPointer());

  EXPECT_EQ(params->mode_parameters_type(),
            schemas::GsplatModeParameters::GsplatParameters);
  const auto* gsplat_params = params->mode_parameters_as_GsplatParameters();
  ASSERT_THAT(gsplat_params, NotNull());

  // Verify initial values are null.
  EXPECT_THAT(gsplat_params->max_screen_size(), IsNull());
  EXPECT_THAT(gsplat_params->min_screen_size(), IsNull());
  EXPECT_THAT(gsplat_params->precomputed_data_texture(), IsNull());
  EXPECT_THAT(gsplat_params->position_data_texture(), IsNull());
  EXPECT_THAT(gsplat_params->cov3d_data_texture(), IsNull());
  EXPECT_THAT(gsplat_params->color_data_texture(), IsNull());
  EXPECT_THAT(gsplat_params->sorted_indices_texture(), IsNull());
  EXPECT_THAT(gsplat_params->splat_scale(), IsNull());
  EXPECT_THAT(gsplat_params->magic_window_projection_quad(), IsNull());
  EXPECT_THAT(gsplat_params->view_resolution(), IsNull());
}

TEST_F(GsplatMaterialSerializerTest, SetMaxScreenSizeSize) {
  std::unique_ptr<GsplatMaterialSerializer> serializer =
      CreateSerializer(schemas::GsplatMode::GSPLAT);

  imp::float2 size{100.0f, 200.0f};
  serializer->SetMaxScreenSize(size);

  flatbuffers::FlatBufferBuilder fbb;
  imp::split_engine::BuiltInTextureParameterCreator texture_parameter_creator(
      true);
  flatbuffers::Offset<void> offset =
      serializer->SerializeParameters(fbb, texture_parameter_creator);
  fbb.Finish(offset);

  const android_xr::schemas::BuiltInMaterialGsplatParameters* params =
      flatbuffers::GetRoot<schemas::BuiltInMaterialGsplatParameters>(
          fbb.GetBufferPointer());

  EXPECT_EQ(params->mode_parameters_type(),
            schemas::GsplatModeParameters::GsplatParameters);
  const auto* gsplat_params = params->mode_parameters_as_GsplatParameters();
  ASSERT_THAT(gsplat_params, NotNull());

  // Verify the parameter was serialized.
  ASSERT_THAT(gsplat_params->max_screen_size(), NotNull());
  EXPECT_EQ(gsplat_params->max_screen_size()->x(), size.x);
  EXPECT_EQ(gsplat_params->max_screen_size()->y(), size.y);
}

TEST_F(GsplatMaterialSerializerTest, MagicWindowCheckDefaultsIsNull) {
  std::unique_ptr<GsplatMaterialSerializer> serializer =
      CreateSerializer(schemas::GsplatMode::MAGIC_WINDOW);
  ASSERT_NE(serializer, nullptr);

  flatbuffers::FlatBufferBuilder fbb;
  imp::split_engine::BuiltInTextureParameterCreator texture_parameter_creator(
      true);
  flatbuffers::Offset<void> offset =
      serializer->SerializeParameters(fbb, texture_parameter_creator);
  fbb.Finish(offset);

  const android_xr::schemas::BuiltInMaterialGsplatParameters* params =
      flatbuffers::GetRoot<schemas::BuiltInMaterialGsplatParameters>(
          fbb.GetBufferPointer());

  EXPECT_EQ(params->mode_parameters_type(),
            schemas::GsplatModeParameters::MagicWindowParameters);
  const auto* mw_params = params->mode_parameters_as_MagicWindowParameters();
  ASSERT_THAT(mw_params, NotNull());

  // Verify initial values are null.
  EXPECT_THAT(mw_params->magic_window_projection_quad(), IsNull());
  EXPECT_THAT(mw_params->magic_window_offscreen_resolution(), IsNull());
}

TEST_F(GsplatMaterialSerializerTest, MagicWindowSetParameters) {
  std::unique_ptr<GsplatMaterialSerializer> serializer =
      CreateSerializer(schemas::GsplatMode::MAGIC_WINDOW);

  imp::TexturePipelineRendererProjectionQuad quad;
  quad.size = {1.0f, 2.0f};
  quad.center = {3.0f, 4.0f, 5.0f};
  quad.rotation = imp::QuatFromEuler({1.0f, 2.0f, 3.0f});
  serializer->SetMagicWindowProjectionQuad(quad);

  imp::uint2 resolution{1024, 768};
  serializer->SetMagicWindowOffscreenResolution(resolution);

  flatbuffers::FlatBufferBuilder fbb;
  imp::split_engine::BuiltInTextureParameterCreator texture_parameter_creator(
      true);
  flatbuffers::Offset<void> offset =
      serializer->SerializeParameters(fbb, texture_parameter_creator);
  fbb.Finish(offset);

  const android_xr::schemas::BuiltInMaterialGsplatParameters* params =
      flatbuffers::GetRoot<schemas::BuiltInMaterialGsplatParameters>(
          fbb.GetBufferPointer());

  EXPECT_EQ(params->mode_parameters_type(),
            schemas::GsplatModeParameters::MagicWindowParameters);
  const auto* mw_params = params->mode_parameters_as_MagicWindowParameters();
  ASSERT_THAT(mw_params, NotNull());

  // Verify the parameters were serialized.
  ASSERT_THAT(mw_params->magic_window_projection_quad(), NotNull());
  EXPECT_EQ(mw_params->magic_window_projection_quad()->size()->x(),
            quad.size.x);
  EXPECT_EQ(mw_params->magic_window_projection_quad()->size()->y(),
            quad.size.y);
  EXPECT_EQ(mw_params->magic_window_projection_quad()->center()->x(),
            quad.center.x);
  EXPECT_EQ(mw_params->magic_window_projection_quad()->center()->y(),
            quad.center.y);
  EXPECT_EQ(mw_params->magic_window_projection_quad()->center()->z(),
            quad.center.z);
  EXPECT_EQ(mw_params->magic_window_projection_quad()->rotation()->x(),
            quad.rotation.x);
  EXPECT_EQ(mw_params->magic_window_projection_quad()->rotation()->y(),
            quad.rotation.y);
  EXPECT_EQ(mw_params->magic_window_projection_quad()->rotation()->z(),
            quad.rotation.z);
  EXPECT_EQ(mw_params->magic_window_projection_quad()->rotation()->w(),
            quad.rotation.w);

  ASSERT_THAT(mw_params->magic_window_offscreen_resolution(), NotNull());
  EXPECT_EQ(mw_params->magic_window_offscreen_resolution()->x(), resolution.x);
  EXPECT_EQ(mw_params->magic_window_offscreen_resolution()->y(), resolution.y);
}

TEST_F(GsplatMaterialSerializerTest, SetParameterUnknownNameIsFatal) {
  std::unique_ptr<GsplatMaterialSerializer> serializer =
      CreateSerializer(schemas::GsplatMode::MAGIC_WINDOW);
  ASSERT_NE(serializer, nullptr);

  EXPECT_DFATAL(serializer->SetParameter("unknown_parameter", 1.0f),
                "No such parameter: unknown_parameter");
}

TEST_F(GsplatMaterialSerializerTest, SetParameterWrongTypeIsFatal) {
  std::unique_ptr<imp::Material> serializer =
      CreateSerializer(schemas::GsplatMode::GSPLAT);
  ASSERT_NE(serializer, nullptr);

  EXPECT_DFATAL(
      serializer->SetParameter(imp::kOpacityScaleParameter,
                               imp::bool4{false, false, false, false}),
      "Type mismatch for parameter: opacityScale");
}

TEST_F(GsplatMaterialSerializerTest, SetGsplatTextures) {
  std::unique_ptr<GsplatMaterialSerializer> serializer =
      CreateSerializer(schemas::GsplatMode::GSPLAT);
  ASSERT_NE(serializer, nullptr);

  imp::BorrowedTexturePtr dummy_texture =
      GetView()->GetTextureFactory().BorrowPlaceholderTextureBlack();

  // These should not crash
  serializer->SetParameter(imp::kPositionDataTextureParameter, dummy_texture,
                           std::nullopt);
  serializer->SetParameter(imp::kCov3dDataTextureParameter, dummy_texture,
                           std::nullopt);
  serializer->SetParameter(imp::kColorDataTextureParameter, dummy_texture,
                           std::nullopt);

  flatbuffers::FlatBufferBuilder fbb;
  imp::split_engine::BuiltInTextureParameterCreator texture_parameter_creator(
      true);
  flatbuffers::Offset<void> offset =
      serializer->SerializeParameters(fbb, texture_parameter_creator);
  fbb.Finish(offset);

  const android_xr::schemas::BuiltInMaterialGsplatParameters* params =
      flatbuffers::GetRoot<schemas::BuiltInMaterialGsplatParameters>(
          fbb.GetBufferPointer());

  EXPECT_EQ(params->mode_parameters_type(),
            schemas::GsplatModeParameters::GsplatParameters);
  const auto* gsplat_params = params->mode_parameters_as_GsplatParameters();
  ASSERT_THAT(gsplat_params, NotNull());

  ASSERT_THAT(gsplat_params->position_data_texture(), NotNull());
  ASSERT_THAT(gsplat_params->cov3d_data_texture(), NotNull());
  ASSERT_THAT(gsplat_params->color_data_texture(), NotNull());
}

TEST_F(GsplatMaterialSerializerTest, HasParameter) {
  std::unique_ptr<GsplatMaterialSerializer> serializer =
      CreateSerializer(schemas::GsplatMode::MAGIC_WINDOW);
  ASSERT_NE(serializer, nullptr);

  EXPECT_TRUE(serializer->HasParameter(imp::kOpacityScaleParameter));
  EXPECT_TRUE(serializer->HasParameter(imp::kSplatScaleParameter));
  EXPECT_TRUE(serializer->HasParameter(imp::kMinScreenSizeParameter));
  EXPECT_TRUE(serializer->HasParameter(imp::kMaxScreenSizeParameter));
  EXPECT_TRUE(
      serializer->HasParameter(imp::kWindowDimensionInMagicWindowParameter));
  EXPECT_TRUE(
      serializer->HasParameter(imp::kMagicWindowOffscreenResolutionParameter));
  EXPECT_TRUE(
      serializer->HasParameter(imp::kMagicWindowFromUserWorldMatrixParameter));
  EXPECT_TRUE(serializer->HasParameter(imp::kSplatDataPrecomputedParameter));
  EXPECT_TRUE(serializer->HasParameter(imp::kSortedIndicesParameter));
  EXPECT_TRUE(serializer->HasParameter(imp::kPositionDataTextureParameter));
  EXPECT_TRUE(serializer->HasParameter(imp::kCov3dDataTextureParameter));
  EXPECT_TRUE(serializer->HasParameter(imp::kColorDataTextureParameter));

  EXPECT_FALSE(serializer->HasParameter("unknown_parameter"));
}

TEST_F(GsplatMaterialSerializerTest, TrySetParameter) {
  std::unique_ptr<GsplatMaterialSerializer> serializer =
      CreateSerializer(schemas::GsplatMode::GSPLAT);
  ASSERT_NE(serializer, nullptr);

  // This should work and not crash.
  serializer->TrySetParameter(imp::kOpacityScaleParameter, 0.5f);

  // This should not crash even though it's an unknown parameter.
  // SetParameter would crash here.
  serializer->TrySetParameter("unknown_parameter", 1.0f);

  // Verify value was set
  flatbuffers::FlatBufferBuilder fbb;
  imp::split_engine::BuiltInTextureParameterCreator texture_parameter_creator(
      true);
  flatbuffers::Offset<void> offset =
      serializer->SerializeParameters(fbb, texture_parameter_creator);
  fbb.Finish(offset);

  const android_xr::schemas::BuiltInMaterialGsplatParameters* params =
      flatbuffers::GetRoot<schemas::BuiltInMaterialGsplatParameters>(
          fbb.GetBufferPointer());

  EXPECT_EQ(params->mode_parameters_type(),
            schemas::GsplatModeParameters::GsplatParameters);
  const auto* gsplat_params = params->mode_parameters_as_GsplatParameters();
  ASSERT_THAT(gsplat_params, NotNull());

  // Verify the parameter was serialized.
  ASSERT_THAT(gsplat_params->opacity_scale(), NotNull());
  EXPECT_EQ(gsplat_params->opacity_scale()->value(), 0.5f);
}

}  // namespace android_xr
