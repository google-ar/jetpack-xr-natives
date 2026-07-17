// Copyright 2026 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/split_engine/texture_pipeline_schema_conversion.h"

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"
#include "core/common/type_helpers.h"
#include "core/math/math.h"
#include "core/ncsb/node.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer_context.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"

namespace imp {
namespace split_engine {

namespace {

// Verify imp::TexturePipelineRendererState::Texture::Format and
// android_xr::schemas::RenderTargetTextureFormat match.
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::AUTO,
                 android_xr::schemas::RenderTargetTextureFormat::AUTO));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGBA8,
                 android_xr::schemas::RenderTargetTextureFormat::RGBA8));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGB8,
                 android_xr::schemas::RenderTargetTextureFormat::RGB8));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RG8,
                 android_xr::schemas::RenderTargetTextureFormat::RG8));
static_assert(DoEnumsMatch(TexturePipelineRendererState::Texture::Format::R8,
                           android_xr::schemas::RenderTargetTextureFormat::R8));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGBA32F,
                 android_xr::schemas::RenderTargetTextureFormat::RGBA32F));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGB32F,
                 android_xr::schemas::RenderTargetTextureFormat::RGB32F));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RG32F,
                 android_xr::schemas::RenderTargetTextureFormat::RG32F));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::R32F,
                 android_xr::schemas::RenderTargetTextureFormat::R32F));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGBA16F,
                 android_xr::schemas::RenderTargetTextureFormat::RGBA16F));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGB16F,
                 android_xr::schemas::RenderTargetTextureFormat::RGB16F));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RG16F,
                 android_xr::schemas::RenderTargetTextureFormat::RG16F));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::R16F,
                 android_xr::schemas::RenderTargetTextureFormat::R16F));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::DEPTH24,
                 android_xr::schemas::RenderTargetTextureFormat::DEPTH24));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::DEPTH32F,
                 android_xr::schemas::RenderTargetTextureFormat::DEPTH32F));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::R8UI,
                 android_xr::schemas::RenderTargetTextureFormat::R8UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RG8UI,
                 android_xr::schemas::RenderTargetTextureFormat::RG8UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGB8UI,
                 android_xr::schemas::RenderTargetTextureFormat::RGB8UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGBA8UI,
                 android_xr::schemas::RenderTargetTextureFormat::RGBA8UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::R16UI,
                 android_xr::schemas::RenderTargetTextureFormat::R16UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RG16UI,
                 android_xr::schemas::RenderTargetTextureFormat::RG16UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGB16UI,
                 android_xr::schemas::RenderTargetTextureFormat::RGB16UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGBA16UI,
                 android_xr::schemas::RenderTargetTextureFormat::RGBA16UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::R32UI,
                 android_xr::schemas::RenderTargetTextureFormat::R32UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RG32UI,
                 android_xr::schemas::RenderTargetTextureFormat::RG32UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGB32UI,
                 android_xr::schemas::RenderTargetTextureFormat::RGB32UI));
static_assert(
    DoEnumsMatch(TexturePipelineRendererState::Texture::Format::RGBA32UI,
                 android_xr::schemas::RenderTargetTextureFormat::RGBA32UI));

static_assert(
    android_xr::schemas::RenderTargetTextureFormat::MAX ==
        android_xr::schemas::RenderTargetTextureFormat::RGBA32UI,
    "New fields added to RenderTargetTextureFormat but assert not updated");

// Verify imp::TexturePipelineRendererState::AutomaticTextureSize::Mode and
// android_xr::schemas::AutomaticTextureSizeMode match.
static_assert(DoEnumsMatch(TexturePipelineRendererState::AutomaticTextureSize::
                               Mode::DEFAULT_VIEW_SIZE_VIRTUAL_PIXELS,
                           android_xr::schemas::AutomaticTextureSizeMode::
                               DEFAULT_VIEW_SIZE_VIRTUAL_PIXELS));
static_assert(DoEnumsMatch(
    TexturePipelineRendererState::AutomaticTextureSize::Mode::
        VIEW_SIZE_PHYSICAL_PIXELS,
    android_xr::schemas::AutomaticTextureSizeMode::VIEW_SIZE_PHYSICAL_PIXELS));

static_assert(
    android_xr::schemas::AutomaticTextureSizeMode::MAX ==
        android_xr::schemas::AutomaticTextureSizeMode::
            VIEW_SIZE_PHYSICAL_PIXELS,
    "New fields added to AutomaticTextureSizeMode but assert not updated");

// Verify imp::render_settings::QualityLevel and
// android_xr::schemas::QualityLevel match.
static_assert(DoEnumsMatch(imp::render_settings::QUALITY_LEVEL_UNDEFINED,
                           android_xr::schemas::QualityLevel::UNDEFINED));
static_assert(DoEnumsMatch(imp::render_settings::QUALITY_LEVEL_LOW,
                           android_xr::schemas::QualityLevel::LOW));
static_assert(DoEnumsMatch(imp::render_settings::QUALITY_LEVEL_MEDIUM,
                           android_xr::schemas::QualityLevel::MEDIUM));
static_assert(DoEnumsMatch(imp::render_settings::QUALITY_LEVEL_HIGH,
                           android_xr::schemas::QualityLevel::HIGH));
static_assert(DoEnumsMatch(imp::render_settings::QUALITY_LEVEL_ULTRA,
                           android_xr::schemas::QualityLevel::ULTRA));

static_assert(android_xr::schemas::QualityLevel::MAX ==
                  android_xr::schemas::QualityLevel::ULTRA,
              "New fields added to QualityLevel but assert not updated");

// Verify imp::render_settings::AntiAliasing and
// android_xr::schemas::AntiAliasing match.
static_assert(DoEnumsMatch(imp::render_settings::ANTI_ALIASING_NONE,
                           android_xr::schemas::AntiAliasing::NONE));
static_assert(DoEnumsMatch(imp::render_settings::ANTI_ALIASING_FXAA,
                           android_xr::schemas::AntiAliasing::FXAA));

static_assert(android_xr::schemas::AntiAliasing::MAX ==
                  android_xr::schemas::AntiAliasing::FXAA,
              "New fields added to AntiAliasing but assert not updated");

// Verify imp::render_settings::Dithering and
// android_xr::schemas::Dithering match.
static_assert(DoEnumsMatch(imp::render_settings::DITHERING_NONE,
                           android_xr::schemas::Dithering::NONE));
static_assert(DoEnumsMatch(imp::render_settings::DITHERING_TEMPORAL,
                           android_xr::schemas::Dithering::TEMPORAL));

static_assert(android_xr::schemas::Dithering::MAX ==
                  android_xr::schemas::Dithering::TEMPORAL,
              "New fields added to Dithering but assert not updated");

// Verify imp::render_settings::ColorGrading::ToneMappingMode and
// android_xr::schemas::ToneMappingMode match.
static_assert(DoEnumsMatch(
    imp::render_settings::ColorGrading::TONE_MAPPING_MODE_UNDEFINED,
    android_xr::schemas::ToneMappingMode::UNDEFINED));
static_assert(
    DoEnumsMatch(imp::render_settings::ColorGrading::TONE_MAPPING_MODE_LINEAR,
                 android_xr::schemas::ToneMappingMode::LINEAR));
static_assert(
    DoEnumsMatch(imp::render_settings::ColorGrading::TONE_MAPPING_MODE_ACES,
                 android_xr::schemas::ToneMappingMode::ACES));
static_assert(DoEnumsMatch(
    imp::render_settings::ColorGrading::TONE_MAPPING_MODE_ACES_LEGACY,
    android_xr::schemas::ToneMappingMode::ACES_LEGACY));
static_assert(
    DoEnumsMatch(imp::render_settings::ColorGrading::TONE_MAPPING_MODE_FILMIC,
                 android_xr::schemas::ToneMappingMode::FILMIC));
static_assert(
    DoEnumsMatch(imp::render_settings::ColorGrading::TONE_MAPPING_MODE_GENERIC,
                 android_xr::schemas::ToneMappingMode::GENERIC));

static_assert(android_xr::schemas::ToneMappingMode::MAX ==
                  android_xr::schemas::ToneMappingMode::GENERIC,
              "New fields added to ToneMappingMode but assert not updated");

}  // namespace

absl::StatusOr<
    flatbuffers::Offset<android_xr::schemas::TexturePipelineRenderer>>
TexturePipelineRendererSchemaFromState(
    flatbuffers::FlatBufferBuilder& fbb,
    const imp::TexturePipelineRendererState& state) {
  std::vector<flatbuffers::Offset<android_xr::schemas::TexturePipelinePass>>
      passes_vector;
  if (!state.passes.empty()) {
    passes_vector.reserve(state.passes.size());
    for (const TexturePipelineRendererState::Pass& pass : state.passes) {
      if (pass.override_material.has_value()) {
        return absl::UnimplementedError(
            "TexturePipelineRenderer: override_material is not "
            "supported in Split Engine.");
      }
      // Camera id serialization isn't supported and we just pass in 0.
      uint64_t camera_id = 0;
      if (pass.camera.IsValid()) {
        return absl::UnimplementedError(
            "TexturePipelineRenderer: camera serialization is "
            "not fully supported yet.");
      }

      flatbuffers::Offset<flatbuffers::String> group =
          fbb.CreateString(pass.group);

      android_xr::schemas::ColorTextureConfig color_texture_config_type =
          android_xr::schemas::ColorTextureConfig::NONE;
      flatbuffers::Offset<void> color_texture_config_offset = 0;

      if (std::holds_alternative<std::string>(pass.color_texture_config)) {
        color_texture_config_type =
            android_xr::schemas::ColorTextureConfig::String;
        color_texture_config_offset =
            android_xr::schemas::CreateString(
                fbb, fbb.CreateString(
                         std::get<std::string>(pass.color_texture_config)))
                .Union();
      } else if (std::holds_alternative<
                     imp::TexturePipelineRendererState::Texture>(
                     pass.color_texture_config)) {
        color_texture_config_type =
            android_xr::schemas::ColorTextureConfig::NewRegisteredTexture;
        const auto& tex = std::get<imp::TexturePipelineRendererState::Texture>(
            pass.color_texture_config);
        auto name = fbb.CreateString(tex.name);
        color_texture_config_offset =
            android_xr::schemas::CreateNewRegisteredTexture(
                fbb, name,
                static_cast<android_xr::schemas::RenderTargetTextureFormat>(
                    tex.format))
                .Union();
      }

      flatbuffers::Offset<android_xr::schemas::NewRegisteredTexture>
          depth_texture;
      if (pass.depth_texture.has_value()) {
        flatbuffers::Offset<flatbuffers::String> name =
            fbb.CreateString(pass.depth_texture->name);
        depth_texture = android_xr::schemas::CreateNewRegisteredTexture(
            fbb, name,
            static_cast<android_xr::schemas::RenderTargetTextureFormat>(
                pass.depth_texture->format));
      }

      android_xr::schemas::TextureSizeConfig texture_size_type =
          android_xr::schemas::TextureSizeConfig::NONE;
      flatbuffers::Offset<void> texture_size_offset = 0;

      if (std::holds_alternative<
              imp::TexturePipelineRendererState::AutomaticTextureSize>(
              pass.texture_size)) {
        const auto& auto_size =
            std::get<imp::TexturePipelineRendererState::AutomaticTextureSize>(
                pass.texture_size);
        texture_size_type =
            android_xr::schemas::TextureSizeConfig::AutomaticTextureSize;
        texture_size_offset =
            android_xr::schemas::CreateAutomaticTextureSize(
                fbb,
                static_cast<android_xr::schemas::AutomaticTextureSizeMode>(
                    auto_size.mode),
                auto_size.factor.value_or(1.0f), auto_size.resize_with_view)
                .Union();
      } else if (std::holds_alternative<imp::uint2>(pass.texture_size)) {
        const auto& pix = std::get<imp::uint2>(pass.texture_size);
        texture_size_type = android_xr::schemas::TextureSizeConfig::Uint2;
        texture_size_offset =
            fbb.CreateStruct(android_xr::schemas::Uint2(pix.x, pix.y)).Union();
      }

      flatbuffers::Offset<android_xr::schemas::ViewRenderSettings>
          render_settings;
      if (pass.render_settings.has_value()) {
        flatbuffers::Offset<android_xr::schemas::RenderQuality> render_quality =
            0;
        if (pass.render_settings->render_quality.has_value()) {
          render_quality = android_xr::schemas::CreateRenderQuality(
              fbb, static_cast<android_xr::schemas::QualityLevel>(
                       pass.render_settings->render_quality->hdr_color_buffer));
        }

        flatbuffers::Offset<android_xr::schemas::ColorGrading> color_grading =
            0;
        if (pass.render_settings->color_grading.has_value()) {
          const auto& cg = *pass.render_settings->color_grading;

          flatbuffers::Offset<android_xr::schemas::ToneMapper> tone_mapper = 0;
          if (cg.tone_mapper.has_value()) {
            tone_mapper = android_xr::schemas::CreateToneMapper(
                fbb,
                static_cast<android_xr::schemas::ToneMappingMode>(
                    cg.tone_mapper->mode),
                cg.tone_mapper->contrast.value_or(0.0f),
                cg.tone_mapper->mid_gray_in.value_or(0.0f),
                cg.tone_mapper->mid_gray_out.value_or(0.0f),
                cg.tone_mapper->hdr_max.value_or(0.0f));
          }

          color_grading = android_xr::schemas::CreateColorGrading(
              fbb,
              cg.quality.has_value()
                  ? static_cast<android_xr::schemas::QualityLevel>(*cg.quality)
                  : android_xr::schemas::QualityLevel::UNDEFINED,
              tone_mapper,
              StructFromOptional<android_xr::schemas::Float>(cg.exposure),
              StructFromOptional<android_xr::schemas::Float>(
                  cg.night_adaptation),
              StructFromOptional<android_xr::schemas::Float>(cg.contrast),
              StructFromOptional<android_xr::schemas::Float>(cg.vibrance),
              StructFromOptional<android_xr::schemas::Float>(cg.saturation));
        }

        flatbuffers::Offset<android_xr::schemas::MultiSampleAntiAliasingOptions>
            msaa_offset = 0;
        const auto& msaa =
            pass.render_settings->multi_sample_anti_aliasing_options;
        if (msaa.enabled.has_value() || msaa.sample_count.has_value() ||
            msaa.custom_resolve.has_value()) {
          msaa_offset =
              android_xr::schemas::CreateMultiSampleAntiAliasingOptions(
                  fbb,
                  StructFromOptional<android_xr::schemas::Bool>(msaa.enabled),
                  StructFromOptional<android_xr::schemas::UInt32>(
                      msaa.sample_count),
                  StructFromOptional<android_xr::schemas::Bool>(
                      msaa.custom_resolve));
        }

        render_settings = android_xr::schemas::CreateViewRenderSettings(
            fbb,
            StructFromOptional<android_xr::schemas::Bool>(
                pass.render_settings->post_processing_enabled),
            static_cast<android_xr::schemas::AntiAliasing>(
                pass.render_settings->anti_aliasing.value_or(
                    imp::render_settings::ANTI_ALIASING_NONE)),
            static_cast<android_xr::schemas::Dithering>(
                pass.render_settings->dithering.value_or(
                    imp::render_settings::DITHERING_NONE)),
            render_quality, color_grading,
            StructFromOptional<android_xr::schemas::Bool>(
                pass.render_settings->shadowing_enabled),
            msaa_offset,
            StructFromOptional<android_xr::schemas::Bool>(
                pass.render_settings->use_srgb_swapchain),
            StructFromOptional<android_xr::schemas::Bool>(
                pass.render_settings->use_stencil_swapchain),
            StructFromOptional<android_xr::schemas::Bool>(
                pass.render_settings->use_msaa_swapchain));
      }

      std::optional<android_xr::schemas::Int2> view_port_left_bottom;
      if (pass.view_port_left_bottom.has_value()) {
        view_port_left_bottom = android_xr::schemas::Int2(
            pass.view_port_left_bottom->x, pass.view_port_left_bottom->y);
      }

      std::optional<android_xr::schemas::Float4> clear_color;
      if (pass.clear_color.has_value()) {
        clear_color = android_xr::schemas::Float4(
            pass.clear_color->x, pass.clear_color->y, pass.clear_color->z,
            pass.clear_color->w);
      }

      android_xr::schemas::Bool use_main_view_settings(
          pass.use_main_view_settings);
      passes_vector.push_back(android_xr::schemas::CreateTexturePipelinePass(
          fbb, group, camera_id, color_texture_config_type,
          color_texture_config_offset,
          StructFromOptional<android_xr::schemas::Int>(
              pass.color_texture_layer),
          depth_texture, texture_size_type, texture_size_offset,
          &use_main_view_settings, render_settings,
          PointerFromOptional(view_port_left_bottom),
          PointerFromOptional(clear_color)));
    }
  }

  return android_xr::schemas::CreateTexturePipelineRenderer(
      fbb, fbb.CreateVector(passes_vector), state.priority);
}

absl::StatusOr<TexturePipelineRendererState>
TexturePipelineRendererStateFromSchema(
    BridgeId bridge_id,
    const android_xr::schemas::TexturePipelineRenderer& schema) {
  // TODO: (broken link) - add more validation (see bug description).

  imp::TexturePipelineRendererState state;
  state.priority = schema.priority();

  if (schema.passes()) {
    for (const android_xr::schemas::TexturePipelinePass* pass_schema :
         *schema.passes()) {
      TexturePipelineRendererState::Pass& pass = state.passes.emplace_back();

      if (pass_schema->camera() != 0) {
        return absl::UnimplementedError(
            "TexturePipelineRenderer: camera serialization is "
            "not fully supported yet.");
      }

      if (!pass_schema->group()) {
        return absl::FailedPreconditionError(
            "TexturePipelineRenderer: group must be specified.");
      }

      if (pass_schema->group()->str() == imp::Node::kMainGroupName) {
        return absl::FailedPreconditionError(
            "TexturePipelineRenderer: group must not be the main group.");
      }

      pass.group = AppContext::GetBridgePrefixedName(
          bridge_id, pass_schema->group()->string_view());

      if (pass_schema->color_texture_config_type() ==
          android_xr::schemas::ColorTextureConfig::NewRegisteredTexture) {
        const auto* color = pass_schema->color_texture_config_as<
            android_xr::schemas::NewRegisteredTexture>();
        if (!color) {
          return absl::FailedPreconditionError(
              "TexturePipelineRenderer: color texture config is null.");
        }
        imp::TexturePipelineRendererState::Texture t;
        if (color->name()) {
          t.name = AppContext::GetBridgePrefixedName(bridge_id,
                                                     color->name()->str());
        }
        if (color->format() >
            android_xr::schemas::RenderTargetTextureFormat::MAX) {
          return absl::InvalidArgumentError(
              "TexturePipelineRenderer: color texture format is invalid.");
        }
        t.format =
            static_cast<imp::TexturePipelineRendererState::Texture::Format>(
                color->format());
        pass.color_texture_config = t;
      } else if (pass_schema->color_texture_config_type() ==
                 android_xr::schemas::ColorTextureConfig::String) {
        auto* reg =
            pass_schema->color_texture_config_as<android_xr::schemas::String>();
        if (!reg || !reg->value() || reg->value()->str().empty()) {
          return absl::FailedPreconditionError(
              "TexturePipelineRenderer: color texture config string is empty.");
        }
        pass.color_texture_config =
            AppContext::GetBridgePrefixedName(bridge_id, reg->value()->str());
      }

      if (pass_schema->color_texture_layer()) {
        pass.color_texture_layer = pass_schema->color_texture_layer()->value();
      }

      if (const android_xr::schemas::NewRegisteredTexture* depth =
              pass_schema->depth_texture()) {
        imp::TexturePipelineRendererState::Texture t;
        if (depth->name()) {
          t.name = AppContext::GetBridgePrefixedName(bridge_id,
                                                     depth->name()->str());
        }
        if (depth->format() >
            android_xr::schemas::RenderTargetTextureFormat::MAX) {
          return absl::InvalidArgumentError(
              "TexturePipelineRenderer: depth texture format is invalid.");
        }
        t.format =
            static_cast<imp::TexturePipelineRendererState::Texture::Format>(
                depth->format());
        pass.depth_texture = t;
      }

      if (pass_schema->texture_size_type() ==
          android_xr::schemas::TextureSizeConfig::AutomaticTextureSize) {
        const auto* automatic =
            pass_schema
                ->texture_size_as<android_xr::schemas::AutomaticTextureSize>();
        if (!automatic) {
          return absl::FailedPreconditionError(
              "TexturePipelineRenderer: automatic texture size is null.");
        }
        imp::TexturePipelineRendererState::AutomaticTextureSize auto_size;
        if (automatic->mode() >
            android_xr::schemas::AutomaticTextureSizeMode::MAX) {
          return absl::InvalidArgumentError(
              "TexturePipelineRenderer: automatic texture size mode is "
              "invalid.");
        }
        auto_size.mode = static_cast<
            imp::TexturePipelineRendererState::AutomaticTextureSize::Mode>(
            automatic->mode());
        auto_size.factor = automatic->factor();
        auto_size.resize_with_view = automatic->resize_with_view();
        pass.texture_size = auto_size;
      } else if (pass_schema->texture_size_type() ==
                 android_xr::schemas::TextureSizeConfig::Uint2) {
        const auto* pixel =
            pass_schema->texture_size_as<android_xr::schemas::Uint2>();
        if (!pixel) {
          return absl::FailedPreconditionError(
              "TexturePipelineRenderer: pixel texture size is null.");
        }
        pass.texture_size = imp::uint2{pixel->x(), pixel->y()};
      }

      if (auto* settings = pass_schema->render_settings()) {
        render_settings::ViewRenderSettings s;
        if (settings->post_processing_enabled()) {
          s.post_processing_enabled =
              settings->post_processing_enabled()->value();
        }

        if (settings->anti_aliasing() >
            android_xr::schemas::AntiAliasing::MAX) {
          return absl::InvalidArgumentError(
              "TexturePipelineRenderer: anti_aliasing is invalid.");
        }
        s.anti_aliasing = static_cast<render_settings::AntiAliasing>(
            settings->anti_aliasing());

        if (settings->dithering() > android_xr::schemas::Dithering::MAX) {
          return absl::InvalidArgumentError(
              "TexturePipelineRenderer: dithering is invalid.");
        }
        s.dithering =
            static_cast<render_settings::Dithering>(settings->dithering());

        if (settings->shadowing_enabled()) {
          s.shadowing_enabled = settings->shadowing_enabled()->value();
        }
        if (settings->use_srgb_swapchain()) {
          s.use_srgb_swapchain = settings->use_srgb_swapchain()->value();
        }
        if (settings->use_stencil_swapchain()) {
          s.use_stencil_swapchain = settings->use_stencil_swapchain()->value();
        }
        if (settings->use_msaa_swapchain()) {
          s.use_msaa_swapchain = settings->use_msaa_swapchain()->value();
        }

        if (const android_xr::schemas::RenderQuality* q =
                settings->render_quality()) {
          render_settings::RenderQuality rq;
          if (q->hdr_color_buffer() > android_xr::schemas::QualityLevel::MAX) {
            return absl::InvalidArgumentError(
                "TexturePipelineRenderer: hdr_color_buffer is invalid.");
          }
          rq.hdr_color_buffer =
              static_cast<render_settings::QualityLevel>(q->hdr_color_buffer());
          s.render_quality = rq;
        }

        if (const android_xr::schemas::ColorGrading* cg =
                settings->color_grading()) {
          render_settings::ColorGrading cgs;
          if (cg->quality() > android_xr::schemas::QualityLevel::MAX) {
            return absl::InvalidArgumentError(
                "TexturePipelineRenderer: quality is invalid.");
          }
          cgs.quality =
              static_cast<render_settings::QualityLevel>(cg->quality());
          if (cg->exposure()) {
            cgs.exposure = cg->exposure()->value();
          }
          if (cg->night_adaptation()) {
            cgs.night_adaptation = cg->night_adaptation()->value();
          }
          if (cg->contrast()) {
            cgs.contrast = cg->contrast()->value();
          }
          if (cg->vibrance()) {
            cgs.vibrance = cg->vibrance()->value();
          }
          if (cg->saturation()) {
            cgs.saturation = cg->saturation()->value();
          }

          if (const android_xr::schemas::ToneMapper* tm = cg->tone_mapper()) {
            render_settings::ColorGrading::ToneMapper tms;
            if (tm->mode() > android_xr::schemas::ToneMappingMode::MAX) {
              return absl::InvalidArgumentError(
                  "TexturePipelineRenderer: tone mapper mode is out of "
                  "bounds.");
            }
            tms.mode =
                static_cast<render_settings::ColorGrading::ToneMappingMode>(
                    tm->mode());
            tms.contrast = tm->contrast();
            tms.mid_gray_in = tm->mid_gray_in();
            tms.mid_gray_out = tm->mid_gray_out();
            tms.hdr_max = tm->hdr_max();
            cgs.tone_mapper = tms;
          }
          s.color_grading = cgs;
        }

        if (const android_xr::schemas::MultiSampleAntiAliasingOptions* msaa =
                settings->multi_sample_anti_aliasing_options()) {
          imp::render_settings::MultiSampleAntiAliasingOptions msaas;
          if (msaa->enabled()) {
            msaas.enabled = msaa->enabled()->value();
          }
          if (msaa->sample_count()) {
            msaas.sample_count = msaa->sample_count()->value();
          }
          if (msaa->custom_resolve()) {
            msaas.custom_resolve = msaa->custom_resolve()->value();
          }
          s.multi_sample_anti_aliasing_options = msaas;
        }

        pass.render_settings = s;
      }

      if (const android_xr::schemas::Int2* view_port =
              pass_schema->view_port_left_bottom()) {
        pass.view_port_left_bottom = int2{view_port->x(), view_port->y()};
      }

      pass.use_main_view_settings =
          pass_schema->use_main_view_settings()
              ? pass_schema->use_main_view_settings()->value()
              : false;

      if (const android_xr::schemas::Float4* c = pass_schema->clear_color()) {
        pass.clear_color = float4{c->x(), c->y(), c->z(), c->w()};
      }
    }
  }
  return state;
}

}  // namespace split_engine
}  // namespace imp
