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

#include "core/view/utils/render_setting_utils.h"

#include <cstdint>

#include "filament/filament/include/filament/ColorGrading.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Options.h"
#include "filament/filament/include/filament/ToneMapper.h"
#include "filament/filament/include/filament/View.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"

namespace imp {

namespace {
constexpr uint32_t kMaxMaterialGlobalIndex = 3;
}

void ConfigureViewRenderSettingsWithOverrides(
    const OverrideMode override_mode, filament::View* target,
    const filament::View* source, const ViewRenderSettings* override_settings,
    filament::Engine* engine) {
  bool is_overriding_source =
      override_mode == render_settings::OVERRIDE_MODE_OVERRIDE_FROM_SOURCE;

  if (is_overriding_source) {
    // (broken link) start
    target->setAmbientOcclusionOptions(source->getAmbientOcclusionOptions());
    target->setAntiAliasing(source->getAntiAliasing());
    target->setBlendMode(source->getBlendMode());
    target->setBloomOptions(source->getBloomOptions());
    target->setDepthOfFieldOptions(source->getDepthOfFieldOptions());
    target->setDithering(source->getDithering());
    target->setDynamicResolutionOptions(source->getDynamicResolutionOptions());
    // target->setDynamicLightingOptions(source->getDynamicLightingOptions());
    target->setFogOptions(source->getFogOptions());
    target->setFrustumCullingEnabled(source->isFrustumCullingEnabled());
    target->setGuardBandOptions(source->getGuardBandOptions());
    target->setPostProcessingEnabled(source->isPostProcessingEnabled());
    target->setRenderQuality(source->getRenderQuality());
    target->setScreenSpaceReflectionsOptions(
        source->getScreenSpaceReflectionsOptions());
    target->setShadowingEnabled(source->isShadowingEnabled());
    // target->setShadowType(source->getShadowType());
    target->setSoftShadowOptions(source->getSoftShadowOptions());
    target->setStereoscopicOptions(source->getStereoscopicOptions());
    target->setTemporalAntiAliasingOptions(
        source->getTemporalAntiAliasingOptions());
    target->setVignetteOptions(source->getVignetteOptions());
    target->setVsmShadowOptions(source->getVsmShadowOptions());
    // (broken link) end

    // Don't set the MSAA options if we're overriding the current settings
    // because setting them at this point might fire asserts that wouldn't fire
    // with the overrides.
    if (override_settings == nullptr) {
      target->setMultiSampleAntiAliasingOptions(
          source->getMultiSampleAntiAliasingOptions());
    }

    for (uint32_t i = 0; i <= kMaxMaterialGlobalIndex; ++i) {
      target->setMaterialGlobal(i, source->getMaterialGlobal(i));
    }
  }

  if (override_settings == nullptr) {
    return;
  }

  if (override_settings->post_processing_enabled.has_value()) {
    target->setPostProcessingEnabled(
        override_settings->post_processing_enabled.value());
  }

  if (override_settings->anti_aliasing.has_value()) {
    filament::View::AntiAliasing anti_aliasing =
        override_settings->anti_aliasing.value() ==
                render_settings::ANTI_ALIASING_NONE
            ? filament::View::AntiAliasing::NONE
            : filament::View::AntiAliasing::FXAA;
    target->setAntiAliasing(anti_aliasing);
  }

  target->setMultiSampleAntiAliasingOptions({
      .enabled =
          override_settings->multi_sample_anti_aliasing_options.enabled
              .value_or(target->getMultiSampleAntiAliasingOptions().enabled),
      .sampleCount = static_cast<uint8_t>(
          override_settings->multi_sample_anti_aliasing_options.sample_count
              .value_or(
                  target->getMultiSampleAntiAliasingOptions().sampleCount)),
      .customResolve =
          override_settings->multi_sample_anti_aliasing_options.custom_resolve
              .value_or(
                  target->getMultiSampleAntiAliasingOptions().customResolve),
  });

  if (override_settings->dithering.has_value()) {
    filament::View::Dithering dithering =
        override_settings->dithering.value() == render_settings::DITHERING_NONE
            ? filament::View::Dithering::NONE
            : filament::View::Dithering::TEMPORAL;
    target->setDithering(dithering);
  }

  if (override_settings->render_quality.has_value()) {
    filament::View::RenderQuality render_quality;
    switch (override_settings->render_quality.value().hdr_color_buffer) {
      case render_settings::QualityLevel::QUALITY_LEVEL_LOW: {
        render_quality.hdrColorBuffer = filament::QualityLevel::LOW;
        break;
      }
      case render_settings::QualityLevel::QUALITY_LEVEL_MEDIUM: {
        render_quality.hdrColorBuffer = filament::QualityLevel::MEDIUM;
        break;
      }
      case render_settings::QualityLevel::QUALITY_LEVEL_HIGH: {
        render_quality.hdrColorBuffer = filament::QualityLevel::HIGH;
        break;
      }
      case render_settings::QualityLevel::QUALITY_LEVEL_ULTRA: {
        render_quality.hdrColorBuffer = filament::QualityLevel::ULTRA;
        break;
      }
      default: {
        render_quality.hdrColorBuffer = filament::QualityLevel::MEDIUM;
      }
    }
    target->setRenderQuality(render_quality);
  }

  if (override_settings->color_grading.has_value()) {
    filament::ColorGrading::Builder color_grading_builder;

    // Quality level
    if (override_settings->color_grading.value().quality.has_value()) {
      switch (override_settings->color_grading.value().quality.value()) {
        case render_settings::QualityLevel::QUALITY_LEVEL_LOW: {
          color_grading_builder.quality(
              filament::ColorGrading::QualityLevel::LOW);
          break;
        }
        case render_settings::QualityLevel::QUALITY_LEVEL_MEDIUM: {
          color_grading_builder.quality(
              filament::ColorGrading::QualityLevel::MEDIUM);
          break;
        }
        case render_settings::QualityLevel::QUALITY_LEVEL_HIGH: {
          color_grading_builder.quality(
              filament::ColorGrading::QualityLevel::HIGH);
          break;
        }
        case render_settings::QualityLevel::QUALITY_LEVEL_ULTRA: {
          color_grading_builder.quality(
              filament::ColorGrading::QualityLevel::ULTRA);
          break;
        }
        default: {
          break;
        }
      }
    }

    // LUT format
    if (override_settings->color_grading.value().format.has_value()) {
      switch (override_settings->color_grading.value().format.value()) {
        case render_settings::ColorGrading::LutFormat::LUT_FORMAT_INTEGER: {
          color_grading_builder.format(
              filament::ColorGrading::LutFormat::INTEGER);
          break;
        }
        case render_settings::ColorGrading::LutFormat::LUT_FORMAT_FLOAT: {
          color_grading_builder.format(
              filament::ColorGrading::LutFormat::FLOAT);
          break;
        }
        default: {
          break;
        }
      }
    }

    // Dimensions
    if (override_settings->color_grading.value().dimensions.has_value()) {
      color_grading_builder.dimensions(
          override_settings->color_grading.value().dimensions.value());
    }

    // Tone mapper
    if (override_settings->color_grading.value().tone_mapper.has_value()) {
      switch (
          override_settings->color_grading.value().tone_mapper.value().mode) {
        case render_settings::ColorGrading::TONE_MAPPING_MODE_LINEAR: {
          color_grading_builder.toneMapper(new filament::LinearToneMapper());
          break;
        }
        case render_settings::ColorGrading::TONE_MAPPING_MODE_ACES: {
          color_grading_builder.toneMapper(new filament::ACESToneMapper());
          break;
        }
        case render_settings::ColorGrading::TONE_MAPPING_MODE_ACES_LEGACY: {
          color_grading_builder.toneMapper(
              new filament::ACESLegacyToneMapper());
          break;
        }
        case render_settings::ColorGrading::TONE_MAPPING_MODE_FILMIC: {
          color_grading_builder.toneMapper(new filament::FilmicToneMapper());
          break;
        }
        case render_settings::ColorGrading::TONE_MAPPING_MODE_GENERIC: {
          filament::GenericToneMapper* generic_tone_mapper =
              new filament::GenericToneMapper();
          const render_settings::ColorGrading::ToneMapper&
              tone_mapper_settings =
                  override_settings->color_grading.value().tone_mapper.value();

          if (tone_mapper_settings.contrast.has_value()) {
            generic_tone_mapper->setContrast(
                tone_mapper_settings.contrast.value());
          }
          if (tone_mapper_settings.hdr_max.has_value()) {
            generic_tone_mapper->setHdrMax(
                tone_mapper_settings.hdr_max.value());
          }
          if (tone_mapper_settings.mid_gray_in.has_value()) {
            generic_tone_mapper->setMidGrayIn(
                tone_mapper_settings.mid_gray_in.value());
          }
          if (tone_mapper_settings.mid_gray_out.has_value()) {
            generic_tone_mapper->setMidGrayOut(
                tone_mapper_settings.mid_gray_out.value());
          }

          color_grading_builder.toneMapper(generic_tone_mapper);
          break;
        }
        default: {
          break;
        }
      }
    }

    // Luminance scaling
    if (override_settings->color_grading.value()
            .luminance_scaling.has_value()) {
      color_grading_builder.luminanceScaling(
          override_settings->color_grading.value().luminance_scaling.value());
    }

    // Gamut mapping
    if (override_settings->color_grading.value().gamut_mapping.has_value()) {
      color_grading_builder.gamutMapping(
          override_settings->color_grading.value().gamut_mapping.value());
    }

    // Exposure
    if (override_settings->color_grading.value().exposure.has_value()) {
      color_grading_builder.exposure(
          override_settings->color_grading.value().exposure.value());
    }

    // Night adaptation
    if (override_settings->color_grading.value().night_adaptation.has_value()) {
      color_grading_builder.nightAdaptation(
          override_settings->color_grading.value().night_adaptation.value());
    }

    // White balance
    if (override_settings->color_grading.value().white_balance.has_value()) {
      const render_settings::ColorGrading::WhiteBalance& white_balance =
          override_settings->color_grading.value().white_balance.value();
      color_grading_builder.whiteBalance(white_balance.temperature,
                                         white_balance.tint);
    }

    // Channel mixer
    if (override_settings->color_grading.value().channel_mixer.has_value()) {
      const render_settings::ColorGrading::ChannelMixer& channel_mixer =
          override_settings->color_grading.value().channel_mixer.value();
      color_grading_builder.channelMixer(channel_mixer.out_red,
                                         channel_mixer.out_green,
                                         channel_mixer.out_blue);
    }

    // Shadows midtones highlights
    if (override_settings->color_grading.value()
            .shadows_midtones_highlights.has_value()) {
      const render_settings::ColorGrading::ShadowsMidtonesHighlights&
          shadows_midtones_highlights =
              override_settings->color_grading.value()
                  .shadows_midtones_highlights.value();
      color_grading_builder.shadowsMidtonesHighlights(
          shadows_midtones_highlights.shadows,
          shadows_midtones_highlights.midtones,
          shadows_midtones_highlights.highlights,
          shadows_midtones_highlights.ranges);
    }

    // Slope offset power
    if (override_settings->color_grading.value()
            .slope_offset_power.has_value()) {
      const render_settings::ColorGrading::SlopeOffsetPower&
          slope_offset_power = override_settings->color_grading.value()
                                   .slope_offset_power.value();
      color_grading_builder.slopeOffsetPower(slope_offset_power.slope,
                                             slope_offset_power.offset,
                                             slope_offset_power.power);
    }

    // Contrast
    if (override_settings->color_grading.value().contrast.has_value()) {
      color_grading_builder.contrast(
          override_settings->color_grading.value().contrast.value());
    }

    // Vibrance
    if (override_settings->color_grading.value().vibrance.has_value()) {
      color_grading_builder.vibrance(
          override_settings->color_grading.value().vibrance.value());
    }

    // Saturation
    if (override_settings->color_grading.value().saturation.has_value()) {
      color_grading_builder.saturation(
          override_settings->color_grading.value().saturation.value());
    }

    // Curve
    if (override_settings->color_grading.value().curves.has_value()) {
      const render_settings::ColorGrading::Curve& curve =
          override_settings->color_grading.value().curves.value();
      color_grading_builder.curves(curve.shadow_gamma, curve.mid_point,
                                   curve.highlight_scale);
    }

    target->setColorGrading(color_grading_builder.build(*engine));
  }

  if (override_settings->shadowing_enabled.has_value()) {
    target->setShadowingEnabled(override_settings->shadowing_enabled.value());
  }
}

void OverrideViewRenderSettings(filament::View* target,
                                const ViewRenderSettings* override_settings,
                                filament::Engine* engine) {
  ConfigureViewRenderSettingsWithOverrides(
      OverrideMode::OVERRIDE_MODE_OVERRIDE_CURRENT, target, nullptr,
      override_settings, engine);
}

ViewRenderSettings GetViewRenderSettings(const filament::View* view) {
  ViewRenderSettings settings;
  settings.post_processing_enabled = view->isPostProcessingEnabled();
  settings.anti_aliasing =
      view->getAntiAliasing() == filament::View::AntiAliasing::FXAA
          ? render_settings::ANTI_ALIASING_FXAA
          : render_settings::ANTI_ALIASING_NONE;
  settings.dithering =
      view->getDithering() == filament::View::Dithering::TEMPORAL
          ? render_settings::DITHERING_TEMPORAL
          : render_settings::DITHERING_NONE;
  filament::QualityLevel hdr_color_buffer =
      view->getRenderQuality().hdrColorBuffer;

  render_settings::RenderQuality render_quality;
  if (hdr_color_buffer == filament::QualityLevel::LOW) {
    render_quality.hdr_color_buffer =
        render_settings::QualityLevel::QUALITY_LEVEL_LOW;
  } else if (hdr_color_buffer == filament::QualityLevel::MEDIUM) {
    render_quality.hdr_color_buffer =
        render_settings::QualityLevel::QUALITY_LEVEL_MEDIUM;
  } else if (hdr_color_buffer == filament::QualityLevel::HIGH) {
    render_quality.hdr_color_buffer =
        render_settings::QualityLevel::QUALITY_LEVEL_HIGH;
  } else if (hdr_color_buffer == filament::QualityLevel::ULTRA) {
    render_quality.hdr_color_buffer =
        render_settings::QualityLevel::QUALITY_LEVEL_ULTRA;
  } else {
    render_quality.hdr_color_buffer =
        render_settings::QualityLevel::QUALITY_LEVEL_MEDIUM;
  }
  settings.render_quality = render_quality;

  // TODO: Get color grading options. Not yet supported.
  settings.shadowing_enabled = view->isShadowingEnabled();
  return settings;
}

}  // namespace imp
