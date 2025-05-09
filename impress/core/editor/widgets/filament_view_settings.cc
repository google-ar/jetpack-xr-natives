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

#include "core/editor/widgets/filament_view_settings.h"

#include <memory>

#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/ColorGrading.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/ToneMapper.h"
#include "filament/filament/include/filament/View.h"

namespace imp::editor {

namespace {
// Creates a new ColorGrading object based on the given settings.
filament::ColorGrading* CreateColorGrading(const ColorGradingSettings& settings,
                                           filament::Engine& engine) {
  std::unique_ptr<filament::ToneMapper> tone_mapper =
      CreateToneMapper(settings);
  filament::ColorGrading* color_grading =
      filament::ColorGrading::Builder()
          .quality(settings.quality)
          .exposure(settings.exposure)
          .nightAdaptation(settings.night_adaptation)
          .whiteBalance(settings.temperature, settings.tint)
          .channelMixer(settings.out_red, settings.out_green, settings.out_blue)
          .shadowsMidtonesHighlights(
              filament::Color::toLinear(settings.shadows),
              filament::Color::toLinear(settings.midtones),
              filament::Color::toLinear(settings.highlights), settings.ranges)
          .slopeOffsetPower(settings.slope, settings.offset, settings.power)
          .contrast(settings.contrast)
          .vibrance(settings.vibrance)
          .saturation(settings.saturation)
          .curves(settings.gamma, settings.mid_point, settings.scale)
          .toneMapper(tone_mapper.get())
          .luminanceScaling(settings.luminance_scaling)
          .gamutMapping(settings.gamut_mapping)
          .outputColorSpace(settings.colorspace)
          .build(engine);
  return color_grading;
}
}  // namespace

bool GenericToneMapperSettings::operator==(
    const GenericToneMapperSettings& rhs) const {
  return contrast == rhs.contrast && mid_gray_in == rhs.mid_gray_in &&
         mid_gray_out == rhs.mid_gray_out && hdr_max == rhs.hdr_max;
}

bool AgxToneMapperSettings::operator==(const AgxToneMapperSettings& rhs) const {
  return look == rhs.look;
}

bool ColorGradingSettings::operator==(const ColorGradingSettings& rhs) const {
  return enabled == rhs.enabled && colorspace == rhs.colorspace &&
         quality == rhs.quality && tone_mapping == rhs.tone_mapping &&
         generic_tone_mapper == rhs.generic_tone_mapper &&
         agx_tone_mapper == rhs.agx_tone_mapper &&
         luminance_scaling == rhs.luminance_scaling &&
         gamut_mapping == rhs.gamut_mapping && exposure == rhs.exposure &&
         night_adaptation == rhs.night_adaptation &&
         temperature == rhs.temperature && tint == rhs.tint &&
         out_red == rhs.out_red && out_green == rhs.out_green &&
         out_blue == rhs.out_blue && shadows == rhs.shadows &&
         midtones == rhs.midtones && highlights == rhs.highlights &&
         ranges == rhs.ranges && contrast == rhs.contrast &&
         vibrance == rhs.vibrance && saturation == rhs.saturation &&
         slope == rhs.slope && offset == rhs.offset && power == rhs.power &&
         gamma == rhs.gamma && mid_point == rhs.mid_point &&
         linked_curves == rhs.linked_curves && scale == rhs.scale;
}

void FilamentViewSettings::ReadFromView(const filament::View& view) {
  anti_aliasing = view.getAntiAliasing();
  dithering = view.getDithering();
  shadow_type = view.getShadowType();
  post_processing_enabled = view.isPostProcessingEnabled();

  ssao = view.getAmbientOcclusionOptions();
  bloom = view.getBloomOptions();
  dof = view.getDepthOfFieldOptions();
  dsr = view.getDynamicResolutionOptions();
  fog = view.getFogOptions();
  guard_band = view.getGuardBandOptions();
  msaa = view.getMultiSampleAntiAliasingOptions();
  render_quality = view.getRenderQuality();
  ssr = view.getScreenSpaceReflectionsOptions();
  stereoscopic = view.getStereoscopicOptions();
  taa = view.getTemporalAntiAliasingOptions();
  vignette = view.getVignetteOptions();
  vsm_shadow = view.getVsmShadowOptions();

  // There is no getter for dynamic lighting options.
  //   dynamic_lighting = view.getDynamicLightingOptions();

  // TODO: (broken link) - There is currently no way to retrieve color grading
  // options.
}

void FilamentViewSettings::ApplyToView(filament::View& view,
                                       filament::Engine& engine) {
  view.setAntiAliasing(anti_aliasing);
  view.setDithering(dithering);
  view.setShadowType(shadow_type);
  view.setPostProcessingEnabled(post_processing_enabled);

  view.setAmbientOcclusionOptions(ssao);
  view.setBloomOptions(bloom);
  view.setDepthOfFieldOptions(dof);
  view.setDynamicResolutionOptions(dsr);
  view.setFogOptions(fog);
  view.setGuardBandOptions(guard_band);
  view.setMultiSampleAntiAliasingOptions(msaa);
  view.setRenderQuality(render_quality);
  view.setScreenSpaceReflectionsOptions(ssr);
  view.setStereoscopicOptions(stereoscopic);
  view.setTemporalAntiAliasingOptions(taa);
  view.setVignetteOptions(vignette);
  view.setVsmShadowOptions(vsm_shadow);

  view.setDynamicLightingOptions(dynamic_lighting.z_light_near,
                                 dynamic_lighting.z_light_far);

  // Enable/disable color grading as needed.
  if (color_grading_settings != previous_color_grading_settings_) {
    if (color_grading_settings.enabled) {
      filament::ColorGrading* color_grading =
          CreateColorGrading(color_grading_settings, engine);
      engine.destroy(previous_color_grading_);
      previous_color_grading_ = color_grading;
      previous_color_grading_settings_ = color_grading_settings;
      view.setColorGrading(previous_color_grading_);
    } else {
      view.setColorGrading(nullptr);
    }
  }
}

std::unique_ptr<filament::ToneMapper> CreateToneMapper(
    const ColorGradingSettings& settings) {
  switch (settings.tone_mapping) {
    case ToneMapping::kLinear:
      return std::make_unique<filament::LinearToneMapper>();
    case ToneMapping::kAcesLegacy:
      return std::make_unique<filament::ACESLegacyToneMapper>();
    case ToneMapping::kAces:
      return std::make_unique<filament::ACESToneMapper>();
    case ToneMapping::kFilmic:
      return std::make_unique<filament::FilmicToneMapper>();
    case ToneMapping::kAgx:
      return std::make_unique<filament::AgxToneMapper>(
          settings.agx_tone_mapper.look);
    case ToneMapping::kGeneric:
      return std::make_unique<filament::GenericToneMapper>(
          settings.generic_tone_mapper.contrast,
          settings.generic_tone_mapper.mid_gray_in,
          settings.generic_tone_mapper.mid_gray_out,
          settings.generic_tone_mapper.hdr_max);
    case ToneMapping::kPbrNeutral:
      return std::make_unique<filament::PBRNeutralToneMapper>();
    case ToneMapping::kDisplayRange:
      return std::make_unique<filament::DisplayRangeToneMapper>();
  }
}

}  // namespace imp::editor
