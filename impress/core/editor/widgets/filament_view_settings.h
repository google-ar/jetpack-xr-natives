/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FILAMENT_VIEW_SETTINGS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FILAMENT_VIEW_SETTINGS_H_

#include <cstdint>
#include <memory>

#include "filament/filament/include/filament/ColorGrading.h"
#include "filament/filament/include/filament/ColorSpace.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Options.h"
#include "filament/filament/include/filament/ToneMapper.h"
#include "core/math/vec.h"

namespace imp::editor {

// The tone mapping technique to use during post-processing. Each enum value
// represents one of the Filament tone mappers.
enum class ToneMapping : uint8_t {
  kLinear = 0,
  kAcesLegacy = 1,
  kAces = 2,
  kFilmic = 3,
  kAgx = 4,
  kGeneric = 5,
  kPbrNeutral = 6,
  kDisplayRange = 7,
};

// Parameters specific to Filament's GenericToneMapper.
struct GenericToneMapperSettings {
  float contrast = 1.55f;
  float mid_gray_in = 0.18f;
  float mid_gray_out = 0.215f;
  float hdr_max = 10.0f;

  bool operator!=(const GenericToneMapperSettings& rhs) const {
    return !(rhs == *this);
  }
  bool operator==(const GenericToneMapperSettings& rhs) const;
};

// Parameters specific to Filament's AgxToneMapper.
struct AgxToneMapperSettings {
  filament::AgxToneMapper::AgxLook look =
      filament::AgxToneMapper::AgxLook::NONE;

  bool operator!=(const AgxToneMapperSettings& rhs) const {
    return !(rhs == *this);
  }
  bool operator==(const AgxToneMapperSettings& rhs) const;
};

// Color grading settings. Used to configure Filament's ColorGrading API.
struct ColorGradingSettings {
  bool enabled = true;
  bool linked_curves = false;
  bool luminance_scaling = false;
  bool gamut_mapping = false;
  filament::ColorGrading::QualityLevel quality =
      filament::ColorGrading::QualityLevel::MEDIUM;
  ToneMapping tone_mapping = ToneMapping::kAcesLegacy;
  bool padding0{};
  AgxToneMapperSettings agx_tone_mapper;
  filament::color::ColorSpace colorspace =
      filament::color::Rec709 - filament::color::sRGB - filament::color::D65;
  GenericToneMapperSettings generic_tone_mapper;
  float4 shadows = {1.0f, 1.0f, 1.0f, 0.0f};
  float4 midtones = {1.0f, 1.0f, 1.0f, 0.0f};
  float4 highlights = {1.0f, 1.0f, 1.0f, 0.0f};
  float4 ranges = {0.0f, 0.333f, 0.550f, 1.0f};
  float3 out_red = {1.0f, 0.0f, 0.0f};
  float3 out_green = {0.0f, 1.0f, 0.0f};
  float3 out_blue = {0.0f, 0.0f, 1.0f};
  float3 slope = {1.0f};
  float3 offset = {0.0f};
  float3 power = {1.0f};
  float3 gamma = {1.0f};
  float3 mid_point = {1.0f};
  float3 scale = {1.0f};
  float exposure = 0.0f;
  float night_adaptation = 0.0f;
  float temperature = 0.0f;
  float tint = 0.0f;
  float contrast = 1.0f;
  float vibrance = 1.0f;
  float saturation = 1.0f;

  bool operator!=(const ColorGradingSettings& rhs) const {
    return !(rhs == *this);
  }
  bool operator==(const ColorGradingSettings& rhs) const;
};

struct DynamicLightingSettings {
  float z_light_near = 5;
  float z_light_far = 100;
};

// Filament view settings. Most of these settings were taken (with small
// modifications to suit Impress) from Filament's ViewerGui settings, under
// libs/viewer/include/viewer/Settings.h in the Filament source.
class FilamentViewSettings {
 public:
  // Reads settings from the given Filament View. Note, currently ColorGrading
  // settings are not exposed by Filament and thus cannot be read.
  void ReadFromView(const filament::View& view);
  // Applies settings to the given Filament View. Only modifies color grading if
  // relevant settings have changed.
  void ApplyToView(filament::View& view, filament::Engine& engine);

  // High level View settings.
  filament::AntiAliasing anti_aliasing = filament::AntiAliasing::FXAA;
  filament::Dithering dithering = filament::Dithering::TEMPORAL;
  filament::ShadowType shadow_type = filament::ShadowType::PCF;
  bool post_processing_enabled = true;

  // Filament option structs (sorted).
  filament::AmbientOcclusionOptions ssao;
  filament::BloomOptions bloom;
  filament::DepthOfFieldOptions dof;
  filament::DynamicResolutionOptions dsr;
  filament::FogOptions fog;
  filament::GuardBandOptions guard_band;
  filament::MultiSampleAntiAliasingOptions msaa;
  filament::RenderQuality render_quality;
  filament::ScreenSpaceReflectionsOptions ssr;
  filament::StereoscopicOptions stereoscopic;
  filament::TemporalAntiAliasingOptions taa;
  filament::VignetteOptions vignette;
  filament::VsmShadowOptions vsm_shadow;

  // Custom options structs.
  DynamicLightingSettings dynamic_lighting;
  ColorGradingSettings color_grading_settings;

 private:
  // We need to store the previous color grading settings in order to avoid
  // recreating it, since the filament View doesn't expose it.
  ColorGradingSettings previous_color_grading_settings_;
  filament::ColorGrading* previous_color_grading_ = nullptr;
};

// Creates a new tonemapper based on the given settings.
std::unique_ptr<filament::ToneMapper> CreateToneMapper(
    const ColorGradingSettings& settings);

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FILAMENT_VIEW_SETTINGS_H_
