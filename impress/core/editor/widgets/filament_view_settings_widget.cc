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

#include "core/editor/widgets/filament_view_settings_widget.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "absl/log/check.h"
#include "dear_imgui/imgui.h"
#include "filament/filament/include/filament/ColorSpace.h"
#include "filament/filament/include/filament/Options.h"
#include "filament/filament/include/filament/ToneMapper.h"
#include "filament/libs/filagui/include/filagui/ImGuiExtensions.h"
#include "filament/libs/math/include/math/scalar.h"
#include "core/editor/widgets/filament_view_settings.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"

namespace imp::editor {
namespace {
using ::filament::AntiAliasing;
using ::filament::DepthOfFieldOptions;
using ::filament::Dithering;
using ::filament::QualityLevel;
using ::filament::TemporalAntiAliasingOptions;

using ::filament::color::D65;
using ::filament::color::Linear;
using ::filament::color::Rec709;
using ::filament::color::sRGB;

using ::filament::math::smoothstep;

// Most of the code in this file has been branched from Filament's ViewerGui
// file, under libs/viewer/src/ViewerGui.cpp in the Filament source tree.

// The base size, in number of data points, for ImGui plots in this widget.
constexpr int kPlotSize = 1024;

void PushSliderColors(float hue) {
  ImGui::PushStyleColor(ImGuiCol_FrameBg,
                        static_cast<ImVec4>(ImColor::HSV(hue, 0.5f, 0.5f)));
  ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,
                        static_cast<ImVec4>(ImColor::HSV(hue, 0.6f, 0.5f)));
  ImGui::PushStyleColor(ImGuiCol_FrameBgActive,
                        static_cast<ImVec4>(ImColor::HSV(hue, 0.7f, 0.5f)));
  ImGui::PushStyleColor(ImGuiCol_SliderGrab,
                        static_cast<ImVec4>(ImColor::HSV(hue, 0.9f, 0.9f)));
}

void PopSliderColors() { ImGui::PopStyleColor(4); }

void TooltipFloat(float value) {
  if (ImGui::IsItemActive() || ImGui::IsItemHovered()) {
    ImGui::SetTooltip("%.2f", value);
  }
}

void ComputeToneMapPlot(const ColorGradingSettings& settings,
                        std::vector<float>& plot) {
  float hdr_max = settings.tone_mapping == ToneMapping::kGeneric
                      ? settings.generic_tone_mapper.hdr_max
                      : 10.0f;
  std::unique_ptr<filament::ToneMapper> mapper = CreateToneMapper(settings);

  float a = std::log10(hdr_max * 1.5f / 1e-6f);
  
  for (int i = 0; i < kPlotSize; ++i) {
    float v = i;
    float x =
        1e-6f * std::pow(10.0f, a * v / (static_cast<float>(kPlotSize) - 1.0f));
    plot[i] = (*mapper)(x).r;
  }
}

void ComputeRangePlot(const ColorGradingSettings& settings,
                      std::vector<float>& plot) {
  const float4& ranges = settings.ranges;
  
  for (int i = 0; i < kPlotSize; ++i) {
    float x = i / static_cast<float>(kPlotSize);
    float s = 1.0f - smoothstep(ranges.x, ranges.y, x);
    float h = smoothstep(ranges.z, ranges.w, x);
    plot[i] = s;
    plot[kPlotSize + i] = 1.0f - s - h;
    plot[2 * kPlotSize + i] = h;
  }
}

void RangePlotSeriesStart(int series) {
  switch (series) {
    case 0:
      ImGui::PushStyleColor(
          ImGuiCol_PlotLines,
          static_cast<ImVec4>(ImColor::HSV(0.4f, 0.25f, 1.0f)));
      break;
    case 1:
      ImGui::PushStyleColor(
          ImGuiCol_PlotLines,
          static_cast<ImVec4>(ImColor::HSV(0.8f, 0.25f, 1.0f)));
      break;
    case 2:
      ImGui::PushStyleColor(
          ImGuiCol_PlotLines,
          static_cast<ImVec4>(ImColor::HSV(0.17f, 0.21f, 1.0f)));
      break;
  }
}

void RangePlotSeriesEnd(int series) {
  if (series < 3) {
    ImGui::PopStyleColor();
  }
}

float GetRangePlotValue(int series, void* data, int index) {
  return static_cast<float*>(data)[series * kPlotSize + index];
}

float3 Curves(float3 v, float3 shadow_gamma, float3 mid_point,
              float3 highlight_scale) {
  float3 d = 1.0f / (pow(mid_point, shadow_gamma - 1.0f));
  float3 dark = pow(v, shadow_gamma) * d;
  float3 light = highlight_scale * (v - mid_point) + mid_point;
  return float3{
      v.r <= mid_point.r ? dark.r : light.r,
      v.g <= mid_point.g ? dark.g : light.g,
      v.b <= mid_point.b ? dark.b : light.b,
  };
}

void ComputeCurvePlot(const ColorGradingSettings& settings,
                      std::vector<float>& plot) {
  
  for (int i = 0; i < kPlotSize; i++) {
    float3 x{i / static_cast<float>(kPlotSize) * 2.0f};
    float3 y = Curves(x, settings.gamma, settings.mid_point, settings.scale);
    plot[i] = y.r;
    plot[kPlotSize + i] = y.g;
    plot[2 * kPlotSize + i] = y.b;
  }
}
}  // namespace

FilamentViewSettingsWidget::FilamentViewSettingsWidget(BaseView& view)
    : view_(view),
      tone_map_plot_(kPlotSize),
      range_plot_(kPlotSize * 3),
      curve_plot_(kPlotSize * 3) {}

void FilamentViewSettingsWidget::DrawImGui() {
  view_settings_.ReadFromView(*view_.GetHost()->GetView());

  if (ImGui::CollapsingHeader("View")) {
    ImGui::Checkbox("Post-processing", &view_settings_.post_processing_enabled);
    ImGui::Indent();
    bool dither = view_settings_.dithering == Dithering::TEMPORAL;
    ImGui::Checkbox("Dithering", &dither);
    view_settings_.dithering = dither ? Dithering::TEMPORAL : Dithering::NONE;
    ImGui::Unindent();

    ImGui::Checkbox("Screen-space Guard Band",
                    &view_settings_.guard_band.enabled);
  }

  if (ImGui::CollapsingHeader("Bloom Options")) {
    ImGui::Checkbox("Enabled##bloomEnabled", &view_settings_.bloom.enabled);
    ImGui::SliderFloat("Strength", &view_settings_.bloom.strength, 0.0f, 1.0f);
    ImGui::Checkbox("Threshold", &view_settings_.bloom.threshold);

    int levels = view_settings_.bloom.levels;
    ImGui::SliderInt("Levels", &levels, 3, 11);
    view_settings_.bloom.levels = levels;

    int quality = static_cast<int>(view_settings_.bloom.quality);
    ImGui::SliderInt("Bloom Quality", &quality, 0, 3);
    view_settings_.bloom.quality = static_cast<QualityLevel>(quality);

    ImGui::Checkbox("Lens Flare", &view_settings_.bloom.lensFlare);
    ImGui::Indent();
    if (ImGui::CollapsingHeader("Lens Flare Options")) {
      ImGui::Checkbox("Starburst", &view_settings_.bloom.starburst);
      ImGui::SliderFloat("Chromatic Aberration",
                         &view_settings_.bloom.chromaticAberration, 0.0f, 1.0f,
                         "%.3f", ImGuiSliderFlags_Logarithmic);
      int ghost_count = view_settings_.bloom.ghostCount;
      ImGui::SliderInt("Ghost Count", &ghost_count, 0, 8);
      view_settings_.bloom.ghostCount = ghost_count;
      ImGui::SliderFloat("Ghost Spacing", &view_settings_.bloom.ghostSpacing,
                         0.0f, 1.0f);
      ImGui::SliderFloat("Ghost Threshold",
                         &view_settings_.bloom.ghostThreshold, 0.0f, 1000.0f,
                         "%.2f", ImGuiSliderFlags_Logarithmic);
      ImGui::SliderFloat("Halo Thickness", &view_settings_.bloom.haloThickness,
                         0.0f, 1.0f);
      ImGui::SliderFloat("Halo Radius", &view_settings_.bloom.haloRadius, 0.0f,
                         0.5f);
      ImGui::SliderFloat("Halo Threshold", &view_settings_.bloom.haloThreshold,
                         0.0f, 1000.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
    }
    ImGui::Unindent();
  }

  if (ImGui::CollapsingHeader("Anti-aliasing Options")) {
    bool fxaa = view_settings_.anti_aliasing == AntiAliasing::FXAA;
    ImGui::Checkbox("FXAA", &fxaa);
    view_settings_.anti_aliasing =
        fxaa ? AntiAliasing::FXAA : AntiAliasing::NONE;

    ImGui::Checkbox("MSAA 4x", &view_settings_.msaa.enabled);
    ImGui::Indent();
    ImGui::Checkbox("Custom resolve", &view_settings_.msaa.customResolve);
    ImGui::Unindent();

    ImGui::Checkbox("TAA##taaEnabled", &view_settings_.taa.enabled);
    ImGui::Indent();
    if (ImGui::CollapsingHeader("TAA Options")) {
      ImGui::Checkbox("Upscaling", &view_settings_.taa.upscaling);
      ImGui::Checkbox("History Reprojection",
                      &view_settings_.taa.historyReprojection);
      ImGui::SliderFloat("Feedback", &view_settings_.taa.feedback, 0.0f, 1.0f);
      ImGui::Checkbox("Filter History", &view_settings_.taa.filterHistory);
      ImGui::Checkbox("Filter Input", &view_settings_.taa.filterInput);
      ImGui::SliderFloat("FilterWidth", &view_settings_.taa.filterWidth, 0.2f,
                         2.0f);
      ImGui::SliderFloat("LOD bias", &view_settings_.taa.lodBias, -8.0f, 0.0f);
      ImGui::Checkbox("Use YCoCg", &view_settings_.taa.useYCoCg);
      ImGui::Checkbox("Prevent Flickering",
                      &view_settings_.taa.preventFlickering);
      int jitterSequence = static_cast<int>(view_settings_.taa.jitterPattern);
      int boxClipping = static_cast<int>(view_settings_.taa.boxClipping);
      int boxType = static_cast<int>(view_settings_.taa.boxType);
      ImGui::Combo(
          "Jitter Pattern", &jitterSequence,
          "RGSS x4\0Uniform Helix x4\0Halton x8\0Halton x16\0Halton x32\0\0");
      ImGui::Combo("Box Clipping", &boxClipping, "Accurate\0Clamp\0None\0\0");
      ImGui::Combo("Box Type", &boxType, "AABB\0Variance\0Both\0\0");
      ImGui::SliderFloat("Variance Gamma", &view_settings_.taa.varianceGamma,
                         0.75f, 1.25f);
      ImGui::SliderFloat("RCAS", &view_settings_.taa.sharpness, 0.0f, 1.0f);
      view_settings_.taa.boxClipping =
          static_cast<TemporalAntiAliasingOptions::BoxClipping>(boxClipping);
      view_settings_.taa.boxType =
          static_cast<TemporalAntiAliasingOptions::BoxType>(boxType);
      view_settings_.taa.jitterPattern =
          static_cast<TemporalAntiAliasingOptions::JitterPattern>(
              jitterSequence);
    }
    ImGui::Unindent();
  }

  if (ImGui::CollapsingHeader("SSAO Options")) {
    auto& ssao = view_settings_.ssao;
    ImGui::Checkbox("Enabled##ssaoEnabled", &ssao.enabled);

    int quality = static_cast<int>(ssao.quality);
    int lowpass = static_cast<int>(ssao.lowPassFilter);
    bool upsampling = ssao.upsampling != QualityLevel::LOW;

    bool half_res = ssao.resolution != 1.0f;

    ImGui::SliderInt("Quality", &quality, 0, 3);
    ImGui::SliderInt("Low Pass", &lowpass, 0, 2);
    ImGui::Checkbox("Bent Normals", &ssao.bentNormals);
    ImGui::Checkbox("High quality upsampling", &upsampling);
    ImGui::SliderFloat("Min Horizon angle", &ssao.minHorizonAngleRad, 0.0f,
                       M_PI_4);
    ImGui::SliderFloat("Bilateral Threshold", &ssao.bilateralThreshold, 0.0f,
                       0.1f);
    ImGui::Checkbox("Half resolution", &half_res);
    ssao.resolution = half_res ? 0.5f : 1.0f;

    ssao.upsampling = upsampling ? QualityLevel::HIGH : QualityLevel::LOW;
    ssao.lowPassFilter = static_cast<QualityLevel>(lowpass);
    ssao.quality = static_cast<QualityLevel>(quality);

    ImGui::Indent();
    if (ImGui::CollapsingHeader("Dominant Light Shadows (experimental)")) {
      int sample_count = ssao.ssct.sampleCount;
      ImGui::Checkbox("Enabled##dlsEnabled", &ssao.ssct.enabled);
      ImGui::SliderFloat("Cone angle", &ssao.ssct.lightConeRad, 0.0f, M_PI_2);
      ImGui::SliderFloat("Shadow distance", &ssao.ssct.shadowDistance, 0.0f,
                         10.0f);
      ImGui::SliderFloat("Contact dist max", &ssao.ssct.contactDistanceMax,
                         0.0f, 100.0f);
      ImGui::SliderFloat("Intensity##dls", &ssao.ssct.intensity, 0.0f, 10.0f);
      ImGui::SliderFloat("Depth bias", &ssao.ssct.depthBias, 0.0f, 1.0f);
      ImGui::SliderFloat("Depth slope bias", &ssao.ssct.depthSlopeBias, 0.0f,
                         1.0f);
      ImGui::SliderInt("Sample count", &sample_count, 1, 32);
      ImGuiExt::DirectionWidget("Direction##dls", ssao.ssct.lightDirection.v);
      ssao.ssct.sampleCount = sample_count;
    }
    ImGui::Unindent();
  }

  if (ImGui::CollapsingHeader("SSR Options")) {
    auto& ssr = view_settings_.ssr;
    ImGui::Checkbox("Enabled##ssrEnabled", &ssr.enabled);
    ImGui::SliderFloat("Ray thickness", &ssr.thickness, 0.001f, 0.2f);
    ImGui::SliderFloat("Bias", &ssr.bias, 0.001f, 0.5f);
    ImGui::SliderFloat("Max distance", &ssr.maxDistance, 0.1, 10.0f);
    ImGui::SliderFloat("Stride", &ssr.stride, 1.0, 10.0f);
  }

  if (ImGui::CollapsingHeader("Dynamic Resolution")) {
    auto& dsr = view_settings_.dsr;
    int quality = static_cast<int>(dsr.quality);
    ImGui::Checkbox("Enabled##dsrEnabled", &dsr.enabled);
    ImGui::Checkbox("Homogeneous scaling", &dsr.homogeneousScaling);
    ImGui::SliderFloat("Min scale", &dsr.minScale.x, 0.25f, 1.0f);
    ImGui::SliderFloat("Max scale", &dsr.maxScale.x, 0.25f, 1.0f);
    ImGui::SliderInt("Quality", &quality, 0, 3);
    ImGui::SliderFloat("Sharpness", &dsr.sharpness, 0.0f, 1.0f);
    dsr.minScale.x = std::min(dsr.minScale.x, dsr.maxScale.x);
    dsr.minScale.y = dsr.minScale.x;
    dsr.maxScale.y = dsr.maxScale.x;
    dsr.quality = static_cast<QualityLevel>(quality);
  }

  if (ImGui::CollapsingHeader("Fog")) {
    int fog_color_source = 0;
    if (view_settings_.fog.skyColor) {
      fog_color_source = 2;
    } else if (view_settings_.fog.fogColorFromIbl) {
      fog_color_source = 1;
    }

    bool exclude_skybox = !std::isinf(view_settings_.fog.cutOffDistance);
    ImGui::Checkbox("Enabled##fogEnabled", &view_settings_.fog.enabled);
    ImGui::SliderFloat("Start [m]", &view_settings_.fog.distance, 0.0f, 100.0f);
    ImGui::SliderFloat("Extinction [1/m]", &view_settings_.fog.density, 0.0f,
                       1.0f);
    ImGui::SliderFloat("Floor [m]", &view_settings_.fog.height, 0.0f, 100.0f);
    ImGui::SliderFloat("Height falloff [1/m]",
                       &view_settings_.fog.heightFalloff, 0.0f, 4.0f);
    ImGui::SliderFloat("Sun Scattering start [m]",
                       &view_settings_.fog.inScatteringStart, 0.0f, 100.0f);
    ImGui::SliderFloat("Sun Scattering size",
                       &view_settings_.fog.inScatteringSize, 0.1f, 100.0f);
    ImGui::Checkbox("Exclude Skybox", &exclude_skybox);
    ImGui::Combo("Color source##fogColorSource", &fog_color_source,
                 "Constant\0IBL\0Skybox\0\0");
    ImGui::ColorPicker3("Color##fogColor", view_settings_.fog.color.v);
    view_settings_.fog.cutOffDistance =
        exclude_skybox ? 1e6f : std::numeric_limits<float>::infinity();
    switch (fog_color_source) {
      case 0:
        view_settings_.fog.skyColor = nullptr;
        view_settings_.fog.fogColorFromIbl = false;
        break;
      case 1:
        view_settings_.fog.skyColor = nullptr;
        view_settings_.fog.fogColorFromIbl = true;
        break;
      case 2:
        // No way to assign texture yet.
        // view_settings_.fog.skyColor = some_texture;
        view_settings_.fog.fogColorFromIbl = false;
        break;
    }
  }

  if (ImGui::CollapsingHeader("Depth of Field")) {
    bool dof_median =
        view_settings_.dof.filter == DepthOfFieldOptions::Filter::MEDIAN;
    int dof_ring_count = view_settings_.dof.fastGatherRingCount;
    int dof_max_coc = view_settings_.dof.maxForegroundCOC;
    if (!dof_ring_count) dof_ring_count = 5;
    if (!dof_max_coc) dof_max_coc = 32;
    ImGui::Checkbox("Enabled##dofEnabled", &view_settings_.dof.enabled);
    ImGui::SliderFloat("Blur scale", &view_settings_.dof.cocScale, 0.1f, 10.0f);
    ImGui::SliderFloat("CoC aspect-ratio", &view_settings_.dof.cocAspectRatio,
                       0.25f, 4.0f);
    ImGui::SliderInt("Ring count", &dof_ring_count, 1, 17);
    ImGui::SliderInt("Max CoC", &dof_max_coc, 1, 32);
    ImGui::Checkbox("Native Resolution", &view_settings_.dof.nativeResolution);
    ImGui::Checkbox("Median Filter", &dof_median);
    view_settings_.dof.filter = dof_median ? DepthOfFieldOptions::Filter::MEDIAN
                                           : DepthOfFieldOptions::Filter::NONE;
    view_settings_.dof.backgroundRingCount = dof_ring_count;
    view_settings_.dof.foregroundRingCount = dof_ring_count;
    view_settings_.dof.fastGatherRingCount = dof_ring_count;
    view_settings_.dof.maxForegroundCOC = dof_max_coc;
    view_settings_.dof.maxBackgroundCOC = dof_max_coc;
  }

  if (ImGui::CollapsingHeader("Vignette")) {
    ImGui::Checkbox("Enabled##vignetteEnabled",
                    &view_settings_.vignette.enabled);
    ImGui::SliderFloat("Mid point", &view_settings_.vignette.midPoint, 0.0f,
                       1.0f);
    ImGui::SliderFloat("Roundness", &view_settings_.vignette.roundness, 0.0f,
                       1.0f);
    ImGui::SliderFloat("Feather", &view_settings_.vignette.feather, 0.0f, 1.0f);
    ImGui::ColorEdit3("Color##vignetteColor", &view_settings_.vignette.color.r);
  }

  DrawColorGradingUI();

  view_settings_.ApplyToView(*view_.GetHost()->GetView(),
                             *view_.GetHost()->GetEngine());
}

void FilamentViewSettingsWidget::DrawColorGradingUI() {
  constexpr ImVec2 kVerticalSliderSize(18.0f, 160.0f);
  constexpr ImVec2 kPlotLinesSize(0.0f, 160.0f);
  constexpr ImVec2 kPlotLinesWideSize(0.0f, 120.0f);

  if (ImGui::CollapsingHeader("Color grading")) {
    ColorGradingSettings& color_grading = view_settings_.color_grading_settings;

    ImGui::Checkbox("Enabled##colorGrading", &color_grading.enabled);

    int quality = static_cast<int>(color_grading.quality);
    ImGui::Combo("Quality##colorGradingQuality", &quality,
                 "Low\0Medium\0High\0Ultra\0\0");
    color_grading.quality =
        static_cast<decltype(color_grading.quality)>(quality);

    int colorspace =
        (color_grading.colorspace == Rec709 - Linear - D65) ? 0 : 1;
    ImGui::Combo("Output color space", &colorspace,
                 "Rec709-Linear-D65\0Rec709-sRGB-D65\0\0");
    color_grading.colorspace =
        (colorspace == 0) ? Rec709 - Linear - D65 : Rec709 - sRGB - D65;

    int tone_mapping = static_cast<int>(color_grading.tone_mapping);
    ImGui::Combo("Tone-mapping", &tone_mapping,
                 "Linear\0ACES (legacy)\0ACES\0Filmic\0AgX\0Generic\0PBR "
                 "Neutral\0Display Range\0\0");
    color_grading.tone_mapping =
        static_cast<decltype(color_grading.tone_mapping)>(tone_mapping);
    if (color_grading.tone_mapping == ToneMapping::kGeneric) {
      if (ImGui::CollapsingHeader("Tonemap parameters")) {
        GenericToneMapperSettings& generic = color_grading.generic_tone_mapper;
        ImGui::SliderFloat("Contrast##genericToneMapper", &generic.contrast,
                           1e-5f, 3.0f);
        ImGui::SliderFloat("Mid-gray in##genericToneMapper",
                           &generic.mid_gray_in, 0.0f, 1.0f);
        ImGui::SliderFloat("Mid-gray out##genericToneMapper",
                           &generic.mid_gray_out, 0.0f, 1.0f);
        ImGui::SliderFloat("HDR max", &generic.hdr_max, 1.0f, 64.0f);
      }
    }
    if (color_grading.tone_mapping == ToneMapping::kAgx) {
      int agxLook = static_cast<int>(color_grading.agx_tone_mapper.look);
      ImGui::Combo("AgX Look", &agxLook, "None\0Punchy\0Golden\0\0");
      color_grading.agx_tone_mapper.look =
          static_cast<decltype(color_grading.agx_tone_mapper.look)>(agxLook);
    }

    ComputeToneMapPlot(color_grading, tone_map_plot_);

    ImGui::PushStyleColor(ImGuiCol_PlotLines, static_cast<ImVec4>(ImColor::HSV(
                                                  0.17f, 0.21f, 0.9f)));
    ImGui::PlotLines("", tone_map_plot_.data(), tone_map_plot_.size(), 0,
                     "Tone map", 0.0f, 1.05f, ImVec2(0, 160));
    ImGui::PopStyleColor();

    ImGui::Checkbox("Luminance scaling", &color_grading.luminance_scaling);
    ImGui::Checkbox("Gamut mapping", &color_grading.gamut_mapping);

    ImGui::SliderFloat("Exposure", &color_grading.exposure, -10.0f, 10.0f);
    ImGui::SliderFloat("Night adaptation", &color_grading.night_adaptation,
                       0.0f, 1.0f);

    ImGui::Indent();
    if (ImGui::CollapsingHeader("White balance")) {
      int temperature = color_grading.temperature * 100.0f;
      int tint = color_grading.tint * 100.0f;
      ImGui::SliderInt("Temperature", &temperature, -100, 100);
      ImGui::SliderInt("Tint", &tint, -100, 100);
      color_grading.temperature = temperature / 100.0f;
      color_grading.tint = tint / 100.0f;
    }

    if (ImGui::CollapsingHeader("Channel mixer")) {
      PushSliderColors(0.0f / 7.0f);
      ImGui::VSliderFloat("##outRed.r", kVerticalSliderSize,
                          &color_grading.out_red.r, -2.0f, 2.0f, "");
      TooltipFloat(color_grading.out_red.r);
      ImGui::SameLine();
      ImGui::VSliderFloat("##outRed.g", kVerticalSliderSize,
                          &color_grading.out_red.g, -2.0f, 2.0f, "");
      TooltipFloat(color_grading.out_red.g);
      ImGui::SameLine();
      ImGui::VSliderFloat("##outRed.b", kVerticalSliderSize,
                          &color_grading.out_red.b, -2.0f, 2.0f, "");
      TooltipFloat(color_grading.out_red.b);
      ImGui::SameLine(0.0f, 18.0f);
      PopSliderColors();

      PushSliderColors(2.0f / 7.0f);
      ImGui::VSliderFloat("##outGreen.r", kVerticalSliderSize,
                          &color_grading.out_green.r, -2.0f, 2.0f, "");
      TooltipFloat(color_grading.out_green.r);
      ImGui::SameLine();
      ImGui::VSliderFloat("##outGreen.g", kVerticalSliderSize,
                          &color_grading.out_green.g, -2.0f, 2.0f, "");
      TooltipFloat(color_grading.out_green.g);
      ImGui::SameLine();
      ImGui::VSliderFloat("##outGreen.b", kVerticalSliderSize,
                          &color_grading.out_green.b, -2.0f, 2.0f, "");
      TooltipFloat(color_grading.out_green.b);
      ImGui::SameLine(0.0f, 18.0f);
      PopSliderColors();

      PushSliderColors(4.0f / 7.0f);
      ImGui::VSliderFloat("##outBlue.r", kVerticalSliderSize,
                          &color_grading.out_blue.r, -2.0f, 2.0f, "");
      TooltipFloat(color_grading.out_blue.r);
      ImGui::SameLine();
      ImGui::VSliderFloat("##outBlue.g", kVerticalSliderSize,
                          &color_grading.out_blue.g, -2.0f, 2.0f, "");
      TooltipFloat(color_grading.out_blue.g);
      ImGui::SameLine();
      ImGui::VSliderFloat("##outBlue.b", kVerticalSliderSize,
                          &color_grading.out_blue.b, -2.0f, 2.0f, "");
      TooltipFloat(color_grading.out_blue.b);
      PopSliderColors();
    }
    if (ImGui::CollapsingHeader("Tonal ranges")) {
      ImGui::ColorEdit3("Shadows", &color_grading.shadows.x);
      ImGui::SliderFloat("Weight##shadowsWeight", &color_grading.shadows.w,
                         -2.0f, 2.0f);
      ImGui::ColorEdit3("Mid-tones", &color_grading.midtones.x);
      ImGui::SliderFloat("Weight##midTonesWeight", &color_grading.midtones.w,
                         -2.0f, 2.0f);
      ImGui::ColorEdit3("Highlights", &color_grading.highlights.x);
      ImGui::SliderFloat("Weight##highlightsWeight",
                         &color_grading.highlights.w, -2.0f, 2.0f);
      ImGui::SliderFloat4("Ranges", &color_grading.ranges.x, 0.0f, 1.0f);
      // Normalize ranges.
      float4& ranges = color_grading.ranges;
      ranges.y = clamp(ranges.y, ranges.x + 1e-5f, ranges.w - 1e-5f);  // darks
      ranges.z = clamp(ranges.z, ranges.x + 1e-5f, ranges.w - 1e-5f);  // lights
      ComputeRangePlot(color_grading, range_plot_);
      ImGuiExt::PlotLinesSeries(
          "", 3, RangePlotSeriesStart, GetRangePlotValue, RangePlotSeriesEnd,
          range_plot_.data(), kPlotSize, 0, "", 0.0f, 1.0f, kPlotLinesWideSize);
    }
    if (ImGui::CollapsingHeader("Color decision list")) {
      ImGui::SliderFloat3("Slope", &color_grading.slope.x, 0.0f, 2.0f);
      ImGui::SliderFloat3("Offset", &color_grading.offset.x, -0.5f, 0.5f);
      ImGui::SliderFloat3("Power", &color_grading.power.x, 0.0f, 2.0f);
    }
    if (ImGui::CollapsingHeader("Adjustments")) {
      ImGui::SliderFloat("Contrast", &color_grading.contrast, 0.0f, 2.0f);
      ImGui::SliderFloat("Vibrance", &color_grading.vibrance, 0.0f, 2.0f);
      ImGui::SliderFloat("Saturation", &color_grading.saturation, 0.0f, 2.0f);
    }
    if (ImGui::CollapsingHeader("Curves")) {
      ImGui::Checkbox("Linked curves", &color_grading.linked_curves);

      ComputeCurvePlot(color_grading, curve_plot_);

      if (!color_grading.linked_curves) {
        PushSliderColors(0.0f / 7.0f);
        ImGui::VSliderFloat("##curveGamma.r", kVerticalSliderSize,
                            &color_grading.gamma.r, 0.0f, 4.0f, "");
        TooltipFloat(color_grading.gamma.r);
        ImGui::SameLine();
        ImGui::VSliderFloat("##curveMid.r", kVerticalSliderSize,
                            &color_grading.mid_point.r, 0.0f, 2.0f, "");
        TooltipFloat(color_grading.mid_point.r);
        ImGui::SameLine();
        ImGui::VSliderFloat("##curveScale.r", kVerticalSliderSize,
                            &color_grading.scale.r, 0.0f, 4.0f, "");
        TooltipFloat(color_grading.scale.r);
        ImGui::SameLine(0.0f, 18.0f);
        PopSliderColors();

        ImGui::PushStyleColor(ImGuiCol_PlotLines,
                              (ImVec4)ImColor::HSV(0.0f, 0.7f, 0.8f));
        ImGui::PlotLines("", curve_plot_.data(), kPlotSize, 0, "Red", 0.0f,
                         2.0f, kPlotLinesSize);
        ImGui::PopStyleColor();

        PushSliderColors(2.0f / 7.0f);
        ImGui::VSliderFloat("##curveGamma.g", kVerticalSliderSize,
                            &color_grading.gamma.g, 0.0f, 4.0f, "");
        TooltipFloat(color_grading.gamma.g);
        ImGui::SameLine();
        ImGui::VSliderFloat("##curveMid.g", kVerticalSliderSize,
                            &color_grading.mid_point.g, 0.0f, 2.0f, "");
        TooltipFloat(color_grading.mid_point.g);
        ImGui::SameLine();
        ImGui::VSliderFloat("##curveScale.g", kVerticalSliderSize,
                            &color_grading.scale.g, 0.0f, 4.0f, "");
        TooltipFloat(color_grading.scale.g);
        ImGui::SameLine(0.0f, 18.0f);
        PopSliderColors();

        ImGui::PushStyleColor(
            ImGuiCol_PlotLines,
            static_cast<ImVec4>(ImColor::HSV(0.3f, 0.7f, 0.8f)));
        ImGui::PlotLines("", curve_plot_.data() + kPlotSize, kPlotSize, 0,
                         "Green", 0.0f, 2.0f, kPlotLinesSize);
        ImGui::PopStyleColor();

        PushSliderColors(4.0f / 7.0f);
        ImGui::VSliderFloat("##curveGamma.b", kVerticalSliderSize,
                            &color_grading.gamma.b, 0.0f, 4.0f, "");
        TooltipFloat(color_grading.gamma.b);
        ImGui::SameLine();
        ImGui::VSliderFloat("##curveMid.b", kVerticalSliderSize,
                            &color_grading.mid_point.b, 0.0f, 2.0f, "");
        TooltipFloat(color_grading.mid_point.b);
        ImGui::SameLine();
        ImGui::VSliderFloat("##curveScale.b", kVerticalSliderSize,
                            &color_grading.scale.b, 0.0f, 4.0f, "");
        TooltipFloat(color_grading.scale.b);
        ImGui::SameLine(0.0f, 18.0f);
        PopSliderColors();

        ImGui::PushStyleColor(
            ImGuiCol_PlotLines,
            static_cast<ImVec4>(ImColor::HSV(0.6f, 0.7f, 0.8f)));
        ImGui::PlotLines("", curve_plot_.data() + 2 * kPlotSize, kPlotSize, 0,
                         "Blue", 0.0f, 2.0f, kPlotLinesSize);
        ImGui::PopStyleColor();
      } else {
        ImGui::VSliderFloat("##curveGamma", kVerticalSliderSize,
                            &color_grading.gamma.r, 0.0f, 4.0f, "");
        TooltipFloat(color_grading.gamma.r);
        ImGui::SameLine();
        ImGui::VSliderFloat("##curveMid", kVerticalSliderSize,
                            &color_grading.mid_point.r, 0.0f, 2.0f, "");
        TooltipFloat(color_grading.mid_point.r);
        ImGui::SameLine();
        ImGui::VSliderFloat("##curveScale", kVerticalSliderSize,
                            &color_grading.scale.r, 0.0f, 4.0f, "");
        TooltipFloat(color_grading.scale.r);
        ImGui::SameLine(0.0f, 18.0f);

        color_grading.gamma = float3{color_grading.gamma.r};
        color_grading.mid_point = float3{color_grading.mid_point.r};
        color_grading.scale = float3{color_grading.scale.r};

        ImGui::PushStyleColor(
            ImGuiCol_PlotLines,
            static_cast<ImVec4>(ImColor::HSV(0.17f, 0.21f, 0.9f)));
        ImGui::PlotLines("", curve_plot_.data(), kPlotSize, 0, "RGB", 0.0f,
                         2.0f, kPlotLinesSize);
        ImGui::PopStyleColor();
      }
    }
    ImGui::Unindent();
  }
}

}  // namespace imp::editor
