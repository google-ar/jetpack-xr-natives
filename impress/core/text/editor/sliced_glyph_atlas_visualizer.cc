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

#include "core/text/editor/sliced_glyph_atlas_visualizer.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/debugging/leak_check.h"
#include "absl/strings/str_format.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "third_party/implot/implot.h"
#include "core/config.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"

namespace imp::editor {

namespace {

#if IMP_PLATFORM(DESKTOP) || IMP_PLATFORM(WASM)
constexpr int kPanelHeight = 600;
#else
constexpr int kPanelHeight = 300;
#endif

constexpr ImU32 kGlyphOutlineColors[] = {
    IM_COL32(0, 255, 0, 128),
    IM_COL32(255, 255, 0, 128),
    IM_COL32(238, 130, 238, 128),
};

constexpr int kGlyphOutlineColorsCount =
    sizeof(kGlyphOutlineColors) / sizeof(ImU32);

constexpr ImU32 kGlyphOriginColor = IM_COL32(255, 0, 255, 128);

}  // namespace

SlicedGlyphAtlasVisualizer::SlicedGlyphAtlasVisualizer(
    BaseView& view, AtlasDataProvider provider)
    : view_(view),
      provider_(std::move(provider)),
      show_all_glyph_bounds_(false),
      show_all_glyph_origins_(false),
      enable_hover_highlight_(false),
      selected_buffer_index_(0) {
  {
    // TODO Either fix the leak or find a way to automatically
    // disable the leak check for all usages of ImPlot.
    absl::LeakCheckDisabler disabler;
    ImPlot::CreateContext();
    rgba_buffer_ = std::make_unique<uint8_t[]>(4);
    for (int i = 0; i < 4; ++i) {
      rgba_buffer_[i] = 0;
    }
  }
}

SlicedGlyphAtlasVisualizer::~SlicedGlyphAtlasVisualizer() = default;

bool SlicedGlyphAtlasVisualizer::HasContent() const {
  return provider_.get_atlas_info_func().texture != nullptr;
}

void SlicedGlyphAtlasVisualizer::DrawImGui() {
  Draw(provider_.get_atlas_info_func());
}

std::string GetSelectedBufferDescription(size_t selected_buffer_index) {
  std::string selected_buffer_description = "Composite texture";
  if (selected_buffer_index > 0) {
    size_t slice_index = selected_buffer_index - 1;
    selected_buffer_description = absl::StrFormat("Slice %d", slice_index);
  }
  return selected_buffer_description;
}

void SlicedGlyphAtlasVisualizer::Draw(const AtlasInfo& atlas_info) {
  float glyph_atlas_utilization = provider_.get_utilization_func();
  ImGui::Text("Glyph atlas, utilization: %f", glyph_atlas_utilization);

  std::string selected_buffer_description =
      GetSelectedBufferDescription(selected_buffer_index_);
  size_t slice_count = atlas_info.slice_textures.size();

  if (ImGui::BeginCombo("Displayed buffer",
                        selected_buffer_description.c_str())) {
    for (size_t i = 0; i < slice_count + 1; ++i) {
      bool isSelected = (i == selected_buffer_index_);
      if (ImGui::Selectable(GetSelectedBufferDescription(i).c_str(),
                            isSelected)) {
        selected_buffer_index_ = static_cast<int>(i);
      }
      if (isSelected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }

  constexpr int kPadding = 32;
  ImVec2 display_size = ImGui::GetIO().DisplaySize;
  int size = std::min(display_size.x, display_size.y) - kPadding;

  if (ImPlot::BeginPlot("##Atlas", ImVec2(size, size),
                        ImPlotFlags_Equal | ImPlotFlags_NoLegend |
                            ImPlotFlags_NoMouseText | ImPlotFlags_Crosshairs)) {
    ImPlotAxisFlags flags =
        ImPlotAxisFlags_NoHighlight | ImPlotAxisFlags_NoGridLines;
    ImPlot::SetupAxes(nullptr, nullptr, flags, flags);

    // Set up initial display axis limits
    ImPlot::SetupAxisLimits(ImAxis_X1, 0, 1);
    ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);

    imp::Texture* texture = nullptr;
    if (selected_buffer_index_ == 0) {
      texture = atlas_info.texture;
    } else {
      size_t slice_index = selected_buffer_index_ - 1;
      texture = atlas_info.slice_textures[slice_index];
    }
    ImPlot::PlotImage("", texture->GetTexture(),
                      /*bounds_min = */ {0, 0},
                      /*bounds_max = */ {1, 1});

    if (ImPlot::IsPlotHovered()) {
      ImPlotPoint mouse = ImPlot::GetPlotMousePos();

#if IMP_PLATFORM(DESKTOP) || IMP_PLATFORM(WASM)
      uint2 composite_size = atlas_info.texture_size * atlas_info.grid_size;
      filament::RenderTarget::Builder render_target_builder;
      render_target_builder.texture(
          filament::RenderTarget::AttachmentPoint::COLOR,
          atlas_info.texture->GetTexture());
      filament::Engine* engine = view_.GetSharedEngine();
      filament::RenderTarget* render_target =
          render_target_builder.build(*engine);
      int32_t mouse_pixel_x = (int32_t)(composite_size.x * mouse.x);
      int32_t mouse_pixel_y = (int32_t)(composite_size.y * (1 - mouse.y));
      filament::backend::PixelBufferDescriptor pixel_buffer_desc(
          rgba_buffer_.get(), 4, filament::Texture::Format::RGBA,
          filament::Texture::Type::UBYTE,
          [](void* buffer, size_t size, void* user) {}, nullptr);
      view_.GetHost()->GetRenderer()->readPixels(render_target, mouse_pixel_x,
                                                 mouse_pixel_y, 1, 1,
                                                 std::move(pixel_buffer_desc));
#endif

      if (mouse.x > 0 && mouse.x < 1 && mouse.y > 0 && mouse.y < 1) {
        float2 uv = float2{mouse.x, 1 - mouse.y};
        std::optional<GlyphInfo> glyph = provider_.get_glyph_info_func(uv);
        if (glyph.has_value()) {
          Rect bounds = glyph->uv;
          bounds.center.y = 1 - bounds.center.y;
          if (enable_hover_highlight_) {
            DrawGlyphBounds(bounds);
          } else {
            DrawGlyphOutline(bounds);
          }
          float2 origin = glyph->origin;
          origin.y = 1 - origin.y;
          DrawGlyphOrigin(bounds, origin);
          DrawToolTip(*glyph, atlas_info);
        }
      }
    }
    if (show_all_glyph_origins_ || show_all_glyph_bounds_) {
      std::vector<GlyphInfo> glyphs = provider_.get_all_glyph_info_func();
      for (int i = 0; i < glyphs.size(); ++i) {
        const GlyphInfo& glyph = glyphs[i];
        Rect bounds = glyph.uv;
        bounds.center.y = 1 - bounds.center.y;
        if (show_all_glyph_bounds_) {
          DrawGlyphOutline(bounds, i);
        }
        if (show_all_glyph_origins_) {
          float2 origin = glyph.origin;
          origin.y = 1 - origin.y;
          DrawGlyphOrigin(bounds, origin);
        }
      }
    }

    ImPlot::EndPlot();
  }

  ImGui::Checkbox("Show glyph bounds", &show_all_glyph_bounds_);
  ImGui::Checkbox("Show glyph origins", &show_all_glyph_origins_);
  ImGui::Checkbox("Enable hover highlight", &enable_hover_highlight_);
}

void SlicedGlyphAtlasVisualizer::DrawGlyphBounds(const Rect& bounds) const {
  ImDrawList* draw_list = ImPlot::GetPlotDrawList();
  if (!draw_list) {
    return;
  }

  ImVec2 min{bounds.GetMin().x, bounds.GetMin().y};
  ImVec2 max{bounds.GetMax().x, bounds.GetMax().y};
  float tool_l = ImPlot::PlotToPixels(min).x;
  float tool_r = ImPlot::PlotToPixels(max).x;
  float tool_t = ImPlot::PlotToPixels(min).y;
  float tool_b = ImPlot::PlotToPixels(max).y;
  ImPlot::PushPlotClipRect();
  draw_list->AddRectFilled(ImVec2(tool_l, tool_t), ImVec2(tool_r, tool_b),
                           IM_COL32(128, 128, 128, 128));
  ImPlot::PopPlotClipRect();
}

void SlicedGlyphAtlasVisualizer::DrawGlyphOutline(const Rect& bounds,
                                                  int color_idx) const {
  ImDrawList* draw_list = ImPlot::GetPlotDrawList();
  if (!draw_list) {
    return;
  }

  ImVec2 min{bounds.GetMin().x, bounds.GetMin().y};
  ImVec2 max{bounds.GetMax().x, bounds.GetMax().y};
  float tool_l = ImPlot::PlotToPixels(min).x;
  float tool_r = ImPlot::PlotToPixels(max).x;
  float tool_t = ImPlot::PlotToPixels(min).y;
  float tool_b = ImPlot::PlotToPixels(max).y;
  ImU32 color = kGlyphOutlineColors[color_idx % kGlyphOutlineColorsCount];
  float inset = 1.0 / 1024.0;

  ImPlot::PushPlotClipRect();
  draw_list->AddRect(ImVec2(tool_l, tool_t), ImVec2(tool_r, tool_b), color);
  draw_list->AddRect(ImVec2(tool_l + inset, tool_t + inset),
                     ImVec2(tool_r - inset, tool_b - inset), color);
  ImPlot::PopPlotClipRect();
}

void SlicedGlyphAtlasVisualizer::DrawGlyphOrigin(const Rect& bounds,
                                                 const float2& origin) const {
  ImDrawList* draw_list = ImPlot::GetPlotDrawList();
  if (!draw_list) {
    return;
  }

  float2 min = bounds.GetMin();
  float2 max = bounds.GetMax();
  ImVec2 bounds_min = ImPlot::PlotToPixels(std::min(min.x, origin.x),
                                           std::min(min.y, origin.y));
  ImVec2 bounds_max = ImPlot::PlotToPixels(std::max(max.x, origin.x),
                                           std::max(max.y, origin.y));
  ImVec2 pixel_origin = ImPlot::PlotToPixels(origin.x, origin.y);
  ImPlot::PushPlotClipRect();
  draw_list->AddLine(ImVec2(bounds_min.x, pixel_origin.y),
                     ImVec2(bounds_max.x, pixel_origin.y), kGlyphOriginColor);
  draw_list->AddLine(ImVec2(pixel_origin.x, bounds_min.y),
                     ImVec2(pixel_origin.x, bounds_max.y), kGlyphOriginColor);
  ImPlot::PopPlotClipRect();
}

void SlicedGlyphAtlasVisualizer::DrawToolTip(
    const GlyphInfo& info, const AtlasInfo& atlas_info) const {
#if IMP_PLATFORM(DESKTOP) || IMP_PLATFORM(WASM)
  ImVec4 rgba = ImVec4(rgba_buffer_[0] / 255.0f, rgba_buffer_[1] / 255.0f,
                       rgba_buffer_[2] / 255.0f, rgba_buffer_[3] / 255.0f);
  ImGui::ColorTooltip("##rgba_preview", &rgba.x,
                      ImGuiColorEditFlags_InputRGB |
                          ImGuiColorEditFlags_AlphaPreview |
                          ImGuiColorEditFlags_AlphaPreviewHalf);
#endif
  ImGui::BeginTooltip();
  if (info.is_stroke) {
    ImGui::Text("Glyph: stroked %s", info.glyph.c_str());
  } else {
    ImGui::Text("Glyph: %s", info.glyph.c_str());
  }
  uint2 composite_size = atlas_info.texture_size * atlas_info.grid_size;
  ImGui::Text("Glyph origin: (%f, %f)", info.origin.x * composite_size.x,
              info.origin.y * composite_size.y);
  ImGui::EndTooltip();
}

}  // namespace imp::editor
