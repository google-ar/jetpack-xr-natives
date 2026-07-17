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

#include "core/editor/widgets/performance/render_info_panel.h"

#include <cmath>

#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "implot/implot.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/config.h"
#include "core/editor/widgets/performance/imgui_helper.h"
#include "core/editor/widgets/performance/performance_window.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_id.h"
#include "core/performance/profiler.h"
#include "core/sprite/sprite_renderer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_renderer.h"

namespace imp::editor {

namespace {
constexpr int kStartingUpperBound = 100;
}  // namespace

RenderInfoPanel::RenderInfoPanel(PerformanceWindow& performance_window,
                                 BaseView& view, int buffer_size)
    : performance_window_(performance_window),
      view_(view),
      buffer_(buffer_size),
      upper_bound_(kStartingUpperBound) {}

RenderInfoPanel::~RenderInfoPanel() = default;

void RenderInfoPanel::DrawLegend(float width, float height) {
  if (ImGui::BeginChild("legend", ImVec2(width, height), true)) {
    ImGui::Text("Rendering");
    ImGui::Separator();

    int color_idx = 0;

    ImGuiHelper::DrawLegendItem("Renderables", show_renderables_, color_idx++);
    if (has_sprites_) {
      ImGuiHelper::DrawLegendItem("Sprites", show_sprites_, color_idx++);
    }
    if (has_gltfs_) {
      ImGuiHelper::DrawLegendItem("Gltfs", show_gltfs_, color_idx++);
    }
  }
  ImGui::EndChild();
}

void RenderInfoPanel::DrawPanel(int width, int height, int time_span_seconds) {
  IMP_TRACE();

  constexpr float legend_width = 150.0f;
  DrawLegend(legend_width, height);
  ImGui::SameLine();  // Place plot to the right of the legend.

  // Provides a border around the plot area since we removed the padding.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  if (ImGui::BeginChild("##RenderInfoChild", ImVec2(width, height), true)) {
    // Remove padding around the plot area
    ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(0, 0));
    ImPlot::PushStyleVar(ImPlotStyleVar_FitPadding, ImVec2(0.0f, 0.1f));

    if (ImPlot::BeginPlot("##RenderInfo", ImVec2(width, height),
                          ImPlotFlags_NoLegend | ImPlotFlags_NoFrame)) {
      ImPlot::SetupAxes(nullptr, nullptr,
                        ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoTickLabels,
                        ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_AutoFit);
      ImPlot::SetupAxisLimits(
          ImAxis_X1,
          Profiler::GetCurrentFrameIndex() -
              time_span_seconds * details::kNumDisplayValuesPerSecond,
          Profiler::GetCurrentFrameIndex(), ImPlotCond_Always);

      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

      if (!buffer_.empty()) {
        if (show_renderables_) {
          ImPlot::PlotLine(
              "Total Filament renderables", &buffer_.data()[0].frame_number,
              &buffer_.data()[0].num_renderables, buffer_.data().size(), 0,
              buffer_.marker(), sizeof(RenderInfo));
        }

        if (has_sprites_ && show_sprites_) {
          ImPlot::PlotLine("Sprite Renderer", &buffer_.data()[0].frame_number,
                           &buffer_.data()[0].num_sprites,
                           buffer_.data().size(), 0, buffer_.marker(),
                           sizeof(RenderInfo));
        }

        if (has_gltfs_ && show_gltfs_) {
          ImPlot::PlotLine("Gltf Renderer", &buffer_.data()[0].frame_number,
                           &buffer_.data()[0].num_gltfs, buffer_.data().size(),
                           0, buffer_.marker(), sizeof(RenderInfo));
        }

        if (ImPlot::IsPlotHovered()) {
          ImDrawList* draw_list = ImPlot::GetPlotDrawList();
          ImPlotPoint mouse = ImPlot::GetPlotMousePos();

          int hovered_frame = static_cast<int>(std::floor(mouse.x));

          DrawHighlightFrame(hovered_frame, draw_list);

          if (hovered_frame > 0) {
            DrawToolTip(hovered_frame);
          }
        }
      }

      int selected_frame_number = performance_window_.GetSelectedFrameNumber();
      ImDrawList* draw_list_overlays = ImPlot::GetPlotDrawList();
      DrawHighlightFrame(selected_frame_number, draw_list_overlays,
                         IM_COL32(255, 255, 255, 200), 0.3f);
      DrawSelectedFrameLabels(selected_frame_number, draw_list_overlays);

      ImPlot::EndPlot();
    }
    ImPlot::PopStyleVar();  // ImPlotStyleVar_FitPadding
    ImPlot::PopStyleVar();  // ImPlotStyleVar_PlotPadding
  }
  ImGui::EndChild();
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding
}

void RenderInfoPanel::DrawSelectedFrameLabels(int frame_number,
                                              ImDrawList* draw_list) {
  if (frame_number < 0 || buffer_.empty()) return;

  const RenderInfo* frame_info = nullptr;
  for (const auto& info : buffer_.data()) {
    if (static_cast<int>(info.frame_number) == frame_number) {
      frame_info = &info;
      break;
    }
  }

  if (!frame_info) return;

  // Order must match plot order.
  int idx = 0;
  frame_value_data_[idx].y_val = frame_info->num_renderables;
  frame_value_data_[idx].unit = "";
  frame_value_data_[idx].color_index = idx;
  frame_value_data_[idx].show_flag = show_renderables_;

  if (has_sprites_) {
    idx++;
    frame_value_data_[idx].y_val = frame_info->num_sprites;
    frame_value_data_[idx].unit = "";
    frame_value_data_[idx].color_index = idx;
    frame_value_data_[idx].show_flag = show_sprites_;
  }

  if (has_gltfs_) {
    idx++;
    frame_value_data_[idx].y_val = frame_info->num_gltfs;
    frame_value_data_[idx].unit = "";
    frame_value_data_[idx].color_index = idx;
    frame_value_data_[idx].show_flag = show_gltfs_;
  }

  ImGuiHelper::DrawFrameValueLabels(
      frame_number, absl::MakeSpan(frame_value_data_).subspan(0, idx + 1),
      draw_list);
}

void RenderInfoPanel::DrawHighlightFrame(int frame_number,
                                         ImDrawList* draw_list, ImU32 color,
                                         float size) {
  if (!draw_list) {
    return;
  }

  float tool_l = ImPlot::PlotToPixels(frame_number - size, 0).x;
  float tool_r = ImPlot::PlotToPixels(frame_number + size, 0).x;
  float tool_t = ImPlot::GetPlotPos().y;
  float tool_b = tool_t + ImPlot::GetPlotSize().y;
  ImPlot::PushPlotClipRect();
  draw_list->AddRectFilled(ImVec2(tool_l, tool_t), ImVec2(tool_r, tool_b),
                           color);
  ImPlot::PopPlotClipRect();
}

void RenderInfoPanel::DrawToolTip(int frame_number) {
  const RenderInfo& info =
      buffer_.data()[(frame_number - 1) % buffer_.capacity()];

  ImGui::BeginTooltip();
  ImGui::Text("Total Filament renderables:  %d", info.num_renderables);

  if (has_sprites_) {
    ImGui::Text("SpriteRenderers: %d", info.num_sprites);
  }

  if (has_gltfs_) {
    ImGui::Text("GltfRenderers:  %d", info.num_gltfs);
  }

  ImGui::EndTooltip();
}

void RenderInfoPanel::Update(absl::Duration elapsed_time,
                             absl::Duration delta_time) {
  int renderable_count = view_.GetHost()->GetScene()->getRenderableCount();
  if (renderable_count > upper_bound_) {
    upper_bound_ = renderable_count;
  }

  BaseComponentPool* sprite_pool =
      view_.GetComponentManager().GetComponentPoolById(
          GetComponentTypeId<SpriteRenderer>());
  int num_sprite_renderers = 0;
  has_sprites_ = sprite_pool != nullptr;
  if (sprite_pool) {
    num_sprite_renderers = sprite_pool->GetComponentCount();
  }

  BaseComponentPool* gltf_pool =
      view_.GetComponentManager().GetComponentPoolById(
          GetComponentTypeId<GltfRenderer>());

  int num_gltf_renderers = 0;
  has_gltfs_ = gltf_pool != nullptr;
  if (gltf_pool) {
    num_gltf_renderers = gltf_pool->GetComponentCount();
  }

  buffer_.push_back(RenderInfo{
      .frame_number = static_cast<int>(Profiler::GetCurrentFrameIndex()),
      .num_renderables = renderable_count,
      .num_sprites = num_sprite_renderers,
      .num_gltfs = num_gltf_renderers});
}

}  // namespace imp::editor
