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
#include "core/editor/widgets/performance/config.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_id.h"
#include "core/sprite/sprite_renderer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_renderer.h"

namespace imp::editor {

namespace {
constexpr int kStartingUpperBound = 100;
constexpr int kMaxUpperBound = 3000;
constexpr float kUpperBoundPadding = 1.25f;
}  // namespace

RenderInfoPanel::RenderInfoPanel(BaseView& view, int buffer_size)
    : view_(view),
      buffer_(buffer_size),
      frame_number_(0),
      upper_bound_(kStartingUpperBound) {}

RenderInfoPanel::~RenderInfoPanel() = default;

void RenderInfoPanel::DrawPanel(int width, int height, int time_span_seconds) {
  if (ImPlot::BeginPlot("##RenderInfo", ImVec2(width, height))) {
    ImPlotCond plot_cond =
        state_ == MonitorState::kPaused ? ImPlotCond_None : ImPlotCond_Always;

    ImPlot::SetupAxes("Frame number", "Number of renderables");
    ImPlot::SetupAxisLimits(
        ImAxis_X1,
        frame_number_ - time_span_seconds * details::kNumDisplayValuesPerSecond,
        frame_number_, plot_cond);

    ImPlot::SetupAxisLimits(ImAxis_Y1, 0, upper_bound_ * kUpperBoundPadding);
    ImPlot::SetupAxisLimitsConstraints(ImAxis_Y1, 0, kMaxUpperBound);
    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

    if (!buffer_.empty()) {
      ImPlot::PlotLine(
          "Total Filament renderables", &buffer_.data()[0].frame_number,
          &buffer_.data()[0].num_renderables, buffer_.data().size(), 0,
          buffer_.marker(), sizeof(RenderInfo));

      if (has_sprites_) {
        ImPlot::PlotLine("Sprite Renderer", &buffer_.data()[0].frame_number,
                         &buffer_.data()[0].num_sprites, buffer_.data().size(),
                         0, buffer_.marker(), sizeof(RenderInfo));
      }

      if (has_gltfs_) {
        ImPlot::PlotLine("Gltf Renderer", &buffer_.data()[0].frame_number,
                         &buffer_.data()[0].num_gltfs, buffer_.data().size(), 0,
                         buffer_.marker(), sizeof(RenderInfo));
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

    ImPlot::EndPlot();
  }
}

void RenderInfoPanel::DrawHighlightFrame(int frame_number,
                                         ImDrawList* draw_list) {
  if (!draw_list) {
    return;
  }

  float tool_l = ImPlot::PlotToPixels(frame_number - 0.5f, 0).x;
  float tool_r = ImPlot::PlotToPixels(frame_number + 0.5f, 0).x;
  float tool_t = ImPlot::GetPlotPos().y;
  float tool_b = tool_t + ImPlot::GetPlotSize().y;
  ImPlot::PushPlotClipRect();
  draw_list->AddRectFilled(ImVec2(tool_l, tool_t), ImVec2(tool_r, tool_b),
                           IM_COL32(128, 128, 128, 64));
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

void RenderInfoPanel::OnStateChanged(MonitorPanel::MonitorState state) {
  state_ = state;
}

void RenderInfoPanel::Update(absl::Duration elapsed_time,
                             absl::Duration delta_time) {
  int renderable_count = view_.GetHost()->GetScene()->getRenderableCount();
  if (renderable_count > upper_bound_) {
    upper_bound_ = renderable_count;
  }

  BaseComponentPool* sprite_pool =
      view_.GetComponentManager().GetComponentPoolById(
          kComponentId<SpriteRenderer>);
  int num_sprite_renderers = 0;
  has_sprites_ = sprite_pool != nullptr;
  if (sprite_pool) {
    num_sprite_renderers = sprite_pool->GetComponentCount();
  }

  BaseComponentPool* gltf_pool =
      view_.GetComponentManager().GetComponentPoolById(
          kComponentId<GltfRenderer>);

  int num_gltf_renderers = 0;
  has_gltfs_ = gltf_pool != nullptr;
  if (gltf_pool) {
    num_gltf_renderers = gltf_pool->GetComponentCount();
  }

  ++frame_number_;
  buffer_.push_back(RenderInfo{.frame_number = frame_number_,
                               .num_renderables = renderable_count,
                               .num_sprites = num_sprite_renderers,
                               .num_gltfs = num_gltf_renderers});
}

}  // namespace imp::editor
