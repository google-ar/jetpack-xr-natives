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

#include "core/editor/widgets/performance/imgui_helper.h"

#include <algorithm>
#include <array>

#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "dear_imgui/imgui.h"
#include "implot/implot.h"

namespace imp::editor {

namespace {
constexpr ImU32 kLabelBackgroundColor = IM_COL32(0, 0, 0, 100);
constexpr float kLabelXOffset = 5.0f;
constexpr float kLabelYPadding = 1.0f;
constexpr float kLabelXPadding = 2.0f;
constexpr float kTextHeightPadding = 2.0f;
constexpr int kMaxLabels = 16;
}  // namespace

void ImGuiHelper::DrawLegendItem(const char* label, bool& show_flag,
                                 int color_index) {
  const ImVec2 swatch_size(10, 10);
  const ImVec2 p = ImGui::GetCursorScreenPos();

  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  ImVec4 color_vec = ImPlot::GetColormapColor(color_index);

  if (!show_flag) color_vec.w *= 0.5f;  // Dim color if not shown

  const ImU32 col = ImGui::ColorConvertFloat4ToU32(color_vec);

  draw_list->AddRectFilled(p, ImVec2(p.x + swatch_size.x, p.y + swatch_size.y),
                           col);

  ImGui::Dummy(swatch_size);
  ImGui::SameLine();

  bool faded = false;
  if (!show_flag) {
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    faded = true;
  }

  // Vertically center the text next to the swatch
  const float text_height = ImGui::GetTextLineHeight();
  const float offset_y = (swatch_size.y - text_height) * 0.5f;
  ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset_y);

  ImGui::Text(label);

  if (ImGui::IsItemClicked()) {
    show_flag = !show_flag;
  }

  if (faded) {
    ImGui::PopStyleVar();
  }
}

void ImGuiHelper::DrawFrameValueLabels(const int frame_number,
                                       absl::Span<const LabelData> labels,
                                       ImDrawList* draw_list) {
  if (frame_number < 0 || !draw_list) return;

  ImPlot::PushPlotClipRect();

  const float label_x =
      ImPlot::PlotToPixels(frame_number + 0.5, 0).x + kLabelXOffset;

  struct LabelLayout {
    const LabelData* data;
    float y_pos;
    float height;
    ImVec2 text_size;
    char buffer[64];
  };

  std::array<LabelLayout, kMaxLabels> layouts;
  int layout_count = 0;

  // In the ideal case labels are drawn where the selected frame line intersects
  // the plot they're associated with.
  // If two plots are close together then their labels can overlap.
  // If the y-value is high/low or there's a lot of labels that were adjusted
  // not to overlap, then they might go off the top/bottom of the plot.

  // To handle this we:
  // 1. Filter out labels that are disabled in the legend.
  // 2. Calculate ideal positions for visible labels.
  // 3. Adjust labels so that they don't overlap each other.
  // 4. Adjust labels so that they don't go beyond the bottom of the plot.
  // 5. Adjust labels so that they don't go beyond the top of the plot.

  // This can still fail in certain cases but for now it works fine.

  // Filter out labels that are disabled in the legend.
  // Calculate initial positions and sizes for visible labels.
  for (int i = 0; i < labels.size(); ++i) {
    if (!labels[i].show_flag) continue;
    if (layout_count >= kMaxLabels) continue;

    const LabelData& label = labels[i];
    LabelLayout& layout = layouts[layout_count];
    layout.data = &label;

    absl::SNPrintF(layout.buffer, sizeof(layout.buffer), "%.2f%s", label.y_val,
                   label.unit);

    layout.text_size = ImGui::CalcTextSize(layout.buffer);
    layout.height = layout.text_size.y + kTextHeightPadding;

    const ImVec2 pos = ImPlot::PlotToPixels(ImPlotPoint(0, label.y_val));
    layout.y_pos = pos.y;
    layout_count++;
  }

  // Sort layouts based on their y-value in descending order.
  std::sort(layouts.begin(), layouts.begin() + layout_count,
            [](const LabelLayout& a, const LabelLayout& b) {
              return a.data->y_val > b.data->y_val;
            });

  const ImVec2 plot_pos = ImPlot::GetPlotPos();
  const ImVec2 plot_size = ImPlot::GetPlotSize();
  const float plot_y_min = plot_pos.y;
  const float plot_y_max = plot_pos.y + plot_size.y;

  // Adjust labels so that they don't overlap.
  for (int i = 1; i < layout_count; ++i) {
    const float prev_label_end = layouts[i - 1].y_pos + layouts[i - 1].height;
    if (layouts[i].y_pos < prev_label_end) {
      layouts[i].y_pos = prev_label_end;
    }
  }

  // Adjust labels so that they don't go beyond the bottom of the plot.
  // This can be a bit confusing to read since y-axis grows downward.
  // We're subtracting to move up on the user's screen.
  float shift_upwards = 0.0f;
  for (int i = layout_count - 1; i >= 0; --i) {
    layouts[i].y_pos += shift_upwards;

    const float label_bottom = layouts[i].y_pos + layouts[i].height;
    if (label_bottom > plot_y_max) {
      const float overflow = label_bottom - plot_y_max;
      layouts[i].y_pos -= overflow;
      shift_upwards -= overflow;  // Propagate shift to labels above
    }
  }
  // Adjust labels so that they don't go beyond the top of the plot.
  if (layout_count > 0 && layouts[0].y_pos < plot_y_min) {
    // We only need the offset of the first label to the top of the plot.
    // All other labels will be shifted the same amount since they're drawn
    // beneath it.
    const float shift_downwards = plot_y_min - layouts[0].y_pos;
    for (int i = 0; i < layout_count; ++i) {
      layouts[i].y_pos += shift_downwards;
    }
  }

  // Draw the labels.
  for (int i = 0; i < layout_count; ++i) {
    const LabelLayout& layout = layouts[i];
    const LabelData& label = *layout.data;

    const ImVec4 color_vec = ImPlot::GetColormapColor(label.color_index);
    const ImU32 color = ImGui::ColorConvertFloat4ToU32(color_vec);

    const float current_y =
        std::clamp(layout.y_pos, plot_y_min, plot_y_max - layout.height);

    const ImVec2 pos(label_x, current_y);

    draw_list->AddRectFilled(
        pos,
        ImVec2(pos.x + layout.text_size.x + 2 * kLabelXPadding,
               pos.y + layout.height),
        kLabelBackgroundColor);
    draw_list->AddText(ImVec2(pos.x + kLabelXPadding, pos.y + kLabelYPadding),
                       color, layout.buffer);
  }

  ImPlot::PopPlotClipRect();
}

}  // namespace imp::editor
