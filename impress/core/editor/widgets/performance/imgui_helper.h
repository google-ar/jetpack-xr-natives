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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_IMGUI_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_IMGUI_HELPER_H_
#include <cstddef>

#include "absl/types/span.h"
#include "dear_imgui/imgui.h"

namespace imp::editor {

class ImGuiHelper {
 public:
  ImGuiHelper() = delete;

  static void SelectFrame(size_t frame_number) {
    selected_frame_number_ = frame_number;
  }
  static size_t GetSelectedFrameNumber() { return selected_frame_number_; }

  // Draws a legend item for the custom legends used by performance graphs.
  static void DrawLegendItem(const char* label, bool& show_flag,
                             int color_index);

  // Data for a single label to be drawn by DrawFrameValueLabels.
  struct LabelData {
    float y_val;
    const char* unit;
    int color_index;
    bool show_flag;
  };

  // Draws labels next to a vertical line on a plot for the given frame number.
  static void DrawFrameValueLabels(int frame_number,
                                   absl::Span<const LabelData> labels,
                                   ImDrawList* draw_list);

 private:
  inline static size_t selected_frame_number_ = -1;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_IMGUI_HELPER_H_
