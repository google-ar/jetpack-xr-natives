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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FLAME_GRAPH_COLORS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FLAME_GRAPH_COLORS_H_

#include <array>

#include "dear_imgui/imgui.h"

// Colors for the flame graph.
// constexpr generation of inactive frame colors needed its own namespace.
namespace flame_graph_colors {

// The number of colors to use for the flame graph.
static constexpr int kNumColors = 16;

// Colors to be used for nodes in the flame graph.
static constexpr std::array<ImU32, kNumColors> kFrameColors = {
    0xFF1E25B3,  // Red 40 - #B3251E
    0xFF575EF5,  // Red 60 - #F55E57

    0xFF015AC0,  // Orange 50 - #C05A01
    0xFF418DFF,  // Orange 70 - #FF8D41

    0xFF6E0DB6,  // Pink 40 - #B60D6E
    0xFFD27DFF,  // Pink 70 - #FF7DD2

    0xFF378912,  // Green 50 - #128937
    0xFF65C244,  // Green 70 - #44C265

    0xFF9B8300,  // Cyan 50 - #00839B
    0xFFDFBB00,  // Cyan 70 - #00BBDF

    0xFFD23874,  // Purple 40 - #7438D2
    0xFFFF72AD,  // Purple 60 - #AD72FF

    0xFF007BD3,  // Yellow 60 - #D37B00
    0xFF00BDFC,  // Yellow 80 - #FCBD00

    0xFFCE5711,  // Blue 40 - #1157CE
    0xFFF88F4E,  // Blue 60 - #4E8FF8
};

// ImU32 is an int32 representing an ABGR color.
// Alpha is the first byte so we clear it and then OR in the desired alpha.
static constexpr ImU32 ToUnselectedColor(ImU32 color) {
  return (color & 0x00FFFFFF) | 0x96000000;
}

// Generate the unselected colors array at compile time.
static constexpr std::array<ImU32, kNumColors> kUnselectedFrameColors = [] {
  std::array<ImU32, kNumColors> colors{};
  for (int i = 0; i < kNumColors; ++i) {
    colors[i] = ToUnselectedColor(kFrameColors[i]);
  }
  return colors;
}();

}  // namespace flame_graph_colors

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FLAME_GRAPH_COLORS_H_
