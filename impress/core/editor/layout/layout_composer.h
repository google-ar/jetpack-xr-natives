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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_LAYOUT_COMPOSER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_LAYOUT_COMPOSER_H_

#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "dear_imgui/imgui.h"
#include "core/common/invocable.h"
#include "core/editor/layout/layout_config.proto.imp.h"
#include "core/editor/widget.h"
#include "core/editor/widget_layout_info.h"
#include "core/view/utils/string_map.h"

namespace imp::editor {

// An API for coordinating Editor UI draw calls in a platform-dependent screen
// layout. Invocables can be queued to be drawn in the Details section, the
// Scene section, or as a standalone tab.
class LayoutComposer {
 public:
  // Information about a sub-window (e.g. scene window, details window) that is
  // drawn by the LayoutComposer for world layout.
  struct SubWindowInfo {
    std::string label;
    ImVec2 window_size;
    ImVec2 window_position;
  };

  explicit LayoutComposer(LayoutConfig layout_config);

  virtual ~LayoutComposer() = default;

  virtual void DrawWidget(const WidgetLayoutInfo& layout_info, Widget* widget);

  // Draws the platform-dependent Editor UI using all queued Invocables, which
  // are afterwards cleared from memory.
  // This function should be called once per frame to draw the Editor UI.
  virtual void DrawLayout();

  // Returns the information about the sub-windows that are registered for world
  // layout.
  absl::Span<const SubWindowInfo> GetSubWindowInfo();

 private:
  // Draws the given Invocable in the Details section.
  void DrawInDetailsSection(imp::Invocable<void()> draw_function);

  // Queues the given Invocable to be drawn in the Details section, under a
  // ImGui::CollapsingHeader.
  void DrawInDetailsSectionAsHeader(
      absl::string_view header_label, imp::Invocable<void()> draw_function,
      imp::Invocable<void()> on_close_button_pressed = {},
      ImGuiTreeNodeFlags additional_flags =
          ImGuiTreeNodeFlags_CollapsingHeader);

  // Queues the given Invocable to be drawn in the Scene section.
  void DrawInSceneSection(imp::Invocable<void()> draw_function);

  // Queues the given Invocable to be drawn in the Scene section, under a
  // ImGui::CollapsingHeader.
  void DrawInSceneSectionAsHeader(
      absl::string_view header_label, imp::Invocable<void()> draw_function,
      imp::Invocable<void()> on_close_button_pressed = Invocable<void()>(),
      ImGuiTreeNodeFlags additional_flags = ImGuiTreeNodeFlags_None);

  // Queues the given Invocable to be drawn as its own tab in a central
  // tabbed window.
  void DrawAsStandaloneTab(
      absl::string_view tab_label, imp::Invocable<void()> draw_function,
      ImGuiTabItemFlags flags = ImGuiTabBarFlags_FittingPolicyScroll,
      bool draw_before_previous_tabs = false);

  // Queues the given Invocable to be drawn as its own menu in the main menu
  // bar.
  void DrawAsMenuInMenuBar(absl::string_view menu_label,
                           imp::Invocable<void()> draw_function);

  // Queues the given Invocable to be drawn in the toolbar section.
  //
  // The toolbar is a panel in the top-middle of the screen that only exists in
  // the MULTIPLE_WINDOWS layout mode.
  void DrawInToolbar(imp::Invocable<void()> draw_function);

  // Queues the given Invocable to be drawn at the end after the layout.
  void DrawAfterLayout(imp::Invocable<void()> draw_function);

  // Draws a window for world layout. The window will be drawn from left to
  // right without overlapping.
  void DrawWindowForWorldLayout(absl::string_view window_label,
                                imp::Invocable<void()> draw_function);

  // Draws a standalone details window.
  void DrawStandaloneDetailsWindow();
  // Draws the details draw functions and a placeholder if none exist.
  void DrawDetailsSectionContents();
  // Draws a standalone scene window.
  void DrawStandaloneSceneWindow();
  // Draws the scene draw functions.
  void DrawSceneSectionContents();
  // Draws an individual tab.
  void DrawTabbedWindow();
  // Draws the main menu.
  void DrawMainMenuBar();
  // Draws the toolbar
  void DrawToolbar();
  // Draws a circular cursor at ImGuiIO::MousePos.
  void DrawCursor();
  //
  void DrawAfterLayout();

  // Resisters or updates the sub-window for world layout.
  void UpdateSubWindowRegistration(absl::string_view label);

  // Wraps an Invocable in a CollapsingHeader.
  imp::Invocable<void()> BuildHeaderDrawFunction(
      absl::string_view header_label, imp::Invocable<void()> draw_function,
      imp::Invocable<void()> on_close_button_pressed,
      ImGuiTreeNodeFlags additional_flags = ImGuiTreeNodeFlags_None);

  // Returns the safe display area excluding the overscan area.
  ImVec2 GetSafeDisplaySize() const;
  ImVec4 GetSafeDisplayBounds() const;

  std::vector<imp::Invocable<void()>> details_draw_functions_;
  std::vector<imp::Invocable<void()>> scene_draw_functions_;
  std::vector<imp::Invocable<void()>> tab_item_draw_functions_;
  std::vector<imp::Invocable<void()>> menu_draw_functions_;
  std::vector<imp::Invocable<void()>> toolbar_draw_functions_;
  std::vector<imp::Invocable<void()>> draw_after_functions_;

  LayoutConfig layout_config_;

  // Layout state
  LayoutConfig::TabbedWindowState tabbed_window_state_;
  float window_y_offset_ = 0;
  ImGuiID selected_tab_id_ = 0;
  ImGuiTextFilter details_filter_;
  StringMap<SubWindowInfo> sub_window_info_map_;
  std::vector<SubWindowInfo> sub_window_info_;
  float window_x_offset_for_world_layout_ = 0;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_LAYOUT_COMPOSER_H_
