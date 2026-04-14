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

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "dear_imgui/imgui.h"
#include "core/common/invocable.h"
#include "core/editor/layout/docking_helper.h"
#include "core/editor/layout/layout_config.proto.imp.h"
#include "core/editor/widget.h"
#include "core/editor/widget_layout_info.h"
#include "core/editor/widgets/window/window_configuration.h"
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

  std::optional<LayoutConfig::LayoutType> GetLayoutType() const;

  virtual void DrawWidget(const WidgetLayoutInfo& layout_info, Widget* widget);

  // Draws the platform-dependent Editor UI using all queued Invocables, which
  // are afterwards cleared from memory.
  // This function should be called once per frame to draw the Editor UI.
  virtual void DrawLayout();

  // Returns the information about the sub-windows that are registered for world
  // layout.
  absl::Span<const SubWindowInfo> GetSubWindowInfo();

  void SetWindowConfiguration(
      std::optional<WindowConfiguration*> window_configuration) {
    window_configuration_ = window_configuration;
  }

  void ResetDockingLayout();

  void SaveLayoutToIniFile();

 private:
  struct WidgetInfo {
    WidgetInfo(absl::string_view label, imp::Invocable<void()> draw_function,
               bool force_focus = false)
        : label(label),
          draw_function(std::move(draw_function)),
          force_focus(force_focus) {}

    // The label of the tab. This is used to name the dockable window of the
    // tab.
    std::string label;
    imp::Invocable<void()> draw_function;
    bool force_focus;
  };

  // Helper functions for queuing widgets for docking layout, which is used for
  // desktop and WASM.
  void DrawOnDockingLayout(const WidgetLayoutInfo& layout_info, Widget* widget);
  // Helper functions for queuing widgets for fixed layouts, which are used for
  // mobile and xr.
  void DrawOnFixedLayout(const WidgetLayoutInfo& layout_info, Widget* widget);

  // Queues the given Invocable to be drawn in the Details section, under a
  // ImGui::CollapsingHeader.
  void DrawInDetailsSectionAsHeader(Widget* widget);

  // Queues the given Invocable to be drawn in the Scene section, under a
  // ImGui::CollapsingHeader.
  void DrawInSceneSectionAsHeader(Widget* widget);

  // Queues the given Invocable to be drawn as its own tab in a central
  // tabbed window.
  void DrawAsStandaloneTab(
      absl::string_view tab_label, imp::Invocable<void()> draw_function,
      ImGuiTabItemFlags flags = ImGuiTabBarFlags_FittingPolicyScroll,
      bool draw_before_previous_tabs = false, bool force_focus = false);

  void DrawInLeftDock(absl::string_view tab_label,
                      imp::Invocable<void()> draw_function,
                      bool force_focus = false);

  // Queues the given Invocable to be drawn as its own tab in a dockable tabbed
  // window.
  // If `draw_before_previous_tabs` is true, the tab will be drawn as the
  // first tab of all existing tabs. Otherwise, the tab will be drawn as the
  // last tab of all existing tabs.
  void DrawAsDockableTab(absl::string_view tab_label,
                         imp::Invocable<void()> draw_function,
                         bool draw_before_previous_tabs = false,
                         bool force_focus = false);

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
  void DrawAfterLayout(absl::string_view label,
                       imp::Invocable<void()> draw_function);

  // Draws a window for world layout. The window will be drawn from left to
  // right without overlapping.
  void DrawWindowForWorldLayout(absl::string_view window_label,
                                imp::Invocable<void()> draw_function);

  // Draws a details window that can be docked to the right of the screen
  // initially and can be moved around.
  void DrawDockableDetailsWindow();
  // Draws the details draw functions and a placeholder if none exist.
  void DrawDetailsSectionContents();

  // Draws a scene window that can be docked to the left of the screen.
  void DrawDockableSceneWindow();
  // Draws the scene draw functions.
  void DrawSceneSectionContents();

  // Draws an individual tab.
  void DrawTabbedWindow();
  // Draws a tabbed window that can be docked to the bottom of the screen.
  void DrawDockableTabbedWindow();

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
  std::vector<WidgetInfo> left_dock_draw_functions_;
  std::vector<WidgetInfo> tab_item_info_;
  std::vector<imp::Invocable<void()>> menu_draw_functions_;
  std::vector<imp::Invocable<void()>> toolbar_draw_functions_;
  std::vector<WidgetInfo> draw_after_functions_;
  // Separate out Node Info to draw before all components
  imp::Invocable<void()> node_info_draw_function_;
  // Separate out Component Library to draw after all components
  imp::Invocable<void()> component_library_draw_function_;

  LayoutConfig layout_config_;
  std::unique_ptr<DockingHelper> docking_helper_;
  std::optional<WindowConfiguration*> window_configuration_;
  std::string saved_layout_filename_;
  bool should_reset_docking_layout_ = false;

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
