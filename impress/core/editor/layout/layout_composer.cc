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

#include "core/editor/layout/layout_composer.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "absl/strings/match.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "core/common/file_helpers.h"
#include "core/common/invocable.h"
#include "core/config.h"
#include "core/editor/editor_constants.h"
#include "core/editor/layout/docking_helper.h"
#include "core/editor/layout/editor_panel_ids.h"
#include "core/editor/layout/helpers.h"
#include "core/editor/layout/layout_config.proto.imp.h"
#include "core/editor/widget.h"
#include "core/editor/widget_layout_info.h"
#include "core/editor/widgets/window/window_configuration.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"

namespace imp::editor {
namespace {
constexpr float kLerpFactor = 0.5f;
constexpr float kWindowAlpha = 0.85f;
// Spacer between component filter and node info at top of NodeDetails panel
constexpr float kComponentLibrarySpacer = 4.0f;
// Spacer at between node info and the first header (NodeDetails)
constexpr float kComponentFilterSpacer = 8.0f;
// Spacer between Component Library title and normal widget UI (NodeDetails)
constexpr float kNodeInfoSpacer = 4.0f;
// Title of Component Library section at bottom of NodeDetails
constexpr std::string_view kComponentLibraryTitle = "Component Library";
constexpr int32_t kToolbarWidth = 200;
constexpr int32_t kSceneItemWidth = 200;
constexpr int32_t kFixedItemWidth = 200;
constexpr int32_t kFixedTabWidth = 500;
// White
constexpr ImColor kCursorDefaultColor = ImColor(1.0f, 1.0f, 1.0f, 1.0f);
// Grey
constexpr ImColor kCursorDownColor = ImColor(0.9f, 0.9f, 0.9f, 1.0f);
constexpr float kCursorDefaultRadius = 7;
constexpr float kCursorDownRadius = 6;
constexpr float kCursorThickness = 3;
// Dark Grey
constexpr ImVec4 kTabButtonDefaultColor = ImVec4(0.9f, 0.9f, 0.9f, 1);
// Grey
constexpr ImVec4 kTabButtonHoverColor = ImVec4(1.0f, 1.0f, 1.0f, 1);
// Black
constexpr ImVec4 kTabButtonTextColor = ImVec4(0.0f, 0.0f, 0.0f, 1);

constexpr absl::string_view kHideLabel = "Hide";
constexpr absl::string_view kShowLabel = "Show";
constexpr absl::string_view kPinToTopLabel = "Top";
constexpr absl::string_view kPinToBottomLabel = "Bottom";

// Padding around the viewport widget image.
constexpr ImVec2 kViewportPadding = ImVec2(2.0f, 2.0f);

bool IsLabelVisible(absl::string_view label) {
  return !absl::StartsWith(label, "##");
}

bool LatchFocusRequest(Widget* widget) {
  const bool focus_requested = widget->IsRequestingFocus();
  widget->SetIsRequestingFocus(false);
  return focus_requested;
}
}  // namespace

LayoutComposer::LayoutComposer(LayoutConfig layout_config)
    : layout_config_(layout_config),
      tabbed_window_state_(layout_config.initial_tabbed_window_state) {
  if (!layout_config_.lerp_factor.has_value() ||
      AlmostEqual(*layout_config_.lerp_factor, 0.0f)) {
    layout_config_.lerp_factor = kLerpFactor;
  }
  layout_config_.lerp_factor = 1;

#if IMP_RUNTIME(DEV)
  if (layout_config.layout_type ==
      LayoutConfig::LayoutType::MULTIPLE_WINDOWS_DEFAULT) {
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  } else {
    ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_DockingEnable;
  }

  ImGui::GetIO().IniFilename = NULL;
#endif
  saved_layout_filename_ =
      GetRepoDirectory() + std::string(kSavedLayoutIniFile);
}

std::optional<LayoutConfig::LayoutType> LayoutComposer::GetLayoutType() const {
  return layout_config_.layout_type;
}

void LayoutComposer::DrawWidget(const WidgetLayoutInfo& layout_info,
                                Widget* widget) {
  if (layout_config_.layout_type ==
      LayoutConfig::LayoutType::MULTIPLE_WINDOWS_DEFAULT) {
    DrawOnDockingLayout(layout_info, widget);
  } else {
    DrawOnFixedLayout(layout_info, widget);
  }
}

void LayoutComposer::DrawOnDockingLayout(const WidgetLayoutInfo& layout_info,
                                         Widget* widget) {
  switch (layout_info.panel_id) {
    case PanelId::kDetailsWindow:
      DrawInDetailsSectionAsHeader(widget);
      break;
    case PanelId::kSceneWindow:
      DrawInSceneSectionAsHeader(widget);
      break;
    case PanelId::kViewport:
      viewport_draw_functions_.push_back([widget]() { widget->DrawImGui(); });
      break;
    case PanelId::kLeftPanel:
      DrawInLeftDock(
          widget->GetName(), [widget]() { widget->DrawImGui(); },
          /*force_focus=*/LatchFocusRequest(widget));
      break;
    case PanelId::kTabBar:
      DrawAsDockableTab(
          widget->GetName(), [widget]() { widget->DrawImGui(); },
          /*draw_before_previous_tabs=*/false,
          /*force_focus=*/LatchFocusRequest(widget));
      break;
    case PanelId::kMenuBar:
      DrawAsMenuInMenuBar(widget->GetName(),
                          [widget]() { widget->DrawImGui(); });
      break;
    case PanelId::kToolBar:
      DrawInToolbar([widget]() { widget->DrawImGui(); });
      break;
    default:
      DrawAfterLayout(widget->GetName(), [widget]() { widget->DrawImGui(); });
  }
}

void LayoutComposer::DrawOnFixedLayout(const WidgetLayoutInfo& layout_info,
                                       Widget* widget) {
  switch (layout_info.panel_id) {
    case PanelId::kDetailsWindow:
      DrawInDetailsSectionAsHeader(widget);
      break;
    case PanelId::kSceneWindow:
    case PanelId::kViewport:
    case PanelId::kLeftPanel:
      DrawInSceneSectionAsHeader(widget);
      break;
    case PanelId::kTabBar:
      DrawAsStandaloneTab(
          widget->GetName(), [widget]() { widget->DrawImGui(); },
          ImGuiTabItemFlags_Leading, /*draw_before_previous_tabs=*/false,
          /*force_focus=*/LatchFocusRequest(widget));
      break;
    case PanelId::kMenuBar:
    case PanelId::kToolBar:
      break;
    default:
      // In fixed layout, all visible freeform widgets are drawn as standalone
      // tabs.
      if (IsLabelVisible(widget->GetName())) {
        DrawAsStandaloneTab(
            widget->GetName(), [widget]() { widget->DrawImGui(); },
            ImGuiTabItemFlags_Leading, /*draw_before_previous_tabs=*/false,
            /*force_focus=*/LatchFocusRequest(widget));
      } else {
        DrawAfterLayout(widget->GetName(), [widget]() { widget->DrawImGui(); });
      }
  }
}

void LayoutComposer::DrawInDetailsSectionAsHeader(Widget* widget) {
  absl::string_view header_label = widget->GetName();

  if (header_label == kNodeWidgetHeaderName) {
    // Pull out node info to draw before other components (no header)
    node_info_draw_function_ = [widget]() { widget->DrawImGui(); };
  } else if (header_label == kComponentLibraryWidgetHeaderName) {
    // Pull out component library to draw after other components (no header)
    component_library_draw_function_ = [widget]() { widget->DrawImGui(); };
  } else if (details_filter_.PassFilter(header_label.data())) {
    // For all other widgets, only draw if filter passes (with header)
    details_draw_functions_.push_back(BuildHeaderDrawFunction(
        header_label, [widget]() { widget->DrawImGui(); },
        widget->OnCloseButton(), widget->GetTreeNodeFlags()));
  }
}

void LayoutComposer::DrawInSceneSectionAsHeader(Widget* widget) {
  scene_draw_functions_.push_back(BuildHeaderDrawFunction(
      widget->GetName(), [widget]() { widget->DrawImGui(); },
      widget->OnCloseButton(), widget->GetTreeNodeFlags()));
}

void LayoutComposer::DrawAsStandaloneTab(absl::string_view tab_label,
                                         imp::Invocable<void()> draw_function,
                                         ImGuiTabItemFlags flags,
                                         bool draw_before_previous_tabs,
                                         bool force_focus) {
  Invocable<void()> draw_function_final = [this, flags,
                                           label = std::string(tab_label),
                                           draw_function =
                                               std::move(draw_function),
                                           force_focus]() {
    // The Dear ImGui TabItem tap-to-select logic depends on hovering
    // and is incompatible with touchscreens, so this code uses
    // ImGuiTabItemFlags_SetSelected to manually handle tab selection.
    // (See ImGuiTreeNodeFlags_AllowItemOverlap).
    ImGuiID tab_id = ImGui::GetCurrentWindow()->GetID(label.c_str());
    ImGuiTabItemFlags tab_flags = flags;
    if (force_focus) {
      selected_tab_id_ = tab_id;
    }
    // The first leading tab will be selected by default.
    if (selected_tab_id_ == 0 &&
        (flags & ImGuiTabItemFlags_Leading) == ImGuiTabItemFlags_Leading) {
      selected_tab_id_ = tab_id;
    }
    if (selected_tab_id_ == tab_id) {
      tab_flags |= ImGuiTabItemFlags_SetSelected;
    }
    ImGui::SetNextItemAllowOverlap();
    if (ImGui::BeginTabItem(label.data(), nullptr, tab_flags)) {
      if (tabbed_window_state_.expanded_state ==
          LayoutConfig::WindowExpandedState::EXPANDED) {
        ImVec2 safe_display_size = GetSafeDisplaySize();
        float max_window_height =
            safe_display_size.y *
            layout_config_.initial_tabbed_window_state
                .max_window_height_multiplier.value_or(1.0f);
        ImGuiStyle& style = ImGui::GetStyle();
        ImGui::SetNextWindowSizeConstraints(
            ImVec2(0, 0),
            ImVec2(safe_display_size.x - style.WindowPadding.x * 2,
                   max_window_height - ImGui::GetCursorPosY() -
                       style.WindowPadding.y * 2));

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        if (ImGui::BeginChild(
                GenerateUniqueImGuiLabel("##tab_content_child", this).c_str(),
                ImVec2(0, 0),
                ImGuiChildFlags_AutoResizeY |
                    ImGuiChildFlags_AlwaysUseWindowPadding)) {
          draw_function();
        }
        ImGui::EndChild();
        ImGui::PopStyleVar();
      }
      ImGui::EndTabItem();
    }
    // After the tab is rendered, check if it's been selected.
    if (ImGui::IsMouseClicked(0) &&
        ImGui::GetCurrentContext()->HoveredId == tab_id) {
      selected_tab_id_ = tab_id;
      // Expand the window if a tab is tapped.
      tabbed_window_state_.expanded_state =
          LayoutConfig::WindowExpandedState::EXPANDED;
    }
  };
  if (draw_before_previous_tabs) {
    tab_item_info_.insert(
        tab_item_info_.begin(),
        WidgetInfo(std::string(tab_label), std::move(draw_function_final)));
  } else {
    tab_item_info_.push_back(
        WidgetInfo{std::string(tab_label), std::move(draw_function_final)});
  }
}

void LayoutComposer::DrawInLeftDock(absl::string_view tab_label,
                                    imp::Invocable<void()> draw_function,
                                    bool force_focus) {
  left_dock_draw_functions_.push_back(
      {std::string(tab_label), std::move(draw_function), force_focus});
}

void LayoutComposer::DrawAsDockableTab(absl::string_view tab_label,
                                       imp::Invocable<void()> draw_function,
                                       bool draw_before_previous_tabs,
                                       bool force_focus) {
  WidgetInfo tab = {std::string(tab_label), std::move(draw_function),
                    force_focus};

  if (draw_before_previous_tabs) {
    tab_item_info_.insert(tab_item_info_.begin(), std::move(tab));
  } else {
    tab_item_info_.push_back(std::move(tab));
  }
}

void LayoutComposer::DrawAsMenuInMenuBar(absl::string_view menu_label,
                                         imp::Invocable<void()> draw_function) {
  menu_draw_functions_.push_back([label = std::string(menu_label),
                                  draw_function = std::move(draw_function)] {
    if (ImGui::BeginMenu(label.data())) {
      draw_function();
      ImGui::EndMenu();
    }
  });
}

void LayoutComposer::DrawInToolbar(imp::Invocable<void()> draw_function) {
  toolbar_draw_functions_.push_back(std::move(draw_function));
}

void LayoutComposer::DrawAfterLayout(absl::string_view label,
                                     imp::Invocable<void()> draw_function) {
  draw_after_functions_.push_back(
      WidgetInfo{std::string(label), std::move(draw_function)});
}

imp::Invocable<void()> LayoutComposer::BuildHeaderDrawFunction(
    absl::string_view header_label, imp::Invocable<void()> draw_function,
    imp::Invocable<void()> on_close_button_pressed,
    ImGuiTreeNodeFlags additional_flags) {
  return [label = std::string(header_label),
          draw_function = std::move(draw_function),
          on_close_button_pressed = std::move(on_close_button_pressed),
          additional_flags]() {
    // Making it so that the header can highlight when hovered or clicked.
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered,
                          ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered]);
    ImGui::PushStyleColor(ImGuiCol_HeaderActive,
                          ImGui::GetStyle().Colors[ImGuiCol_ButtonActive]);
    // A pointer of p_visible is passed into ImGui::CollapsingHeader and is set
    // to false if the close button is pressed.
    bool p_visible = true;
    ImGui::Separator();
    if (ImGui::CollapsingHeader(label.c_str(),
                                on_close_button_pressed ? &p_visible : nullptr,
                                additional_flags)) {
      ImGui::Indent();
      draw_function();
      ImGui::Unindent();
    }

    ImGui::PopStyleColor(2);

    // The close button has been pressed, so run the callback.
    if (!p_visible) {
      on_close_button_pressed();
    }
  };
}

void LayoutComposer::DrawDockableDetailsWindow() {
  ImGui::SetNextWindowBgAlpha(kWindowAlpha);
  // Without explicitly setting collapsed to false, the panel defaults to
  // collapsed on desktop.
  ImGui::SetNextWindowCollapsed(false, ImGuiCond_Appearing);

  ImGuiWindowFlags flags = ImGuiWindowFlags_NoFocusOnAppearing |
                           ImGuiWindowFlags_AlwaysVerticalScrollbar |
                           ImGuiWindowFlags_HorizontalScrollbar;

  if (window_configuration_ &&
      (*window_configuration_)->ShouldHideAllWindows()) {
    return;
  }

  if (ImGui::Begin(PanelIdToString(PanelId::kDetailsWindow).c_str(), nullptr,
                   flags)) {
    DrawDetailsSectionContents();
  }
  ImGui::End();
}

void LayoutComposer::DrawDetailsSectionContents() {
  if (!node_info_draw_function_) {
    ImGui::Text("Select a Node to see details.");
    return;
  }

  // Draw component filter
  details_filter_.Draw(GenerateUniqueImGuiLabel("filter", this).c_str());
  ImGui::Dummy(ImVec2(0.0f, kComponentFilterSpacer));

  // Draw node info
  if (node_info_draw_function_) {
    node_info_draw_function_();
    ImGui::Dummy(ImVec2(0.0f, kNodeInfoSpacer));
  }

  // Draw regular components
  for (auto& draw_function : details_draw_functions_) {
    draw_function();
  }

  // Draw add-component ui
  ImGui::Separator();
  if (component_library_draw_function_) {
    ImGui::Indent();
    ImGui::Text(kComponentLibraryTitle.data());
    ImGui::Dummy(ImVec2(0.0f, kComponentLibrarySpacer));
    component_library_draw_function_();
    ImGui::Unindent();
  }
}

void LayoutComposer::DrawWindowForWorldLayout(
    absl::string_view window_label, imp::Invocable<void()> draw_function) {
  ImVec2 safe_display_size = GetSafeDisplaySize();
  ImVec4 safe_display_bounds = GetSafeDisplayBounds();

  ImVec2 window_size = ImVec2(safe_display_size.x, safe_display_size.y);
  ImGui::SetNextWindowSize(window_size, ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowBgAlpha(kWindowAlpha);
  // Anchor the window next to last window on its left.
  ImGui::SetNextWindowPos(
      {safe_display_bounds.x + window_x_offset_for_world_layout_,
       window_y_offset_},
      ImGuiCond_Always);
  // Without explicitly setting collapsed to false, the panel defaults to
  // collapsed on XR.
  ImGui::SetNextWindowCollapsed(false, ImGuiCond_Appearing);

  if (ImGui::Begin(window_label.data(), nullptr,
                   ImGuiWindowFlags_AlwaysAutoResize |
                       ImGuiWindowFlags_HorizontalScrollbar |
                       ImGuiWindowFlags_NoFocusOnAppearing)) {
    ImGui::PushItemWidth(kFixedItemWidth);
    draw_function();
    window_x_offset_for_world_layout_ += ImGui::GetWindowWidth();
    UpdateSubWindowRegistration(window_label);
    ImGui::PopItemWidth();
  }
  ImGui::End();
}

void LayoutComposer::DrawDockableSceneWindow() {
  ImGui::SetNextWindowBgAlpha(kWindowAlpha);
  // Without explicitly setting collapsed to false, the panel defaults to
  // collapsed on desktop.
  ImGui::SetNextWindowCollapsed(false, ImGuiCond_Appearing);
  ImGuiWindowFlags flags = ImGuiWindowFlags_HorizontalScrollbar |
                           ImGuiWindowFlags_NoFocusOnAppearing;

  if (window_configuration_ &&
      (*window_configuration_)->ShouldHideAllWindows()) {
    return;
  }

  if (ImGui::Begin(PanelIdToString(PanelId::kSceneWindow).c_str(), nullptr,
                   flags)) {
    ImGui::PushItemWidth(kSceneItemWidth);
    DrawSceneSectionContents();
    ImGui::PopItemWidth();
  }
  ImGui::End();
}

void LayoutComposer::DrawSceneSectionContents() {
  for (auto& draw_function : scene_draw_functions_) {
    draw_function();
  }
}

void LayoutComposer::DrawDockableViewportWindow() {
  if (viewport_draw_functions_.empty()) return;

  ImGui::SetNextWindowBgAlpha(kWindowAlpha);
  ImGui::SetNextWindowCollapsed(false, ImGuiCond_Appearing);
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoFocusOnAppearing |
                           ImGuiWindowFlags_NoScrollbar |
                           ImGuiWindowFlags_NoScrollWithMouse;

  if (window_configuration_ &&
      (*window_configuration_)->ShouldHideAllWindows()) {
    return;
  }

  // Setting padding to 0 to use full window space for image.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, kViewportPadding);
  std::string window_label = PanelIdToString(PanelId::kViewport);
  if (ImGui::Begin(window_label.c_str(), nullptr, flags)) {
    for (auto& draw_function : viewport_draw_functions_) {
      draw_function();
    }
  }

  ImGui::End();
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding
}

void LayoutComposer::DrawDockableTabbedWindow() {
  ImGui::SetNextWindowBgAlpha(kWindowAlpha);
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoFocusOnAppearing;

  for (auto& info : tab_item_info_) {
    if (should_reset_docking_layout_) {
      ImGui::SetNextWindowDockID(
          docking_helper_->GetDockId(DockingHelper::DockingType::kBottom),
          ImGuiCond_Always);
    } else {
      ImGui::SetNextWindowDockID(
          docking_helper_->GetDockId(DockingHelper::DockingType::kBottom),
          ImGuiCond_FirstUseEver);
    }
    if (info.force_focus) {
      ImGui::SetNextWindowFocus();
    }
    if (ImGui::Begin(info.label.c_str(), nullptr, flags)) {
      info.draw_function();
    }
    ImGui::End();
  }

  for (auto& info : left_dock_draw_functions_) {
    ImGui::SetNextWindowDockID(
        docking_helper_->GetDockId(DockingHelper::DockingType::kLeft),
        ImGuiCond_Appearing);
    if (info.force_focus) {
      ImGui::SetNextWindowFocus();
    }
    if (ImGui::Begin(info.label.c_str(), nullptr, flags)) {
      info.draw_function();
    }
    ImGui::End();
  }
}

void LayoutComposer::DrawTabbedWindow() {
  ImVec2 safe_display_size = GetSafeDisplaySize();
  ImVec4 safe_display_bounds = GetSafeDisplayBounds();
  float window_width = safe_display_size.x;
  if (layout_config_.layout_type.value() ==
      LayoutConfig::LayoutType::MULTIPLE_WINDOWS_WORLD_LAYOUT) {
    // Put the window next to the last window on its left.
    safe_display_bounds.x += window_x_offset_for_world_layout_;
    // The tabbed window is fixed width for world layout.
    safe_display_bounds.z = safe_display_bounds.x + kFixedTabWidth;
    window_width = kFixedTabWidth;
  }

  ImGui::SetNextWindowBgAlpha(kWindowAlpha);

  // Initialize the position of the tabbed window.
  switch (tabbed_window_state_.pin_state.value()) {
    case LayoutConfig::WindowPinState::PINNED_TO_TOP: {
      ImGui::SetNextWindowPos({0, 0}, ImGuiCond_Appearing);
      break;
    }
    default:
    case LayoutConfig::WindowPinState::PINNED_TO_BOTTOM_DEFAULT: {
      ImGui::SetNextWindowPos({0, safe_display_bounds.w}, ImGuiCond_Appearing);
      break;
    }
  }

  float max_window_height =
      safe_display_size.y * layout_config_.initial_tabbed_window_state
                                .max_window_height_multiplier.value_or(1.0f);

  ImGui::SetNextWindowSizeConstraints(ImVec2(window_width, 0),
                                      ImVec2(window_width, max_window_height));
  if (ImGui::Begin(
          PanelIdToString(PanelId::kTabBar).c_str(), nullptr,
          ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoFocusOnAppearing |
              ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar |
              ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground |
              ImGuiWindowFlags_NoScrollbar |
              ImGuiWindowFlags_NoScrollWithMouse)) {
    // Draw tabs.
    ImGui::BeginGroup();
    ImGui::BeginTabBar(PanelIdToString(PanelId::kTabBar).c_str(),
                       ImGuiTabBarFlags_FittingPolicyResizeDown);
    for (auto& info : tab_item_info_) {
      info.draw_function();
    }
    UpdateSubWindowRegistration(PanelIdToString(PanelId::kTabBar).c_str());

    // The tab buttons are colored differently to indicate they are not tabs.
    ImGui::PushStyleColor(ImGuiCol_Tab, kTabButtonDefaultColor);
    ImGui::PushStyleColor(ImGuiCol_TabHovered, kTabButtonHoverColor);
    ImGui::PushStyleColor(ImGuiCol_Text, kTabButtonTextColor);

    // In LayoutType::kSingleTabbedWindow, draw a button that allows rotating
    // between LayoutConfig::WindowPinState values.
    if (layout_config_.layout_type.value() ==
        LayoutConfig::LayoutType::SINGLE_TABBED_WINDOW) {
      switch (tabbed_window_state_.pin_state.value()) {
        case LayoutConfig::WindowPinState::PINNED_TO_TOP: {
          if (ImGui::TabItemButton(kPinToBottomLabel.data(),
                                   ImGuiTabItemFlags_Trailing)) {
            tabbed_window_state_.pin_state =
                LayoutConfig::WindowPinState::PINNED_TO_BOTTOM_DEFAULT;
          }
          break;
        }
        default:
        case LayoutConfig::WindowPinState::PINNED_TO_BOTTOM_DEFAULT: {
          if (ImGui::TabItemButton(kPinToTopLabel.data(),
                                   ImGuiTabItemFlags_Trailing)) {
            tabbed_window_state_.pin_state =
                LayoutConfig::WindowPinState::PINNED_TO_TOP;
          }
          break;
        }
      }
    }

    // Draw Hide/Show button.
    switch (tabbed_window_state_.expanded_state.value()) {
      case LayoutConfig::WindowExpandedState::EXPANDED: {
        if (ImGui::TabItemButton(kHideLabel.data(),
                                 ImGuiTabItemFlags_Trailing)) {
          tabbed_window_state_.expanded_state =
              LayoutConfig::WindowExpandedState::COLLAPSED_DEFAULT;
        }
        break;
      }
      default:
      case LayoutConfig::WindowExpandedState::COLLAPSED_DEFAULT: {
        if (ImGui::TabItemButton(kShowLabel.data(),
                                 ImGuiTabItemFlags_Trailing)) {
          tabbed_window_state_.expanded_state =
              LayoutConfig::WindowExpandedState::EXPANDED;
        }
        break;
      }
    }
    ImGui::PopStyleColor();
    ImGui::PopStyleColor();
    ImGui::PopStyleColor();

    ImGui::EndTabBar();
    ImGui::EndGroup();

    ImGui::SetNextWindowSizeConstraints(
        ImVec2(window_width, 0), ImVec2(window_width, max_window_height));
    ImVec2 auto_fit_size =
        ImGui::CalcWindowNextAutoFitSize(ImGui::GetCurrentWindow());

    // Position the tabbed window according to the LayoutConfig::WindowPinState.
    switch (tabbed_window_state_.pin_state.value()) {
      case LayoutConfig::WindowPinState::PINNED_TO_TOP: {
        ImGui::SetWindowPos(
            LerpImVec2(ImGui::GetWindowPos(),
                       ImVec2(safe_display_bounds.x, window_y_offset_),
                       *layout_config_.lerp_factor),
            ImGuiCond_Always);
        break;
      }
      default:
      case LayoutConfig::WindowPinState::PINNED_TO_BOTTOM_DEFAULT: {
        ImGui::SetWindowPos(
            LerpImVec2(ImGui::GetWindowPos(),
                       ImVec2(safe_display_bounds.x,
                              safe_display_bounds.w - auto_fit_size.y),
                       *layout_config_.lerp_factor),
            ImGuiCond_Always);
        break;
      }
    }

    ImVec2 window_position = ImGui::GetWindowPos();
    ImGuiStyle style = ImGui::GetStyle();
    ImGui::GetBackgroundDrawList()->AddRectFilled(
        {window_position.x, window_position.y + style.WindowPadding.y +
                                style.FramePadding.y * 2.0f +
                                ImGui::GetTextLineHeight()},
        {window_position.x + ImGui::GetWindowWidth(),
         window_position.y + auto_fit_size.y},
        ImGui::GetColorU32(ImGuiCol_WindowBg));
  }
  ImGui::End();
}

void LayoutComposer::DrawMainMenuBar() {
  ImGui::BeginMainMenuBar();
  for (auto& draw_function : menu_draw_functions_) {
    draw_function();
  }
  window_y_offset_ = ImGui::GetWindowHeight();
  ImGui::EndMainMenuBar();
}

void LayoutComposer::DrawToolbar() {
  if (toolbar_draw_functions_.empty()) {
    return;
  }

  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowBgAlpha(kWindowAlpha);

  if (ImGui::Begin("Toolbar", nullptr,
                   ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove |
                       ImGuiWindowFlags_NoFocusOnAppearing |
                       ImGuiWindowFlags_NoTitleBar |
                       ImGuiWindowFlags_NoCollapse)) {
    ImGui::PushItemWidth(kToolbarWidth);

    for (auto& draw_function : toolbar_draw_functions_) {
      draw_function();
    }

    ImGui::PopItemWidth();
  }
  ImVec2 auto_fit_size =
      ImGui::CalcWindowNextAutoFitSize(ImGui::GetCurrentWindow());
  // Adjust the position of the Toolbar window based on the rendered size to be
  // the top middle of the screen.
  ImGui::SetWindowPos("Toolbar",
                      ImVec2((io.DisplaySize.x / 2.0) - (auto_fit_size.x / 2.0),
                             window_y_offset_),
                      ImGuiCond_Always);
  ImGui::End();
}

void LayoutComposer::DrawAfterLayout() {
  for (auto& [label, draw_function, _] : draw_after_functions_) {
    // Freeform windows with invisible labels draw invisible widgets, so we
    // don't need to draw them in a freeform window.
    if (label.empty() || !IsLabelVisible(label)) {
      draw_function();
      continue;
    }

    ImVec2 window_size = GetSafeDisplaySize();
    // Set the freeform window in the middle of the screen, with a size of half
    // of the screen size in both dimensions.
    ImGui::SetNextWindowSize(ImVec2(window_size.x * 0.5f, window_size.y * 0.5f),
                             ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(
        ImVec2(window_size.x * 0.25f, window_size.y * 0.25f),
        ImGuiCond_FirstUseEver);
    if (window_configuration_ &&
        layout_config_.layout_type ==
            LayoutConfig::LayoutType::MULTIPLE_WINDOWS_DEFAULT) {
      // If the window configuration is available, show a close button for the
      // window.
      bool show_draw = (*window_configuration_)->IsWindowVisible(label);
      ImGui::Begin(label.c_str(), &show_draw);
      draw_function();
      ImGui::End();
      // Allow other mechanisms to hide the window while the window is shown and
      // the close button is not clicked.
      show_draw &= (*window_configuration_)->IsWindowVisible(label);
      (*window_configuration_)
          ->SetWindowVisibility(
              label, show_draw
                         ? WindowConfiguration::WindowVisibility::kVisible
                         : WindowConfiguration::WindowVisibility::kHidden);
    } else {
      ImGui::Begin(label.c_str());
      draw_function();
      ImGui::End();
    }
  }
}

void LayoutComposer::DrawCursor() {
  ImGuiIO* io = &ImGui::GetIO();
  ImDrawList* draw_list = ImGui::GetForegroundDrawList();
  // A decagon to approximate a bordered circle.
  ImColor cursor_color =
      io->MouseDown[0] ? kCursorDownColor : kCursorDefaultColor;
  float cursor_radius =
      io->MouseDown[0] ? kCursorDownRadius : kCursorDefaultRadius;
  draw_list->AddCircle(io->MousePos, cursor_radius, cursor_color, 10,
                       kCursorThickness);
}

void LayoutComposer::DrawLayout() {
  ImGuiStyle& style = ImGui::GetStyle();
  style.DisplaySafeAreaPadding =
      ImVec2(layout_config_.overscan.x, layout_config_.overscan.y);

  if (layout_config_.layout_type ==
      LayoutConfig::LayoutType::MULTIPLE_WINDOWS_DEFAULT) {
    if (!docking_helper_) {
      docking_helper_ = std::make_unique<DockingHelper>();
      if (!docking_helper_->IsInitializedWithSavedLayout()) {
        ResetDockingLayout();
      } else {
        for (const auto& label :
             docking_helper_->GetInitialVisibleWindowLabels()) {
          (*window_configuration_)
              ->SetWindowVisibility(
                  label, WindowConfiguration::WindowVisibility::kVisible);
        }
      }
    }
    ImGui::DockSpaceOverViewport(
        docking_helper_->GetDockableSpaceId(), nullptr,
        ImGuiDockNodeFlags_PassthruCentralNode  // PassthruCentralNode makes the
                                                // central node transparent
    );
  }

  // Draw the menu bar first to know if we should draw all other windows
  // below the menu bar.
  if (!menu_draw_functions_.empty()) {
    DrawMainMenuBar();
  } else {
    window_y_offset_ = layout_config_.overscan.y;
  }

  // Pick where to render the Details and Scene panels.
  switch (layout_config_.layout_type.value()) {
    case LayoutConfig::LayoutType::SINGLE_TABBED_WINDOW: {
      DrawAsStandaloneTab(
          PanelIdToString(PanelId::kDetailsWindow),
          [this]() { DrawDetailsSectionContents(); }, ImGuiTabItemFlags_Leading,
          /*draw_before_previous_tabs=*/true);
      DrawAsStandaloneTab(
          PanelIdToString(PanelId::kSceneWindow),
          [this]() { DrawSceneSectionContents(); }, ImGuiTabItemFlags_Leading,
          /*draw_before_previous_tabs=*/true);
      DrawTabbedWindow();
      break;
    }
    case LayoutConfig::LayoutType::MULTIPLE_WINDOWS_WORLD_LAYOUT: {
      window_x_offset_for_world_layout_ = 0;
      DrawWindowForWorldLayout(PanelIdToString(PanelId::kSceneWindow),
                               [this]() { DrawSceneSectionContents(); });
      DrawWindowForWorldLayout(PanelIdToString(PanelId::kDetailsWindow),
                               [this]() { DrawDetailsSectionContents(); });
      DrawTabbedWindow();
      break;
    }
    default:
    case LayoutConfig::LayoutType::MULTIPLE_WINDOWS_DEFAULT: {
      DrawDockableDetailsWindow();
      DrawDockableSceneWindow();
      DrawToolbar();
      DrawDockableTabbedWindow();
      break;
    }
  }

  if (layout_config_.show_cursor) {
    DrawCursor();
  }

  DrawAfterLayout();

  // Save on clicking on "Save Layout" button.
  if (window_configuration_ &&
      (*window_configuration_)->ShouldSaveLayoutToIniFile()) {
    SaveLayoutToIniFile();
    (*window_configuration_)->NotifyLayoutSaved();
  }

  // Flush the queued draw functions immediately after drawing.
  details_draw_functions_.clear();
  scene_draw_functions_.clear();
  viewport_draw_functions_.clear();
  left_dock_draw_functions_.clear();
  tab_item_info_.clear();
  menu_draw_functions_.clear();
  toolbar_draw_functions_.clear();
  draw_after_functions_.clear();
  node_info_draw_function_ = imp::Invocable<void()>();
  component_library_draw_function_ = imp::Invocable<void()>();

  should_reset_docking_layout_ = false;
}

void LayoutComposer::ResetDockingLayout() {
  docking_helper_->Initialize();
  should_reset_docking_layout_ = true;
}

void LayoutComposer::SaveLayoutToIniFile() {
  ImGui::GetIO().WantSaveIniSettings = true;
  ImGui::SaveIniSettingsToDisk(saved_layout_filename_.c_str());
  IMP_LOG(imp::INFO) << "Saved editor layout to " << saved_layout_filename_;
  ImGui::GetIO().WantSaveIniSettings = false;
}

absl::Span<const LayoutComposer::SubWindowInfo>
LayoutComposer::GetSubWindowInfo() {
  sub_window_info_.clear();

  if (sub_window_info_map_.empty()) {
    // If there are no windows, add a window that covers the entire screen.
    sub_window_info_.push_back({.label = std::string(kEntireTexture),
                                .window_size = ImGui::GetIO().DisplaySize,
                                .window_position = ImVec2(0, 0)});
  } else {
    for (const auto& [label, info] : sub_window_info_map_) {
      sub_window_info_.push_back(info);
    }
  }
  return sub_window_info_;
}

void LayoutComposer::UpdateSubWindowRegistration(absl::string_view label) {
  if (layout_config_.layout_type.value() !=
      LayoutConfig::LayoutType::MULTIPLE_WINDOWS_WORLD_LAYOUT) {
    return;
  }

  if (sub_window_info_map_.contains(label)) {
    sub_window_info_map_[label.data()].window_size = ImGui::GetWindowSize();
    sub_window_info_map_[label.data()].window_position = ImGui::GetWindowPos();
  } else {
    sub_window_info_map_[label.data()] = {
        .label = label.data(),
        .window_size = ImGui::GetWindowSize(),
        .window_position = ImGui::GetWindowPos()};
  }
}

ImVec2 LayoutComposer::GetSafeDisplaySize() const {
  const ImGuiIO& io = ImGui::GetIO();
  return ImVec2(io.DisplaySize.x - layout_config_.overscan.x * 2,
                io.DisplaySize.y - layout_config_.overscan.y * 2);
}

ImVec4 LayoutComposer::GetSafeDisplayBounds() const {
  const ImGuiIO& io = ImGui::GetIO();
  return ImVec4(layout_config_.overscan.x, layout_config_.overscan.y,
                io.DisplaySize.x - layout_config_.overscan.x,
                io.DisplaySize.y - layout_config_.overscan.y);
}

}  // namespace imp::editor
