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

#include "core/editor/widget_ui_system.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "dear_imgui/imgui.h"
#include "core/common/registry.h"
#include "core/common/type_traits.h"
#include "core/editor/components/spatial_ui_canvas.h"
#include "core/editor/components/world_space_editor_ui.h"
#include "core/editor/layout/editor_panel_ids.h"
#include "core/editor/layout/layout_composer.h"
#include "core/editor/layout/layout_config.proto.imp.h"
#include "core/editor/widget.h"
#include "core/editor/widget_layout_info.h"
#include "core/editor/widgets/window/window_configuration.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/path_manager.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"
#include "core/view/view_events.h"

namespace imp::editor {

WidgetUiSystem::WidgetUiSystem(BaseView* view, bool enabled,
                               std::unique_ptr<LayoutComposer> layout_composer)
    : System(view),
      layout_composer_(std::move(layout_composer)),
      enabled_(enabled) {
  if (layout_composer_ && layout_composer_->GetLayoutType().has_value() &&
      layout_composer_->GetLayoutType() ==
          LayoutConfig::LayoutType::MULTIPLE_WINDOWS_DEFAULT) {
    window_configuration_ =
        &view->GetRegistry().GetOrCreate<WindowConfiguration>();
    layout_composer_->SetWindowConfiguration(*window_configuration_);
  }

  view->GetDispatcher().Connect(
      [this](const ImGuiPreRenderEvent& event) mutable {
        if (!enabled_) {
          return;
        }
        for (auto& [widget, layout_info] : widgets_) {
          if (pending_removes_.contains(widget.get()) ||
              !widget->HasContent()) {
            continue;
          }

          if (layout_composer_) {
            if (layout_composer_->GetLayoutType() ==
                    LayoutConfig::LayoutType::MULTIPLE_WINDOWS_DEFAULT &&
                window_configuration_ &&
                !(window_configuration_->IsWindowVisible(widget->GetName()))) {
              continue;
            }
            // Widgets can queue draw functions here using LayoutComposer.
            layout_composer_->DrawWidget(layout_info, widget.get());
          } else {
            widget->DrawImGui();
          }
        }
        if (layout_composer_) {
          // Widgets are drawn and draw functions are flushed.
          layout_composer_->DrawLayout();

          NodeHandle world_ui = GetView().GetPathManager().Find(
              absl::StrCat("//", WorldSpaceEditorUi::kWorldUiName));
          if (world_ui && world_ui->GetComponent<WorldSpaceEditorUi>()) {
            absl::Span<const LayoutComposer::SubWindowInfo> window_info =
                layout_composer_->GetSubWindowInfo();
            uint8_t window_count = 0;
            for (const auto& window_info : window_info) {
              SpatialUiCanvas::SpatialUiCanvasSettings settings = {
                  .name = window_info.label,
                  .content_position = {window_info.window_position.x,
                                       window_info.window_position.y},
                  .content_size = {window_info.window_size.x,
                                   window_info.window_size.y}};
              window_count++;
              world_ui->GetComponent<WorldSpaceEditorUi>()
                  ->UpdateSpatialUiCanvas(settings)
                  .KeptBy(world_ui);
            }
          }
        }

        ProcessPendingRemoves();
      },
      this);
}

void WidgetUiSystem::SetLayoutComposer(
    std::unique_ptr<LayoutComposer> layout_composer) {
  if (layout_composer && !layout_composer->GetLayoutType().has_value()) {
    LayoutConfig::LayoutType new_layout_type =
        layout_composer->GetLayoutType().value();
    if (new_layout_type != LayoutConfig::LayoutType::MULTIPLE_WINDOWS_DEFAULT) {
      // Switch to a non-multiple windows layout, so remove the window
      // configuration.
      window_configuration_ = nullptr;
    } else {
      if (!window_configuration_ && !widgets_.empty()) {
        // Switch to a multiple windows layout, but widgets are already added
        // without window configuration.
        IMP_LOG(imp::WARNING) << "Set layout composer after widgets are added. This may "
                        "mess up the UI.";
      }
      window_configuration_ =
          &GetView().GetRegistry().GetOrCreate<WindowConfiguration>();
    }
  }

  layout_composer_ = std::move(layout_composer);
}

void WidgetUiSystem::AddWidget(const WidgetLayoutInfo& layout_info,
                               std::unique_ptr<Widget> widget) {
  AddWindowConfiguration(widget->GetName(), layout_info);
  widgets_.push_back({std::move(widget), layout_info});
  type_hashes_.push_back(type_traits::kTypeHash<Widget>);
}

void WidgetUiSystem::RemoveWidget(Widget* widget) {
  pending_removes_.insert(widget);
}

void WidgetUiSystem::ProcessPendingRemoves() {
  int index = 0;
  for (auto it = widgets_.begin(); it != widgets_.end();) {
    auto pending_remove_itr = pending_removes_.find(it->first.get());
    if (pending_remove_itr != pending_removes_.end()) {
      pending_removes_.erase(pending_remove_itr);
      if (window_configuration_) {
        window_configuration_->RemoveWindow(it->first->GetName());
      }
      widgets_.erase(it);
      type_hashes_.erase(type_hashes_.begin() + index);
      continue;
    }
    it++;
    index++;
  }
}

void WidgetUiSystem::AddWindowConfiguration(
    absl::string_view name, const WidgetLayoutInfo& layout_info) {
  // Windows in other locations are not managed by the WindowConfiguration.
  if (window_configuration_ && !name.empty() && !absl::StartsWith(name, "##") &&
      (layout_info.panel_id == PanelId::kTabBar ||
       layout_info.panel_id == PanelId::kLeftPanel ||
       layout_info.panel_id == PanelId::kRightPanel ||
       layout_info.panel_id == PanelId::kFreeform)) {
    window_configuration_->AddWindow(
        name, layout_info.show_by_default
                  ? WindowConfiguration::WindowVisibility::kVisible
                  : WindowConfiguration::WindowVisibility::kHidden);
  }
}

void WidgetUiSystem::SetEnabled(bool enabled) { enabled_ = enabled; }

}  // namespace imp::editor
