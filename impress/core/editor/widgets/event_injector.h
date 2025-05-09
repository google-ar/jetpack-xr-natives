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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_EVENT_INJECTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_EVENT_INJECTOR_H_

#include <functional>
#include <optional>
#include <string>
#include <type_traits>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/editor/editor_proto_visitor.h"
#include "core/editor/ui/filterable_combo.h"
#include "core/editor/widget.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

namespace imp::editor {

// Allows injection of events.
class EventInjector : public editor::Widget {
 public:
  explicit EventInjector(BaseView& view);

  absl::string_view GetName() const override { return "Events"; }
  void DrawImGui() override;

  // Adds the given event type to the event injection panel.
  // Optional default instance provides default values for the event UI.
  template <typename EventT>
  void RegisterEvent(std::optional<EventT> default_instance = {});

 private:
  BaseView& view_;
  FilterableCombo event_ui_;
};

template <typename EventT>
void EventInjector::RegisterEvent(std::optional<EventT> default_instance) {
  static_assert(std::is_base_of<Event, EventT>::value,
                "EventT must derive from imp::Event");

  EventT event;
  if (default_instance) {
    event = *default_instance;
  }
  event_ui_.Add(type_traits::kTypeName<EventT>, [this, event]() mutable {
    imp::editor::EditorProtoVisitor<EventT> visitor(event);
    event.Visit(visitor, 0, nullptr);
    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(30, 90, 30, 255));
    if (ImGui::Button("Inject")) {
      view_.GetDispatcher().Send(event);
    }
    ImGui::PopStyleColor();
    return FilterableCombo::SelectionStatus::kSelected;
  });
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_EVENT_INJECTOR_H_
