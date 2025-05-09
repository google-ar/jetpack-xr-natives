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

#include "core/editor/ui/filterable_combo.h"

#include <algorithm>
#include <string>
#include <utility>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/editor/layout/helpers.h"

namespace imp::editor {

FilterableCombo::FilterableCombo(absl::string_view label,
                                 SortStrategy sort_strategy)
    : label_(label), sort_strategy_(sort_strategy), selected_(nullptr) {}

void FilterableCombo::DrawImGui() {
  ImGui::Text("Filter:");
  ImGui::SameLine();
  filter_.Draw(
      GenerateUniqueImGuiLabel("filter", this, EditorControlFlags::kNone)
          .c_str());
  std::string combo_text = selected_ ? selected_->first : std::string(label_);
  if (ImGui::BeginCombo(
          GenerateUniqueImGuiLabel("combo", this, EditorControlFlags::kNone)
              .c_str(),
          combo_text.c_str(), ImGuiComboFlags_None)) {
    selected_ = nullptr;
    for (const std::pair<std::string, RenderSelectableFn>& entry :
         render_selectable_fns_) {
      if (filter_.PassFilter(entry.first.c_str())) {
        if (entry.second(SelectionStatus::kNotSelected) ==
            SelectionStatus::kSelected) {
          selected_ = &entry;
        }
      }
    }
    ImGui::EndCombo();
  } else if (selected_) {
    if (selected_->second(SelectionStatus::kSelected) ==
        SelectionStatus::kNotSelected) {
      selected_ = nullptr;
    }
  }
}

void FilterableCombo::Add(absl::string_view label, RenderFn render_fn) {
  std::string label_str = std::string(label);
  render_selectable_fns_.emplace_back(
      label_str,
      [this, label_str, fn = std::move(render_fn)](SelectionStatus status) {
        if (status == SelectionStatus::kNotSelected) {
          std::string l = GenerateUniqueImGuiLabel(label_str, this);
          bool selected = false;
          ImGui::Selectable(GenerateUniqueImGuiLabel(label_str, this).c_str(),
                            &selected);
          if (selected) {
            status = SelectionStatus::kSelected;
          }
        } else {
          status = fn();
        }
        return status;
      });
  if (sort_strategy_ == SortStrategy::kAlphabetical) {
    std::sort(render_selectable_fns_.begin(), render_selectable_fns_.end(),
              [](const ComboEntry& a, const ComboEntry& b) {
                return a.first < b.first;
              });
  }
}

}  // namespace imp::editor
