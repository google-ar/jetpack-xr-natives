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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_FILTERABLE_COMBO_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_FILTERABLE_COMBO_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/invocable.h"
#include "core/editor/widget.h"

namespace imp::editor {

// A filterable combo box UI for arbitrary UI content.
class FilterableCombo : public Widget {
 public:
  enum class SortStrategy {
    kAlphabetical,
    kFIFO,
  };

  // Creates a filterable combo box that shows the label if nothing is selected.
  explicit FilterableCombo(
      absl::string_view label,
      SortStrategy sort_strategy = SortStrategy::kAlphabetical);

  // The return type of a combo item render function (see RenderFn for details).
  enum class SelectionStatus {
    kNotSelected,
    kSelected,
  };

  // Function to render the custom UI. This is called when the item is selected.
  // The fn should return kNotSelected if it no longer wants to be selected.
  using RenderFn = Invocable<SelectionStatus()>;
  // Adds an entry with the given label and function to render ui when selected.
  void Add(absl::string_view label, RenderFn render_fn);

  void DrawImGui() override;
  absl::string_view GetName() const override { return label_; }

 private:
  std::string label_;
  SortStrategy sort_strategy_;
  // Function that renders a Selectable and returns whether it was selected.
  using RenderSelectableFn = Invocable<SelectionStatus(SelectionStatus)>;
  using ComboEntry = std::pair<std::string, RenderSelectableFn>;
  std::vector<ComboEntry> render_selectable_fns_;
  ImGuiTextFilter filter_;
  const ComboEntry* selected_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_FILTERABLE_COMBO_H_
