// Copyright 2026 Google LLC
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

#include "core/editor/widgets/debug_draw_widget.h"

#include <string>

#include "absl/algorithm/container.h"
#include "absl/types/span.h"
#include "dear_imgui/imgui.h"
#include "core/common/debug_draw.h"

namespace imp::editor {
namespace {

constexpr ImVec4 kEnabledColor(0.263f, 0.627f, 0.278f, 1.0f);
constexpr ImVec4 kDisabledColor(0.898f, 0.224f, 0.208f, 1.0f);

constexpr char kHelpText[] =
    "This widget controls *tagged* debug draws\n"
    "(e.g. imp::DebugDrawLocal, see core/common/debug_draw.h)\n"
    "Direct calls to debug_draw functions are not tracked here.";

}  // namespace

void DebugDrawWidget::DrawImGui() {
  absl::Span<const std::string> tags = debug_draw::GetAllRegisteredTags();

  if (tags.empty()) {
    ImGui::Text("No debug draw tags have been used.\n\n");
    ImGui::Text(kHelpText);
    return;
  }

  filter_.Draw("Filter", /*width=*/200.0f);

  ImGui::SameLine();
  ImGui::TextDisabled("(?)");
  if (ImGui::BeginItemTooltip()) {
    ImGui::Text(kHelpText);
    ImGui::EndTooltip();
  }

  if (!ImGui::BeginTable(
          "DebugDrawTable", /*columns=*/2,
          ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
              ImGuiTableFlags_Sortable | ImGuiTableFlags_SizingFixedFit |
              ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuter)) {
    return;
  }

  bool all_enabled = true;
  for (const std::string& tag : tags) {
    if (filter_.PassFilter(tag.c_str()) && !debug_draw::IsTagEnabled(tag)) {
      all_enabled = false;
      break;
    }
  }

  ImGui::TableSetupColumn("Enabled", ImGuiTableColumnFlags_NoSort);
  ImGui::TableSetupColumn("Tag", ImGuiTableColumnFlags_DefaultSort);

  ImGui::TableSetupScrollFreeze(/*cols=*/0, /*rows=*/1);

  int id = 0;

  // Manually setup header rows so we can put a checkbox in the first column to
  // control all tags matching the filter.
  ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
  ImGui::PushID(id++);
  ImGui::TableSetColumnIndex(0);
  if (ImGui::Checkbox("##Enable All", &all_enabled)) {
    for (const std::string& tag : tags) {
      if (filter_.PassFilter(tag.c_str())) {
        debug_draw::SetTagEnabled(tag, all_enabled);
      }
    }
  }
  ImGui::SameLine();
  ImGui::TableHeader("##Enabled");
  ImGui::PopID();

  ImGui::PushID(id++);
  ImGui::TableSetColumnIndex(1);
  ImGui::TableHeader("Tag");
  ImGui::PopID();

  ImGuiTableSortSpecs* sort_specs = ImGui::TableGetSortSpecs();

  // We can only ever insert new tags, never remove, so the list of tags is
  // dirty if the size has changed.
  const bool table_is_dirty = sorted_tags_.size() != tags.size();
  const bool spec_is_dirty = sort_specs && sort_specs->SpecsDirty;
  if (sort_specs) {
    sort_ascending_ =
        sort_specs->Specs[0].SortDirection == ImGuiSortDirection_Ascending;
    sort_specs->SpecsDirty = false;
  }

  if (table_is_dirty || spec_is_dirty) {
    SortTags();
  }

  for (const std::string& tag : sorted_tags_) {
    if (!filter_.PassFilter(tag.c_str())) {
      continue;
    }

    ImGui::PushID(id++);

    ImGui::TableNextRow();
    ImGui::TableNextColumn();

    bool is_enabled = debug_draw::IsTagEnabled(tag);

    if (ImGui::Checkbox("##Enabled", &is_enabled)) {
      debug_draw::SetTagEnabled(tag, is_enabled);
    }

    ImGui::TableNextColumn();

    ImGui::TextColored(is_enabled ? kEnabledColor : kDisabledColor, "%s",
                       tag.c_str());

    ImGui::PopID();
  }

  ImGui::EndTable();
}

void DebugDrawWidget::SortTags() {
  absl::Span<const std::string> tags = debug_draw::GetAllRegisteredTags();

  sorted_tags_.clear();
  sorted_tags_.reserve(tags.size());

  for (const std::string& tag : tags) {
    sorted_tags_.push_back(tag);
  }

  absl::c_stable_sort(sorted_tags_, [this](auto& lhs, auto& rhs) {
    return sort_ascending_ ? lhs < rhs : lhs > rhs;
  });
}

}  // namespace imp::editor
