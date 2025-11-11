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

#include "core/editor/widgets/performance/hierarchy_panel.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <thread>  // NOLINT: Need to sort things by thread id.
#include <vector>

#include "absl/container/btree_map.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/performance/profiler.h"

namespace imp::editor {

HierarchyPanel::HierarchyPanel() {}

HierarchyPanel::~HierarchyPanel() = default;

absl::string_view HierarchyPanel::GetSelectedSampleName() {
  return selected_sample_name_;
}

void HierarchyPanel::DrawPanel(int frame_index,
                               SampleProcessor& sample_processor) {
  IMP_TRACE();

  // Minimum height for the panel no matter how small the window is.
  constexpr float kMinPanelHeight = 250.0f;
  // Width of the columns that display details about the sample.
  constexpr float kDetailColumnWidth = 100.0f;
  // Number of columns to display in the table.
  constexpr int kNumColumns = 4;

  ProcessedFrame& processed_frame =
      sample_processor.GetProcessedFrame(frame_index);

  if (!thread_set_) {
    current_thread_id_ = Profiler::GetMainThreadId();
    thread_set_ = true;
  }

  absl::string_view it_thread_name;

  // Allow us to switch between threads and see their samples.
  if (ImGui::BeginCombo("##combo", current_thread_)) {
    for (auto& it : processed_frame.samples_by_thread_and_name) {
      it_thread_name = Profiler::GetThreadName(it.first);
      bool is_selected = (current_thread_ == it_thread_name);
      if (ImGui::Selectable(it_thread_name.data(), is_selected)) {
        current_thread_ = it_thread_name.data();
        current_thread_id_ = it.first;
      }
      if (is_selected) {
        ImGui::SetItemDefaultFocus();
      }
    }

    ImGui::EndCombo();
  }

  bool has_samples =
      processed_frame.samples_by_thread_and_name.contains(current_thread_id_);

  // Build Table UI and draw recursively.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

  // Hierarchy table takes up the remaining space in the window.
  // If the window is too small it won't be displayed so we set a min height.
  const float available_height = ImGui::GetContentRegionAvail().y;
  const float child_height = std::max(kMinPanelHeight, available_height);

  ImGui::BeginChild("tree_table_child", ImVec2(0, child_height), true);

  if (ImGui::BeginTable("tree table", kNumColumns,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY)) {
    ImGui::TableSetupColumn("Hierarchy", ImGuiTableColumnFlags_WidthStretch,
                            1.0f);
    ImGui::TableSetupColumn("Calls", ImGuiTableColumnFlags_WidthFixed,
                            kDetailColumnWidth);
    ImGui::TableSetupColumn("Frametime", ImGuiTableColumnFlags_WidthFixed,
                            kDetailColumnWidth);
    ImGui::TableSetupColumn("% of Frame", ImGuiTableColumnFlags_WidthFixed,
                            kDetailColumnWidth);
    ImGui::TableSetupScrollFreeze(0, 1);  // Freeze the first row.
    ImGui::TableHeadersRow();             // First row contains the headers.

    if (has_samples) {
      // Passed by reference so that we don't have to calculate what the row
      // index should be for each recursive call to DrawTree. As a result we
      // can just increment the value rather than returning the current index
      // from each recursive call as it draws the node's children.
      int row_index = 0;
      ProcessedThreadSamples& processed_thread_samples =
          processed_frame.samples_by_thread_and_name[current_thread_id_];
      ProfilerSampleNode* root;
      size_t root_count = processed_thread_samples.sample_roots.size();
      for (size_t i = 0; i < root_count; ++i) {
        root = processed_thread_samples.sample_roots[i];
        root_duration_ns_ = root->total_time;
        DrawTreeNode(root, 0, row_index);
      }
    } else {
      // If the root is null, we still draw the table/headers.
      // Also add a message to the first column to indicate no samples.
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("No samples available for this frame.");
    }

    ImGui::EndTable();
  }
  ImGui::EndChild();
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding
}

void HierarchyPanel::DrawTreeNode(ProfilerSampleNode* node, int depth,
                                  int& row_index) {
  if (depth > kMaxTreeDepth) return;

  ImGuiTreeNodeFlags flag = ImGuiTreeNodeFlags_OpenOnArrow;

  ProfileResult* result = node->result;
  uint32_t elapsed_time = node->total_time;

  ProfilerSampleNode* child = node->first_child;

  if (!child) {
    // No children, don't show a dropdown arrow.
    flag |= ImGuiTreeNodeFlags_Leaf;
  } else {
    // Has children, create groups of the same call and add their frametimes.
    // Using btree map to ensure consistent ordering of the children.
    absl::btree_map<absl::string_view, ProfilerSampleNode*> unique_children;
    absl::string_view child_name;

    ProfilerSampleNode* grandchild;

    while (child != nullptr) {
      child_name = child->result->name;

      // If this is a new group, add it to the map.
      if (unique_children.find(child_name) == unique_children.end()) {
        unique_children[child_name] = child;
      } else {
        // If this group exists, increment the call count and add the
        // frametime.
        ProfilerSampleNode* child_node = unique_children[child_name];
        child_node->calls += 1;
        child_node->total_time += child->total_time;

        // Add child's children to the group's children.
        grandchild = child->first_child;
        while (grandchild != nullptr) {
          ProfilerSampleNode* next_grandchild = grandchild->next_sibling;
          // Detach grandchild from its original sibling list before adding.
          grandchild->next_sibling = nullptr;
          child_node->AddChild(grandchild);
          grandchild = next_grandchild;
        }
      }

      child = child->next_sibling;
    }

    // Sort grouped children by total frametime used.
    std::vector<ProfilerSampleNode*> sorted_children;
    sorted_children.reserve(unique_children.size());

    for (auto& [_, child] : unique_children) {
      sorted_children.push_back(child);
    }

    std::sort(sorted_children.begin(), sorted_children.end(),
              [](const ProfilerSampleNode* a, const ProfilerSampleNode* b) {
                return a->total_time < b->total_time;
              });

    // Replace existing children with grouped/sorted ones.
    node->first_child = nullptr;
    for (size_t i = 0; i < sorted_children.size(); ++i) {
      node->AddChild(sorted_children[i]);
    }
  }

  // Alternate background colors for each row.
  ImGui::TableNextRow();
  ImU32 row_color;
  if (selected_sample_name_ == result->name) {
    // Highlight color for the selected row
    row_color = ImGui::GetColorU32(ImGuiCol_HeaderHovered);
  } else {
    row_color = row_index % 2 == 0 ? ImGui::GetColorU32(ImGuiCol_TableRowBg)
                                   : ImGui::GetColorU32(ImGuiCol_TableRowBgAlt);
  }
  ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, row_color);
  row_index++;

  // Fill out information for this row.
  ImGui::TableSetColumnIndex(1);
  ImGui::Text("%i", node->calls);

  ImGui::TableSetColumnIndex(2);
  float duration = static_cast<float>(elapsed_time) / 1000000.0f;
  ImGui::Text("%.3f ms", duration);

  ImGui::TableSetColumnIndex(3);
  ImGui::Text("%.1f %%", 100.0f * static_cast<float>(elapsed_time) /
                             static_cast<float>(root_duration_ns_));

  // Recursively draw children.
  ImGui::TableSetColumnIndex(0);

  bool is_open = ImGui::TreeNodeEx(result->name.data(), flag);
  if (ImGui::IsItemClicked()) {
    selected_sample_name_ = result->name;
  }

  if (is_open) {
    ProfilerSampleNode* child = node->first_child;
    while (child != nullptr) {
      DrawTreeNode(child, depth + 1, row_index);
      child = child->next_sibling;
    }
    ImGui::TreePop();
  }
}
}  // namespace imp::editor
