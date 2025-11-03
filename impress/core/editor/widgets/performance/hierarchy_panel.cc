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
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <stack>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/trace.h"
#include "core/performance/profiler.h"

namespace imp::editor {

HierarchyPanel::HierarchyPanel() {}

HierarchyPanel::~HierarchyPanel() = default;

ProfilerSampleNode* HierarchyPanel::CreateTree(int frame_index) {
  IMP_TRACE();

  // Returns 0 if the frame is not currently available or contains no samples.
  int sample_count = Profiler::GetSampleCount(frame_index);
  if (sample_count <= 0) {
    return nullptr;
  }

  std::array<ProfileResult, Profiler::kMaxSamples>& samples =
      Profiler::GetSamples(frame_index);

  frame_duration_ = Profiler::GetRenderNextFrameDurationNanos(frame_index);

  ProfilerSampleNode* root = &sample_nodes_[0];
  // Effectively a call stack. Each sample is added to the stack in the loop.
  // When a sample does not take place during the start/end time of the sample
  // at the top of the stack, it gets popped and that repeats until we find a
  // sample that encapsulates the current sample.
  std::stack<ProfilerSampleNode*> active_nodes;

  root->result = &samples[0];
  root->total_time = frame_duration_;
  root->calls = 1;
  root->children.clear();
  active_nodes.push(root);
  for (size_t i = 1; i < sample_count; ++i) {
    if (samples[i].thread_id != Profiler::GetMainThreadId()) {
      continue;
    }

    // Pop nodes that have ended before the current sample starts
    while (!active_nodes.empty() && active_nodes.top()->result->sample_end_id <=
                                        samples[i].sample_end_id) {
      active_nodes.pop();
    }

    if (active_nodes.empty()) {
      // Should not happen if samples are well formed.
      // This means samples were not added in the order they occurred in or
      // that the root sample was not actually the true root.
      // For example if IMP_TRACE() was removed from
      // FilamentHost::RenderNextFrame() then the child samples would not have
      // that as a common root and lead to this code path.
      // Every IMP_TRACE should share the same root.
      IMP_LOG(imp::WARNING) << "Sample " << samples[i].name
                   << " does not have a parent sample on frame "
                   << (frame_index - 1);
      return nullptr;
    }

    // Create a new node and add it as a child of the sample atop the stack.
    ProfilerSampleNode* new_node = &sample_nodes_[i];
    new_node->result = &samples[i];
    new_node->total_time = samples[i].duration_ns;
    new_node->calls = 1;
    new_node->children.clear();
    active_nodes.top()->children.push_back(new_node);
    active_nodes.push(new_node);
  }

  return root;
}

void HierarchyPanel::DrawTable(ProfilerSampleNode* root) {
  IMP_TRACE();

  // Build Table UI and draw recursively.
  ImGui::BeginChild("tree_table_child", ImVec2(0, 500), true);

  constexpr float kDetailColumnWidth = 100.0f;
  constexpr int kNumColumns = 4;
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

    if (root) {
      // Passed by reference so that we don't have to calculate what the row
      // index should be for each recursive call to DrawTree. As a result we
      // can just increment the value rather than returning the current index
      // from each recursive call as it draws the node's children.
      int row_index = 0;
      DrawTreeNode(root, 0, row_index);
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
}

void HierarchyPanel::DrawPanel(int frame_index) {
  IMP_TRACE();

  ProfilerSampleNode* root = CreateTree(frame_index);

  DrawTable(root);
}

void HierarchyPanel::DrawTreeNode(ProfilerSampleNode* node, int depth,
                                  int& row_index) {
  if (depth > kMaxTreeDepth) return;

  ImGuiTreeNodeFlags flag = 0;

  ProfileResult* result = node->result;
  uint32_t elapsed_time = node->total_time;

  if (node->children.empty()) {
    // No children, don't show a dropdown arrow.
    flag |= ImGuiTreeNodeFlags_Leaf;
  } else {
    // Has children, create groups of the same call and add their frametimes.
    std::map<absl::string_view, ProfilerSampleNode*> unique_children;

    absl::string_view child_name;
    for (auto& child : node->children) {
      child_name = child->result->name;

      // If this is a new group, add it to the map.
      if (unique_children.find(child_name) == unique_children.end()) {
        unique_children[child_name] = child;
      } else {
        // If this group exists, increment the call count and add the frametime.
        unique_children[child_name]->calls += 1;
        unique_children[child_name]->total_time += child->total_time;

        // Add child's children to the group's children.
        for (auto& child_child : child->children) {
          unique_children[child_name]->children.push_back(child_child);
        }
      }
    }

    // Sort grouped children by total frametime used.
    std::vector<ProfilerSampleNode*> sorted_children;
    sorted_children.reserve(unique_children.size());

    for (auto& [_, child] : unique_children) {
      sorted_children.push_back(child);
    }

    std::sort(sorted_children.begin(), sorted_children.end(),
              [](const ProfilerSampleNode* a, const ProfilerSampleNode* b) {
                return a->total_time > b->total_time;
              });

    // Replace existing children vector with grouped/sorted one.
    node->children = sorted_children;
  }

  // Alternate background colors for each row.
  ImGui::TableNextRow();
  ImU32 row_color = row_index % 2 == 0
                        ? ImGui::GetColorU32(ImGuiCol_TableRowBg)
                        : ImGui::GetColorU32(ImGuiCol_TableRowBgAlt);
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
                             static_cast<float>(frame_duration_));

  // Recursively draw children.
  ImGui::TableSetColumnIndex(0);

  if (ImGui::TreeNodeEx(result->name.data(), flag)) {
    for (auto& child : node->children) {
      DrawTreeNode(child, depth + 1, row_index);
    }
    ImGui::TreePop();
  }
}
}  // namespace imp::editor
