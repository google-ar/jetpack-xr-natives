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
#include <memory>
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

void HierarchyPanel::DrawPanel(int frame_index) {
  IMP_TRACE_NAME("HierarchyPanel::DrawPanel");

  int sample_count = Profiler::GetSampleCount(frame_index - 1);

  if (sample_count <= 0) {
    return;
  }

  std::array<ProfileResult, Profiler::kMaxSamples> samples =
      Profiler::GetSamples(frame_index - 1);

  frame_duration_ = Profiler::GetFrameDuration(frame_index - 1);

  std::unique_ptr<ProfilerSampleNode> root =
      std::make_unique<ProfilerSampleNode>();
  // Effectively a call stack. Each sample is added to the stack in the loop.
  // When a sample does not take place during the start/end time of the sample
  // at the top of the stack, it gets popped and that repeats until we find a
  // sample that encapsulates the current sample.
  std::stack<ProfilerSampleNode*> active_nodes;

  root->result = &samples[0];
  root->total_time = frame_duration_;
  root->calls = 1;
  active_nodes.push(root.get());

  for (size_t i = 1; i < sample_count; ++i) {
    IMP_TRACE_NAME("Create Sample Tree");
    if (samples[i].thread_id != Profiler::GetMainThreadId()) {
      continue;
    }

    // Pop nodes that have ended before the current sample starts
    while (!active_nodes.empty() &&
           active_nodes.top()->result->end_time <= samples[i].start_time) {
      active_nodes.pop();
    }
    if (active_nodes.empty()) {
      // Should not happen if samples are well formed.
      IMP_LOG(imp::WARNING) << "Sample " << samples[i].name
                   << " does not have a parent sample on frame "
                   << (frame_index - 1);
      return;
    }
    // Create a new node and add it as a child of the sample atop the stack.
    std::unique_ptr<ProfilerSampleNode> new_node =
        std::make_unique<ProfilerSampleNode>();
    new_node->result = &samples[i];
    new_node->total_time = samples[i].end_time - samples[i].start_time;
    new_node->calls = 1;
    ProfilerSampleNode* new_node_ptr = new_node.get();
    active_nodes.top()->children.push_back(std::move(new_node));
    active_nodes.push(new_node_ptr);
  }

  {
    IMP_TRACE_NAME("Draw Imgui Table");
    // Build Table UI and draw recursively.
    ImGui::BeginChild("tree_table_child", ImVec2(0, 500), true);

    constexpr float kDetailColumnWidth = 100.0f;
    if (ImGui::BeginTable("tree table", 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY)) {
      ImGui::TableSetupColumn("Hierarchy", ImGuiTableColumnFlags_WidthStretch,
                              1.0f);
      ImGui::TableSetupColumn("Calls", ImGuiTableColumnFlags_WidthFixed,
                              kDetailColumnWidth);
      ImGui::TableSetupColumn("Frametime", ImGuiTableColumnFlags_WidthFixed,
                              kDetailColumnWidth);
      ImGui::TableSetupColumn("% of Frame", ImGuiTableColumnFlags_WidthFixed,
                              kDetailColumnWidth);
      ImGui::TableSetupScrollFreeze(0, 1);
      ImGui::TableHeadersRow();

      // Passed by reference so that we don't have to calculate what the row
      // index should be for each recursive call to DrawTree. As a result we can
      // just increment the value rather than returning the current index from
      // each recursive call as it draws the node's children.
      int row_index = 0;
      DrawTree(root.get(), 0, row_index);

      ImGui::EndTable();
    }
    ImGui::EndChild();
  }
}

void HierarchyPanel::DrawTree(ProfilerSampleNode* node, int depth,
                              int& row_index) {
  if (depth > kMaxTreeDepth) return;

  ImGuiTreeNodeFlags flag = 0;

  ProfileResult* result = node->result;
  int64_t elapsed_time = node->total_time;

  if (node->children.empty()) {
    // No children, don't show a dropdown arrow.
    flag |= ImGuiTreeNodeFlags_Leaf;
  } else {
    // Has children, create groups of the same call and add their frametimes.
    std::map<absl::string_view, std::unique_ptr<ProfilerSampleNode>>
        unique_children;

    absl::string_view child_name;
    for (auto& child : node->children) {
      child_name = child->result->name;
      if (unique_children.find(child_name) == unique_children.end()) {
        unique_children[child_name] = std::move(child);
      } else {
        unique_children[child_name]->calls += 1;
        unique_children[child_name]->total_time += child->total_time;
        // Add child's children to the group's children.
        for (auto& child_child : child->children) {
          unique_children[child_name]->children.push_back(
              std::move(child_child));
        }
      }
    }

    // Sort grouped children by total frametime used.
    std::vector<std::unique_ptr<ProfilerSampleNode>> sorted_children;
    sorted_children.reserve(unique_children.size());

    for (auto& [_, child] : unique_children) {
      sorted_children.push_back(std::move(child));
    }

    std::sort(sorted_children.begin(), sorted_children.end(),
              [](const std::unique_ptr<ProfilerSampleNode>& a,
                 const std::unique_ptr<ProfilerSampleNode>& b) {
                return a->total_time > b->total_time;
              });

    // Replace existing children vector with grouped/sorted one.
    node->children = std::move(sorted_children);
  }

  // Alternate background colors for each row.
  ImGui::TableNextRow();
  if (row_index % 2 == 0) {
    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                           ImGui::GetColorU32(ImGuiCol_TableRowBg));
  } else {
    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                           ImGui::GetColorU32(ImGuiCol_TableRowBgAlt));
  }
  row_index++;

  // Fill out information for this row.
  ImGui::TableSetColumnIndex(1);
  ImGui::Text("%i", node->calls);

  ImGui::TableSetColumnIndex(2);
  float duration = (float)elapsed_time / 1000000.0f;
  ImGui::Text("%.3f ms", duration);

  ImGui::TableSetColumnIndex(3);
  ImGui::Text("%.1f %%", 100.0f * elapsed_time / frame_duration_);

  // Recursively draw children.
  ImGui::TableSetColumnIndex(0);

  if (ImGui::TreeNodeEx(result->name.data(), flag)) {
    for (auto& child : node->children) {
      DrawTree(child.get(), depth + 1, row_index);
    }
    ImGui::TreePop();
  }
}
}  // namespace imp::editor
