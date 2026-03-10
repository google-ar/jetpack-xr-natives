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
#include <thread>  // NOLINT: Need to use std::thread::id.
#include <vector>

#include "absl/container/btree_map.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/frame_time_panel.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/profiler.h"
#include "core/performance/profiler_structs.h"

namespace imp::editor {

namespace {
// Minimum height for the panel no matter how small the window is.
constexpr float kMinPanelHeight = 250.0f;
// Width of the columns that display details about the sample.
constexpr float kDetailColumnWidthWide = 100.0f;
constexpr float kDetailColumnWidthNarrow = 50.0f;
constexpr bool kShowMemoryColumns = Profiler::IsMemoryTrackingSupported();
// Number of columns to display in the table.
constexpr int kNumColumns = kShowMemoryColumns ? 6 : 4;
constexpr float kNanosPerMs = 1000000.0f;
}  // namespace

void HierarchyPanel::DrawPanel(float width, int frame_index,
                               SampleProcessor& sample_processor,
                               FrameTimePanel& frame_time_panel) {
  IMP_TRACE();

  if (!thread_set_) {
    current_thread_id_ = Profiler::GetMainThreadId();
    thread_set_ = true;
  }

  // Choose which thread to display samples for.
  DrawThreadSelector();

  bool valid_frame = Profiler::HasFrameRecorded(frame_index);

  // Build Table UI and draw recursively.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

  // Hierarchy table takes up the remaining space in the window.
  // If the window is too small it won't be displayed so we set a min height.
  const ImVec2 available_size = ImGui::GetContentRegionAvail();
  const float child_height = std::max(kMinPanelHeight, available_size.y);

  if (ImGui::BeginTable("tree table", kNumColumns,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                        ImVec2(width, child_height))) {
    ImGui::TableSetupColumn("Hierarchy", ImGuiTableColumnFlags_WidthStretch,
                            1.0f);
    ImGui::TableSetupColumn("Calls", ImGuiTableColumnFlags_WidthFixed,
                            kDetailColumnWidthNarrow);
    if (kShowMemoryColumns) {
      ImGui::TableSetupColumn("Allocations", ImGuiTableColumnFlags_WidthFixed,
                              kDetailColumnWidthWide);
      ImGui::TableSetupColumn("Alloc Size", ImGuiTableColumnFlags_WidthFixed,
                              kDetailColumnWidthWide);
    }
    ImGui::TableSetupColumn("Frametime", ImGuiTableColumnFlags_WidthFixed,
                            kDetailColumnWidthWide);
    ImGui::TableSetupColumn("% of Frame", ImGuiTableColumnFlags_WidthFixed,
                            kDetailColumnWidthWide);
    ImGui::TableSetupScrollFreeze(0, 1);  // Freeze the first row.
    ImGui::TableHeadersRow();             // First row contains the headers.

    if (valid_frame) {
      if (current_thread_id_ == Profiler::GetMainThreadId()) {
        DrawMainThreadSamples(frame_index, sample_processor, frame_time_panel);
      } else {
        DrawWorkerThreadSamples(frame_index, sample_processor,
                                current_thread_id_, frame_time_panel);
      }
    } else {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Choose a recorded frame to view the hierarchy.");
    }

    ImGui::EndTable();
  }
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding
}

void HierarchyPanel::DrawMainThreadSamples(int frame_index,
                                           SampleProcessor& sample_processor,
                                           FrameTimePanel& frame_time_panel) {
  ProcessedSamples& processed_frame =
      sample_processor.GetProcessedFrame(frame_index);

  int row_index = 0;
  SampleNode* root;
  // Create a hard copy of the tree before we start modifying its data.
  // We modify the sample data, children, and ordering in DrawTreeNode.
  const std::vector<SampleNode*> tree_copy =
      GetTreeHardCopy(processed_frame.sample_roots, main_thread_node_pool_);
  const size_t root_count = tree_copy.size();
  for (size_t i = 0; i < root_count; ++i) {
    root = tree_copy[i];
    root_duration_ns_ = root->total_time_ns;
    DrawTreeNode(frame_time_panel, root, 0, row_index);
  }
}

void HierarchyPanel::DrawWorkerThreadSamples(int frame_index,
                                             SampleProcessor& sample_processor,
                                             std::thread::id thread_id,
                                             FrameTimePanel& frame_time_panel) {
  const FrameMetaData& frame_metadata = Profiler::GetFrameMetaData(frame_index);
  const uint64_t start_time = frame_metadata.frame_start_time_ns;
  const uint64_t end_time = frame_metadata.total_duration_ns + start_time;
  std::vector<WorkerProfileResult> raw_samples =
      Profiler::GetWorkerThreadSamples(start_time, end_time, thread_id);
  const ProcessedSamples current_worker_samples_ =
      sample_processor.ProcessWorkerThreadSamples(raw_samples);
  int row_index = 0;
  const size_t root_count = current_worker_samples_.sample_roots.size();
  root_duration_ns_ = frame_metadata.total_duration_ns;
  for (size_t i = 0; i < root_count; ++i) {
    SampleNode* root = current_worker_samples_.sample_roots[i];
    DrawTreeNode(frame_time_panel, root, 0, row_index);
  }
}

void HierarchyPanel::DrawTreeNode(FrameTimePanel& frame_time_panel,
                                  SampleNode* node, int depth, int& row_index) {
  if (depth > kMaxTreeDepth) return;

  ImGuiTreeNodeFlags flag = ImGuiTreeNodeFlags_OpenOnArrow;

  SampleNode* child = node->first_child;

  if (!child) {
    // No children, don't show a dropdown arrow.
    flag |= ImGuiTreeNodeFlags_Leaf;
  } else {
    // Has children, create groups of the same call and add their frametimes.
    // Using btree map to ensure consistent ordering of the children.
    absl::btree_map<absl::string_view, SampleNode*> unique_children;
    absl::string_view child_name;

    SampleNode* grandchild;

    while (child != nullptr) {
      child_name = child->result->GetName();

      // If this is a new group, add it to the map.
      if (unique_children.find(child_name) == unique_children.end()) {
        unique_children[child_name] = child;
      } else {
        // If this group exists, increment the call count and add other values.
        SampleNode* child_node = unique_children[child_name];
        child_node->calls += 1;
        child_node->total_time_ns += child->total_time_ns;
        child_node->total_memory_allocated += child->total_memory_allocated;
        child_node->total_memory_allocations_count +=
            child->total_memory_allocations_count;

        // Add child's children to the group's children.
        grandchild = child->first_child;
        while (grandchild != nullptr) {
          SampleNode* next_grandchild = grandchild->next_sibling;
          // Detach grandchild from its original sibling list before adding.
          grandchild->next_sibling = nullptr;
          child_node->AddChild(grandchild);
          grandchild = next_grandchild;
        }
      }

      child = child->next_sibling;
    }

    // Sort grouped children by total frametime used.
    std::vector<SampleNode*> sorted_children;
    sorted_children.reserve(unique_children.size());

    for (auto& [_, child] : unique_children) {
      sorted_children.push_back(child);
    }

    std::sort(sorted_children.begin(), sorted_children.end(),
              [](const SampleNode* a, const SampleNode* b) {
                return a->total_time_ns < b->total_time_ns;
              });

    // Replace existing children with grouped/sorted ones.
    node->first_child = nullptr;
    for (size_t i = 0; i < sorted_children.size(); ++i) {
      node->AddChild(sorted_children[i]);
    }
  }

  const absl::string_view name = node->result->GetName();

  DrawTableRow(frame_time_panel, name.data(), node->total_time_ns, node->calls,
               node->total_memory_allocated,
               node->total_memory_allocations_count, row_index);

  // Recursively draw children.
  ImGui::TableSetColumnIndex(0);

  const bool is_open = ImGui::TreeNodeEx(name.data(), flag);
  if (ImGui::IsItemClicked()) {
    frame_time_panel.SetSelectedSampleName(name);
  }

  if (is_open) {
    SampleNode* child = node->first_child;
    while (child != nullptr) {
      DrawTreeNode(frame_time_panel, child, depth + 1, row_index);
      child = child->next_sibling;
    }
    ImGui::TreePop();
  }
}

void HierarchyPanel::DrawTableRow(FrameTimePanel& frame_time_panel,
                                  const char* name, uint32_t time, int calls,
                                  size_t memory_allocated,
                                  size_t allocations_count, int& row_index) {
  // Alternate background colors for each row.
  ImGui::TableNextRow();
  ImU32 row_color;
  const absl::string_view selected_sample_name =
      frame_time_panel.GetSelectedSampleName();
  if (selected_sample_name == name) {
    // Highlight color for the selected row
    row_color = ImGui::GetColorU32(ImGuiCol_HeaderHovered);
  } else {
    row_color = row_index % 2 == 0 ? ImGui::GetColorU32(ImGuiCol_TableRowBg)
                                   : ImGui::GetColorU32(ImGuiCol_TableRowBgAlt);
  }
  ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, row_color);
  row_index++;

  int column_index = 1;  // Skip the first column, it's handled last.

  // Fill out information for this row.
  ImGui::TableSetColumnIndex(column_index++);
  ImGui::Text("%i", calls);

  // Memory tracking may not be supported on all platforms.
  if (Profiler::IsMemoryTrackingSupported()) {
    ImGui::TableSetColumnIndex(column_index++);
    ImGui::Text("%zu", allocations_count);

    ImGui::TableSetColumnIndex(column_index++);
    const size_t allocated = memory_allocated;
    constexpr size_t kKilobyte = 1024;
    constexpr size_t kMegabyte = 1024 * 1024;

    if (allocated > kMegabyte) {
      ImGui::Text("%.1f MB", (float)allocated / kMegabyte);
    } else if (allocated > kKilobyte) {
      ImGui::Text("%.1f KB", (float)allocated / kKilobyte);
    } else {
      ImGui::Text("%zu B", allocated);
    }
  }

  ImGui::TableSetColumnIndex(column_index++);
  const float duration = static_cast<float>(time) / kNanosPerMs;
  ImGui::Text("%.3f ms", duration);

  ImGui::TableSetColumnIndex(column_index++);
  ImGui::Text("%.1f %%", 100.0f * static_cast<float>(time) /
                             static_cast<float>(root_duration_ns_));
}

void HierarchyPanel::DrawThreadSelector() {
  // Allow us to switch between threads and see their samples.
  if (!ImGui::BeginCombo("##combo", current_thread_)) return;

  absl::string_view it_thread_name;
  const std::vector<std::thread::id> thread_ids =
      Profiler::GetWorkerThreadIds();

  for (int i = 0; i < thread_ids.size(); ++i) {
    it_thread_name = Profiler::GetThreadName(thread_ids[i]);
    bool is_selected = (current_thread_ == it_thread_name);

    if (ImGui::Selectable(it_thread_name.data(), is_selected)) {
      current_thread_ = it_thread_name.data();
      current_thread_id_ = thread_ids[i];
    }

    if (is_selected) {
      ImGui::SetItemDefaultFocus();
    }
  }

  ImGui::EndCombo();
}

std::vector<SampleNode*> HierarchyPanel::GetTreeHardCopy(
    std::vector<SampleNode*>& roots, NodePool& node_pool) {
  std::vector<SampleNode*> tree_copy;
  tree_copy.reserve(roots.size());
  node_pool.ResetIndex();
  for (SampleNode* source_root : roots) {
    tree_copy.push_back(CopyNodeToPool(source_root, node_pool));
  }
  return tree_copy;
}

SampleNode* HierarchyPanel::CopyNodeToPool(SampleNode* node,
                                           NodePool& node_pool) {
  if (!node) return nullptr;

  SampleNode* node_copy = node_pool.GetNext();
  node_copy->result = node->result;
  node_copy->total_time_ns = node->total_time_ns;
  node_copy->first_child = CopyNodeToPool(node->first_child, node_pool);
  node_copy->next_sibling = CopyNodeToPool(node->next_sibling, node_pool);
  node_copy->total_memory_allocated = node->total_memory_allocated;
  node_copy->total_memory_allocations_count =
      node->total_memory_allocations_count;
  node_copy->calls = 1;
  return node_copy;
}

}  // namespace imp::editor
