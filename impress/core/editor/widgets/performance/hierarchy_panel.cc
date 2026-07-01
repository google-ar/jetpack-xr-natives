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
#include <limits>
#include <memory>
#include <string>
#include <thread>  // NOLINT: Need to use std::thread::id.
#include <vector>

#include "absl/container/btree_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/profiler_data_provider.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/editor/widgets/performance/search_filter.h"
#include "core/performance/profiler.h"
#include "core/performance/profiler_structs.h"

namespace imp::editor {

namespace {
// Color to highlight a selected row.
constexpr ImU32 kSelectedRowColor = IM_COL32(11, 87, 208, 128);
// Minimum height for the panel no matter how small the window is.
constexpr float kMinPanelHeight = 250.0f;
// Width of the columns that display details about the sample.
constexpr float kDetailColumnWidthWide = 85.0f;
constexpr float kDetailColumnWidthNarrow = 40.0f;
constexpr bool kShowMemoryColumns = Profiler::IsMemoryTrackingSupported();
// Number of columns to display in the table.
constexpr int kNumColumns = kShowMemoryColumns ? 6 : 4;
constexpr float kNanosPerMs = 1000000.0f;
}  // namespace

void HierarchyPanel::DrawPanel(const float width,
                               ProfilerDataProvider& data_provider,
                               const std::thread::id thread_id,
                               const absl::string_view search_query) {
  IMP_TRACE();

  // Detect transitions on search query
  if (search_query.empty() && !last_search_query_.empty()) {
    should_restore_states_ = true;
    should_expand_selected_ = selected_during_search_;
    selected_during_search_ = false;
  } else if (!search_query.empty() && last_search_query_.empty()) {
    selected_during_search_ = false;
  }
  last_search_query_ = std::string(search_query);

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

    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, kSelectedRowColor);
    ImGui::PushStyleColor(ImGuiCol_HeaderActive, kSelectedRowColor);
    bool any_valid_frames = false;
    if (thread_id == Profiler::GetMainThreadId()) {
      any_valid_frames = DrawMainThreadSamples(data_provider, search_query);
    } else {
      any_valid_frames =
          DrawWorkerThreadSamples(thread_id, data_provider, search_query);
    }
    ImGui::PopStyleColor();  // ImGuiCol_HeaderActive
    ImGui::PopStyleColor();  // ImGuiCol_HeaderHovered

    if (!any_valid_frames) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Choose a recorded frame to view the hierarchy.");
    }

    ImGui::EndTable();
  }
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding

  if (!search_query.empty() && data_provider.HasSelectedSampleChanged()) {
    selected_during_search_ = true;
  }
  should_restore_states_ = false;
  should_expand_selected_ = false;
}

bool HierarchyPanel::DrawMainThreadSamples(
    ProfilerDataProvider& data_provider, const absl::string_view search_query) {
  const int start_frame = data_provider.GetSelectedFrameStart();
  const int end_frame = data_provider.GetSelectedFrameEnd();
  SampleProcessor& sample_processor = data_provider.GetSampleProcessor();

  bool any_valid_frames = false;
  for (int frame_index = start_frame, i = 0; frame_index <= end_frame;
       ++frame_index, ++i) {
    // Need to push a unique ID otherwise opening a sample with a duplicate name
    // will open all samples with that name.

    // The reason we choose i instead of frame_index is for UX.
    // If the ID pushed changes for different frames then it won't remember
    // the open/closed state of the samples across frames.
    // This would mean that you navigate through the tree to a sample you're
    // interested in, and then it closes when you navigate to the next frame.

    // If the ID is always the same though then in the multi-frame case it will
    // open the tree for every sample across every frame which we don't want.

    // By using i we ensure the ID is always 0 for the single-frame case but
    // unique across each frame for the multi-frame case.

    // (TODO: (broken link)) - Support grouping main thread samples when multiple
    // frames are selected. Push/PopID would no longer be needed.
    ImGui::PushID(i);
    if (DrawMainThreadFrame(frame_index, sample_processor, data_provider,
                            search_query)) {
      any_valid_frames = true;
    }
    ImGui::PopID();
  }
  return any_valid_frames;
}

bool HierarchyPanel::DrawMainThreadFrame(const int frame_index,
                                         SampleProcessor& sample_processor,
                                         ProfilerDataProvider& data_provider,
                                         const absl::string_view search_query) {
  const ProcessedSamples& processed_frame =
      sample_processor.GetProcessedFrame(frame_index);

  if (processed_frame.sample_roots.empty()) return false;

  int row_index = 0;
  // Create a hard copy of the tree before we start modifying its data.
  // We modify the sample data, children, and ordering in DrawTreeNode.
  const std::vector<SampleNode*> tree_copy =
      GetTreeHardCopy(processed_frame.sample_roots, main_thread_node_pool_);
  const size_t root_count = tree_copy.size();

  HierarchyDrawArgs args = {
      .data_provider = data_provider,
      .thread_id = Profiler::GetMainThreadId(),
      .row_index = row_index,
      .root_duration_ns = 0,
      .node = nullptr,
      .depth = 0,
      .search_query = search_query,
      .should_expand_selected = should_expand_selected_,
      .selected_sample_name = data_provider.GetSelectedSampleName(),
      .saved_node_states = saved_node_states_,
      .should_restore_states = should_restore_states_,
  };

  // Static vector to avoid reallocating every time.
  static std::vector<SampleNode*> filtered_roots;
  filtered_roots.clear();
  filtered_roots.reserve(root_count);
  for (size_t i = 0; i < root_count; ++i) {
    SampleNode* root = tree_copy[i];
    if (FilterTree(root, search_query)) {
      filtered_roots.push_back(root);
    }
  }

  const size_t filtered_root_count = filtered_roots.size();
  for (size_t i = 0; i < filtered_root_count; ++i) {
    args.node = filtered_roots[i];
    args.root_duration_ns = args.node->total_time_ns;
    args.depth = 0;
    DrawTreeNode(args);
  }

  return true;
}

bool HierarchyPanel::DrawWorkerThreadSamples(
    const std::thread::id thread_id, ProfilerDataProvider& data_provider,
    const absl::string_view search_query) {
  const int start_frame = data_provider.GetSelectedFrameStart();
  const int end_frame = data_provider.GetSelectedFrameEnd();
  SampleProcessor& sample_processor = data_provider.GetSampleProcessor();

  absl::StatusOr<FrameMetaData> start_frame_metadata =
      Profiler::GetFrameMetaData(start_frame);

  if (!start_frame_metadata.ok()) return false;

  absl::StatusOr<FrameMetaData> end_frame_metadata =
      Profiler::GetFrameMetaData(end_frame);

  if (!end_frame_metadata.ok()) return false;

  const uint64_t start_time = start_frame_metadata->frame_start_time_ns;
  const uint64_t end_time = end_frame_metadata->frame_start_time_ns +
                            end_frame_metadata->total_duration_ns;

  const std::vector<WorkerProfileResult> raw_samples =
      Profiler::GetWorkerThreadSamples(start_time, end_time, thread_id);

  if (raw_samples.empty()) return false;

  const ProcessedSamples current_worker_samples_ =
      sample_processor.ProcessWorkerThreadSamples(raw_samples);

  int row_index = 0;

  // Create a fake root sample to contain all the actual root samples and allow
  // them to be grouped/sorted together.
  // Creating the fake result here so it stays in scope for DrawTreeNode.
  WorkerProfileResult fake_result;
  std::unique_ptr<SampleNode> fake_root = CreateFakeRootSample(
      current_worker_samples_.sample_roots, start_time, end_time, fake_result);

  if (!FilterTree(fake_root.get(), search_query)) return true;

  HierarchyDrawArgs args = {
      .data_provider = data_provider,
      .thread_id = thread_id,
      .row_index = row_index,
      .root_duration_ns = static_cast<int64_t>(end_time - start_time),
      .node = fake_root.get(),
      .depth = 0,
      .search_query = search_query,
      .should_expand_selected = should_expand_selected_,
      .selected_sample_name = data_provider.GetSelectedSampleName(),
      .saved_node_states = saved_node_states_,
      .should_restore_states = should_restore_states_,
  };

  DrawTreeNode(args);

  return true;
}

void HierarchyPanel::DrawTreeNode(HierarchyDrawArgs& args) {
  SampleNode* node = args.node;
  const int depth = args.depth;
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

  DrawTableRow(args);

  // Recursively draw children.
  ImGui::TableSetColumnIndex(0);

  const ImGuiID id = ImGui::GetID(name.data());

  if (args.should_restore_states || !args.search_query.empty()) {
    ImGui::SetNextItemOpen(ShouldNodeBeOpen(args, node, id), ImGuiCond_Always);
  }

  const bool is_open = ImGui::TreeNodeEx(name.data(), flag);
  if (args.search_query.empty()) {
    args.saved_node_states[id] = is_open;
  }

  if (ImGui::IsItemClicked()) {
    args.data_provider.SetSelectedSampleName(name);
    args.data_provider.SetSelectedSampleThreadId(args.thread_id);
  }

  if (is_open) {
    SampleNode* child = node->first_child;
    while (child != nullptr) {
      args.node = child;
      args.depth = depth + 1;
      DrawTreeNode(args);
      child = child->next_sibling;
    }
    ImGui::TreePop();
  }
}

bool HierarchyPanel::ShouldNodeBeOpen(const HierarchyDrawArgs& args,
                                      SampleNode* node,
                                      const uint32_t id) const {
  // We're actively searching so we want to open all nodes that have matching
  // search results below them.
  if (!args.should_restore_states) {
    return !args.search_query.empty() && node->first_child != nullptr;
  }

  // We're done searching and want to expand the node if it's an ancestor of
  // the selected sample.
  if (args.should_expand_selected &&
      IsAncestorOfSelected(node, args.selected_sample_name)) {
    return true;
  }

  // Restoring states after search. Revert to the user's previously saved
  // open/closed state for this node if it exists; otherwise, default to closed.
  const auto it = args.saved_node_states.find(id);
  return it != args.saved_node_states.end() ? it->second : false;
}

void HierarchyPanel::DrawTableRow(HierarchyDrawArgs& args) {
  const SampleNode* node = args.node;
  const absl::string_view name = node->result->GetName();
  const uint64_t time = node->total_time_ns;
  const int calls = node->calls;
  const size_t memory_allocated = node->total_memory_allocated;
  const size_t allocations_count = node->total_memory_allocations_count;

  // Alternate background colors for each row.
  ImGui::TableNextRow();
  ImU32 row_color;
  if (args.data_provider.GetSelectedSampleName() == name) {
    // Highlight color for the selected row
    row_color = ImGui::GetColorU32(ImGuiCol_HeaderHovered);
  } else {
    row_color = args.row_index % 2 == 0
                    ? ImGui::GetColorU32(ImGuiCol_TableRowBg)
                    : ImGui::GetColorU32(ImGuiCol_TableRowBgAlt);
  }
  ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, row_color);
  args.row_index++;

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
                             static_cast<float>(args.root_duration_ns));
}

std::vector<SampleNode*>& HierarchyPanel::GetTreeHardCopy(
    const std::vector<SampleNode*>& roots, NodePool& node_pool) {
  // Static vector to avoid allocating memory each time a tree is drawn.
  // Safe to do since we are only calling this from main thread.
  static std::vector<SampleNode*> tree_copy;
  tree_copy.clear();
  tree_copy.reserve(roots.size());
  node_pool.ResetIndex();
  for (const SampleNode* source_root : roots) {
    tree_copy.push_back(CopyNodeToPool(source_root, node_pool));
  }
  return tree_copy;
}

SampleNode* HierarchyPanel::CopyNodeToPool(const SampleNode* node,
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

// Creates a fake root sample to contain all the actual root samples and allow
// them to be grouped/sorted together.
std::unique_ptr<SampleNode> HierarchyPanel::CreateFakeRootSample(
    const std::vector<SampleNode*>& roots, uint64_t start_time,
    uint64_t end_time, WorkerProfileResult& fake_result) {
  auto fake_root = std::make_unique<SampleNode>();
  const int root_count = roots.size();

  fake_result.name = "Selected Frame Window";
  // Must be the highest value of any sample in the timeframe in order for
  // grouping/sorting to work properly. Set to uint64_t max to ensure it is
  // always considered the last to complete.
  fake_result.sample_end_id = std::numeric_limits<uint64_t>::max();
  fake_result.start_time_ns = start_time;
  fake_result.end_time_ns = end_time;
  fake_result.thread_id = roots[0]->result->GetThreadId();
  fake_result.allocation_bytes = 0;
  fake_result.allocation_count = 0;
  fake_result.callstack_start_index =
      roots[0]->result->GetCallstackStartIndex();
  fake_result.callstack_end_index =
      roots[root_count - 1]->result->GetCallstackEndIndex();

  fake_root->SetInitialValues(&fake_result);

  // Aggregate memory stats for all the root samples.
  for (size_t i = 0; i < roots.size(); ++i) {
    fake_root->total_memory_allocated += roots[i]->total_memory_allocated;
    fake_root->total_memory_allocations_count +=
        roots[i]->total_memory_allocations_count;

    // Build sibling linked list for the fake root.
    if (i < root_count - 1) {
      roots[i]->next_sibling = roots[i + 1];
    }
  }

  if (!roots.empty()) {
    fake_root->first_child = roots[0];
  }

  return fake_root;
}

}  // namespace imp::editor
