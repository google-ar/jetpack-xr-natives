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

#include "core/editor/widgets/recipes/recipe_editor.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/ascii.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/misc/cpp/imgui_stdlib.h"
#include "third_party/imgui_node_editor/imgui_node_editor.h"
#include "core/async/future.h"
#include "core/common/robin_map.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/editor/widgets/recipes/editor_constants.h"
#include "core/editor/widgets/recipes/editor_helpers.h"
#include "core/editor/widgets/recipes/recipe_editor_graph.h"
#include "core/editor/widgets/recipes/simple_connection_node_placements.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_graph.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/recipe_runner.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::editor {

namespace {

using NodePlacements = recipe_internal::SimpleConnectionNodePlacements;

constexpr float kWindowHeightPct = 0.5f;
constexpr int kRecipeVariablesWindowWidth = 450;

}  // namespace

RecipeEditor::RecipeEditor(BaseView& view) : view_(view) {
  ax::NodeEditor::Config config;
  config.EnableSmoothZoom = true;

  context_ = ax::NodeEditor::CreateEditor(&config);

  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  editor.GetDispatcher().Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        if (!event.selected) {
          recipe_runner_ = ComponentHandle<RecipeRunner>();
          ClearCurrentGraph();
          return;
        }

        recipe_runner_ = event.selected->GetComponent<RecipeRunner>();
        if (recipe_runner_) {
          graph_loading_future_ = Future<absl::Status>::Schedule(
              [this, &graph = recipe_runner_->GetRuntimeGraph()]() {
                return LoadRecipeGraph(graph);
              });
        } else {
          ClearCurrentGraph();
        }
      },
      this);
}

RecipeEditor::~RecipeEditor() { ax::NodeEditor::DestroyEditor(context_); }

bool RecipeEditor::HasContent() const { return graph_ != nullptr; }

void RecipeEditor::DrawImGui() {
  if (!graph_) {
    return;
  }

  ImGui::BeginGroup();
  DrawSearchBar();
  DrawRecipeVariables();
  ImGui::EndGroup();

  ImGui::SameLine();

  ax::NodeEditor::SetCurrentEditor(context_);

  ax::NodeEditor::Begin("Recipe Editor",
                        ImVec2(-1, view_.GetSize().y * kWindowHeightPct));

  CheckForGraphUpdates();
  DrawGraph();

  ax::NodeEditor::End();

  if (first_frame_) {
    first_frame_ = false;
    ax::NodeEditor::NavigateToContent(0.0f);
  }

  if (search_selection_changed_ && !search_results_.empty()) {
    ax::NodeEditor::SelectNode(
        ax::NodeEditor::NodeId(search_results_[search_result_index_]));
    ax::NodeEditor::NavigateToSelection();
    search_selection_changed_ = false;
  }

  ax::NodeEditor::SetCurrentEditor(nullptr);
}

absl::Status RecipeEditor::LoadRecipeGraph(RecipeRuntimeGraph& graph) {
  ClearCurrentGraph();

  MP_ASSIGN_OR_RETURN(graph_, RecipeEditorGraph::Create(graph));

  return absl::OkStatus();
}

void RecipeEditor::ClearCurrentGraph() { graph_.reset(); }

void RecipeEditor::CheckForGraphUpdates() {
  if (!graph_) {
    return;
  }

  // Check if the user clicked the gui to create a node, pin, or link
  if (ax::NodeEditor::BeginCreate()) {
    // TODO: (broken link) - handle creating new nodes
    ax::NodeEditor::PinId inputPinId, outputPinId;
    if (ax::NodeEditor::QueryNewLink(&inputPinId, &outputPinId)) {
      if (inputPinId && outputPinId) {
        if (ax::NodeEditor::AcceptNewItem()) {
          // TODO: (broken link) - determine correct connection type
          RecipeEditorGraph::LinkId nextId = graph_->links.size();
          graph_->links.push_back(RecipeEditorGraph::Link{
              .id = nextId,
              .start_socket_id = ConvertId(inputPinId),
              .end_socket_id = ConvertId(outputPinId),
              .type = RecipeEditorGraph::ConnectionType::Flow});
        }

        // TODO: (broken link) - reject illegal connection between nodes
        // use ax::NodeEditor::RejectNewItem() for visual feedback
      }
    }
  }
  ax::NodeEditor::EndCreate();
}

void RecipeEditor::DrawRecipeVariables() {
  if (!graph_) {
    return;
  }

  ImGui::BeginGroup();
  ImGui::Text("Recipe Variables");
  if (ImGui::BeginListBox("##recipe-variables",
                          ImVec2(kRecipeVariablesWindowWidth,
                                 view_.GetSize().y * kWindowHeightPct))) {
    if (recipe_runner_) {
      RecipeScope& scope = recipe_runner_->GetScope();
      StringMap<recipe::Variable> variables = scope.GetVariables();
      for (auto& [name, variable] : variables) {
        ImGui::Text(
            absl::StrCat(name, ": ", recipe::ToString(variable)).c_str());
      }
    }
    ImGui::EndListBox();
  }

  ImGui::EndGroup();
}

void RecipeEditor::DrawSearchBar() {
  if (ImGui::InputText("Search", &search_buffer_, 16)) {
    // Update search results.
    std::string lower_case_buffer = absl::AsciiStrToLower(search_buffer_);
    search_results_.clear();
    for (auto& [id, node] : graph_->nodes) {
      if (absl::StrContains(absl::AsciiStrToLower(node.name),
                            lower_case_buffer) ||
          absl::StrContains(absl::AsciiStrToLower(node.type_name),
                            lower_case_buffer) ||
          absl::StrContains(absl::AsciiStrToLower(node.content),
                            lower_case_buffer)) {
        search_results_.push_back(node.id);
      }
    }
    search_result_index_ = 0;
    search_selection_changed_ = true;
  }

  ImGui::Text(
      absl::StrFormat("%u of %u",
                      search_results_.empty() ? 0 : search_result_index_ + 1,
                      search_results_.size())
          .c_str());

  ImGui::SameLine();
  if (ImGui::Button("<")) {
    --search_result_index_;
    search_selection_changed_ = true;
  }
  ImGui::SameLine();
  if (ImGui::Button(">")) {
    ++search_result_index_;
    search_selection_changed_ = true;
  }

  search_result_index_ =
      std::clamp(search_result_index_, size_t(0), search_results_.size() - 1);
}

void RecipeEditor::DrawGraph() {
  if (!graph_) {
    return;
  }

  for (auto& [_, node] : graph_->nodes) {
    DrawNode(node);
  }

  if (first_frame_) {
    // TODO : Place nodes after their sizes stop changing.
    RobinMap<int, float2> placements =
        NodePlacements().GeneratePlacements(*graph_);

    for (auto& [id, pos] : placements) {
      ax::NodeEditor::SetNodePosition(ax::NodeEditor::NodeId(id),
                                      ImVec2(pos.x, pos.y));
    }
  }

  for (auto& link : graph_->links) {
    DrawLink(link);
  }
}

void RecipeEditor::DrawNode(const RecipeEditorGraph::Node& node) {
  ax::NodeEditor::NodeId node_id = ax::NodeEditor::NodeId(node.id);
  ax::NodeEditor::BeginNode(node_id);

  ImGui::BeginGroup();
  ImGui::Text(node.type_name.c_str());
  ImGui::Text(node.name.c_str());
  ImGui::Text(node.content.c_str());
  for (auto& [name, value] : node.return_values) {
    ImGui::Text(absl::StrCat(name, ": ", recipe::ToString(value)).c_str());
  }
  ImGui::EndGroup();

  ImGui::BeginGroup();
  for (const auto& [key, socket] : node.sockets) {
    if (socket.kind != RecipeEditorGraph::SocketKind::Input) {
      continue;
    }
    ax::NodeEditor::BeginPin(ax::NodeEditor::PinId(socket.id),
                             ax::NodeEditor::PinKind::Input);
    DrawSocket(socket, /*connected=*/true);
    ImGui::SameLine();
    ImGui::Text(socket.name.c_str());
    ax::NodeEditor::EndPin();
  }
  ImGui::EndGroup();

  ImGui::SameLine(ax::NodeEditor::GetNodeSize(node_id).y);

  ImGui::BeginGroup();
  for (const auto& [key, socket] : node.sockets) {
    if (socket.kind != RecipeEditorGraph::SocketKind::Output) {
      continue;
    }
    ImGui::BeginGroup();
    ax::NodeEditor::BeginPin(ax::NodeEditor::PinId(socket.id),
                             ax::NodeEditor::PinKind::Output);
    ax::NodeEditor::PinPivotAlignment(ImVec2(1.0f, 0.5f));
    ax::NodeEditor::PinPivotSize(ImVec2(0, 0));
    ImGui::Text(socket.name.c_str());
    ImGui::SameLine();
    // TODO Check if sockets are connected.
    DrawSocket(socket, /*connected=*/true);
    ax::NodeEditor::EndPin();
    ImGui::EndGroup();
  }
  ImGui::EndGroup();

  ax::NodeEditor::EndNode();
}

void RecipeEditor::DrawLink(const RecipeEditorGraph::Link& link) {
  ImColor color;
  switch (link.type) {
    case RecipeEditorGraph::ConnectionType::Flow:
      color = kRecipeFlowConnectionColor;
      break;
    case RecipeEditorGraph::ConnectionType::Value:
      color = kRecipeValueConnectionColor;
      break;
    default:
      IMP_LOG(imp::FATAL) << "Unknown link type in DrawLink";
      return;
  }

  ax::NodeEditor::Link(ax::NodeEditor::LinkId(link.id),
                       ax::NodeEditor::PinId(link.start_socket_id),
                       ax::NodeEditor::PinId(link.end_socket_id), color);
}

void RecipeEditor::DrawSocket(const RecipeEditorGraph::Socket& socket,
                              bool connected) {
  RecipeIconType icon_type;
  ImColor color;
  switch (socket.type) {
    case RecipeEditorGraph::ConnectionType::Flow:
      icon_type = RecipeIconType::Flow;
      color = kRecipeFlowConnectionColor;
      break;
    case RecipeEditorGraph::ConnectionType::Value:
      icon_type = RecipeIconType::Circle;
      color = kRecipeValueConnectionColor;
      break;
    default:
      return;
  }

  ImVec2 pin_icon_size = ImVec2(24, 24);
  DrawRecipeIcon(pin_icon_size, icon_type, connected, color,
                 ImColor(32, 32, 32, 125));
}

RecipeEditorGraph::NodeId RecipeEditor::ConvertId(ax::NodeEditor::NodeId id) {
  return RecipeEditorGraph::NodeId(id.Get());
}

RecipeEditorGraph::PinId RecipeEditor::ConvertId(ax::NodeEditor::PinId id) {
  return RecipeEditorGraph::PinId(id.Get());
}

RecipeEditorGraph::LinkId RecipeEditor::ConvertId(ax::NodeEditor::LinkId id) {
  return RecipeEditorGraph::LinkId(id.Get());
}

}  // namespace imp::editor
