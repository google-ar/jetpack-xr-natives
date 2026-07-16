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
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/ascii.h"
#include "absl/strings/match.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/misc/cpp/imgui_stdlib.h"
#include "third_party/imgui_node_editor/imgui_node_editor.h"
#include "core/async/future.h"
#include "core/common/robin_map.h"
#include "core/editor/editor.h"
#include "core/editor/editor_proto_visitor.h"
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
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/recipe_runner.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::editor {

namespace {

using NodePlacements = recipe_internal::SimpleConnectionNodePlacements;

constexpr float kWindowHeightPct = 0.5f;
constexpr int kLeftPanelWidth = 300;
// The multiplier for the zoom power of the node editor. The larger the number,
// the larger the jumps for each zoom level.
constexpr float kWindowZoomPower = 1.02f;
constexpr int kRightMouseButtonIndex = 1;
constexpr char kCreateNodePopupName[] = "selected_popup";

}  // namespace

namespace {
constexpr absl::string_view kAssignmentStatementName = "Assignment Statement";
constexpr absl::string_view kAsyncCallStatementName = "Async Call Statement";
constexpr absl::string_view kBinaryExpressionName = "Binary Expression";
constexpr absl::string_view kBranchStatementName = "Branch Statement";
constexpr absl::string_view kCallExpressionName = "Call Expression";
constexpr absl::string_view kCallStatementName = "Call Statement";
constexpr absl::string_view kCustomStatementName = "Custom Statement";
constexpr absl::string_view kEventTriggerName = "Event Trigger";
constexpr absl::string_view kIdentifierName = "Identifier";
constexpr absl::string_view kLoopStatementName = "Loop Statement";
constexpr absl::string_view kSequenceStatementName = "Sequence Statement";
constexpr absl::string_view kSwitchStatementName = "Switch Statement";
constexpr absl::string_view kUnaryExpressionName = "Unary Expression";
constexpr absl::string_view kVariableDeclarationStatementName =
    "Variable Declaration Statement";
constexpr absl::string_view kWhileStatementName = "While Statement";
constexpr absl::string_view kEventName = "Event";

// Helper to update a vector of value connection arguments by index.
bool UpdateValueArgs(std::vector<ValueConnection>& args,
                     const SocketConnection& new_connection,
                     absl::string_view end_socket_name) {
  absl::string_view arg_prefix = "Arg";
  if (absl::StartsWith(end_socket_name, arg_prefix)) {
    int arg_index;
    if (absl::SimpleAtoi(
            absl::string_view(end_socket_name).substr(arg_prefix.length()),
            &arg_index)) {
      if (args.size() <= arg_index) {
        args.resize(arg_index + 1);
      }
      args[arg_index] = {.connection = new_connection};
      return true;
    }
  }
  return false;
}

}  // namespace

RecipeEditor::RecipeEditor(BaseView& view) : view_(view) {
  ax::NodeEditor::Config config;
  config.EnableSmoothZoom = true;
  config.SmoothZoomPower = kWindowZoomPower;

  context_ = ax::NodeEditor::CreateEditor(&config);

  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  editor.GetDispatcher().Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        // We only support single selection for the recipe editor.
        NodeHandle selected_node =
            view_.GetRegistry().Get<Editor>()->get().GetSingleSelectedNode();
        if (!selected_node.IsValid()) {
          recipe_runner_ = ComponentHandle<RecipeRunner>();
          ClearCurrentGraph();
          return;
        }

        recipe_runner_ = selected_node->GetComponent<RecipeRunner>();
        if (recipe_runner_) {
          graph_loading_future_ = Future<absl::Status>(LoadRecipeEditorGraph());
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

  ax::NodeEditor::SetCurrentEditor(context_);

  ImGui::BeginGroup();
  DrawSearchBar();
  DrawRecipeVariables();
  DrawNodeInspector();
  ImGui::EndGroup();

  ImGui::SameLine();

  // We're using IsBackgroundDoubleClicked since this is the only way to detect
  // a right click within the editor window in NodeEditor.
  // TODO: (broken link) - Have right click occur on single click
  if (ImGui::IsMouseDown(kRightMouseButtonIndex) &&
      ax::NodeEditor::IsBackgroundDoubleClicked() && !create_node_popup_open_) {
    ImGui::OpenPopup(kCreateNodePopupName);
    create_node_popup_open_ = true;
  }
  if (create_node_popup_open_) {
    DrawCreateNodePopup();
  }
  ax::NodeEditor::Begin("Recipe Editor",
                        ImVec2(-1, view_.GetSize().y * kWindowHeightPct));

  if (ImGui::IsKeyDown(ImGuiKey_Backspace)) {
    int node_count = ax::NodeEditor::GetSelectedNodes(nullptr, 0);
    if (node_count > 0) {
      std::vector<ax::NodeEditor::NodeId> node_ids(node_count);
      ax::NodeEditor::GetSelectedNodes(node_ids.data(), node_count);
      // Convert to RecipeEditorGraph::NodeId
      std::vector<RecipeEditorGraph::NodeId> editor_node_ids;
      editor_node_ids.reserve(node_ids.size());
      for (const auto& id : node_ids) {
        editor_node_ids.push_back(ConvertId(id));
      }
      // We loop since we can select multiple nodes at once, and we want to
      // delete all of them.
      for (const auto& node_id : editor_node_ids) {
        DeleteNode(node_id);
      }
    }
  }
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

absl::Status RecipeEditor::LoadRecipeEditorGraph() {
  ClearCurrentGraph();

  RecipeGraph& recipe_graph =
      RecipeRunner::IsfInfo::GetState(recipe_runner_).graph;

  MP_ASSIGN_OR_RETURN(graph_, RecipeEditorGraph::Create(recipe_graph));

  recipe_runner_->GetRuntimeGraph().SetSocketValueListener(
      [editor_graph = graph_.get()](const imp::NodeId& recipe_node_id,
                                    const recipe::Variables& return_values) {
        auto it = editor_graph->node_lookup.find(recipe_node_id);
        if (it == editor_graph->node_lookup.end()) {
          IMP_LOG(imp::WARNING) << "Recipe node not found in graph: "
                       << recipe_node_id.index;
        } else {
          editor_graph->nodes[it->second].return_values = return_values;
        }
      });

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
          // Accessing the NodeID and Socket Id of the pins so we can
          // Updating the node connections within the recipe_runner

          RecipeEditorGraph::PinId start_pin_id = ConvertId(inputPinId);
          RecipeEditorGraph::PinId end_pin_id = ConvertId(outputPinId);

          RecipeEditorGraph::NodeId start_node_id =
              RecipeEditorGraph::NodeId(-1);
          RecipeEditorGraph::NodeId end_node_id = RecipeEditorGraph::NodeId(-1);
          absl::string_view start_socket_name;
          absl::string_view end_socket_name;

          for (auto const& [node_id, node] : graph_->nodes) {
            for (auto const& [socket_name, socket] : node.sockets) {
              if (socket.id == start_pin_id) {
                start_node_id = node_id;
                start_socket_name = socket_name;
                break;
              }
            }
            for (auto const& [socket_name, socket] : node.sockets) {
              if (socket.id == end_pin_id) {
                end_node_id = node_id;
                end_socket_name = socket_name;
                break;
              }
            }
          }

          if (start_node_id != RecipeEditorGraph::NodeId(-1) &&
              end_node_id != RecipeEditorGraph::NodeId(-1)) {
            // Now we have the node IDs: start_node_id and end_node_id
            // And the sockets: start_socket and end_socket
            absl::Status status = UpdateNodeConnections(
                start_node_id, end_node_id, start_socket_name, end_socket_name);
            if (!status.ok()) {
              IMP_LOG(imp::ERROR) << "Failed to update node connections: " << status;
            } else {
              // If the recipe node is edited, we need to reload the recipe
              // editor graph to reflect the changes.
              (void)LoadRecipeEditorGraph();
            }
          }

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

absl::Status RecipeEditor::UpdateNodeConnections(
    RecipeEditorGraph::NodeId start_node_id,
    RecipeEditorGraph::NodeId end_node_id, absl::string_view start_socket_name,
    absl::string_view end_socket_name) {
  RecipeGraph& recipe_graph =
      RecipeRunner::IsfInfo::GetState(recipe_runner_).graph;

  auto start_socket_it =
      graph_->nodes[start_node_id].sockets.find(start_socket_name);
  if (start_socket_it == graph_->nodes[start_node_id].sockets.end()) {
    return absl::NotFoundError("Start socket not found.");
  }
  auto end_socket_it = graph_->nodes[end_node_id].sockets.find(end_socket_name);
  if (end_socket_it == graph_->nodes[end_node_id].sockets.end()) {
    return absl::NotFoundError("End socket not found.");
  }
  RecipeNode* start_node = &recipe_graph.recipe_nodes[start_node_id];
  RecipeNode* end_node = &recipe_graph.recipe_nodes[end_node_id];

  bool connection_updated = false;
  bool is_flow_connection = false;

  // Handle Value Connections
  // TODO: (broken link) - Add a more robust way to properly handle adding/editing
  // sockets.
  if (start_socket_it->second.type ==
          RecipeEditorGraph::ConnectionType::Value &&
      end_socket_it->second.type == RecipeEditorGraph::ConnectionType::Value) {
    SocketConnection new_connection = {.node_id = start_node->id,
                                       .socket_name = "out"};

    if (auto* end_exec_node = std::get_if<ExecutableNode>(&end_node->node)) {
      if (auto* call_statement =
              std::get_if<CallStatement>(&end_exec_node->statement)) {
        connection_updated = UpdateValueArgs(call_statement->expression.args,
                                             new_connection, end_socket_name);
      }
    } else if (auto* end_value_node = std::get_if<ValueNode>(&end_node->node)) {
      if (auto* end_binary_expr =
              std::get_if<BinaryExpression>(&end_value_node->value)) {
        if (absl::StartsWith(end_socket_name, "Left")) {
          end_binary_expr->left = {.connection = new_connection};
          connection_updated = true;
        } else if (absl::StartsWith(end_socket_name, "Right")) {
          end_binary_expr->right = {.connection = new_connection};
          connection_updated = true;
        } else {
          return absl::InvalidArgumentError(
              "Unknown end socket name for BinaryExpression.");
        }
      } else if (auto* end_call_expr =
                     std::get_if<CallExpression>(&end_value_node->value)) {
        connection_updated = UpdateValueArgs(end_call_expr->args,
                                             new_connection, end_socket_name);
      } else if (auto* end_unary_expr =
                     std::get_if<UnaryExpression>(&end_value_node->value)) {
        if (end_socket_name == "Value") {
          end_unary_expr->input = {.connection = new_connection};
          connection_updated = true;
        }
      }
    }
  } else if (start_socket_it->second.type ==
                 RecipeEditorGraph::ConnectionType::Flow &&
             end_socket_it->second.type ==
                 RecipeEditorGraph::ConnectionType::Flow) {
    // Handle Flow Connections
    is_flow_connection = true;
    ExecutableNodeConnection new_connection = {.node_id = end_node->id};
    if (auto* start_exec_node =
            std::get_if<ExecutableNode>(&start_node->node)) {
      if (auto* call_statement =
              std::get_if<CallStatement>(&start_exec_node->statement)) {
        if (start_socket_name == "Next") {
          call_statement->next_node = new_connection;
          connection_updated = true;
        }
      }
    } else if (auto* start_event_node =
                   std::get_if<EventNode>(&start_node->node)) {
      if (start_socket_name == "Next") {
        start_event_node->next_node = new_connection;
        connection_updated = true;
      }
    }
  }

  if (connection_updated) {
    if (is_flow_connection) {
      // Flow connections update the starting node.
      recipe_graph.recipe_nodes[start_node_id] = *start_node;
    } else {
      // Value connections update the ending node.
      recipe_graph.recipe_nodes[end_node_id] = *end_node;
    }
    RecipeRunner::IsfInfo::GetState(recipe_runner_).graph = recipe_graph;
  }

  return absl::OkStatus();
}
void RecipeEditor::DrawRecipeVariables() {
  if (!graph_) {
    return;
  }

  ImGui::BeginGroup();
  ImGui::Text("Recipe Variables");
  if (ImGui::BeginListBox(
          "##recipe-variables",
          ImVec2(kLeftPanelWidth, view_.GetSize().y * kWindowHeightPct))) {
    if (recipe_runner_) {
      RecipeScope& scope = recipe_runner_->GetScope();
      StringMap<recipe::Variable> variables = scope.GetVariables();
      for (auto& [name, variable] : variables) {
        ImGui::Text(
            "%s", absl::StrCat(name, ": ", recipe::ToString(variable)).c_str());
      }
    }
    ImGui::EndListBox();
  }

  ImGui::EndGroup();
}

void RecipeEditor::DrawSearchBar() {
  ImGui::PushItemWidth(kLeftPanelWidth);
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
  ImGui::PopItemWidth();

  ImGui::Text("%zu of %zu",
              search_results_.empty() ? 0 : search_result_index_ + 1,
              search_results_.size());

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

void RecipeEditor::DrawCreateNodePopup() {
  if (ImGui::BeginPopup(kCreateNodePopupName)) {
    ImGui::InputText("Search", &create_node_search_buffer_);
    ImGui::Separator();
    const char* node_names[] = {kAssignmentStatementName.data(),
                                kAsyncCallStatementName.data(),
                                kBinaryExpressionName.data(),
                                kBranchStatementName.data(),
                                kCallExpressionName.data(),
                                kCallStatementName.data(),
                                kCustomStatementName.data(),
                                kEventName.data(),
                                kEventTriggerName.data(),
                                kIdentifierName.data(),
                                kLoopStatementName.data(),
                                kSequenceStatementName.data(),
                                kSwitchStatementName.data(),
                                kUnaryExpressionName.data(),
                                kVariableDeclarationStatementName.data(),
                                kWhileStatementName.data()};
    ImGui::SeparatorText("Available Nodes");
    // Convert the search buffer to lower case for case-insensitive matching.
    std::string lower_case_buffer =
        absl::AsciiStrToLower(create_node_search_buffer_);

    // Returns true if the item name matches the search buffer.
    auto item_matches = [&](absl::string_view item_name) {
      return create_node_search_buffer_.empty() ||
             absl::StrContains(absl::AsciiStrToLower(item_name),
                               lower_case_buffer);
    };

    for (const char* node_name : node_names) {
      bool show_menu = item_matches(node_name);

      // If the main item doesn't match, check its sub-nodes.
      // TODO: (broken link) - Current implementation is not scalable and will
      // require refactoring. A good approach would be to have a method that
      // recursively digs into every field in a protobuf and check if the string
      // mattaches anything in the field, and draw the menu item if it does.
      if (!show_menu && !create_node_search_buffer_.empty()) {
        if (strcmp(node_name, kAssignmentStatementName.data()) == 0) {
          for (const auto op : proto::EnumMetaData<
                   AssignmentStatement::AssignmentOps>::kValues) {
            if (item_matches(
                    proto::EnumMetaData<
                        AssignmentStatement::AssignmentOps>::GetName(op))) {
              show_menu = true;
              break;
            }
          }
        } else if (strcmp(node_name, kBinaryExpressionName.data()) == 0) {
          for (const auto op :
               proto::EnumMetaData<BinaryExpression::BinaryOps>::kValues) {
            if (item_matches(
                    proto::EnumMetaData<BinaryExpression::BinaryOps>::GetName(
                        op))) {
              show_menu = true;
              break;
            }
          }
        } else if (strcmp(node_name, kCallExpressionName.data()) == 0) {
          if (item_matches("GetLocalPosition") ||
              item_matches(kCallExpressionName.data())) {
            show_menu = true;
          }
        } else if (strcmp(node_name, kCallStatementName.data()) == 0) {
          if (item_matches("SetLocalPosition") ||
              item_matches(kCallStatementName.data())) {
            show_menu = true;
          }
        } else if (strcmp(node_name, kEventName.data()) == 0) {
          for (const absl::string_view event_name : recipe::kEventNames) {
            if (item_matches(event_name)) {
              show_menu = true;
              break;
            }
          }
        } else if (strcmp(node_name, kIdentifierName.data()) == 0) {
          if (item_matches("NodeSelf")) {
            show_menu = true;
          }
        } else if (strcmp(node_name, kUnaryExpressionName.data()) == 0) {
          for (const auto op :
               proto::EnumMetaData<UnaryExpression::UnaryOps>::kValues) {
            if (item_matches(
                    proto::EnumMetaData<UnaryExpression::UnaryOps>::GetName(
                        op))) {
              show_menu = true;
              break;
            }
          }
        }
      }

      // Display the popup menu if the main item matches or if its sub-nodes
      // match. If any item matches a search query, they will be displayed in
      // the menu. If the search query is empty, all nodes will be displayed.
      if (show_menu) {
        if (ImGui::BeginMenu(node_name)) {
          if (strcmp(node_name, kAssignmentStatementName.data()) == 0) {
            auto assignment_ops = proto::EnumMetaData<
                AssignmentStatement::AssignmentOps>::kValues;
            for (const AssignmentStatement::AssignmentOps assignment_op :
                 assignment_ops) {
              const std::string op_name = std::string(
                  proto::EnumMetaData<AssignmentStatement::AssignmentOps>::
                      GetName(assignment_op));
              if (item_matches(op_name)) {
                if (ImGui::MenuItem(op_name.c_str())) {
                  CreateGenericExecutableNode(
                      kAssignmentStatementName.data(), *graph_,
                      AssignmentStatement{.op = assignment_op});
                }
              }
            }
          }
          if (strcmp(node_name, kAsyncCallStatementName.data()) == 0) {
            if (item_matches(kAsyncCallStatementName.data())) {
              if (ImGui::MenuItem(kAsyncCallStatementName.data())) {
                CreateGenericExecutableNode(kAsyncCallStatementName.data(),
                                            *graph_, AsyncCallStatement{});
              }
            }
          }
          if (strcmp(node_name, kBinaryExpressionName.data()) == 0) {
            for (const BinaryExpression::BinaryOps binary_op :
                 proto::EnumMetaData<BinaryExpression::BinaryOps>::kValues) {
              const std::string op_name = std::string(
                  proto::EnumMetaData<BinaryExpression::BinaryOps>::GetName(
                      binary_op));
              if (item_matches(op_name)) {
                if (ImGui::MenuItem(op_name.c_str())) {
                  CreateBinaryExpressionNode(binary_op, *graph_);
                }
              }
            }
          }
          if (strcmp(node_name, kBranchStatementName.data()) == 0) {
            if (item_matches(kBranchStatementName.data())) {
              if (ImGui::MenuItem(kBranchStatementName.data())) {
                CreateGenericExecutableNode(kBranchStatementName.data(),
                                            *graph_, BranchStatement{});
              }
            }
          }
          if (strcmp(node_name, kCallExpressionName.data()) == 0) {
            if (item_matches("GetLocalPosition")) {
              if (ImGui::MenuItem("GetLocalPosition")) {
                CreateGenericValueNode(
                    "GetLocalPosition", *graph_,
                    CallExpression{.name = "GetLocalPosition", .args = {}});
              }
            }
            if (item_matches("GetLocalRotationVec4")) {
              if (ImGui::MenuItem("GetLocalRotationVec4")) {
                CreateGenericValueNode(
                    "GetLocalRotationVec4", *graph_,
                    CallExpression{.name = "GetLocalRotationVec4", .args = {}});
              }
            }
            if (item_matches(kCallExpressionName.data())) {
              if (ImGui::MenuItem(kCallExpressionName.data())) {
                CreateGenericValueNode(kCallExpressionName.data(), *graph_,
                                       CallExpression{});
              }
            }
          }
          if (strcmp(node_name, kCallStatementName.data()) == 0) {
            if (item_matches("SetLocalPosition")) {
              if (ImGui::MenuItem("SetLocalPosition")) {
                CreateGenericExecutableNode(
                    "SetLocalPosition", *graph_,
                    CallStatement{.expression = {.name = "SetLocalPosition",
                                                 .args = {}}});
              }
            }
            if (item_matches("SetLocalRotationVec4")) {
              if (ImGui::MenuItem("SetLocalRotationVec4")) {
                CreateGenericExecutableNode(
                    "SetLocalRotationVec4", *graph_,
                    CallStatement{.expression = {.name = "SetLocalRotationVec4",
                                                 .args = {}}});
              }
            }
            if (item_matches(kCallStatementName.data())) {
              if (ImGui::MenuItem(kCallStatementName.data())) {
                CreateGenericExecutableNode(kCallStatementName.data(), *graph_,
                                            CallStatement{});
              }
            }
          }
          if (strcmp(node_name, kCustomStatementName.data()) == 0) {
            if (item_matches(kCustomStatementName.data())) {
              if (ImGui::MenuItem(kCustomStatementName.data())) {
                CreateGenericExecutableNode(kCustomStatementName.data(),
                                            *graph_, CustomStatement{});
              }
            }
          }
          if (strcmp(node_name, kEventName.data()) == 0) {
            for (const absl::string_view event_name : recipe::kEventNames) {
              if (item_matches(event_name)) {
                if (ImGui::MenuItem(event_name.data())) {
                  CreateEventNode(event_name.data(), *graph_);
                }
              }
            }
          }
          if (strcmp(node_name, kEventTriggerName.data()) == 0) {
            if (item_matches(kEventTriggerName.data())) {
              if (ImGui::MenuItem(kEventTriggerName.data())) {
                CreateGenericExecutableNode(kEventTriggerName.data(), *graph_,
                                            EventTrigger{});
              }
            }
          }
          if (strcmp(node_name, kIdentifierName.data()) == 0) {
            if (item_matches("NodeSelf")) {
              if (ImGui::MenuItem("NodeSelf")) {
                CreateGenericValueNode("NodeSelf", *graph_,
                                       Identifier{.name = "NodeSelf"});
              }
            }
          }
          if (strcmp(node_name, kUnaryExpressionName.data()) == 0) {
            for (const UnaryExpression::UnaryOps unary_op :
                 proto::EnumMetaData<UnaryExpression::UnaryOps>::kValues) {
              const std::string op_name = std::string(
                  proto::EnumMetaData<UnaryExpression::UnaryOps>::GetName(
                      unary_op));
              if (item_matches(op_name)) {
                if (ImGui::MenuItem(op_name.c_str())) {
                  CreateGenericValueNode(kUnaryExpressionName.data(), *graph_,
                                         UnaryExpression{.op = unary_op});
                }
              }
            }
          }
          ImGui::EndMenu();
        }
      }
    }
    ImGui::EndPopup();
  } else {
    create_node_popup_open_ = false;
  }
}

void RecipeEditor::DeleteNode(RecipeEditorGraph::NodeId node_id) {
  RecipeGraph recipe_graph = recipe_runner_->GetRuntimeGraph().GetRecipeGraph();
  recipe_graph.recipe_nodes.erase(
      std::remove_if(recipe_graph.recipe_nodes.begin(),
                     recipe_graph.recipe_nodes.end(),
                     [node_id](const RecipeNode& node) {
                       return node.id.index == node_id;
                     }),
      recipe_graph.recipe_nodes.end());
  RecipeRunner::IsfInfo::GetState(recipe_runner_).graph = recipe_graph;
  if (graph_loading_future_.Ready()) {
    graph_loading_future_ = Future<absl::Status>::Schedule(
        [this, &graph = recipe_runner_->GetRuntimeGraph()]() {
          return LoadRecipeEditorGraph();
        });
  } else {
    graph_loading_future_ = graph_loading_future_.Then(
        [this, &graph = recipe_runner_->GetRuntimeGraph()]() {
          return LoadRecipeEditorGraph();
        });
  }
}

void RecipeEditor::UpdateRecipeGraph(const RecipeNode& new_node) {
  RecipeGraph& recipe_graph =
      RecipeRunner::IsfInfo::GetState(recipe_runner_).graph;
  recipe_graph.recipe_nodes.push_back(new_node);

  if (graph_loading_future_.Ready()) {
    graph_loading_future_ = Future<absl::Status>(LoadRecipeEditorGraph());
  } else {
    graph_loading_future_ = graph_loading_future_.Then(
        [this]() { return LoadRecipeEditorGraph(); });
  }
}

template <typename T>
void RecipeEditor::CreateGenericValueNode(const char* name,
                                          imp::editor::RecipeEditorGraph& graph,
                                          T expression) {
  RecipeEditorGraph::NodeId nextId = graph.nodes.size();
  std::string node_name = absl::StrFormat("%s [Recipe node id %d]",
                                          absl::AsciiStrToLower(name), nextId);
  const RecipeNode new_node = RecipeNode{
      .id = {.index = static_cast<uint32_t>(nextId)},
      .name = node_name,
      .node = ValueNode{.value = expression},
  };
  UpdateRecipeGraph(new_node);
}
template <typename T>
void RecipeEditor::CreateGenericExecutableNode(
    const char* name, imp::editor::RecipeEditorGraph& graph, T statement) {
  RecipeEditorGraph::NodeId nextId = graph.nodes.size();
  std::string node_name = absl::StrFormat("%s [Recipe node id %d]",
                                          absl::AsciiStrToLower(name), nextId);
  const RecipeNode new_node = RecipeNode{
      .id = {.index = static_cast<uint32_t>(nextId)},
      .name = node_name,
      .node = ExecutableNode{.statement = statement},
  };
  UpdateRecipeGraph(new_node);
}

void RecipeEditor::CreateEventNode(const char* name,
                                   imp::editor::RecipeEditorGraph& graph) {
  RecipeEditorGraph::NodeId nextId = graph.nodes.size();
  std::string node_name = absl::StrFormat("%s [Recipe node id %d]",
                                          absl::AsciiStrToLower(name), nextId);
  const RecipeNode new_node = RecipeNode{
      .id = {.index = static_cast<uint32_t>(nextId)},
      .name = node_name,
      .node = EventNode{.event_name = name},
  };
  UpdateRecipeGraph(new_node);
}
void RecipeEditor::CreateBinaryExpressionNode(
    const BinaryExpression::BinaryOps& binary_op,
    imp::editor::RecipeEditorGraph& graph) {
  // TODO: (broken link) -  Have the values be editable by either user selection
  // or connection to an existing node.
  int default_left_value = 0;
  int default_right_value = 0;
  RecipeEditorGraph::NodeId nextId = graph.nodes.size();
  std::string ExpressionName =
      "math/" +
      absl::AsciiStrToLower(
          std::string(proto::EnumMetaData<BinaryExpression::BinaryOps>::GetName(
              binary_op))) +
      absl::StrFormat(" [Recipe node id %d]", nextId);
  const RecipeNode new_node = RecipeNode{
      .id = {.index = static_cast<uint32_t>(nextId)},
      .name = ExpressionName,
      .node =
          imp::ValueNode{
              .value =
                  BinaryExpression{
                      .op = binary_op,
                      .left = {.connection = imp::Literal(default_left_value)},
                      .right = {.connection =
                                    imp::Literal(default_right_value)}}},
  };
  UpdateRecipeGraph(new_node);
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
  ImGui::Text("%s", node.type_name.c_str());
  ImGui::Text("%s", node.name.c_str());
  ImGui::Text("%s", node.content.c_str());
  for (auto& [name, value] : node.return_values) {
    ImGui::Text("%s",
                absl::StrCat(name, ": ", recipe::ToString(value)).c_str());
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
    ImGui::Text("%s", socket.name.c_str());
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
    ImGui::Text("%s", socket.name.c_str());
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

void RecipeEditor::DrawNodeInspector() {
  ImGui::PushItemWidth(kLeftPanelWidth);
  ImGui::BeginGroup();
  ImGui::Text("Node Inspector");
  if (ax::NodeEditor::GetSelectedObjectCount() > 0) {
    ax::NodeEditor::NodeId selected_node;

    if (ax::NodeEditor::GetSelectedNodes(&selected_node, 1)) {
      RecipeEditorGraph::NodeId editor_graph_node_id = ConvertId(selected_node);
      auto it = graph_->nodes.find(editor_graph_node_id);
      if (it != graph_->nodes.end()) {
        RecipeNode* recipe_node = it->second.recipe_node;
        if (recipe_node) {
          EditorProtoVisitor<RecipeNode> visitor(*recipe_node);
          recipe_node->Visit(visitor, 0, nullptr);
          if (visitor.AnyFieldEdited()) {
            // If the recipe node is edited, we need to reload the recipe
            // editor graph to reflect the changes.
            (void)LoadRecipeEditorGraph();
          }
        }
      }
    }
  }
  ImGui::EndGroup();
  ImGui::PopItemWidth();
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
