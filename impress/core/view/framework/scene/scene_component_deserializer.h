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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_SCENE_COMPONENT_DESERIALIZER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_SCENE_COMPONENT_DESERIALIZER_H_

#include <string>
#include <type_traits>
#include <vector>

#include "absl/container/btree_map.h"
#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/async/future.h"
#include "core/common/bit_flag.h"
#include "core/common/hash.h"
#include "core/common/robin_map.h"
#include "core/config.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_manager.h"
#include "core/ncsb/component_traits.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/parse_message_visitor.h"
#include "core/proto/textproto_reader.h"
#include "core/proto/textproto_writer.h"
#if IMP_RUNTIME(DEV)
#include "absl/status/statusor.h"
#include "dear_imgui/imgui.h"
#include "core/async/executor.h"
#include "core/editor/command_manager.h"
#include "core/editor/editor_proto_visitor.h"
#include "core/editor/node_value_command.h"
#include "core/editor/ui/filterable_combo.h"
#include "core/editor/widgets/component_editor_helpers.h"
#include "core/editor/widgets/isf_component_widget.h"
#include "core/editor/widgets/stateless_isf_component_widget.h"
#include "core/ncsb/scene_metadata.h"
#include "mediapipe/framework/port/status_macros.h"
#endif

namespace imp {

// Helper class used by SceneSystem to help handle the deserialization of
// components within a .isf file.
//
// This class is used to register a component's IsfInfo in a type-erased
// manner so that it can be used by SceneSystem to add and setup components.
class SceneComponentDeserializer {
 public:
  using ParseFunc = bool(NodeHandle, ComponentManager*,
                         const std::vector<absl::string_view>&,
                         proto::ParseMessageVisitor*,
                         bool should_add_component);
  using SaveFunc = SaveIsfStateResult(utils::Entity, ComponentManager&,
                                      google::protobuf::imp_proto::Any&, bool&,
                                      absl::Span<const std::string>);
  using HasComponentFunc = bool(utils::Entity, ComponentManager*);
  using RemoveComponentFunc = void(utils::Entity, ComponentManager*);
  using VisitFunc = absl::optional<Future<absl::Status>>(
      NodeHandle, ComponentManager*, BaseStateVisitor*,
      bool should_enable_component);
  using DefaultSetupFunc = Future<absl::Status>(utils::Entity,
                                                ComponentManager*,
                                                bool should_enable_component);
  using OnIsfStateChangedFunc = Future<absl::Status>(utils::Entity,
                                                     ComponentManager*);
#if IMP_RUNTIME(DEV)
  using CreateComponentWidgetFunc = std::unique_ptr<editor::Widget>(
      NodeHandle node, Dispatcher& editor_dispatcher);
#endif

  // A struct of function pointers.  If the component type doesn't implement a
  // given function (i.e. async setup), that pointer will be null.
  struct Handler {
    // The proto type url of the State class that this info applies to.
    absl::string_view type_url;
    // The component id of the component that this info applies to.
    ComponentId component_id;
    // Function to create a new Component and parse a string proto into that
    // component's state_ field.
    ParseFunc* parse = nullptr;
    // Function to check if a component exists.
    HasComponentFunc* has_component = nullptr;
    // Function to remove a component.
    RemoveComponentFunc* remove_component = nullptr;
    // Function to save a component out to the value field of a protobuf::Any.
    SaveFunc* save = nullptr;
    // Function called before setting up the component to do any final work
    // via BaseStateVisitor. If BaseStateVisitor calls a Setup overload,
    // returns the result. Otherwise, returns nullopt.
    VisitFunc* visit = nullptr;
    // Function to fetch a component instance and call the default setup
    // method if there is one.
    DefaultSetupFunc* default_setup = nullptr;
    // Function to notify the component that its ISF state has changed.
    OnIsfStateChangedFunc* on_isf_state_changed = nullptr;

#if IMP_RUNTIME(DEV)
    std::string component_typename = "<unknown>";
    CreateComponentWidgetFunc* create_component_widget = nullptr;
#endif
  };

  SceneComponentDeserializer();

  // Registers a component's IsfInfo so that the component can be
  // deserialized.
  template <typename T, IsfOptionalFeature::Flags flags>
  void RegisterComponentIsfInfo();

#if IMP_RUNTIME(DEV)
  template <typename T>
  void RegisterComponentWidget(Handler& handler);
  template <typename T>
  void RegisterAddComponentUi();
#endif

  // Get type-erased handlers from registered component's IsfInfo by the
  // type_url of the IsfInfo.
  const Handler* GetHandler(HashValue type_hash) const;
  const absl::btree_map<HashValue, Handler>& GetHandlers() const;

  // Get the dependencies from registered comopnent's IsfInfo by the type_url
  // of the IsfInfo. Used to control the other that component's Setup is
  // called when they are deserialized from a .isf file.
  const absl::flat_hash_set<HashValue>& GetDeps(HashValue type_hash);

#if IMP_RUNTIME(DEV)
  // Displays a menu of all component types that can be added to the given
  // node.
  void ShowAddComponentUi(NodeHandle node);
  std::unique_ptr<editor::Widget> CreateComponentWidget(
      HashValue component_state_type_url_hash, NodeHandle node,
      Dispatcher& editor_dispatcher);
#endif

 private:
  absl::btree_map<HashValue, Handler> handlers_;
  RobinMap<HashValue, absl::flat_hash_set<HashValue>> deps_;
#if IMP_RUNTIME(DEV)
  editor::FilterableCombo add_component_ui_;
  NodeHandle selected_node_;
  editor::CommandManager* command_manager_;
#endif
};

template <typename T, IsfOptionalFeature::Flags flags>
void SceneComponentDeserializer::RegisterComponentIsfInfo() {
  static_assert(
      component_traits::kIsIsfInfoDefined<T>,
      "To declare that a component can be registered with the SceneSystem, "
      "it "
      "must include a using statement named IsfInfo for template IsfInfo or "
      "StatelessIsfInfo. See isf_info.h for more details.");
  using ComponentIsfInfo = typename T::IsfInfo;
  using ComponentStateT = typename ComponentIsfInfo::StateT;
  using ComponentDependenciesT = typename ComponentIsfInfo::DependenciesT;
  using ComponentDependentsT = typename ComponentIsfInfo::DependentsT;

  // If the component is already registered, return early.
  auto [itr, was_inserted] =
      handlers_.try_emplace(ComponentIsfInfo::kTypeUrlHash);
  Handler& handler = itr->second;

  // This is the first time this component is being registered.
  // Fill in all the required fields in the handler.
  if (was_inserted) {
    handler.type_url = ComponentIsfInfo::kTypeUrl;
    handler.component_id = GetComponentTypeId<T>();
    handler.parse = ComponentIsfInfo::Parse;

    handler.has_component = shared_isf_info_handlers::HasComponent<T>;
    handler.remove_component = shared_isf_info_handlers::RemoveComponent<T>;
    handler.visit = ComponentIsfInfo::Visit;
    handler.default_setup = shared_isf_info_handlers::Setup<T>;
    handler.on_isf_state_changed =
        shared_isf_info_handlers::OnIsfStateChanged<T>;

    absl::flat_hash_set<HashValue>& deps =
        deps_[ComponentIsfInfo::kTypeUrlHash];

    // Add the dependencies that are implicit from the fields in StateT.
    if constexpr (!std::is_void_v<ComponentStateT>) {
      absl::flat_hash_set<HashValue> visited_types;
      IsfStateDepsAdder<ComponentStateT>::AddDeps(deps, visited_types);
    }

    // Add the dependencies for the component if there are any.
    if constexpr (!std::is_void_v<ComponentDependenciesT>) {
      for (HashValue dep_type_hash : ComponentDependenciesT::kDependencies) {
        deps.insert(dep_type_hash);
      }
    }

    if constexpr (!std::is_void_v<ComponentDependentsT>) {
      for (HashValue dependent_type_hash : ComponentDependentsT::kDependents) {
        deps_[dependent_type_hash].insert(ComponentIsfInfo::kTypeUrlHash);
      }
    }

#if IMP_RUNTIME(DEV)
    RegisterComponentWidget<T>(handler);
    RegisterAddComponentUi<T>();
#endif
  }

  // Register optional features.

  // kAutomatic is zero, so compare directly instead of checking bit flags.
  constexpr bool kAutomaticFeaturesActive =
      flags == IsfOptionalFeature::kAutomatic && IMP_RUNTIME(DEV);

  if constexpr (kAutomaticFeaturesActive ||
                CheckBit(flags, IsfOptionalFeature::kSaving)) {
    handler.save = ComponentIsfInfo::Save;
  }

  if constexpr (!std::is_void_v<ComponentStateT> &&
                (kAutomaticFeaturesActive ||
                 CheckBit(flags, IsfOptionalFeature::kTextprotoLoading))) {
    proto::TextprotoReader::RegisterKnownType<ComponentStateT>();
  }

  if constexpr (!std::is_void_v<ComponentStateT> &&
                (kAutomaticFeaturesActive ||
                 CheckBit(flags, IsfOptionalFeature::kTextProtoSaving))) {
    proto::TextprotoWriter::RegisterKnownType<ComponentStateT>();
  }
}  // namespace imp

#if IMP_RUNTIME(DEV)
template <typename T>
void SceneComponentDeserializer::RegisterComponentWidget(Handler& handler) {
  if constexpr (T::kExcludeFromEditor) {
    return;
  }

  using ComponentIsfInfo = typename T::IsfInfo;
  using ComponentStateT = typename ComponentIsfInfo::StateT;

  handler.create_component_widget =
      [](NodeHandle node,
         Dispatcher& editor_dispatcher) -> std::unique_ptr<editor::Widget> {
    if (!node) {
      return {};
    }

    auto component = node->GetComponent<T>();
    if (!component) {
      return {};
    }

    if constexpr (!std::is_void_v<ComponentStateT>) {
      return std::make_unique<editor::IsfComponentWidget<T>>(component,
                                                             editor_dispatcher);
    } else {
      return std::make_unique<editor::StatelessIsfComponentWidget<T>>(
          component);
    }
  };
}

template <typename T>
void SceneComponentDeserializer::RegisterAddComponentUi() {
  if constexpr (T::kExcludeFromEditor) {
    return;
  }

  using IsfInfoT = typename T::IsfInfo;
  using StateT = typename IsfInfoT::StateT;
  using OptionalStateT = std::optional<StateT>;

  if constexpr (!std::is_void_v<StateT>) {
    // Setup the add-component UI panel.
    // Note: since this local is captured in the lambda below, it persists as
    // a "member-variable" in that lambda closure's generated class. This
    // will cause its state to persist across frames to make it useful.
    StateT state;
    add_component_ui_.Add(type_traits::kTypeName<T>, [this, state]() mutable {
      if (!selected_node_) {
        IMP_LOG(imp::FATAL) << "Call to show add-component UI without selected node.";
        return editor::FilterableCombo::SelectionStatus::kNotSelected;
      }

      // TODO: This type should be greyed-out in the combo.
      if (selected_node_->GetComponent<T>()) {
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(180, 30, 30, 255));
        ImGui::Text("Node already has this component type!");
        ImGui::PopStyleColor();
        return editor::FilterableCombo::SelectionStatus::kSelected;
      }

      imp::editor::EditorProtoVisitor<StateT> visitor(state);
      state.Visit(visitor, 0, static_cast<StateT*>(nullptr));
      ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(30, 90, 30, 255));
      bool clicked = ImGui::Button("Add");
      if (clicked) {
        command_manager_
            ->PerformCommand<editor::NodeValueCommand<OptionalStateT>>(
                selected_node_, std::nullopt, state,
                [](NodeHandle target,
                   const OptionalStateT& value) -> absl::Status {
                  auto scene_metadata = target->GetComponent<SceneMetadata>();
                  if (value) {
                    // TODO: allow async commands.
                    absl::StatusOr<ComponentHandle<T>> result =
                        editor::AddComponentWithStateSync<T>(target, *value);
                    if (scene_metadata) {
                      scene_metadata->SetComponentAuthored(
                          IsfInfoT::kTypeUrlHash, /*is_authored=*/true);
                    }
                    MP_RETURN_IF_ERROR(result.status());
                  } else {
                    target->RemoveComponent<T>();
                    if (scene_metadata) {
                      scene_metadata->SetComponentAuthored(
                          IsfInfoT::kTypeUrlHash, /*is_authored=*/false);
                    }
                  }

                  return absl::OkStatus();
                });
      }
      ImGui::PopStyleColor();
      return clicked ? editor::FilterableCombo::SelectionStatus::kNotSelected
                     : editor::FilterableCombo::SelectionStatus::kSelected;
    });
  } else {
    add_component_ui_.Add(type_traits::kTypeName<T>, [this]() {
      if (!selected_node_) {
        IMP_LOG(imp::FATAL) << "Call to show add-component UI without selected node.";
        return editor::FilterableCombo::SelectionStatus::kNotSelected;
      }

      // TODO: This type should be greyed-out in the combo.
      if (selected_node_->GetComponent<T>()) {
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(180, 30, 30, 255));
        ImGui::Text("Node already has this component type!");
        ImGui::PopStyleColor();
        return editor::FilterableCombo::SelectionStatus::kSelected;
      }

      ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(30, 90, 30, 255));
      bool clicked = ImGui::Button("Add");
      if (clicked) {
        command_manager_->PerformCommand<editor::NodeValueCommand<bool>>(
            selected_node_, false, true,
            [](NodeHandle target, bool value) -> absl::Status {
              auto scene_metadata = target->GetComponent<SceneMetadata>();
              if (value) {
                // TODO: allow async commands.
                absl::StatusOr<ComponentHandle<T>> result =
                    editor::AddComponentSync<T>(target);
                if (scene_metadata) {
                  scene_metadata->SetComponentAuthored(IsfInfoT::kTypeUrlHash,
                                                       /*is_authored=*/true);
                }
                MP_RETURN_IF_ERROR(result.status());
              } else {
                target->RemoveComponent<T>();
                if (scene_metadata) {
                  scene_metadata->SetComponentAuthored(IsfInfoT::kTypeUrlHash,
                                                       /*is_authored=*/false);
                }
              }

              return absl::OkStatus();
            });
      }
      ImGui::PopStyleColor();
      return clicked ? editor::FilterableCombo::SelectionStatus::kNotSelected
                     : editor::FilterableCombo::SelectionStatus::kSelected;
    });
  }
}
#endif

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_SCENE_COMPONENT_DESERIALIZER_H_
