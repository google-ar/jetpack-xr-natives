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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_ISF_INFO_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_ISF_INFO_H_

#include <array>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/async/future.h"
#include "core/common/bit_flag.h"
#include "core/common/hash.h"
#include "core/common/platform_helpers.h"
#include "core/common/template_helpers.h"
#include "core/common/type_traits.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_manager.h"
#include "core/ncsb/component_traits.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_data.proto.imp.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/parse_message_visitor.h"
#include "core/proto/proto_common.h"
#include "core/proto/proto_differ.h"
#include "core/proto/proto_reader.h"
#include "core/proto/proto_writer.h"
#include "core/view/async/future.h"

namespace imp {

// Optional features that can be passed in as flags to
// SceneSystem::RegisterComponentsIsfInfo.
//
// These features aren't registered by default because they can come with a
// binary size cost.
struct IsfOptionalFeature {
  using Flags = BitFlag;
  enum Flag : Flags {
    // Indicates that all optional features will be registered in dev builds,
    // but not in production builds.
    kAutomatic = 0,
    // Registers the IsfInfo so that the component state can be saved back out
    // via the SceneSystem.
    kSaving = 1 << 0,
    // Register the IsfInfo so that the component state can be loaded from a
    // textproto file.
    kTextprotoLoading = 1 << 1,
    // Registers the IsfInfo so that the component state can be written out to a
    // textproto file.
    kTextProtoSaving = 1 << 2,
  };
};

enum class SaveIsfStateResult { kSuccess, kEmpty, kFailed };

// Interface for defining a visitor that accesses a savable's state during the
// PreSetup phase of loading. Used to implement LoadSceneVisitor.
class BaseStateVisitor {
 public:
  virtual ~BaseStateVisitor() {}

  // Takes a type-erased state for a component, and the node
  // that the component was attached to.
  virtual absl::optional<Future<absl::Status>> Accept(
      NodeHandle node, HashValue state_type_hash, void* erased_state,
      bool should_enable_component) = 0;
};

// Used to declare dependencies between components that controls the order
// component Setup methods are called in when deserializing .isf files with
// SceneSystem::LoadScene.
//
// Ex:
//   using IsfInfo = IsfInfo<&GltfAnimator::state_,
//   IsfDependencies<GltfRenderer>>;
//
// The above example declares that when deserializing GltfAnimator, it should
// not be Setup until after the GltfRenderer component has finished being setup.
//
// For more examples:
//   third_party/impress/core/view/framework/tests/scene_system_test.cc
template <typename... Dependencies>
struct IsfDependencies {
  static constexpr std::array<HashValue, sizeof...(Dependencies)> kDependencies{
      Dependencies::IsfInfo::kTypeUrlHash...};
};

namespace shared_isf_info_handlers {
// Helper used by SceneComponentDeserializer shared by both IsfInfo and
// StatelessIsfInfo. Unifies calling all valid types of setup methods (or no
// setup at all) into a single type-erased method.
template <typename ComponentT, typename... Args>
Future<absl::Status> Setup(utils::Entity entity,
                           ComponentManager* component_manager,
                           bool should_enable_component, Args&&... args) {
  auto comp = component_manager->Get<ComponentT>(entity);

  using SetupResultT = decltype(component_manager->SetupWithoutAddFromIsf(
      comp, should_enable_component, std::forward<Args>(args)...));

  // Call Setup method and transform the result of Setup (i.e. void,
  // absl::Status, or Future<absl::Status>) into a single return type
  // (Future<absl::Status>).
  if constexpr (std::is_same_v<SetupResultT, void>) {
    component_manager->SetupWithoutAddFromIsf(comp, should_enable_component,
                                              std::forward<Args>(args)...);
    return Future<absl::Status>(absl::OkStatus());
  } else if constexpr (std::is_same_v<SetupResultT, absl::Status>) {
    return Future<absl::Status>(component_manager->SetupWithoutAddFromIsf(
        comp, should_enable_component, std::forward<Args>(args)...));
  } else {
    static_assert(std::is_same_v<SetupResultT, Future<absl::Status>>);
    return component_manager->SetupWithoutAddFromIsf(
        comp, should_enable_component, std::forward<Args>(args)...);
  }
}

// Helper used by SceneComponentDeserializer shared by both IsfInfo and
// StatelessIsfInfo. Used to determine if the component exists on the entity.
template <typename ComponentT>
bool HasComponent(utils::Entity entity, ComponentManager* component_manager) {
  auto comp_handle = component_manager->Get<ComponentT>(entity);
  return static_cast<bool>(comp_handle);
}

// Helper used by SceneComponentDeserializer shared by both IsfInfo and
// StatelessIsfInfo. Used to remove a component.
template <typename ComponentT>
void RemoveComponent(utils::Entity entity,
                     ComponentManager* component_manager) {
  component_manager->Remove<ComponentT>(entity);
}

// Helper used by SceneComponentDeserializer shared by both IsfInfo and
// StatelessIsfInfo. Used to invoke the OnIsfStateChanged() method on a
// component.
template <typename ComponentT>
Future<absl::Status> OnIsfStateChanged(utils::Entity entity,
                                       ComponentManager* component_manager) {
  auto comp_handle = component_manager->Get<ComponentT>(entity);
  using OnIsfStateChangedResultT = decltype(comp_handle->OnIsfStateChanged());
  // Call OnIsfStateChanged method and transform the result (i.e. void,
  // absl::Status, or Future<absl::Status>) into a single return type
  // (Future<absl::Status>).
  if constexpr (std::is_same_v<OnIsfStateChangedResultT, void>) {
    comp_handle->OnIsfStateChanged();
    return Future<absl::Status>(absl::OkStatus());
  } else if constexpr (std::is_same_v<OnIsfStateChangedResultT, absl::Status>) {
    return Future<absl::Status>(comp_handle->OnIsfStateChanged());
  } else {
    static_assert(
        std::is_same_v<OnIsfStateChangedResultT, Future<absl::Status>>);
    return comp_handle->OnIsfStateChanged();
  }
}
}  // namespace shared_isf_info_handlers

namespace isf_info_type_traits {
// Deduce component type from StateFieldAddress
template <typename Component, typename State>
static Component DeduceComponentHelper(State Component::*);

// Deduce data type from state_field
template <typename Component, typename State>
static State DeduceStateHelper(State Component::*);
}  // namespace isf_info_type_traits

// IsfInfo that allows the Component type to be specified explicitly instead of
// dedudced automatically from the state_field.
//
// In the vast majority of cases, IsfInfo should be used instead of this.
//
// In rare cases, this can be useful when the State field is declared in a
// templated base class instead of in the concrete derived class that is
// registered. Care should be taken when using this, since there must still be a
// 1:1 mapping between registered component types and state types.
template <typename Comp, auto state_field, typename Dependencies = void>
struct IsfInfoWithExplicitComp {
  constexpr IsfInfoWithExplicitComp() {
    static_assert(
        type_traits::IsTemplateType<Dependencies, IsfDependencies>::value ||
            std::is_same_v<void, Dependencies>,
        "Dependencies template parameter must be a template expansion of type "
        "IsfDependencies");
    using StateContainerT =
        decltype(isf_info_type_traits::DeduceComponentHelper(state_field));
    static_assert(std::is_base_of_v<StateContainerT, Comp> ||
                      std::is_same_v<StateContainerT, Comp>,
                  "Comp must either be the type containing the state_field or "
                  "a subclass of it.");
  }

  // The type of the component.
  using ComponentT = Comp;

  // The type of the component's State field.
  using StateT = decltype(isf_info_type_traits::DeduceStateHelper(state_field));

  // The type url expected in the .isf file for this component.
  // This is the type url for the proto of the state_field passed into the
  // template parameter.
  static constexpr absl::string_view kTypeUrl = StateT::kTypeUrl;

  static constexpr HashValue kTypeUrlHash = StateT::kTypeUrlHash;

  using DependenciesT = Dependencies;

  // Helper for adding a component to the passed in entity and deserializing the
  // proto data into the component's state_field. Does not call Setup, that is
  // done later as part of the LoadScene process.
  static bool Parse(NodeHandle node, ComponentManager* component_manager,
                    const std::vector<absl::string_view>& component_data,
                    proto::ParseMessageVisitor* parse_message_visitor,
                    bool should_add_component) {
    ComponentT* comp = nullptr;
    if (should_add_component) {
      comp = component_manager->AddWithoutSetup<ComponentT>(node).Get();
    } else {
      comp = component_manager->Get<ComponentT>(node.GetEntity()).Get();
    }

    // Parse takes a list of protos because each one could be coming from a
    // different .isf that is being merged together using the NodeData base
    // field. The binary data is parsed in-order to merge together the data from
    // each .isf file. The data is ordered to prioritize the last .isf.
    for (absl::string_view component_binary_data : component_data) {
      if (!proto::ParseMessage(component_binary_data, &(comp->*state_field),
                               parse_message_visitor)) {
        return false;
      }
    }
    return true;
  }

  // Returns a reference to the ISF state object for a component.
  static StateT& GetState(ComponentHandle<ComponentT> component) {
    return component.Get()->*state_field;
  }

  // Helper for saving out the component on this entity (if it exists) into a
  // proto any by serializing the state_field into the any passed in.
  static SaveIsfStateResult Save(
      utils::Entity entity, ComponentManager& component_manager,
      google::protobuf::imp_proto::Any& out_any, bool& out_is_enabled,
      absl::Span<const std::string> base_component_sources) {
    auto comp_handle = component_manager.Get<ComponentT>(entity);
    ComponentT* comp = comp_handle.Get();
    if (!comp) {
      return SaveIsfStateResult::kFailed;
    }
    if constexpr (component_traits::kHasPreSaveFunc<ComponentT>) {
      comp->PreSave();
    }

    out_is_enabled = comp->IsEnabled();
    out_any.type_url = StateT::kTypeUrl;

    // There are no base sources, simply serialize the current state field.
    if (base_component_sources.empty()) {
      if (!proto::SerializeTo(&(comp->*state_field), &out_any.value)) {
        return SaveIsfStateResult::kFailed;
      }
      return SaveIsfStateResult::kSuccess;
    }

    StateT current_state = comp->*state_field;

    // Combine the base sources into one.
    StateT state_to_compare;
    for (const auto& source : base_component_sources) {
      if (!proto::ParseMessage(source, &state_to_compare)) {
        return SaveIsfStateResult::kFailed;
      }
    }

    // Diff the base sources against the current state field.
    proto::ProtoDiffer::Result diff_result =
        proto::RemoveMatchingFields(current_state, state_to_compare);

    // The base sources match the current state field, that means there is no
    // change from the bases, report that.
    if (diff_result == proto::ProtoDiffer::Result::kFoundAllFieldsMatch) {
      return SaveIsfStateResult::kEmpty;
    } else if (diff_result ==
               proto::ProtoDiffer::Result::kFoundUnmergableDifferences) {
      return SaveIsfStateResult::kFailed;
    }

    // Serialize just the delta between the current state field and the bases.
    if (!proto::SerializeTo(&current_state, &out_any.value)) {
      return SaveIsfStateResult::kFailed;
    }
    return SaveIsfStateResult::kSuccess;
  }

  // Helper for visiting the component before the default Setup is called.
  // If the visitor calls a setup overload, returns the result. Otherwise
  // returns nullopt.
  static absl::optional<Future<absl::Status>> Visit(
      NodeHandle node, ComponentManager* component_manager,
      BaseStateVisitor* state_visitor, bool should_enable_component) {
    auto comp_handle = component_manager->Get<ComponentT>(node.GetEntity());
    ComponentT* comp = comp_handle.Get();

    if (state_visitor) {
      return state_visitor->Accept(node, type_traits::kTypeHash<StateT>,
                                   &(comp->*state_field),
                                   should_enable_component);
    }

    return absl::nullopt;
  }
};

// Used to declare that a component type can be deserialized from a .isf file
// when calling SceneSystem::LoadScene.
//
// Provides information about the proto field that .isf data should be
// deserialized into, how to add/remove the component, and the dependency order
// for calling Setup on all the deserialized components.
//
// To use, the component type must declare a using statement for IsfInfo. Then,
// SceneSystem::RegisterComponentsIsfInfo must be called for the component type,
// which provides the information to the SceneSystem.
//
// In Foo Component:
//   using IsfInfo = IsfInfo<&FooComponent::state_>;
//
// Elsewhere:
//   GetSceneSystem().RegisterComponentsIsfInfo<FooComponent>();
//
// See Examples:
//   third_party/impress/core/view/framework/tests/scene_system_test.cc
template <auto state_field, typename Dependencies = void>
using IsfInfo = IsfInfoWithExplicitComp<
    decltype(isf_info_type_traits::DeduceComponentHelper(state_field)),
    state_field, Dependencies>;

// Alternative to IsfnInfo for declaring that a component type can be
// deserialized from a .isf file when calling SceneSystem::LoadScene.
//
// Unlike IsfInfo, this doesn't require a proto field to deserialize into. It
// can be used to deserialize a component with no additional data beyond the
// component type.
//
// To use, the component type must declare a using statement for
// StatelessIsfInfo. Then, SceneSystem::RegisterComponentsIsfInfo must be called
// for the component type, which provides the information to the
// SceneSystem.
//
// In Foo Component:
//   using IsfInfo = StatelessIsfInfo<FooComponent, kFooTypeUrl>;
//
// Elsewhere:
//   GetSceneSystem().RegisterComponentsIsfInfo<FooComponent>();
//
// See Examples:
//   third_party/impress/core/view/framework/tests/scene_system_test.cc
template <typename Comp, const absl::string_view& type_url,
          typename Dependencies = void>
struct StatelessIsfInfo {
  constexpr StatelessIsfInfo() {
    static_assert(
        type_traits::IsTemplateType<Dependencies, IsfDependencies>::value ||
            std::is_same_v<void, Dependencies>,
        "Dependencies template parameter must be a template expansion of type "
        "IsfDependencies");
  }

  // The type of the component.
  using ComponentT = Comp;

  // StatelessIsfInfo has no state field.
  // This is useful for detecting if a component's isf info contains a State.
  using StateT = void;

  // The type url expected in the .isf file for this component.
  // this is the type_url template parameter passed in.
  static constexpr absl::string_view kTypeUrl = type_url;

  static constexpr HashValue kTypeUrlHash = imp::Hash(type_url);

  using DependenciesT = Dependencies;

  // Helper for adding a component to the passed in entity. Does not call Setup,
  // that is done later as part of the LoadScene process.
  static bool Parse(NodeHandle node, ComponentManager* component_manager,
                    const std::vector<absl::string_view>& component_data,
                    proto::ParseMessageVisitor* parse_message_visitor,
                    bool should_add_component) {
    if (should_add_component) {
      component_manager->AddWithoutSetup<ComponentT>(node);
    }
    return true;
  }

  // Helper for saving out the component on this entity (if it exists) into a
  // proto any by serializing a StatelessComponent proto with this components
  // type_url into the any passed in.
  static SaveIsfStateResult Save(
      utils::Entity entity, ComponentManager& component_manager,
      google::protobuf::imp_proto::Any& out_any, bool& out_is_enabled,
      absl::Span<const std::string> base_component_sources) {
    auto comp_handle = component_manager.Get<ComponentT>(entity);
    out_is_enabled = comp_handle->IsEnabled();

    StatelessComponent comp;
    comp.type = std::string(type_url);
    out_any.type_url = StatelessComponent::kTypeUrl;
    if (!proto::SerializeTo(&comp, &out_any.value)) {
      return SaveIsfStateResult::kFailed;
    }

    return SaveIsfStateResult::kSuccess;
  }

  // Helper for performing work done on the component just before Setup is
  // called. Currently, this does nothing for StatelessIsfInfo.
  // TODO: Support passing parameters to Setup via .isf for
  // Stateless components.
  static absl::optional<Future<absl::Status>> Visit(
      NodeHandle node, ComponentManager* component_manager,
      BaseStateVisitor* state_visitor, bool should_enable_component) {
    return absl::nullopt;
  }
};

namespace internal {

template <typename T,
          std::enable_if_t<!std::is_same_v<void, typename T::IsfDependencies>,
                           int> = 0>
static constexpr bool IsIsfDepsDefinedImpl(int) {
  return true;
}

template <typename T>
static constexpr bool IsIsfDepsDefinedImpl(...) {
  return false;
}

// Compile-time check to determine if type T contains IsfDependencies.
template <typename T>
static constexpr bool kIsIsfDepsDefined = internal::IsIsfDepsDefinedImpl<T>(0);

}  // namespace internal

// Helper class that takes the type of a component's State and recursively
// crawls through all of its fields to search for IsfDependencies.
//
// This makes it possible to setup IsfDependencies implicitly based on what
// fields are in a Component's state proto instead of only explicitly.
template <typename T>
struct IsfStateDepsAdder {
  // Adds all the dependencies of all fields to the deps set.
  static void AddDeps(absl::flat_hash_set<HashValue>& deps,
                      absl::flat_hash_set<HashValue>& visited_types) {
    // This type has IsfDependencies, add each dependency to the set.
    if constexpr (internal::kIsIsfDepsDefined<T>) {
      for (HashValue dep_type_hash : T::IsfDependencies::kDependencies) {
        deps.insert(dep_type_hash);
      }
    }

    // This type is a proto message that contains fields, check for deps on each
    // field.
    if constexpr (imp::proto::HasFields<T>::value) {
      // Ensures that the type T is only visited once which prevents this from
      // crashing for protos with recursive types.
      if (visited_types.contains(type_traits::kTypeHash<T>)) {
        return;
      }
      visited_types.insert(type_traits::kTypeHash<T>);

      ForConstexpr<0, T::kFieldsCount>([&deps, &visited_types](auto i) {
        using FieldType = typename T::template FieldType<i>::Type;
        IsfStateDepsAdder<FieldType>::AddDeps(deps, visited_types);
      });
    }
  }
};

// Partial specialization to unwrap repeated proto fields.
template <typename T>
struct IsfStateDepsAdder<std::vector<T>> {
  static void AddDeps(absl::flat_hash_set<HashValue>& deps,
                      absl::flat_hash_set<HashValue>& visited_types) {
    IsfStateDepsAdder<T>::AddDeps(deps, visited_types);
  }
};

// Partial specialization to unwrap unique_ptr proto fields.
template <typename T>
struct IsfStateDepsAdder<std::unique_ptr<T>> {
  static void AddDeps(absl::flat_hash_set<HashValue>& deps,
                      absl::flat_hash_set<HashValue>& visited_types) {
    IsfStateDepsAdder<T>::AddDeps(deps, visited_types);
  }
};

// Partial specialization to unwrap optional proto fields.
template <typename T>
struct IsfStateDepsAdder<absl::optional<T>> {
  static void AddDeps(absl::flat_hash_set<HashValue>& deps,
                      absl::flat_hash_set<HashValue>& visited_types) {
    IsfStateDepsAdder<T>::AddDeps(deps, visited_types);
  }
};

// Parial specialization to unwrap map proto fields.
template <typename Key, typename Value>
struct IsfStateDepsAdder<std::map<Key, Value>> {
  static void AddDeps(absl::flat_hash_set<HashValue>& deps,
                      absl::flat_hash_set<HashValue>& visited_types) {
    IsfStateDepsAdder<Value>::AddDeps(deps, visited_types);
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_ISF_INFO_H_
