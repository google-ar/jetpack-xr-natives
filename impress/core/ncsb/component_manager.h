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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_MANAGER_H_

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/async/future.h"
#include "core/common/robin_map.h"
#include "core/common/trace.h"
#include "core/common/type_traits.h"
#include "core/config.h"
#include "core/graph/dependency_graph.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_pool.h"
#include "core/ncsb/component_pool_with_updater.h"
#include "core/ncsb/component_system_pool.h"
#include "core/ncsb/component_traits.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_system.h"

#if IMP_RUNTIME(DEV)
#include "core/editor/editor_info.h"
#endif

namespace imp {

class BaseView;

// Manages the memory for all components.
class ComponentManager {
 public:
  explicit ComponentManager(BaseView* view);

  ComponentManager(const ComponentManager&) = delete;
  ComponentManager& operator=(const ComponentManager&) = delete;

  ComponentManager(ComponentManager&&) = default;

  template <typename T, typename... Args>
  Component::AddResult<T, Args...> Add(NodeHandle node, Args&&... args);

  template <typename T, typename... Args>
  Component::AddWithStateResult<T, Args...> AddWithState(
      NodeHandle node, typename T::IsfInfo::StateT state, Args&&... args);

  // Special method for adding a component without actually setting it up.
  // This is used as part of deserialization by SceneSystem.
  // Note: Anything using this needs to ensure SetupWithoutAdd called.
  template <typename T>
  ComponentHandle<T> AddWithoutSetup(NodeHandle node);

  // Special method for setting up a component after it's already been added.
  // This is used as part of deserialization by SceneSystem.
  template <typename T, typename... Args>
  auto SetupWithoutAddFromIsf(ComponentHandle<T> comp,
                              bool should_enable_component, Args&&... args);

  // Returns a handle to the component attached to the given entity.
  template <typename T>
  ComponentHandle<T> Get(utils::Entity entity);

  // Returns a handle to the component that matches the predicate.
  // Predicate is a function or lambda with the signature bool(const T*).
  template <typename T, typename Fn>
  ComponentHandle<T> Get(const Fn& predicate);

  // Checks if the component exists on the given entity.
  template <typename T>
  bool Has(utils::Entity entity);

  // Checks if the component exists based on the given predicate.
  // Predicate is a function or lambda with the signature bool(T*).
  template <typename T, typename Fn>
  bool Has(const Fn& predicate);

  template <typename T>
  void Remove(utils::Entity entity);

  template <typename T, typename... Args>
  Component::AddResult<T, Args...> GetOrAdd(NodeHandle node, Args&&... args);

  template <typename T, typename... Args>
  Component::AddWithStateResult<T, Args...> GetOrAddWithState(
      NodeHandle node, typename T::IsfInfo::StateT state, Args&&... args);

  // Loop through all components of type T and call the given function.
  template <typename T, typename Fn>
  void ForEach(Fn&& fn);

  // Loop through all components of type T that should be updated on this frame
  // and call the given function.
  template <typename T, typename Fn>
  void UpdateEach(Fn&& fn);

  template <typename Fn>
  void ForEachPool(Fn&& fn);

  // Notifies all components attached to the entity of its active state.
  void NotifyActiveForEntity(utils::Entity entity, bool active);

  // Removes all components from all the nodes passed in.
  void RemoveAllFromNodes(const std::vector<NodeHandle>& nodes);

  // Completely detaches all components from all entities.
  void DetachAll();

  BaseComponentPool* GetComponentPoolById(ComponentId component_id);

  // Returns the System for a given type of component.
  //
  // If the type of component doesn't declare a System, this will not compile.
  //
  // See component_system.h for more information.
  template <typename T>
  typename T::System& GetComponentSystem();

  // Return true if the System for a given type of component exists.
  //
  // If the type of component doesn't declare a System, this will not compile.
  //
  // ComponentSystem is constructed automatically in a few cases, and this is
  // mostly useful if your ComponentSystem constructor needs custom parameters
  // and cannot be constructed automatically.
  //
  // See component_system.h for more information.
  template <typename T>
  bool HasComponentSystem();

  // Used to pass in custom parameters to a ComponentSystem by explicitly
  // creating it in advance.
  //
  // Otherwise, a Component's System will be created automatically
  // by passing in just the BaseView parameter to the constructor.
  //
  // Must be called before a component of type T is added if additional
  // parameters are required.
  //
  // See component_system.h for more information.
  template <typename T, typename... Args>
  void CreateComponentSystem(Args&&... args);

 private:
  using ComponentPoolMap =
      RobinMap<ComponentId, std::unique_ptr<BaseComponentPool>>;
  using ComponentPoolListView = std::vector<BaseComponentPool*>;

  // Used when setting up a component to help control when SetupWithState
  // methods vs. regular Setup methods are called.
  enum class SetupMode {
    // Mode for calling SetupWithState methods while loading an Isf or calling
    // AddWithState, falls back to regular Setup.
    kSetupWithStateOrSetup,
    // Mode for calling regular Setup methods only.
    kNormal
  };

  template <SetupMode setup_mode, typename T, typename... Args>
  auto SetupComponentAfterAdd(ComponentHandle<T> comp, Args&&... args);

  // Helper method for invoking Setup and SetupWithState methods on a component
  // that return a result.
  template <SetupMode setup_mode, typename T, typename... Args>
  auto InvokeSetup(ComponentHandle<T> comp, Args&&... args);

  // Helper method for invoking Setup and SetupWithState methods on a component
  // that return void.
  template <SetupMode setup_mode, typename T, typename... Args>
  void InvokeSetupVoid(ComponentHandle<T> comp, Args&&... args);

  // Special method for setting up a component after it's already been added.
  // This is called when normally adding a component, or by
  // SetupWithoutAddFromIsf if the component does not have a SetupWithState
  // method to call.
  template <SetupMode setup_mode, typename T, typename... Args>
  auto SetupWithoutAdd(ComponentHandle<T> comp, bool should_enable_component,
                       Args&&... args);

  template <typename T>
  ComponentPool<T>& GetComponentPool();

  UpdateSystem& GetUpdateSystem();

  template <typename T>
  void AddPoolToCleanupGraph(BaseComponentPool* pool);

  // The component pools for each type of component that has been added mapped
  // by the type of component.
  ComponentPoolMap component_pools_;

  // A list of all the component pools that can be quickly iterated through.
  //
  // This is just a view into the pools stored in component_pools_.
  //
  // This can be used to iterate over the pools quickly using indices in cases
  // where the component pools could become invalidated.
  ComponentPoolListView component_pools_list_view_;

  // This is used to determine the order in which components are removed when a
  // node is destroyed.
  DependencyGraph<ComponentId, BaseComponentPool*> cleanup_graph_;

  template <SetupMode setup_mode, typename T, typename... Args>
  auto OptionalAddResultTypeUnpacker();

  // Helper function for GetOrAdd. If the entity has a component of type T, this
  // will return either a ComponentHandle<T> or a StatusOr<ComponentHandle<T>>
  // or a Future<ComponentHandle<T>>, depending on the return type of
  // Setup(Args...).
  //
  // If a component of type T is not found and if Setup(Args...) is aync, check
  // to see if there's any component of type T that's currently being added. If
  // so, returns Future<ComponentHandle<T>>.
  //
  // If none of the above applies, return std::nullopt.
  template <SetupMode setup_mode, typename T, typename... Args>
  auto GetOrAddHelper(utils::Entity entity);

  BaseView* view_;
};

// -----------------------------------------------------------------------------
// Template implementation follows.

template <typename T, typename... Args>
Component::AddResult<T, Args...> ComponentManager::Add(NodeHandle node,
                                                       Args&&... args) {
  ComponentHandle<T> comp = AddWithoutSetup<T>(node);

  return SetupComponentAfterAdd<SetupMode::kNormal>(
      comp, std::forward<Args>(args)...);
}

template <typename T, typename... Args>
Component::AddWithStateResult<T, Args...> ComponentManager::AddWithState(
    NodeHandle node, typename T::IsfInfo::StateT state, Args&&... args) {
  ComponentHandle<T> comp = AddWithoutSetup<T>(node);

  // Copy the state to the component's state field.
  T::IsfInfo::GetState(comp) = std::move(state);

  return SetupComponentAfterAdd<SetupMode::kSetupWithStateOrSetup>(
      comp, std::forward<Args>(args)...);
}

template <ComponentManager::SetupMode setup_mode, typename T, typename... Args>
auto ComponentManager::SetupComponentAfterAdd(ComponentHandle<T> comp,
                                              Args&&... args) {
  using SetupResultT =
      decltype(InvokeSetup<setup_mode>(comp, std::forward<Args>(args)...));

  static_assert(std::is_same_v<SetupResultT, Future<absl::Status>> ||
                    std::is_same_v<SetupResultT, absl::Status> ||
                    std::is_same_v<SetupResultT, void>,
                "Component Setup method must return either "
                "imp::Future<absl::Status>, absl::Status, or void.");

  // Call Setup method and transform the result of Setup (i.e. void,
  // absl::Status, or Future<absl::Status>) into the result of Add (i.e.
  // ComponentHandle<T>, absl::StatusOr<ComponentHandle<T>>, or
  // Future<ComponentHandle<T>>).
  if constexpr (std::is_same_v<SetupResultT, void>) {
    SetupWithoutAdd<setup_mode>(comp, true, std::forward<Args>(args)...);
    return comp;
  } else if constexpr (std::is_same_v<SetupResultT, absl::Status>) {
    using ReturnT = absl::StatusOr<ComponentHandle<T>>;
    absl::Status status =
        SetupWithoutAdd<setup_mode>(comp, true, std::forward<Args>(args)...);
    if (!status.ok()) {
      return ReturnT(status);
    }
    return ReturnT(comp);
  } else {
    static_assert(std::is_same_v<SetupResultT, Future<absl::Status>>);
    return SetupWithoutAdd<setup_mode>(comp, true, std::forward<Args>(args)...)
        .Then([comp]() { return comp; });
  }
}

template <typename T>
ComponentHandle<T> ComponentManager::AddWithoutSetup(NodeHandle node) {
  ComponentPool<T>& pool = GetComponentPool<T>();
  T* component = static_cast<T*>(pool.Add(node.GetEntity()));
  ComponentHandle<T> comp =
      ComponentHandle<T>(node.GetEntity(), &pool, component);
  comp->PostCreated(node, kComponentId<T>);
  return comp;
}

template <ComponentManager::SetupMode setup_mode, typename T, typename... Args>
auto ComponentManager::InvokeSetup(ComponentHandle<T> comp, Args&&... args) {
  IMP_TRACE_NAME_TEMPLATED("Setup", T);
  if constexpr (setup_mode == SetupMode::kSetupWithStateOrSetup &&
                component_traits::kHasSetupWithStateFunc<T, Args...>) {
    return comp->SetupWithState(std::forward<Args>(args)...);
  } else {
    return comp->Setup(std::forward<Args>(args)...);
  }
}

template <ComponentManager::SetupMode setup_mode, typename T, typename... Args>
void ComponentManager::InvokeSetupVoid(ComponentHandle<T> comp,
                                       Args&&... args) {
  IMP_TRACE_NAME_TEMPLATED("Setup", T);
  if constexpr (setup_mode == SetupMode::kSetupWithStateOrSetup &&
                component_traits::kHasSetupWithStateFunc<T, Args...>) {
    comp->SetupWithState(std::forward<Args>(args)...);
  } else {
    comp->Setup(std::forward<Args>(args)...);
  }
}

template <ComponentManager::SetupMode setup_mode, typename T, typename... Args>
auto ComponentManager::SetupWithoutAdd(ComponentHandle<T> comp,
                                       bool should_enable_component,
                                       Args&&... args) {
  using SetupResultT =
      decltype(InvokeSetup<setup_mode>(comp, std::forward<Args>(args)...));

  ComponentPool<T>& pool = GetComponentPool<T>();

  // If the editor is in edit mode, skip running Setup for components that
  // shouldn't run in edit mode. This is so components remain in their initial
  // state until the Play button is pressed. When switching to Play mode, the
  // entire scene graph is re-created which will cause this method to be called
  // again and Setup to be invoked.
#if IMP_RUNTIME(DEV)
  if (!comp->IsEditorStaging()) {
    if constexpr (!component_traits::kShouldRunInEditMode<T>) {
      if (editor::IsInEditMode(view_->GetRegistry())) {
        pool.PostSetup(comp.GetEntity(), should_enable_component);

        if constexpr (std::is_same_v<SetupResultT, void>) {
          return;
        } else if constexpr (std::is_same_v<SetupResultT, absl::Status>) {
          return absl::OkStatus();
        } else {
          static_assert(std::is_same_v<SetupResultT, Future<absl::Status>>);
          return Future<absl::Status>(absl::OkStatus());
        }
      }
    }
  }
#endif

  if constexpr (std::is_same_v<SetupResultT, void>) {
    InvokeSetupVoid<setup_mode>(comp, std::forward<Args>(args)...);
    pool.PostSetup(comp.GetEntity(), should_enable_component);
    return;
  } else if constexpr (std::is_same_v<SetupResultT, absl::Status>) {
    absl::Status status =
        InvokeSetup<setup_mode>(comp, std::forward<Args>(args)...);
    if (!status.ok()) {
      pool.Remove(comp.GetEntity());
      return status;
    }
    pool.PostSetup(comp.GetEntity(), should_enable_component);
    return absl::OkStatus();
  } else {
    static_assert(std::is_same_v<SetupResultT, Future<absl::Status>>);
    return pool.MakeSetupFuture(
        comp.GetEntity(),
        InvokeSetup<setup_mode>(comp, std::forward<Args>(args)...),
        should_enable_component);
  }
}

template <typename T, typename... Args>
auto ComponentManager::SetupWithoutAddFromIsf(ComponentHandle<T> comp,
                                              bool should_enable_component,
                                              Args&&... args) {
  return SetupWithoutAdd<SetupMode::kSetupWithStateOrSetup>(
      comp, should_enable_component, std::forward<Args>(args)...);
}

template <typename T>
ComponentHandle<T> ComponentManager::Get(utils::Entity entity) {
  ComponentPool<T>& pool = GetComponentPool<T>();
  Component* component = pool.TryGetRawComponentFromEntity(entity);
  if (!component || pool.Pending(entity)) {
    return ComponentHandle<T>();
  }

  return ComponentHandle<T>(entity, &pool, static_cast<T*>(component));
}

template <typename T, typename Fn>
ComponentHandle<T> ComponentManager::Get(const Fn& predicate) {
  imp::ComponentHandle<T> result;
  ForEach<T>([this, &result, &predicate](const T* comp) {
    if (!result && predicate(comp)) {
      result = Get<T>(comp->GetNode()->GetEntity());
    }
  });
  return result;
}

template <typename T>
bool ComponentManager::Has(utils::Entity entity) {
  ComponentPool<T>& pool = GetComponentPool<T>();
  return pool.Has(entity);
}

template <typename T, typename Fn>
bool ComponentManager::Has(const Fn& predicate) {
  bool contains_component = false;
  ForEach<T>([&contains_component, &predicate](const T* comp) {
    if (!contains_component && predicate(comp)) {
      contains_component = true;
    }
  });
  return contains_component;
}

template <typename T>
void ComponentManager::Remove(utils::Entity entity) {
  ComponentPool<T>& pool = GetComponentPool<T>();
  pool.Remove(entity);
}

template <ComponentManager::SetupMode setup_mode, typename T, typename... Args>
auto ComponentManager::OptionalAddResultTypeUnpacker() {
  if constexpr (setup_mode == SetupMode::kNormal) {
    return std::optional<component_traits::AddResult<T, Args...>>();

  } else {
    return std::optional<component_traits::AddWithStateResult<T, Args...>>();
  }
}

template <ComponentManager::SetupMode setup_mode, typename T, typename... Args>
auto ComponentManager::GetOrAddHelper(utils::Entity entity) {
  using ResultT =
      decltype(OptionalAddResultTypeUnpacker<setup_mode, T, Args...>());

  ComponentHandle<T> result = Get<T>(entity);
  if (result) {
    return ResultT(result);
  }

  constexpr bool kIsSetupAsync =
      std::is_same_v<ResultT, std::optional<Future<ComponentHandle<T>>>>;

  if constexpr (kIsSetupAsync) {
    ComponentPool<T>& pool = GetComponentPool<T>();
    if (auto optional_future = pool.GetSetupFuture(entity)) {
      return ResultT(optional_future->Then(
          [entity,
           this](absl::Status status) -> absl::StatusOr<ComponentHandle<T>> {
            if (!status.ok()) {
              return status;
            }

            return Get<T>(entity);
          }));
    }
  }

  return ResultT(std::nullopt);
}

template <typename T, typename... Args>
Component::AddResult<T, Args...> ComponentManager::GetOrAdd(NodeHandle node,
                                                            Args&&... args) {
  std::optional<Component::AddResult<T, Args...>> result =
      GetOrAddHelper<SetupMode::kNormal, T, Args...>(node.GetEntity());

  if (result) {
    return *result;
  }

  return Add<T>(node, std::forward<Args>(args)...);
}

template <typename T, typename... Args>
Component::AddWithStateResult<T, Args...> ComponentManager::GetOrAddWithState(
    NodeHandle node, typename T::IsfInfo::StateT state, Args&&... args) {
  std::optional<Component::AddWithStateResult<T, Args...>> result =
      GetOrAddHelper<SetupMode::kSetupWithStateOrSetup, T, Args...>(
          node.GetEntity());

  if (result) {
    return *result;
  }

  return AddWithState<T>(node, std::move(state), std::forward<Args>(args)...);
}

template <typename T, typename Fn>
void ComponentManager::ForEach(Fn&& fn) {
  ComponentPool<T>& pool = GetComponentPool<T>();
  pool.ForEach(std::forward<Fn>(fn));
}

template <typename T, typename Fn>
void ComponentManager::UpdateEach(Fn&& fn) {
  ComponentPool<T>& pool = GetComponentPool<T>();
  pool.UpdateEach(std::forward<Fn>(fn));
}

template <typename Fn>
void ComponentManager::ForEachPool(Fn&& fn) {
  for (size_t i = 0; i < component_pools_list_view_.size(); i++) {
    BaseComponentPool* pool = component_pools_list_view_[i];
    fn(pool);
  }
}

template <typename T>
ComponentPool<T>& ComponentManager::GetComponentPool() {
  ComponentId component_id = kComponentId<T>;
  auto itr = component_pools_.find(component_id);
  if (itr == component_pools_.end()) {
    if constexpr (!component_traits::kIsComponentSystemDefined<T>) {
      if constexpr (component_traits::kHasUpdateFunc<T>) {
        component_pools_[component_id] =
            std::make_unique<ComponentPoolWithUpdater<T>>(*view_);
      } else {
        component_pools_[component_id] =
            std::make_unique<ComponentPool<T>>(*view_);
      }
    } else {
      using TSystem = typename T::System;
      if constexpr (std::is_constructible_v<TSystem, BaseView*>) {
        component_pools_[component_id] =
            std::make_unique<ComponentSystemPool<T>>(
                *view_, std::make_unique<TSystem>(view_));
      } else {
        IMP_LOG(imp::FATAL) << "Unable to create ComponentPool because the component "
                      "requires a ComponentSystem with custom parameters. Call "
                      "ComponentManager::CreateComponentSystem first.";
      }
    }

    BaseComponentPool* base_pool = component_pools_.at(component_id).get();
    component_pools_list_view_.push_back(base_pool);
    AddPoolToCleanupGraph<T>(base_pool);
    ComponentPool<T>& pool = *static_cast<ComponentPool<T>*>(base_pool);
    return pool;
  }

  BaseComponentPool* pool = itr->second.get();
  return *static_cast<ComponentPool<T>*>(pool);
}

template <typename T>
typename T::System& ComponentManager::GetComponentSystem() {
  static_assert(component_traits::kIsComponentSystemDefined<T>,
                "Cannot call GetComponentSystem with Component type that "
                "doesn't declare a ComponentSystem.");

  using TSystem = typename T::System;
  return *static_cast<TSystem*>(GetComponentPool<T>().GetComponentSystem());
}

template <typename T>
bool ComponentManager::HasComponentSystem() {
  static_assert(component_traits::kIsComponentSystemDefined<T>,
                "Cannot call HasComponentSystem with Component type that "
                "doesn't declare a ComponentSystem.");

  ComponentId component_id = kComponentId<T>;
  return component_pools_.contains(component_id);
}

template <typename T, typename... Args>
void ComponentManager::CreateComponentSystem(Args&&... args) {
  ComponentId component_id = kComponentId<T>;
  auto itr = component_pools_.find(component_id);
  if (itr != component_pools_.end()) {
    IMP_LOG(imp::FATAL)
        << "Cannot create ComponentSystem, the ComponentPool has already been "
           "created.";
  }

  component_pools_[component_id] = std::make_unique<ComponentSystemPool<T>>(
      *view_,
      std::make_unique<typename T::System>(view_, std::forward<Args>(args)...));
  BaseComponentPool* base_pool = component_pools_.at(component_id).get();
  component_pools_list_view_.push_back(base_pool);
  AddPoolToCleanupGraph<T>(base_pool);
}

template <typename T>
void ComponentManager::AddPoolToCleanupGraph(BaseComponentPool* pool) {
  constexpr bool kHasCleanupDependencies =
      component_traits::kAreCleanupDependenciesDefined<T>;
  constexpr bool kHasCleanupDependents =
      component_traits::kAreCleanupDependentsDefined<T>;
  constexpr ComponentId component_id = kComponentId<T>;

  if constexpr (kHasCleanupDependencies) {
    static_assert(type_traits::IsTemplateType<typename T::CleanupDependencies,
                                              CleanupIds>::value,
                  "CleanupDependencies must be of type CIds.");
    static_assert(!T::CleanupDependencies::kIds.empty());
    for (ComponentId id : T::CleanupDependencies::kIds) {
      cleanup_graph_.AddDependency(component_id, id);
    }
  }

  if constexpr (kHasCleanupDependents) {
    static_assert(type_traits::IsTemplateType<typename T::CleanupDependents,
                                              CleanupIds>::value,
                  "CleanupDependents must be of type CleanupIds.");
    static_assert(!T::CleanupDependents::kIds.empty());
    for (ComponentId id : T::CleanupDependents::kIds) {
      cleanup_graph_.AddDependency(id, component_id);
    }
  }

  if constexpr (!kHasCleanupDependencies && !kHasCleanupDependents) {
    cleanup_graph_.AddNode(component_id);
  }

  cleanup_graph_.SetExtra(component_id, pool);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_MANAGER_H_
