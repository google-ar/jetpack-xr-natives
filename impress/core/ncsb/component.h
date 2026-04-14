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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_H_

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/bit_flag.h"
#include "core/common/hash.h"
#include "core/config.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_traits.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_phase.h"
#include "core/view/async/future.h"

namespace imp {

class BaseView;

// Base class for all components. Components can be attached to a Node to
// compose pieces of functionality together to make a Node act as needed for a
// given use case.
//
// In the subclass, you can optionally define some public methods that will be
// automatically called under certain conditions. These are not virtual methods.
// They are called based on inspecting the type at compile time.
// The methods are as follows:
//
// Setup(...)
// Use instead of defining a constructor, which allows you to signal failure, do
// asynchronous initialization work, or call virtual methods.
// (broken link)
//
// The Setup method will automatically be called when using Node.AddComponent.
// You can define overloads that take any arguments.
// You can pass the arguments into Node.AddComponent.
// You can also declare an async Setup method by returning Future<absl::Status>,
//   in which case, Node::AddComponent is also async. See Node::AddComponent for
//   details. If the Component is removed before the async work is complete
//   (i.e. the node is destroyed), then the future will be cancelled.
// You can also declare a Setup method that returns an absl::Status if setting
//   up the component can fail. In this case, Node::AddComponent will return an
//   absl::StatusOr<ComponentHandle<T>>. If Setup returns a failure status, the
//   component will be automatically removed.
// You can also name the method SetupWithState to define a Setup method that can
//   only be called when loading a scene from the SceneSystem or when calling
//   AddComponentWithState. This is useful if you want to have the Isf version
//   of Setup have a different return type from the programmatic version.
//
//
// Cleanup()
// Called just before a Component is removed while GetNode() is still available.
// Use instead of defining a destructor.
//
//
// Update(const FrameTime& frame_time)
// Called once each frame. Information about the time elapsed since the previous
// frame, and the time elapsed since the view was created is passed in. See
// below for more information about the order Update is called in.
//
// Also, Update is only called when a component is active by default. This
// behavior can be changed by doing the following in the declaration of a
// component subclass:
//   static constexpr UpdateMode kUpdateMode = UpdateMode::kUpdateWhileActive;
//
//
// OnActiveStatusChanged
// Called whenever the active state of the Component changes.
//   A Component is active if it is enabled and its node is active.
//
//   Components become enabled after Setup finishes, so typically this method is
//   called immediately after Setup finishes unless it's attached to an inactive
//   node or the component was disabled in a .isf file.
//
//   Components are also disabled before Cleanup, so typically this method is
//   called just before Cleanup unless the component was already inactive.
//
//   Use this method to execute changes to the Component depending on its active
//   state.
//
// DrawEditorUi
// Called whenever drawing the editor UI for the current component. Ensure that
// the declaration and definition are guarded by the IMP_RUNTIME(DEV)
// preprocessor.
//
// Example:
// class FooComponent : Component {
//  public:
//   void Setup(int x);
//   Future<absl::Status> Setup(bool foo);
//   void Cleanup();
//   void Update(const FrameTime& frame_time);
//   void OnActiveStatusChanged(bool is_active) {
//     if (is_active) {
//       TurnOnSomething();
//     } else {
//       TurnOffSomething();
//     }
//   }
// }
// node.AddComponent<FooComponent>(/*x=*/5);
//
// // === Update Order Information ===
//
// Update is called by type-order, NOT by node order. This means that Update
// will be called for all components of type A before Update is called on any
// components of type B.
//
// The order that Update is called on components of the same type is not
// defined. This is important, because it allows us to iterate over the
// behaviors of the same type in contiguous memory.
//
// The order that component types are updated in can be controlled by specifying
// update dependencies. If no dependencies are specified, the order is not
// defined. Dependencies are not inherited between types.
//
// Example Usage:
//   third_party/impress/core/ncsb/component_test.cc
//
// class Foo : public Component {
//   public:
//    // Specifies that Foo components won't update until after Bar components.
//    using UpdateDependencies = UpdateIds<Bar>;
//
//    // Specifies that Bazz components won't update until after Foo components.
//    using UpdateDependents = UpdateIds<Bazz>;
// };
//
// // === Cleanup Order Information ===
//
// When destroying a node, components are removed first by type-order, then by
// depth-first node order. This means that Cleanup will be called for all
// components of type A before Cleanup is called on any components of type B
// for the set of nodes being destroyed (The node passed into DestroyNode and
// all of its children).
//
// The order that component types are removed in can be controlled by specifying
// cleanup dependencies when defining the Component. If no dependencies are
// specified, the order is not defined.
//
// Example Usage:
//   third_party/impress/core/ncsb/component_test.cc
//
// class Example : public Component {
//   public:
//    // Example components won't be removed until after Foo components.
//    using CleanupDependencies = CleanupIds<Foo>;
//
//    // Bar components won't be removed until after Example components.
//    using CleanupDependents = CleanupIds<Bar>;
// };
//
class Component {
 public:
  // Used to determine rules for when a component's update method is called
  enum class UpdateMode {
    // Makes it so a type of component is only updated while the component is
    // active. This is the default behavior.
    kUpdateWhileActive,
    // Makes it so a type of component is always updated regardless of if it is
    // active or not. The current active status can still be checked within
    // Update by calling IsActive.
    kAlwaysUpdate
  };

  // Default UpdateMode setting.
  // This can be changed in Component subclasses by doing the following:
  //
  // class Foo : Component {
  //  public:
  //   static constexpr UpdateMode kUpdateMode = UpdateMode::kAlwaysUpdate;
  // };
  static constexpr UpdateMode kUpdateMode = UpdateMode::kUpdateWhileActive;

  // Default UpdatePhase setting.
  // Used to control the order of component updates in a coarse way.
  // UpdateDependencies & UpdateDependents can be used to control the order of
  // component updates in a fine-grained way.
  //
  // This can be changed in Component subclasses by doing the following:
  //
  // class Foo : Component {
  //  public:
  //   static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEarly;
  // };
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kDefault;

  // Indicates if a Component should be excluded from the editor. If true, then
  // the component won't show up in the Node details panel and can't be added
  // via the component library UI.
  //
  // This can be enabled in Component subclasses by doing the following:
  // class Foo : Component {
  //  public:
  //   static constexpr bool kExcludeFromEditor = true;
  // };
  static constexpr bool kExcludeFromEditor = false;

  // Indicates that a Component should run (i.e. call Setup, Update,
  // OnActiveStatusChanged, and Cleanup) even when in EditMode.
  //
  // EditMode is a feature of the Impress Editor Sandbox that can be used to
  // construct scenes. When using the Impress Sandbox, the editor starts in
  // EditMode. Components will only run once the Play button is pressed unless
  // kRunInEditMode is set to true.
  //
  // In general, a component should use this when it is necessary for the
  // component to render while editing the scene so that it can be laid out
  // in a WYSIWYG manner. For instance, GltfRenderer and MeshRenderer.
  //
  // However, this is usually undesirable for components that perform UX logic
  // that should only run when the scene is playing to prevent the component
  // from modifying the state that gets saved out.
  //
  // This can be enabled in Component subclasses by doing the following:
  // class Foo : Component {
  //  public:
  //   static constexpr bool kRunInEditMode = true;
  // };
  static constexpr bool kRunInEditMode = false;

  // The result returned when adding a component based on the type of component
  // and which overload of Setup is called.
  //
  // If Setup is synchronous (returns void) or no Setup is declared, then the
  // result is ComponentHandle<T>.
  //
  // If Setup is asynchronous (returns Future<absl::Status>), then the result is
  // Future<ComponentHandle<T>>.
  template <typename T, typename... Args>
  using AddResult = component_traits::AddResult<T, Args...>;

  // Similar to AddResult, but with one important difference.
  //
  // If the component defines a SetupWithState overload that matches the args,
  // then that method determines the result.
  //
  // If no SetupWithState overload is defined, then this falls back to the
  // regular Setup methods.
  template <typename T, typename... Args>
  using AddWithStateResult = component_traits::AddWithStateResult<T, Args...>;

  // You should not do work in the constructor or define a non-default
  // constructor, instead you should implement a Setup() method.
  Component();

  // Sets this component to enabled/disabled. If this causes the component's
  // active state to change, OnActiveStatusChanged will then be called.
  void SetEnabled(bool enabled);

  // Returns if this component is enabled.
  //
  // Components starts out disabled during their Setup method.
  // They are automatically enabled after Setup completes.
  //
  // If a component is added via a .isf file and marked as disabled, then it
  // won't be automatically enabled after Setup.
  //
  // Components can be manually disabled and re-enabled by calling SetEnabled.
  //
  // Components are also automatically disabled before Cleanup when they are
  // removed.
  bool IsEnabled() const;

  // Returns if this component is active. A component is only active if it is
  // enabled and its node is active.
  bool IsActive() const;

  // Returns if this component is being removed.
  bool IsRemoving() const;

#if IMP_RUNTIME(DEV)
  // Returns if this component is attached to a node that belongs to background
  // staging of the Editor, which should always still run even when the Editor
  // is not in play mode.
  bool IsEditorStaging() const;
#endif

  // Called when the component is first created to pass in the underlying node
  // that this component is attached to.
  void PostCreated(NodeHandle node, ComponentKey key, BaseComponentPool& pool);

  // Called after PostCreated.  Subclasses may provide their own
  // implementations.  Subclass implementations may take arguments.
  void Setup() {}

  // Called when removing a component.  Subclasses may provide their own
  // implementations and should use this instead of a destructor.
  void Cleanup() {}

  // Notifies the component when its active state changes, allowing the
  // subclasses to make changes to itself according to the active state if
  // needed. This method is called automatically whenever the active state is
  // updated.
  void OnActiveStatusChanged(bool is_active) {}

  // Called when the state proto for this component is changed.
  // Components should use this hook for all logic that uses data in their ISF
  // state so that the Component can be modified with behavior consistent with
  // the original Setup() behavior, i.e. it is recommended that this method is
  // called from Setup() so the logic of initialization and modification is
  // shared.
  void OnIsfStateChanged() {}

#if IMP_RUNTIME(DEV)
  // Called when drawing the editor UI for a component.  Subclasses may provide
  // their own implementations to draw custom editor UI.
  void DrawEditorUi() {}
#endif  // IMP_RUNTIME(DEV)

  // Returns the Node that this component is attached to. This is always valid.
  NodeHandle GetNode() const;

  // Returns the View that the Node this component is attached to is part of.
  BaseView& GetView() const;

  // Returns the underlying filament entity of the Node that this component is
  // attached to. Do not use this API unless you understand the underlying
  // details of filament.
  utils::Entity GetEntity() const;

  // Gets the id for this type of component. Note, this method may return
  // a different result than the static method T::GetComponentId. This is
  // because GetHash() behaves polymorphically.
  ComponentId GetComponentId() const;

  // Returns true if the component's Setup method returned a future that has
  // not yet completed.
  inline bool IsRunningAsyncSetup() const {
    return CheckBit(status_flags_, StatusFlags::kComponentIsRunningAsyncSetup);
  }

  // Returns the key for this component used by the underlying PoolAllocator.
  inline ComponentKey GetComponentKey() const { return key_; }

  // Associates the lifetime of the holdable with the component.
  // Part of the Remember protocol, which makes it possible to pass a component
  // into Future::KeptBy or as an owner to Dispatcher::Connect.
  // See third_party/impress/core/common/rememberer.h for details.
  Invocable<void()> Remember(Holdable holdable);

  // Calls Dispatcher::Send() with this component's node as the target. The
  // |EventType| object must inherit from imp::Event. See dispatcher.h for more
  // details.
  //
  // Returns: The last NodeHandle that the |event| was sent to, or the null
  // NodeHandle for global scope.
  template <typename EventType>
  NodeHandle Send(const EventType& event);

  // Calls Dispatcher::Connect() with this component's node as the target and
  // this component as the owner.  The handler can return a
  // Dispatcher::PropagationResult or void, which defaults to
  // Dispatcher::kContinue. See dispatcher.h for more details.
  //
  // Returns: Connection which can be used to disconnect the function directly.
  template <typename Fn>
  imp::Dispatcher::Connection Connect(Fn&& handler);

  // Same as above, but with a separate owner to control the lifetime of the
  // connection. It is also disconnected when this component's node is
  // destroyed because that is the target.
  //
  // Returns: Connection which can be used to disconnect the function directly.
  template <typename Owner, typename Fn,
            std::enable_if_t<
                std::is_constructible<ConnectionOwner, Owner>::value, int> = 0>
  Dispatcher::Connection Connect(Fn&& handler, Owner owner);

  // Returns a ComponentHandle<T> that wraps the raw component pointer passed
  // in. This method works polymorphically. i.e. if T is SuperComponent, but the
  // actual object is SubComponent, then a valid ComponentHandle<SuperComponent>
  // will be returned that wraps the SubComponent object.
  template <typename T>
  static ComponentHandle<T> GetHandle(const T* component);

  BaseComponentPool& GetBaseComponentPool() const;

 private:
  // Flags used to track if a Component is enabled.
  // It also is used to cache if a component is active so that it
  // can be checked without accessing the node.
  // This is intentionally not a scoped enumeration so that it works with
  // the BitFlag API.
  enum StatusFlags : BitFlag {
    kComponentInitialFlags = 0,
    kComponentIsEnabled = 1 << 0,
    kComponentIsActive = 1 << 1,
    // This flag is set when the component's Setup method returned a future that
    // has not yet completed.
    kComponentIsRunningAsyncSetup = 1 << 2,
    // This flag is set when the component is being removed.
    kComponentIsBeingRemoved = 1 << 3
  };

  Dispatcher& GetDispatcher() const;

  void SetActiveFlagInternal(bool active);

  void SetRunningAsyncSetupFlagInternal(bool is_running_async_setup);

  void SetRemovingFlagInternal(bool is_removing);

  NodeHandle node_;
  ComponentKey key_;
  BaseComponentPool* pool_ = nullptr;
  BitFlag status_flags_ = 0;

  template <typename T>
  friend class ComponentPool;
  friend class BaseComponentPool;
};

template <typename EventType>
NodeHandle Component::Send(const EventType& event) {
  return GetDispatcher().Send(GetNode(), std::forward<const EventType>(event));
}

template <typename Fn>
imp::Dispatcher::Connection Component::Connect(Fn&& handler) {
  return Connect(std::forward<Fn>(handler), this);
}

template <
    typename Owner, typename Fn,
    std::enable_if_t<std::is_constructible<ConnectionOwner, Owner>::value, int>>
Dispatcher::Connection Component::Connect(Fn&& handler, Owner owner) {
  return GetDispatcher().Connect(GetNode(), std::forward<Fn>(handler),
                                 std::forward<Owner>(owner));
}

template <typename T>
ComponentHandle<T> Component::GetHandle(const T* component) {
  // TODO: Handle const-correctness correctly with the
  // ComponentHandle type.
  return ComponentHandle<T>(const_cast<T&>(*component));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_H_
