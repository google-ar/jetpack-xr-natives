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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_SYSTEM_H_

#include "core/ncsb/component_manager.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// A ComponentSystem is a System associated with a particular type of Component
// that receives lifecycle methods related to the Component.
//
// The ComponentSystem will be created when the first component of type T is
// added (or it's explicitly created via the ComponentManager). It will then
// persist for the lifetime of the Impress View.
//
// This is useful for use cases where one needs to do the following:
//   1. Share fields between all instances of a component.
//   2. Perform batch updates that requires access to all component instances.
//   3. Provide API that acts on or accesses all component instances.
//
// Best Practice:
//
// Since ComponentSystem's are long-lived, they are best used to implement core
// functionality of a component, not to manage the behavior of groups of
// components that are specific to a current scene, since that could lead to
// bugs with cleaning up and maintaining the fields of the system as the scene
// changes. In those cases, it is likely preferable to use a Component to manage
// the behavior, which also allows for more granular grouping than simply by
// component type.
//
// When adding fields to a ComponentSystem, think carefully about
// the lifetime of those fields. Use BeforeFirstComponentAdded and
// AfterLastComponentRemoved to create & cleanup state.
//
// How To Use:
//
// To use a ComponentSystem, it must be declared as an inner class named
// "System" in a component.
//
// class Foo : public Component {
//   public:
//     class System : public ComponentSystem<Foo> { ... };
// };
//
// This will cause the ComponentManager to automatically find the type of
// System, create it, and call its lifecycle methods.
//
// The ComponentSystem can be accessed via the ComponentManager:
//
// GetView().GetComponentManager().GetComponentSystem<Foo>();
//
// BaseView* is always passed into the ComponentSystem's constructor as the
// first parameter. ComponentSystem's can also be created explicitly with
// additional parameters like this:
//
// class System : public ComponentSystem<Foo> {
//   public:
//     System(BaseView* view, int val);
// };
//
// GetView().GetComponentManager().CreateComponentSystem<Foo>(10);
//
template <typename T>
class ComponentSystem : public System {
 public:
  explicit ComponentSystem(BaseView* view);

  // Called prior to Component::Setup when the first component of type T is
  // added.
  //
  // If all added components have been removed, then this will be called again
  // the next time a component of type T is added.
  virtual void BeforeFirstComponentAdded() {}

  // Called every time after a component has been added.
  // If this is the first component, called after BeforeFirstComponentAdded.
  virtual void AfterComponentAdded(T& component) {}

  // Called every time a component is removed.
  // If this is the last component, called before AfterLastComponentRemoved.
  virtual void BeforeComponentRemoved(T& component) {}

  // Called after Component::Cleanup when the last component of type T is
  // removed.
  //
  // If components are added again after all of them have been removed, then
  // this will be called again the next time all the components are removed.
  virtual void AfterLastComponentRemoved() {}

  // Called just before all components of component type T are updated.
  //
  // This respects the update dependencies specified for component type T.
  virtual void PreComponentsUpdated(const FrameTime& frame_time) {}

  // Called just after all components of component type T are updated.
  //
  // This respects the update dependencies specified for component type T.
  virtual void PostComponentsUpdated(const FrameTime& frame_time) {}

  // Helper to iterate over each component of type T.
  //
  // Example:
  //
  // ForEachComponent([](MyComponent* my_component) { });
  template <typename Fn>
  void ForEachComponent(Fn&& fn);
};

template <typename T>
ComponentSystem<T>::ComponentSystem(BaseView* view) : System(view) {}

template <typename T>
template <typename Fn>
void ComponentSystem<T>::ForEachComponent(Fn&& fn) {
  GetComponentManager().template ForEach<T>(std::forward<Fn>(fn));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_SYSTEM_H_
