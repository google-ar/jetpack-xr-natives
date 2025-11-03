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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_H_

#include <cstddef>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/TransformManager.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"
#include "core/config.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/base_node.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_manager.h"
#include "core/ncsb/dispatcher/connection_owner.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/groups_manager.h"
#include "core/ncsb/node_controller.h"
#include "core/ncsb/node_handle.h"

namespace imp {

class BaseView;

// A Node within the scene graph of an imp::View.
//
// A Node is created using imp::View::CreateNode, and its lifetime is tied
// to the lifetime of the View. It can have an arbitrary number of
// child nodes, and one parent. If it has no parent, then it is considered a
// top-level node of the scene graph which means that its local and world
// position, rotation, and scale is the same.
//
// Node's are always accessed via a NodeHandle. NodeHandle has semantics similar
// to a smart pointer, and will remain valid until imp::View::DestroyNode is
// called for this node or its ancestor.
//
// Components can be attached to a Node to compose pieces of functionality
// together to make a Node act as needed for a given use case.
// See //depot/google3/third_party/impress/core/ncsb/component.h
//
// Example Usage:
// see //depot/google3/third_party/impress/samples/simple/simple_view.cc
//
// NodeHandle node = view->CreateNode();
// node->SetParent(parent);
// node->SetLocalPosition({0, 0, -1f});
//
// // Add a component that makes the node spin.
// node->AddComponent<Spin>(180.0f);
//
// // Some time later, if desired:
// // (Not Required, node will automatically be cleaned up when the view is.)
// view->DestroyNode(node);
class Node final : public BaseNode {
 public:
  // The name of the group representing the main render pass that
  // outputs to the surface each frame.
  //
  // All nodes are part of this group by default. If a node is
  // removed from this group, it won't be visible on the main surface.
  //
  // The name is "Main".
  static constexpr absl::string_view kMainGroupName =
      GroupsManager::kMainGroupName;

  // Returns the View that this Node is part of.
  inline BaseView& GetView() const { return node_controller_->GetView(); }

  // Returns the name of the node, or the empty string if unnamed.
  absl::string_view GetName() const;

  // Sets the name of the node.
  void SetName(absl::string_view name);

  // Adds a component of type T to this node.
  //
  // If the Component has declared a Setup(...) method, then it will be called.
  // The args are forwarded to the Component's Setup method, or must be empty
  // if no Setup(...) method is declared.
  //
  // Only one component of each type is allowed. If this node already has a
  // component of type T, then the old component is removed and replaced with
  // a new one.
  //
  // Typically, this returns a ComponentHandle<T> to the added component.
  //
  // If this calls an async Setup method (one that returns
  // Future<absl::Status>), then this method returns a
  // Future<ComponentHandle<T>>. When the async work successfully finished, the
  // future returns the ComponentHandle. If the async work fails, then the
  // Component is removed and the future returns with the Status result of the
  // Setup method.
  //
  // If this calls a Setup method that returns an absl::Status, then this method
  // returns an absl::StatusOr<ComponentHandle<T>>. If the Setup method returns
  // a failure status the component is removed and the StatusOr will contain the
  // status returned by the Setup method.
  template <typename T, typename... Args>
  Component::AddResult<T, Args...> AddComponent(Args&&... args);

  // Adds a component of type T to this node.
  //
  // This behaves very similar to AddComponent with one major difference:
  // AddComponentWithState will copy the T::IsfInfo::StateT to the state field
  // of the component before trying to call Setup. Here's a quick overview of
  // what AddComponentWithState does internally:
  // 1. Creates a Component without calling setup.
  // 2. Copies the state argument over to the Component's state field.
  // 3. Calls Setup(...) if there are any matches.
  // 4. Returns a ComponentHandle, an absl::Status or a Future depending on the
  //    Setup function return type.
  //
  // This is meant to automate the dynamic setup process for Components with
  // state field so that the users do not need to manually create a Setup(State)
  // overload and copy the state data over then call the other Setup(...)
  // functions afterwards.
  //
  // AddComponentWithState also supports variadic arguments, meaning that
  // depending on the arguments passed into it, it will invoke different
  // Setup(...) functions. For example:
  // AddComponentWithState(State) will invoke Setup()
  // AddComponentWithState(State, int) will invoke Setup(int)
  // AddComponentWithState(State, double, int) will invoke Setup(double, int)
  //
  // If a SetupWithState(Args..) method is defined, it will be called instead of
  // Setup(Args..).
  template <typename T, typename... Args>
  Component::AddWithStateResult<T, Args...> AddComponentWithState(
      typename T::IsfInfo::StateT state, Args&&... args);

  // Gets the component of type T that is attached to this Node. If there is
  // none, then an invalid ComponentHandle is returned.
  template <typename T>
  ComponentHandle<T> GetComponent() const;

  // Removes the component of type T that is attached to this Node. If there is
  // none, then this method does nothing.
  //
  // When the component is removed, the method Cleanup() will be called if it
  // has been declared for type T.
  template <typename T>
  void RemoveComponent();

  // Gets the component of type T that is attached to this Node. If there is
  // none, then adds a component of type T to this Node, passing through args.
  //
  // If adding the component would be async (see AddComponent for details) then
  // this returns a Future<ComponentHandle<T>> is returned. Otherwise,
  // a ComponentHandle<T> is returned.
  //
  // Note: If the component was already added, then the args are ignored.
  template <typename T, typename... Args>
  Component::AddResult<T, Args...> GetOrAddComponent(Args&&... args);

  // Similar to GetOrAddComponent, this also tries to get the component of type
  // T that is attached to this Node and if there's none, it will add a
  // component of type T with args.
  //
  // This method also takes a state proto for setting up the component in case
  // it needs to add a new component.
  //
  //
  // If Setup(Args...) is async (see AddComponent for details) then this
  // returns a Future<ComponentHandle<T>> is returned. Otherwise, a
  // ComponentHandle<T> or a absl::StatusOr<ComponentHandle<T>> is returned.
  //
  // If a SetupWithState(Args..) method is defined, it will be called instead of
  // Setup(Args..).
  //
  // Note: If the component was already added, then the state proto and the args
  // are ignored.
  template <typename T, typename... Args>
  Component::AddWithStateResult<T, Args...> GetOrAddComponentWithState(
      typename T::IsfInfo::StateT state, Args&&... args);

  // Sets if this node is enabled.
  // A node may be enabled but still not active if its parent isn't active.
  void SetEnabled(bool enabled);

  // Returns if this node is enabled.
  // A node may be be enabled but still not active if its parent isn't active.
  bool IsEnabled() const;

  // Returns if this node is active.
  // A node is active if it's enabled and its parent is active.
  // A node is only rendered if it is active.
  bool IsActive() const;

  // Returns true if this node does not have a parent.
  bool IsRoot() const;

#if IMP_RUNTIME(DEV)
  // Sets if this node is considered to be part of the Editor staging.
  // This will be allow the node to update even when the Editor is not in play
  // mode.
  void SetAsEditorStaging(bool is_editor_staging);

  // Returns true if this node is considered to be part of the Editor staging.
  bool IsEditorStaging() const;
#endif

  // Sets the groups that this node will be included in for
  // rendering.
  //
  // Groups are automatically inherited from a node's ancestors if
  // not explicitly set.
  //
  // By default, nodes are in the Node::kMainGroupName group which is
  // used to render the main pass. Additional groups can be rendered in other
  // render passes using components like TexturePipelineRenderer.
  void SetGroups(absl::Span<const absl::string_view> group_names);

  // Same as SetGroups except set by a Vector.
  //
  // This method has a different name from SetGroups to avoid overload
  // ambiguity when using brace initialization.
  void SetGroupsFromVector(const std::vector<std::string>& group_names);

  // Adds the node to a group.
  // If the node is already in that group, this will be no-op.
  void AddToGroup(absl::string_view group_name);

  // Removed the node from a group.
  // If the node is not in that group, this will be no-op.
  void RemoveFromGroup(absl::string_view group_name);

  // Clears the  groups explicitly set on this node (if there are any)
  // and instead uses the inherited  groups.
  void ClearGroups();

  // Returns the  groups that this node is a part of. This could be
  // the set of groups explicitly set on this node or the groups inherited from
  // its ancestors.
  //
  // By default, a Node is part of the  group Node::kMainGroupName.
  //
  // By default, only the main group is rendered. To render other groups,
  // use the components TexturePipelineRenderer, or other similar components.
  std::vector<std::string> GetGroups() const;

  // Returns true if this node is in the given group.
  bool IsInGroup(absl::string_view group_name) const;

  // Changes the parent of this node. If set to an invalid NodeHandle, this node
  // will be detached from its current parent if it has one.
  //
  // If the node has no parent, then it is considered top-level. In that case,
  // the local and world position, rotation, and scale will be the same.
  //
  // The local position, rotation, and scale of the node remains the same after
  // the parent changes. Therefore, the world position, rotation, and scale may
  // become different.
  void SetParent(NodeHandle parent);

  // Changes the parent of this node. If set to an invalid NodeHandle, this node
  // will be detached from its current parent if it has one.
  //
  // If the node has no parent, then it is considered top-level. In that case,
  // the local and world position, rotation, and scale will be the same.
  //
  // The world position, rotation, and scale of the node remains the same after
  // the parent changes. Therefore, the local position, rotation, and scale may
  // become different.
  void SetParentKeepWorldTransform(NodeHandle parent);

  // Returns the parent of this node. If this node has no parent, then an
  // invalid NodeHandle is returned. In that case, this is a top-level node.
  NodeHandle GetParent() const;

  // Returns the children of this node.
  std::vector<NodeHandle> GetChildren() const;

  // Creates a node that's a child of the current node. This guarantees that the
  // node created has an identity transformation matrix relative to the parent,
  // i.e. it has the same world transform as the parent.
  NodeHandle CreateChildNode() const;

  // Sets the position of this node relative to its parent (local-space). If the
  // node is top-level, then this is the same as SetWorldPosition.
  void SetLocalPosition(const float3& position);
  // Sets the position of this node relative to its parent (local-space). If the
  // node is top-level, then this is the same as SetWorldPositionPrecise.
  // Calling this function will automatically enable precise translation mode.
  void SetLocalPositionPrecise(const double3& position);

  // Gets the position of this node relative to its parent (local-space). If the
  // node is top-level, then this is the same as SetWorldPosition.
  float3 GetLocalPosition() const;
  // Gets the position of this node relative to its parent (local-space). If the
  // node is top-level, then this is the same as SetWorldPosition.
  double3 GetLocalPositionPrecise() const;

  // Sets the rotation of this node relative to its parent (local-space). If the
  // node is top-level, then this is the same as SetWorldRotation.
  void SetLocalRotation(const quatf& rotation);

  // Gets the rotation of this node relative to its parent (local-space). If the
  // node is top-level, then this is the same as SetWorldRotation.
  quatf GetLocalRotation() const;

  // Sets the scale of this node relative to its parent (local-space). If the
  // node is top-level, then this is the same as SetWorldScale.
  void SetLocalScale(const float3& scale);

  // Gets the scale of this node relative to its parent (local-space). If the
  // node is top-level, then this is the same as SetWorldScale.
  float3 GetLocalScale() const;

  // Returns the result of rotating float3(0, 0, -1) by the local rotation from
  // this node.
  float3 GetLocalForward() const;

  // Sets forward direction of the node using local space coordinates.
  //
  // After calling this, GetLocalForward() will match look_direction. A default
  // up direction of (0, 1, 0) will be used.
  //
  // Note: look_direction should not be coincident (parallel) with the default
  // up direction or the result will be undefined.
  void SetLocalForward(const float3& look_direction);

  // Sets forward direction of the node using local space coordinates.
  //
  // After calling this, GetLocalForward() will match look_direction.
  // in. up_direction will determine the orientation of the node around the
  // direction.
  //
  // Note: look direction should not be coincident (parallel) with up direction
  // as the result will be undefined.
  void SetLocalForward(const float3& look_direction,
                       const float3& up_direction);

  // Sets the position of this node relative to the view (world-space).
  //
  // Calculates the local position required for the final position of the node
  // to be the position passed in, and assigns it as the local position.
  void SetWorldPosition(const float3& position);
  // Sets the position of this node relative to the view (world-space).
  //
  // Calculates the local position required for the final position of the node
  // to be the position passed in, and assigns it as the local position.
  //
  // Calling this function will automatically enable precise translation mode.
  void SetWorldPositionPrecise(const double3& position);

  // Gets the position of this node in the global coordinates of the view's
  // scene graph (world-space).
  //
  // The result is a combination of the position, rotation, and scale of all
  // this node's ancestors with the local position of this node.
  float3 GetWorldPosition() const;
  // Gets the position of this node in the global coordinates of the view's
  // scene graph (world-space).
  //
  // The result is a combination of the position, rotation, and scale of all
  // this node's ancestors with the local position of this node.
  double3 GetWorldPositionPrecise() const;

  // Sets the rotation of this node in the global coordinates of the view's
  // scene graph (world-space).
  //
  // Calculates the local rotation required for the final rotation of the node
  // to be the rotation passed in, and assigns it as the local rotation.
  void SetWorldRotation(const quatf& rotation);

  // Gets the rotation of this node in the global coordinates of the view's
  // scene graph (world-space).
  //
  // The result is a combination of the position, rotation, and scale of all
  // this node's ancestors with the local rotation of this node.
  quatf GetWorldRotation() const;

  // Sets the scale of this node in the global coordinates of the view's scene
  // graph (world-space).
  //
  // Calculates the local scale required for the final scale of the node
  // to be the scale passed in, and assigns it as the local scale.
  void SetWorldScale(const float3& scale);

  // Gets the scale of this node in the global coordinates of the view's scene
  // graph (world-space).
  //
  // The result is a combination of the position, rotation, and scale of all
  // this node's ancestors with the local scale of this node.
  //
  // Note: The return scale is meaningless, when the node has non-trivial
  // rotation and one or more of the ancestors has non-uniform scale, because
  // the node is skewed and cannot be decomposed into translation, rotation and
  // scale.
  float3 GetWorldScale() const;

  // Returns the result of rotating float3(0, 0, -1) by the world rotation from
  // this node.
  float3 GetWorldForward() const;

  // Sets forward direction of the node using world space coordinates.
  //
  // After calling this, GetWorldForward() will match look_direction. A default
  // up direction of (0, 1, 0) will be used.
  //
  // Note 1: look_direction should not be coincident (parallel) with the default
  // up direction or the result may be undefined.
  // Note 2: Using non-normalized look_direction will result in scaling the
  // node.
  void SetWorldForward(const float3& look_direction);

  // Sets forward direction of the node using world space coordinates.
  //
  // After calling this, GetWorldForward() will match look_direction.
  // in. up_direction will determine the orientation of the node around the
  // direction. The look direction and up direction cannot be coincident
  // (parallel) or the result may be undefined.
  void SetWorldForward(const float3& look_direction,
                       const float3& up_direction);

  // Converts a point from the local-space of this node to world-space.
  float3 WorldFromLocalPoint(const float3& position) const;

  // Converts a point from the local-space of this node to world-space.
  double3 WorldFromLocalPointPrecise(const double3& position) const;

  // Transform a vector from local-space to the world-space of this node.
  float3 WorldFromLocalVector(const float3& vector) const;

  // Transform a vector from local-space to the world-space of this node.
  double3 WorldFromLocalVectorPrecise(const double3& vector) const;

  // Converts a point from world-space to the local-space of this node.
  float3 LocalFromWorldPoint(const float3& position) const;

  // Converts a point from world-space to the local-space of this node.
  double3 LocalFromWorldPointPrecise(const double3& position) const;

  // Transform a vector from world-space to the local-space of this node.
  float3 LocalFromWorldVector(const float3& vector) const;

  // Transform a vector from world-space to the local-space of this node.
  double3 LocalFromWorldVectorPrecise(const double3& vector) const;

  // Nodes can only be created as value types.
  void* operator new(size_t size) = delete;
  void* operator new[](size_t size) = delete;
  void operator delete(void* p) = delete;
  void operator delete[](void* p) = delete;

  // Associates the holdable with the lifetime of the node.
  // Part of the Remember protocol, which makes it possible to pass a component
  // into Future::KeptBy or as an owner to Dispatcher::Connect.
  // See third_party/impress/core/common/rememberer.h for details.
  Invocable<void()> Remember(Holdable holdable);

  // Get local transform (in Transform<float>) of the node.
  Transform<float> GetLocalTransform() const;
  // Set local transform (in Transform<float>) of the node.
  void SetLocalTransform(const Transform<float>& transform);
  // Get local transform (in Transform<double>) of the node with precise
  // translation.
  // TODO: (broken link) - Switch return type to PreciseTransform, since we only
  // need precision for translation.
  Transform<double> GetLocalTransformPrecise() const;
  // Set local transform (in Transform<double>) of the node with precise
  // translation. Calling this function will automatically enable precise
  // translation mode.
  void SetLocalTransformPrecise(const Transform<double>& transform);

  // Get local transform (in mat4f) of the node.
  // Use GetLocalTransform() if you need to access the rotation or scale. Don't
  // convert the mat4f to a Transform<float> because it will lose precision.
  const mat4f& GetLocalTrs() const;
  // Get local transform (in mat4) of the node with precise translation.
  // Use GetLocalTransformPrecise() if you need to access the rotation or scale.
  // Don't convert the mat4 to a Transform<double> because it will lose
  // precision.
  mat4 GetLocalTrsPrecise() const;
  // Set local transform (in mat4f) of the node.
  // Use SetLocalTransform() if the data comes from a Transform<float>. Don't
  // convert the Transform<float> to a mat4f because it will lose precision.
  void SetLocalTrs(const mat4f& trs);
  // Set local transform (in mat4) of the node.
  // Use SetLocalTransformPrecise() if the data comes from a Transform<double>.
  // Don't convert the Transform<double> to a mat4 because it will lose
  // precision.
  // Calling this function will automatically enable precise translation mode.
  void SetLocalTrsPrecise(const mat4& trs);

  // Get world transform of the node.
  const mat4f& GetWorldTrs() const;
  // Get world transform of the node with precise translation.
  mat4 GetWorldTrsPrecise() const;
  // Set world transform of the node.
  void SetWorldTrs(const mat4f& trs);
  // Set world transform of the node.
  // Calling this function will automatically enable precise translation mode.
  void SetWorldTrsPrecise(const mat4& trs);

  // Calls Dispatcher::Send() with this node as the target. The |EventType|
  // object must inherit from imp::Event. See dispatcher.h for more details.
  //
  // Returns: The last NodeHandle that the |event| was sent to, or the null
  // NodeHandle for global scope.
  template <typename EventType>
  NodeHandle Send(const EventType& event);

  // Calls Dispatcher::Connect() with this node as the target and owner. The
  // handler can return a Dispatcher::PropagationResult or void, which defaults
  // to Dispatcher::kContinue. See dispatcher.h for more details.
  //
  // Returns: Connection which can be used to disconnect the function directly.
  template <typename Fn>
  Dispatcher::Connection Connect(Fn&& handler);

  // Same as above, but with a separate owner to control the lifetime of the
  // connection. It is also disconnected when this node is destroyed because
  // this is the target.
  //
  // Returns: Connection which can be used to disconnect the function directly.
  template <typename Owner, typename Fn,
            std::enable_if_t<
                std::is_constructible<ConnectionOwner, Owner>::value, int> = 0>
  Dispatcher::Connection Connect(Fn&& handler, Owner owner);

  // Searches this node and all its descendants (children, grandchildren, etc.)
  // for node with a matching name.
  //
  // Has O(N) runtime, this is provided for convenience.
  // Returns a NodeHandle with the matching name or nothing.
  NodeHandle FindByName(absl::string_view name);

 private:
  // Pass into Node::SetParent to determine what should happen to the node's
  // transform when the parent changes.
  enum class SetParentMode {
    // Default value. The local position, rotation, and scale will remain the
    // same, the world position, rotation, and scale will change to be relative
    // to the new parent.
    kKeepLocalTransform,
    // The world position, rotation, and scale will remain the same by changing
    // the local position, rotation, and scale to be relative to the new parent.
    kKeepWorldTransform,
  };

  // Node is never created or destroyed, only static_casted from BaseNode.
  Node() = delete;
  ~Node() = delete;

  void SetParentInternal(NodeHandle parent, SetParentMode set_parent_mode);
  // Internal calls to set transforms should always go through these methods.
  // They serialize Node transforms via SplitEngine if it is active.

  void SetFilamentTransformInternal(const Transform<float>& transform);
  void SetFilamentTransformInternal(const Transform<double>& transform);
  void SetFilamentTransformInternal(
      ::filament::TransformManager::Instance instance, const mat4f& transform);
  void SetFilamentTransformInternal(
      ::filament::TransformManager::Instance instance, const mat4& transform);

  inline filament::Engine* GetEngine() const {
    return BaseView::GetSharedEngine();
  }

  inline filament::TransformManager& GetTransformManager() const {
    return GetEngine()->getTransformManager();
  }

  inline ComponentManager& GetComponentManager() const {
    return GetView().GetComponentManager();
  }

  inline Dispatcher& GetDispatcher() const { return GetView().GetDispatcher(); }

  friend class NodeHandle;
};

template <typename T, typename... Args>
Component::AddResult<T, Args...> Node::AddComponent(Args&&... args) {
  return GetComponentManager().Add<T>(NodeHandle(*this),
                                      std::forward<Args>(args)...);
}

template <typename T, typename... Args>
Component::AddWithStateResult<T, Args...> Node::AddComponentWithState(
    typename T::IsfInfo::StateT state, Args&&... args) {
  return GetComponentManager().AddWithState<T>(
      NodeHandle(*this), std::move(state), std::forward<Args>(args)...);
}

template <typename T>
ComponentHandle<T> Node::GetComponent() const {
  return GetComponentManager().Get<T>(entity_);
}

template <typename T>
void Node::RemoveComponent() {
  GetComponentManager().Remove<T>(entity_);
}

template <typename T, typename... Args>
Component::AddResult<T, Args...> Node::GetOrAddComponent(Args&&... args) {
  return GetComponentManager().GetOrAdd<T>(NodeHandle(*this),
                                           std::forward<Args>(args)...);
}

template <typename T, typename... Args>
Component::AddWithStateResult<T, Args...> Node::GetOrAddComponentWithState(
    typename T::IsfInfo::StateT state, Args&&... args) {
  return GetComponentManager().GetOrAddWithState<T>(
      NodeHandle(*this), std::move(state), std::forward<Args>(args)...);
}

template <typename EventType>
NodeHandle Node::Send(const EventType& event) {
  return GetDispatcher().Send(NodeHandle(*this),
                              std::forward<const EventType>(event));
}

template <typename Fn>
Dispatcher::Connection Node::Connect(Fn&& handler) {
  NodeHandle handle(*this);
  return Connect(std::forward<Fn>(handler), handle);
}

template <
    typename Owner, typename Fn,
    std::enable_if_t<std::is_constructible<ConnectionOwner, Owner>::value, int>>
Dispatcher::Connection Node::Connect(Fn&& handler, Owner owner) {
  NodeHandle handle(*this);
  return GetDispatcher().Connect(handle, std::forward<Fn>(handler),
                                 std::forward<Owner>(owner));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_H_
