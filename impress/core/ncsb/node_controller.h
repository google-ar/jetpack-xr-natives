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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_CONROLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_CONROLLER_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/bit_flag.h"
#include "core/common/hash.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"
#include "core/common/rememberer.h"
#include "core/config.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::imp_internal {

// Controller that is used to implement functionality that all nodes support
// like enabled/active, groups, name, etc.
//
// Each node is a filament Entity that has a NodeController associated with it.
//
// Since a Node is primarily just an entity, it can't contain any state outside
// of this controller or the components attached to it.
class NodeController {
 public:
  NodeController(BaseView* view, utils::Entity entity, std::size_t index);

  // Called after the NodeController is created and tracked by the
  // NodeAttachmentManager.
  //
  // This must be separate from the constructor to avoid
  // ordering problems caused by work being done prior to or during insertion
  // into NodeAttachmentManager's tracking data structures.
  void PostCreated();

  // Called just before the NodeController is removed from the
  // NodeAttachmentManager's tracking data structures and then destroyed.
  //
  // This must be separate from the destructor to avoid
  // ordering problems caused by work being done after removal from
  // NodeAttachmentManager's tracking data structures.
  void PreDestroyed();

  void SetEnabled(bool enabled);

  bool IsEnabled() const;

  bool IsActive() const;

  bool IsRoot() const;

#if IMP_RUNTIME(DEV)
  // Sets if this node is considered to be part of the Editor staging.
  // This will be allow the node to update even when the Editor is not in play
  // mode.
  //
  // Note: Should only be called on Editor staging root, besides within the
  // NodeController.
  //
  // Note: All descendants of a Editor staging node will also be Editor staging.
  // Try to unset this flag on a child node of a Editor staging node will not
  // work.
  void SetAsEditorStaging(bool is_editor_staging);

  bool IsEditorStaging() const;
#endif

  inline BaseView& GetView() const { return *view_; }

  inline NodeHandle GetNode() const { return node_; }

  inline utils::Entity GetEntity() const { return node_.GetEntity(); }

  // Returns the index of this NodeController in the list of NodeControllers
  // stored by the View.
  //
  // This is stored in the NodeController itself to avoid an extra map lookup to
  // get the index when nodes are removed in NodeAttachmentManager.
  std::size_t GetIndex() const;

  // Sets the index representing this NodeController in the list of
  // NodeControllers stored by the View.
  void SetIndex(std::size_t index);

  BitFlag GetFlags() const;

  void OnParentChanged();

  void UpdateActive();

  void SetGroups(
      std::optional<absl::Span<const absl::string_view>> group_names);

  void SetGroupsSelf(absl::Span<const absl::string_view> group_names,
                     bool is_inherited);

  // Adds the node and its children to the group.
  void AddToGroup(absl::string_view group_name);

  // Removes the node and its children from the group.
  void RemoveFromGroup(absl::string_view group_name);

  std::vector<std::string> GetGroups() const;

  bool IsInGroup(absl::string_view group_name) const;

  void SetLocalRotation(const quatf& rotation);
  const quatf& GetLocalRotation() const;
  void SetLocalScale(const float3& scale);
  const float3& GetLocalScale() const;

  absl::string_view GetName() const;

  void SetName(absl::string_view name);

  Invocable<void()> Remember(Holdable holdable);

 private:
  bool CheckParentActive();

  void AddToGroupSelf(absl::string_view group_name);

  void RemoveFromGroupSelf(absl::string_view group_name);

  // Called when the groups change. This is used to notify external systems
  // (such as the SplitEngineSerializer) that the groups have changed.
  void OnGroupsChanged();

  void UpdateActiveRecursive(bool is_parent_active);

  void UpdateActiveSelf(bool is_parent_active);

#if IMP_RUNTIME(DEV)
  bool IsParentEditorStaging() const;
#endif

  // Propagate inherited groups to children, not including self.
  void PropagateInheritedGroupsRecursive();

  // Propagate inherited groups to children, including self if self is not
  // overridden.
  void UpdateInheritedGroupsRecursive();

  // Helps detect if the groups in this component match the groups in
  // another instance of this component. Used to compare this component's groups
  // to its parents.
  bool DoGroupsMatch(NodeController& other_node_controller);

  void UpdateInheritedGroupsSelf();

  // Returns the current explicit groups, or Main group if null (Implicit Main).
  // This internal helper is for read-only access to the storage.
  const std::vector<HashValue>& GetGroupHashes() const;

  // Returns mutable reference to groups, allocating if necessary (copying
  // Main).
  std::vector<HashValue>& MutableGroups();

  BaseView* view_;

  NodeHandle node_;

  // This is the index of this NodeController in the list of NodeControllers
  // stored by the View.
  std::size_t index_;

  // TODO: Explore generalized API for hereditary values.
  BitFlag flags_;

  // If group_hashes_ is null, then this node is part of the main
  // layer: GroupsManager::kMainSceneName
  //
  // Otherwise, group_hashes_ contains the sorted unique hash values for the
  // layers that this node is part of.
  //
  // This uses a vector for performance (contiguous memory, fast iteration) and
  // simplicity. It is kept sorted to allow efficient set operations.
  //
  // This would be simpler to implement as a plain set<string>, but this data
  // structure is being used to minimize the impact of layers on memory usage.
  //
  // This way, for nodes that are only part of the default main layer, the only
  // memory overhead is a single pointer. for nodes that are part of custom
  // layers, hash values is less memory than a set of strings.
  std::unique_ptr<std::vector<HashValue>> group_hashes_;

  quatf local_rotation_ = kIdentityQuatf;
  float3 local_scale_ = kOne3;

  std::string name_;

  std::optional<Rememberer> rememberer_;
};

}  // namespace imp::imp_internal

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_CONROLLER_H_
