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

#include "core/ncsb/node_controller.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/fixed_array.h"
#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/bit_flag.h"
#include "core/common/hash.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"
#include "core/config.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/groups_manager.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/path_manager.h"
#include "core/view/base_view.h"

namespace imp::imp_internal {

NodeController::NodeController(BaseView* view, utils::Entity entity,
                               std::size_t index)
    : view_(view),
      node_(entity, this),
      index_(index),
      flags_(NodeFlags::kIsEnabled) {}

void NodeController::PostCreated() {
  view_->GetGroupsManager().AddNodeToGroup(
      GroupsManager::kMainGroupName, GroupsManager::kMainGroupHash, GetNode());
  UpdateActiveSelf(CheckParentActive());
#if IMP_RUNTIME(DEV)
  SetAsEditorStaging(IsParentEditorStaging());
#endif
}

void NodeController::PreDestroyed() {
  NodeHandle node = GetNode();

  // Clear the remembered objects before doing anything else during destruction.
  // This is to ensure that the node is still valid when the remembered objects
  // are destroyed.
  rememberer_.ClearRemembered();

  // It's possible that the node was destroyed when ClearRemembered was running.
  // In that case, we don't need to do anything.
  if (!node) {
    return;
  }

  if (!group_hashes_) {
    // If group_hashes_, just in the main group.
    view_->GetGroupsManager().RemoveNodeFromGroup(GroupsManager::kMainGroupHash,
                                                  node);
  } else {
    // Remove from all the groups.
    for (HashValue group_hash : *group_hashes_) {
      view_->GetGroupsManager().RemoveNodeFromGroup(group_hash, node);
    }
  }
  view_->GetPathManager().SetRoot(node, false);
}

void NodeController::SetEnabled(bool enabled) {
  if (enabled == IsEnabled()) {
    return;
  }
  flags_ = SetBitFromBool(flags_, NodeFlags::kIsEnabled, enabled);
  UpdateActive();
}

bool NodeController::IsEnabled() const {
  return CheckBit(flags_, imp::NodeFlags::kIsEnabled);
}

bool NodeController::IsRoot() const {
  return CheckBit(flags_, imp::NodeFlags::kIsRoot);
}

bool NodeController::IsActive() const {
  return CheckBit(flags_, imp::NodeFlags::kIsActive);
}

#if IMP_RUNTIME(DEV)
bool NodeController::IsEditorStaging() const {
  return CheckBit(flags_, imp::NodeFlags::kIsEditorStaging);
}

void NodeController::SetAsEditorStaging(bool is_editor_staging) {
  if (IsParentEditorStaging()) {
    if (!is_editor_staging) {
      IMP_LOG(imp::WARNING) << "Cannot remove node from editor staging because parent "
                      "is editor staging.";
      return;
    }
  }
  flags_ = SetBitFromBool(flags_, imp::NodeFlags::kIsEditorStaging,
                          is_editor_staging);
  for (auto child : GetNode()->GetChildren()) {
    child->node_controller_->SetAsEditorStaging(is_editor_staging);
  }
}

bool NodeController::IsParentEditorStaging() const {
  NodeHandle parent = GetNode()->GetParent();
  if (parent.IsValid()) {
    return parent->node_controller_->IsEditorStaging();
  }
  return false;
}
#endif

BaseView& NodeController::GetView() const { return *view_; }

NodeHandle NodeController::GetNode() const { return node_; }

utils::Entity NodeController::GetEntity() const { return node_.GetEntity(); }

std::size_t NodeController::GetIndex() const { return index_; }

void NodeController::SetIndex(std::size_t index) { index_ = index; }

BitFlag NodeController::GetFlags() const { return flags_; }

void NodeController::SetLocalRotation(const quatf& rotation) {
  local_rotation_ = rotation;
}

const quatf& NodeController::GetLocalRotation() const {
  return local_rotation_;
}

void NodeController::SetLocalScale(const float3& scale) {
  local_scale_ = scale;
}

const float3& NodeController::GetLocalScale() const { return local_scale_; }

absl::string_view NodeController::GetName() const { return name_; }

void NodeController::SetName(absl::string_view name) {
  name_ = std::string(name);
}

imp::Invocable<void()> NodeController::Remember(imp::Holdable holdable) {
  return rememberer_.Remember(std::move(holdable));
}

void NodeController::OnParentChanged() {
  UpdateInheritedGroupsRecursive();
  UpdateActive();
#if IMP_RUNTIME(DEV)
  SetAsEditorStaging(IsParentEditorStaging());
#endif
}

void NodeController::UpdateActive() {
  UpdateActiveRecursive(CheckParentActive());
}

void NodeController::AddToGroup(absl::string_view group_name) {
  AddToGroupSelf(group_name);
  PropagateInheritedGroupsRecursive();
}

void NodeController::AddToGroupSelf(absl::string_view group_name) {
  if (group_name == GroupsManager::kMainGroupName && !group_hashes_) {
    // If group_hashes_ is unset, then already in the main group.
    return;
  }

  HashValue group_hash = Hash(group_name);
  if (!group_hashes_) {
    group_hashes_ = std::make_unique<absl::flat_hash_set<HashValue>>();
    group_hashes_->insert(GroupsManager::kMainGroupHash);
  } else if (group_hashes_->count(group_hash) > 0) {
    // This node is already present in this group.
    return;
  }

  view_->GetGroupsManager().AddNodeToGroup(group_name, group_hash, GetNode());

  if (group_hashes_->empty() && group_name == GroupsManager::kMainGroupName) {
    // If only in the main group, then destroy group_hashes_.
    group_hashes_.reset();
  } else {
    group_hashes_->insert(group_hash);
  }
}

void NodeController::RemoveFromGroup(absl::string_view group_name) {
  RemoveFromGroupSelf(group_name);
  PropagateInheritedGroupsRecursive();
}

void NodeController::RemoveFromGroupSelf(absl::string_view group_name) {
  if (!group_hashes_) {
    if (group_name == GroupsManager::kMainGroupName) {
      // Removing from main group. Not in any other group.
      group_hashes_ = std::make_unique<absl::flat_hash_set<HashValue>>();
      view_->GetGroupsManager().RemoveNodeFromGroup(
          GroupsManager::kMainGroupHash, GetNode());
    }
    return;
  }

  HashValue group_hash = Hash(group_name);
  auto itr = group_hashes_->find(group_hash);
  if (itr == group_hashes_->end()) {
    // This node isn't in this layer.
    return;
  }

  view_->GetGroupsManager().RemoveNodeFromGroup(group_hash, GetNode());
  group_hashes_->erase(itr);

  // If only in the main group, then destroy group_hashes_.
  if (group_hashes_->size() == 1 &&
      *group_hashes_->begin() == GroupsManager::kMainGroupHash) {
    group_hashes_.reset();
  }
}

void NodeController::SetGroups(
    std::optional<absl::Span<const absl::string_view>> group_names) {
  if (group_names.has_value()) {
    SetGroupsSelf(*group_names, false);
  } else {
    ClearBit(NodeFlags::kIsGroupsOverridden, flags_);
    UpdateInheritedGroupsSelf();
  }

  PropagateInheritedGroupsRecursive();
}

void NodeController::SetGroupsSelf(
    absl::Span<const absl::string_view> group_names, bool is_inherited) {
  flags_ =
      SetBitFromBool(flags_, NodeFlags::kIsGroupsOverridden, !is_inherited);
  std::vector<std::string> oldgroups = GetGroups();
  std::vector<std::string> groups_to_remove;

  std::set_difference(oldgroups.begin(), oldgroups.end(), group_names.begin(),
                      group_names.end(), std::back_inserter(groups_to_remove));

  for (const absl::string_view oldgroup_to_remove : groups_to_remove) {
    RemoveFromGroupSelf(oldgroup_to_remove);
  }

  // AddToGroup already guards against the group being added multiple
  // times so we can just call it to add all the new groups.
  for (const absl::string_view newgroup : group_names) {
    AddToGroupSelf(newgroup);
  }
}

std::vector<std::string> NodeController::GetGroups() const {
  std::vector<std::string> group_names;
  if (!group_hashes_) {
    group_names.push_back(std::string(GroupsManager::kMainGroupName));
  } else {
    for (HashValue group_hash : *group_hashes_) {
      absl::string_view group_name =
          view_->GetGroupsManager().GetGroupName(group_hash);
      if (group_name.empty()) {
        // This should never happen, implies a bug in GroupsManager or
        // NodeController.
        IMP_LOG(imp::FATAL) << "Node is part of a  group that doesn't exist.";
      }
      group_names.push_back(std::string(group_name));
    }
  }

  return group_names;
}

bool NodeController::IsInGroup(absl::string_view group_name) const {
  return group_hashes_ ? group_hashes_->contains(Hash(group_name))
                       : group_name == GroupsManager::kMainGroupName;
}

bool NodeController::CheckParentActive() {
  NodeHandle parent = GetNode()->GetParent();
  bool was_root = IsRoot();
  bool is_root = !parent.IsValid();
  if (is_root != was_root) {
    view_->GetPathManager().SetRoot(GetNode(), is_root);
    if (is_root) {
      flags_ = SetBit(flags_, NodeFlags::kIsRoot);
    } else {
      flags_ = ClearBit(flags_, NodeFlags::kIsRoot);
    }
  }

  if (parent) {
    return parent->IsActive();
  }

  // If there is no parent, then this is considered active.
  return true;
}

void NodeController::UpdateActiveRecursive(bool is_parent_active) {
  bool was_active = IsActive();
  UpdateActiveSelf(is_parent_active);

  // Only update active state in components and children if there's a change in
  // active state.
  bool is_active = IsActive();
  if (was_active != is_active) {
    // TODO (Fix by March 16) Optimize NotifyActiveForEntity as
    // it's quite expensive.
    view_->GetComponentManager().NotifyActiveForEntity(GetEntity(), is_active);
    for (auto child : GetNode()->GetChildren()) {
      child->node_controller_->UpdateActiveRecursive(is_active);
    }
  }
}

void NodeController::UpdateActiveSelf(bool is_parent_active) {
  bool active = IsEnabled() && is_parent_active;
  bool was_active = CheckBit(NodeFlags::kIsActive, flags_);
  flags_ = SetBitFromBool(flags_, NodeFlags::kIsActive, active);

  if (was_active != active) {
    if (!group_hashes_) {
      view_->GetGroupsManager().SetNodeActiveInGroup(
          GroupsManager::kMainGroupHash, GetNode(), active);
    } else {
      for (HashValue group_hash : *group_hashes_) {
        view_->GetGroupsManager().SetNodeActiveInGroup(group_hash, GetNode(),
                                                       active);
      }
    }
  }
}

void NodeController::PropagateInheritedGroupsRecursive() {
  for (auto child : GetNode()->GetChildren()) {
    child->node_controller_->UpdateInheritedGroupsRecursive();
  }
}

void NodeController::UpdateInheritedGroupsRecursive() {
  if (!CheckBit(NodeFlags::kIsGroupsOverridden, flags_)) {
    UpdateInheritedGroupsSelf();

    for (auto child : GetNode()->GetChildren()) {
      child->node_controller_->UpdateInheritedGroupsRecursive();
    }
  }
}

bool NodeController::DoGroupsMatch(NodeController& other_node_controller) {
  if (!other_node_controller.group_hashes_ && !group_hashes_) {
    // If both nodes have default groups, then they match.
    return true;
  }

  if (!group_hashes_ || !other_node_controller.group_hashes_) {
    // If only one group is set, then they don't match.
    return false;
  }

  // Neither component is default, compare the actual groups.
  return *group_hashes_ == *other_node_controller.group_hashes_;
}

void NodeController::UpdateInheritedGroupsSelf() {
  NodeHandle parent = GetNode()->GetParent();
  if (!parent) {
    return;
  }

  // If this component's groups don't match the parent component, then set this
  // component's groups to match.
  if (!DoGroupsMatch(*parent->node_controller_)) {
    std::vector<std::string> parentgroups =
        parent->node_controller_->GetGroups();
    SetGroupsSelf(absl::FixedArray<absl::string_view>(parentgroups.begin(),
                                                      parentgroups.end()),
                  true);
  }
}

}  // namespace imp::imp_internal
