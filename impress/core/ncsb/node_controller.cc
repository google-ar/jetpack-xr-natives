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
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/no_destructor.h"
#include "absl/container/fixed_array.h"
#include "absl/container/inlined_vector.h"
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
#include "core/split_engine/split_engine_serializer.h"
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
  if (rememberer_.has_value()) {
    rememberer_->ClearRemembered();
  }

  // It's possible that the node was destroyed when ClearRemembered was running.
  // In that case, we don't need to do anything.
  if (!node) {
    return;
  }

  for (const HashValue& group_hash : GetGroupHashes()) {
    view_->GetGroupsManager().RemoveNodeFromGroup(group_hash, node);
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
  if (!rememberer_.has_value()) {
    rememberer_.emplace();
  }
  return rememberer_->Remember(std::move(holdable));
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
  HashValue group_hash = Hash(group_name);

  // Special case logic: if the node was previously in zero groups and is added
  // to the main group, delete group_hashes_ (return to default state).
  if (group_hash == GroupsManager::kMainGroupHash && group_hashes_ &&
      group_hashes_->empty()) {
    // Optimization: If the node is only in the 'Main' group (which is the
    // default), release the vector storage to save memory.
    group_hashes_.reset();
    view_->GetGroupsManager().AddNodeToGroup(group_name, group_hash, GetNode());
    OnGroupsChanged();
    return;
  }

  std::vector<HashValue>& hashes = MutableGroups();
  auto it = std::lower_bound(hashes.begin(), hashes.end(), group_hash);
  if (it == hashes.end() || *it != group_hash) {
    // This group is new.
    hashes.insert(it, group_hash);
    view_->GetGroupsManager().AddNodeToGroup(group_name, group_hash, GetNode());
    OnGroupsChanged();
  }
}

void NodeController::RemoveFromGroup(absl::string_view group_name) {
  RemoveFromGroupSelf(group_name);
  PropagateInheritedGroupsRecursive();
}

void NodeController::RemoveFromGroupSelf(absl::string_view group_name) {
  HashValue group_hash = Hash(group_name);
  // Optimization: If implicit main and removing something else, do nothing.
  if (!group_hashes_ && group_hash != GroupsManager::kMainGroupHash) {
    return;
  }

  std::vector<HashValue>& hashes = MutableGroups();
  auto it = std::lower_bound(hashes.begin(), hashes.end(), group_hash);
  if (it == hashes.end() || *it != group_hash) {
    // This group is not present.
    return;
  }

  hashes.erase(it);
  view_->GetGroupsManager().RemoveNodeFromGroup(group_hash, GetNode());
  OnGroupsChanged();

  // Optimization: If the node is only in the 'Main' group (which is the
  // default), release the vector storage to save memory.
  if (group_hashes_ && group_hashes_->size() == 1 &&
      (*group_hashes_)[0] == GroupsManager::kMainGroupHash) {
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

  // Using absl::InlinedVector<HashValue, kGroupHashInlineCapacity> for
  // temporary group name hash lists. Since HashValue is usually 4-8 bytes, a
  // size of 16 fits comfortably on the stack (approx 64-128 bytes), covering
  // the vast majority of use cases without heap allocation.
  static constexpr size_t kGroupHashInlineCapacity = 16;

  // Prepare new sorted unique hashes.
  absl::InlinedVector<HashValue, kGroupHashInlineCapacity> new_hashes;
  new_hashes.reserve(group_names.size());
  std::transform(group_names.begin(), group_names.end(),
                 std::back_inserter(new_hashes),
                 [](absl::string_view name) { return Hash(name); });
  std::sort(new_hashes.begin(), new_hashes.end());
  // Standard idiom to remove duplicates from a sorted vector.
  // std::unique moves duplicates to the end and returns an iterator to the new
  // logical end; erase then removes the undefined tail.
  new_hashes.erase(std::unique(new_hashes.begin(), new_hashes.end()),
                   new_hashes.end());

  const std::vector<HashValue>& old_hashes = GetGroupHashes();

  if (std::equal(new_hashes.begin(), new_hashes.end(), old_hashes.begin(),
                 old_hashes.end())) {
    return;
  }

  // Compute Diff between old and new.
  // std::set_difference expects sorted ranges and outputs the elements present
  // in the first range but not the second.
  absl::InlinedVector<HashValue, kGroupHashInlineCapacity> to_remove;
  std::set_difference(old_hashes.begin(), old_hashes.end(), new_hashes.begin(),
                      new_hashes.end(), std::back_inserter(to_remove));

  absl::InlinedVector<HashValue, kGroupHashInlineCapacity> to_add;
  std::set_difference(new_hashes.begin(), new_hashes.end(), old_hashes.begin(),
                      old_hashes.end(), std::back_inserter(to_add));

  // Apply changes.
  for (const HashValue& h : to_remove) {
    view_->GetGroupsManager().RemoveNodeFromGroup(h, GetNode());
  }

  // Notify the groups manager of the new groups.
  // We need names for AddNodeToGroup. Since we only have hashes, we must find
  // the name.
  // Assuming group_names is small, linear scan is fine for performance.
  for (const HashValue& h : to_add) {
    // Find name corresponding to hash h.
    std::optional<absl::string_view> name;
    for (absl::string_view n : group_names) {
      if (Hash(n) == h) {
        name = n;
        break;
      }
    }

    if (name.has_value()) {
      view_->GetGroupsManager().AddNodeToGroup(*name, h, GetNode());
    } else {
      IMP_LOG(imp::FATAL) << "Could not find name for group hash during SetGroupsSelf";
    }
  }

  if (new_hashes.size() == 1 &&
      new_hashes[0] == GroupsManager::kMainGroupHash) {
    // Optimization: If the node is only in the 'Main' group (which is the
    // default), release the vector storage to save memory.
    group_hashes_.reset();
  } else {
    // Update Storage with the new entries.
    MutableGroups().assign(new_hashes.begin(), new_hashes.end());
  }

  OnGroupsChanged();
}

std::vector<std::string> NodeController::GetGroups() const {
  std::vector<std::string> group_names;
  const std::vector<HashValue>& group_hashes = GetGroupHashes();
  group_names.reserve(group_hashes.size());
  for (HashValue group_hash : group_hashes) {
    absl::string_view group_name =
        view_->GetGroupsManager().GetGroupName(group_hash);
    if (group_name.empty()) {
      // Should not happen.
      IMP_LOG(imp::FATAL) << "Node is part of a group that doesn't exist.";
    }
    group_names.push_back(std::string(group_name));
  }
  return group_names;
}

bool NodeController::IsInGroup(absl::string_view group_name) const {
  HashValue group_hash = Hash(group_name);
  const std::vector<HashValue>& group_hashes = GetGroupHashes();
  return std::binary_search(group_hashes.begin(), group_hashes.end(),
                            group_hash);
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
    for (HashValue group_hash : GetGroupHashes()) {
      view_->GetGroupsManager().SetNodeActiveInGroup(group_hash, GetNode(),
                                                     active);
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

const std::vector<HashValue>& NodeController::GetGroupHashes() const {
  static const absl::NoDestructor<std::vector<HashValue>> kMainGroupVector(
      {GroupsManager::kMainGroupHash});
  return group_hashes_ ? *group_hashes_ : *kMainGroupVector;
}

std::vector<HashValue>& NodeController::MutableGroups() {
  if (!group_hashes_) {
    group_hashes_ = std::make_unique<std::vector<HashValue>>();
    group_hashes_->push_back(GroupsManager::kMainGroupHash);
  }
  return *group_hashes_;
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
  // Vectors are sorted, so direct comparison works.
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

void NodeController::OnGroupsChanged() {
  if (split_engine::SplitEngineSerializer* serializer =
          view_->GetSplitEngineSerializer()) {
    if (serializer->GetApiLevel() ==
        split_engine::kSplitEngineExperimentalApiLevel) {
      std::vector<std::string> groups = GetGroups();
      serializer->SetGroups(GetEntity(), absl::FixedArray<absl::string_view>(
                                             groups.begin(), groups.end()));
    }
  }
}

}  // namespace imp::imp_internal
