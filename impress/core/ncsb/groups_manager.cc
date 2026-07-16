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

#include "core/ncsb/groups_manager.h"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <string>
#include <utility>
#include <variant>

#include "absl/container/inlined_vector.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Scene.h"
#include "core/common/hash.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_or_unowned_memory.h"
#include "core/common/owned_ptr.h"
#include "core/lighting/environment_light.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"

namespace imp {
namespace {
using AutomatedLightingMode = GroupsManager::AutomatedLightingMode;
}

GroupsManager::GroupsManager(BaseView* view, filament::Scene* main_scene)
    : view_(view) {
  groups_.insert_or_assign(kMainGroupHash, Group{.scene = main_scene});
  hashes_to_names_.insert_or_assign(kMainGroupHash,
                                    std::string(kMainGroupName));
}

GroupsManager::Group* GroupsManager::CreateGroup(absl::string_view group_name,
                                                 HashValue group_hash) {
  filament::Scene* scene = BaseView::GetSharedEngine()->createScene();
  groups_.insert_or_assign(group_hash, Group{.scene = scene});
  hashes_to_names_.insert_or_assign(group_hash, std::string(group_name));
  view_->GetDispatcher().Send(GroupCreatedEvent(group_name));
  return &groups_[group_hash];
}

void GroupsManager::AddNodeToGroup(absl::string_view group_name,
                                   HashValue group_hash, NodeHandle node) {
  Group* group = nullptr;
  auto itr = groups_.find(group_hash);
  if (itr != groups_.end()) {
    group = &itr->second;
  } else {
    CreateGroup(group_name, group_hash);
    group = &groups_[group_hash];
  }

  group->num_nodes_in_layer++;

  // Only actually add the node to the scene if it is currently active.
  // Nodes internally track which groups they are part of. When the node becomes
  // active, it will be added to the scene when SetNodeActiveInGroup is called.
  if (node->IsActive()) {
    group->scene->addEntity(node.GetEntity());
  }
}

void GroupsManager::DestroyGroup(HashValue group_hash) {
  Group& group = groups_[group_hash];
  BaseView::GetSharedEngine()->destroy(group.scene);
  view_->GetDispatcher().Send(GroupDestroyedEvent(GetGroupName(group_hash)));
  groups_.erase(group_hash);
  hashes_to_names_.erase(group_hash);
}

void GroupsManager::RemoveNodeFromGroup(HashValue group_hash, NodeHandle node) {
  auto itr = groups_.find(group_hash);
  if (itr == groups_.end()) {
    IMP_LOG(imp::FATAL) << "Attempting to remove node from a group that doesn't exist.";
    return;
  }

  Group& group = itr->second;
  group.num_nodes_in_layer--;
  group.scene->remove(node.GetEntity());
  if (group.num_nodes_in_layer == 0 && group_hash != kMainGroupHash &&
      !GroupHasEnvironmentLight(group)) {
    DestroyGroup(group_hash);
  }
}

void GroupsManager::UpdateNodeGroups(
    absl::Span<const HashValue> old_hashes,
    absl::Span<const HashValue> new_hashes, NodeHandle node,
    absl::Span<const absl::string_view> new_group_names) {
  static constexpr size_t kGroupHashInlineCapacity = 16;

  absl::InlinedVector<HashValue, kGroupHashInlineCapacity> to_remove;
  std::set_difference(old_hashes.begin(), old_hashes.end(), new_hashes.begin(),
                      new_hashes.end(), std::back_inserter(to_remove));

  absl::InlinedVector<HashValue, kGroupHashInlineCapacity> to_add;
  std::set_difference(new_hashes.begin(), new_hashes.end(), old_hashes.begin(),
                      old_hashes.end(), std::back_inserter(to_add));

  for (HashValue h : to_remove) {
    RemoveNodeFromGroup(h, node);
  }

  for (HashValue h : to_add) {
    Group* group = nullptr;
    auto itr = groups_.find(h);
    if (itr != groups_.end()) {
      group = &itr->second;
    } else if (!new_group_names.empty()) {
      // Slow path where we need to search for the group name based on the hash
      // so that we can create the group.
      //
      // Alternatively, UpdateNodeGroups could take sorted pairs of hash and
      // name, or a map from hash to name to speed this up. This intentionally
      // isn't done because this path is not expected to be hit frequently and
      // the overhead for creating and sorting those data structures every time
      // this is called isn't worth it.
      auto group_name_itr = std::find_if(
          new_group_names.begin(), new_group_names.end(),
          [h](absl::string_view group_name) { return Hash(group_name) == h; });
      if (group_name_itr != new_group_names.end()) {
        group = CreateGroup(*group_name_itr, h);
      }
    }

    if (group) {
      group->num_nodes_in_layer++;
      if (node->IsActive()) {
        group->scene->addEntity(node.GetEntity());
      }
    } else {
      IMP_LOG(imp::FATAL) << "Unable to find or create group.";
    }
  }
}

bool GroupsManager::GroupHasEnvironmentLight(GroupsManager::Group& group) {
  AutomatedLightingMode* lighting =
      std::get_if<AutomatedLightingMode>(&group.environment_light);
  if (lighting && *lighting == AutomatedLightingMode::kNoEnvironmentLight) {
    return false;
  }
  return true;
}

void GroupsManager::SetNodeActiveInGroup(HashValue group_hash, NodeHandle node,
                                         bool is_active) {
  auto itr = groups_.find(group_hash);
  if (itr == groups_.end()) {
    IMP_LOG(imp::FATAL) << "Attempting to set node active in a group that doesn't "
                  "exist.";
    return;
  }

  Group& group = itr->second;
  if (is_active) {
    group.scene->addEntity(node.GetEntity());
  } else {
    group.scene->remove(node.GetEntity());
  }
}

filament::Scene* GroupsManager::GetScene(absl::string_view group_name) {
  HashValue group_hash = Hash(group_name);
  auto itr = groups_.find(group_hash);
  if (itr == groups_.end()) {
    return nullptr;
  }
  Group& group = itr->second;
  return group.scene;
}

size_t GroupsManager::GetNumNodesInGroup(absl::string_view group_name) {
  HashValue group_hash = Hash(group_name);
  auto itr = groups_.find(group_hash);
  if (itr == groups_.end()) {
    return 0;
  }
  Group& group = itr->second;
  return group.num_nodes_in_layer;
}

absl::string_view GroupsManager::GetGroupName(HashValue group_hash) {
  auto itr = hashes_to_names_.find(group_hash);
  if (itr == hashes_to_names_.end()) {
    return {};
  }

  return itr->second;
}

void GroupsManager::SetGroupEnvironmentLight(
    absl::string_view group_name, EnvironmentLightHolder environment_light) {
  AutomatedLightingMode* input_auto_light =
      std::get_if<AutomatedLightingMode>(&environment_light);
  if (input_auto_light &&
      *input_auto_light ==
          AutomatedLightingMode::kUseMainGroupEnvironmentLight &&
      group_name == kMainGroupName) {
    IMP_LOG(imp::WARNING) << "Setting Main group to use Main group environment light "
                    "does nothing.";
    return;
  }

  bool has_no_env_light =
      input_auto_light &&
      *input_auto_light == AutomatedLightingMode::kNoEnvironmentLight;

  HashValue group_hash = Hash(group_name);
  auto itr = groups_.find(group_hash);
  Group* group = nullptr;
  if (itr != groups_.end()) {
    group = &itr->second;
    // If removing environment light from the empty group, destroy the group.
    if (has_no_env_light && group->num_nodes_in_layer == 0 &&
        group_hash != kMainGroupHash) {
      DestroyGroup(group_hash);
      return;
    }
  } else {
    // If the group doesn't exist and the environment light is not
    // kNoEnvironmentLight, it will be created.
    if (has_no_env_light) {
      return;
    }
    group = CreateGroup(group_name, group_hash);
  }

  AutomatedLightingMode* current_auto_light =
      std::get_if<AutomatedLightingMode>(&(group->environment_light));
  if (current_auto_light && input_auto_light &&
      *current_auto_light == *input_auto_light) {
    // Return early if the environment light is unchanged.
    return;
  }

  group->environment_light = std::move(environment_light);

  ProcessGroupEnvironmentLightChange(*group);

  if (group_name == kMainGroupName) {
    // Notify all groups using the main group's lighting that the main group's
    // lighting status has changed.
    for (auto& [group_hash, group] : groups_) {
      if (group_hash == kMainGroupHash) {
        continue;
      }
      AutomatedLightingMode* auto_light =
          std::get_if<AutomatedLightingMode>(&group.environment_light);
      if (auto_light && *auto_light == GroupsManager::AutomatedLightingMode::
                                           kUseMainGroupEnvironmentLight) {
        ProcessGroupEnvironmentLightChange(group);
      }
    }
  }
}

BorrowedPtr<EnvironmentLight> GroupsManager::GetEnvironmentLight(
    absl::string_view group_name) {
  HashValue group_hash = Hash(group_name);
  auto itr = groups_.find(group_hash);
  if (itr == groups_.end()) {
    return BorrowedPtr<EnvironmentLight>();
  }

  EnvironmentLightHolder& effective_light =
      GetEffectiveEnvironmentLight(itr->second);

  if (OwnedOrBorrowedPtr<EnvironmentLight>* light =
          std::get_if<OwnedOrBorrowedPtr<EnvironmentLight>>(&effective_light);
      light) {
    return light->Borrow();
  }

  return BorrowedPtr<EnvironmentLight>();
}

EnvironmentLight* GroupsManager::GetRawEnvironmentLight(
    absl::string_view group_name) {
  HashValue group_hash = Hash(group_name);
  auto itr = groups_.find(group_hash);
  if (itr == groups_.end()) {
    return nullptr;
  }

  EnvironmentLightHolder& effective_light =
      GetEffectiveEnvironmentLight(itr->second);

  if (OwnedOrUnownedMemory<EnvironmentLight>* light =
          std::get_if<OwnedOrUnownedMemory<EnvironmentLight>>(&effective_light);
      light) {
    return light->Get();
  }

  return nullptr;
}

GroupsManager::EnvironmentLightHolder&
GroupsManager::GetEffectiveEnvironmentLight(GroupsManager::Group& group) {
  EnvironmentLightHolder* effective_light = &group.environment_light;
  if (AutomatedLightingMode* auto_light =
          std::get_if<AutomatedLightingMode>(effective_light);
      auto_light) {
    if (*auto_light == AutomatedLightingMode::kUseMainGroupEnvironmentLight) {
      effective_light = &groups_[kMainGroupHash].environment_light;
    }
  }
  return *effective_light;
}

void GroupsManager::ProcessGroupEnvironmentLightChange(
    GroupsManager::Group& group) {
  EnvironmentLightHolder& effective_light = GetEffectiveEnvironmentLight(group);

  // Sync the environment light to Filament.
  if (OwnedOrUnownedMemory<EnvironmentLight>* raw_environment_light =
          std::get_if<OwnedOrUnownedMemory<EnvironmentLight>>(&effective_light);
      raw_environment_light) {
    group.scene->setIndirectLight(
        raw_environment_light->Get()->GetIndirectLight());
  } else if (OwnedOrBorrowedPtr<EnvironmentLight>* environment_light =
                 std::get_if<OwnedOrBorrowedPtr<EnvironmentLight>>(
                     &effective_light);
             environment_light) {
    group.scene->setIndirectLight(
        environment_light->Borrow()->GetIndirectLight());
  } else {
    group.scene->setIndirectLight(nullptr);
  }
}

// BorrowedPtr<EnvironmentLight>
// GroupsManager::EnvironmentLightHolder::GetEnvironmentLight() {
//   // OwnedOrBorrowedPtr<EnvironmentLight>* result =
//   // std::get_if<OwnedOrBorrowedPtr<EnvironmentLight>>(&environment_light_);

//   if (std::holds_alternative<OwnedOrBorrowedPtr<EnvironmentLight>>(
//           environment_light_)) {
//     return std::get<OwnedOrBorrowedPtr<EnvironmentLight>>(environment_light_)
//         .Borrow();
//   }
//   return BorrowedPtr<EnvironmentLight>();
// }

// EnvironmentLight*
// GroupsManager::EnvironmentLightHolder::GetRawEnvironmentLight() {
//   if (std::holds_alternative<OwnedOrUnownedMemory<EnvironmentLight>>(
//           environment_light_)) {
//     return
//     std::get<OwnedOrUnownedMemory<EnvironmentLight>>(environment_light_)
//         .Get();
//   }
//   return nullptr;
// }

// AutomatedLightingMode*
// GroupsManager::EnvironmentLightHolder::GetAutomatedLightingMode() {
//   if (std::holds_alternative<AutomatedLightingMode>(environment_light_)) {
//     return &std::get<AutomatedLightingMode>(environment_light_);
//   }
//   return nullptr;
// }

}  // namespace imp
