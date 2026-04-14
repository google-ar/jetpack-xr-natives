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

#include "core/ncsb/component.h"

#include <utility>

#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/bit_flag.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"
#include "core/config.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_manager.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp {

Component::Component() {}

void Component::PostCreated(NodeHandle node, ComponentKey key,
                            BaseComponentPool& pool) {
  node_ = node;
  key_ = key;
  pool_ = &pool;
}

NodeHandle Component::GetNode() const { return node_; }

BaseView& Component::GetView() const { return GetNode()->GetView(); }

utils::Entity Component::GetEntity() const { return node_.GetEntity(); }

ComponentId Component::GetComponentId() const {
  return pool_->GetComponentId();
}

BaseComponentPool& Component::GetBaseComponentPool() const {
  // Guaranteed to exist, because this component exists.
  return *pool_;
}

Dispatcher& Component::GetDispatcher() const {
  return GetView().GetDispatcher();
}

void Component::SetEnabled(bool enabled) {
  bool was_enabled = IsEnabled();
  status_flags_ =
      SetBitFromBool(status_flags_, StatusFlags::kComponentIsEnabled, enabled);
  if (GetNode()->IsActive() && was_enabled != enabled) {
    BaseComponentPool& pool = GetBaseComponentPool();
    pool.NotifyActive(this, /*active=*/enabled);
  }
}

bool Component::IsActive() const {
  return CheckBit(status_flags_, StatusFlags::kComponentIsActive);
}

bool Component::IsEnabled() const {
  return CheckBit(status_flags_, StatusFlags::kComponentIsEnabled);
}

void Component::SetActiveFlagInternal(bool active) {
  status_flags_ =
      SetBitFromBool(status_flags_, StatusFlags::kComponentIsActive, active);
}

void Component::SetRunningAsyncSetupFlagInternal(bool is_running_async_setup) {
  status_flags_ =
      SetBitFromBool(status_flags_, StatusFlags::kComponentIsRunningAsyncSetup,
                     is_running_async_setup);
}

#if IMP_RUNTIME(DEV)
bool Component::IsEditorStaging() const { return GetNode()->IsEditorStaging(); }
#endif

Invocable<void()> Component::Remember(Holdable holdable) {
  return GetBaseComponentPool().Remember(GetEntity(), std::move(holdable));
}

void Component::SetRemovingFlagInternal(bool is_removing) {
  status_flags_ = SetBitFromBool(
      status_flags_, StatusFlags::kComponentIsBeingRemoved, is_removing);
}

bool Component::IsRemoving() const {
  return CheckBit(status_flags_, StatusFlags::kComponentIsBeingRemoved);
}

}  // namespace imp
