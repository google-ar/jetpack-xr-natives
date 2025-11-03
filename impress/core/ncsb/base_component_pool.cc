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

#include "core/ncsb/base_component_pool.h"

#include <cassert>
#include <cstddef>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/compiler.h"
#include "core/async/future.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"
#include "core/common/rememberer.h"
#include "core/common/vector_helpers.h"
#include "core/ncsb/component.h"
#include "core/view/base_view.h"

namespace imp {

BaseComponentPool::ComponentStore::ComponentStore(BaseComponentPool* pool)
    : pool_(pool) {}

bool BaseComponentPool::ComponentStore::Has(utils::Entity entity) const {
  return entities_to_indices_.count(entity) > 0;
}

BaseComponentPool::ComponentIndex BaseComponentPool::ComponentStore::GetIndex(
    utils::Entity entity) const {
  return entities_to_indices_.at(entity);
}

bool BaseComponentPool::ComponentStore::Empty() const {
  return components_.size() <= free_indices_count_;
}

size_t BaseComponentPool::ComponentStore::GetComponentCount() const {
  return components_.size() - free_indices_count_;
}

utils::Entity BaseComponentPool::ComponentStore::BackEntity() const {
  return components_.back()->GetEntity();
}

void BaseComponentPool::ComponentStore::Remove(utils::Entity entity) {
  auto itr = entities_to_indices_.find(entity);
  if (itr == entities_to_indices_.end()) {
    return;
  }

  ComponentIndex index = itr->second;

  // Remove from the map first.
  entities_to_indices_.erase(itr);

  ComponentIndex last_index = components_.size() - 1;

  // Special case for removing a component while in the midst of iterating over
  // the components. In this case, we can't swap and pop because it can cause
  // the iteration to skip over components. Instead we track the free indices
  // so we can densify the vector later.
  if (iterating_depth_ > 0) {
    components_[index].reset();
    ++free_indices_count_;
    
    return;
  }

  // Swap and pop the last component to ensure the vector of components stays
  // dense.
  if (index != last_index) {
    ComponentPtr& last_component = components_.at(last_index);
    utils::Entity swapped_entity = last_component->GetEntity();
    std::swap(components_.at(index), last_component);
    entities_to_indices_[swapped_entity] = index;
  }

  components_.pop_back();
}

Component& BaseComponentPool::ComponentStore::AtRaw(ComponentIndex index) {
  return *components_[index];
}

const Component& BaseComponentPool::ComponentStore::AtRaw(
    ComponentIndex index) const {
  return *components_[index];
}

Component* BaseComponentPool::ComponentStore::TryGetRaw(utils::Entity entity) {
  auto itr = entities_to_indices_.find(entity);
  if (itr == entities_to_indices_.end()) {
    return nullptr;
  }

  return components_[itr->second].get();
}

void BaseComponentPool::ComponentStore::TryDensifyComponentsVector() {
  // If we are in the middle of iterating, we can't densify the vector.
  if (iterating_depth_ > 0) {
    return;
  }

  // If there are no free indices, we don't need to densify.
  if (free_indices_count_ == 0) {
    return;
  }

  CompactVector(components_, [this](size_t new_index) {
    // When a component's index is change within the vector, the entity to
    // index map needs to be updated to accurately reflect the new index.
    entities_to_indices_[components_[new_index]->GetEntity()] = new_index;
  });

  free_indices_count_ = 0;
}

BaseComponentPool::BaseComponentPool(BaseView& view)
    : view_(view), components_(this) {}

BaseComponentPool::~BaseComponentPool() {
  // RemoveAll should be called prior to getting here.
  assert(components_.Empty());
}

bool BaseComponentPool::Has(utils::Entity entity) const noexcept {
  return components_.Has(entity);
}

BaseComponentPool::ComponentIndex BaseComponentPool::Get(
    utils::Entity entity) const noexcept {
  return components_.GetIndex(entity);
}

Component* BaseComponentPool::Add(utils::Entity entity) noexcept {
  if (UTILS_UNLIKELY(Has(entity))) {
    Remove(entity);
  }

  if (components_.Empty()) {
    BeforeFirstAdded();
  }

  return Emplace(entity);
}

void BaseComponentPool::PostSetup(Component& component,
                                  bool should_enable_component) noexcept {
  

  if (should_enable_component) {
    component.SetEnabled(true);
  }
  AfterAdd(component);
}

Component& BaseComponentPool::GetRawComponent(
    ComponentIndex instance) noexcept {
  return components_.AtRaw(instance);
}

Component* BaseComponentPool::TryGetRawComponentFromEntity(
    utils::Entity entity) noexcept {
  return components_.TryGetRaw(entity);
}

utils::Entity BaseComponentPool::GetEntity(
    ComponentIndex instance) const noexcept {
  return components_.AtRaw(instance).GetEntity();
}

size_t BaseComponentPool::GetComponentCount() const noexcept {
  return components_.GetComponentCount();
}

void BaseComponentPool::Remove(utils::Entity entity) noexcept {
  CancelPending(entity);
  Forget(entity);

  // CancelPending or Forget may have removed the component already.
  Component* component = components_.TryGetRaw(entity);
  if (component == nullptr) {
    return;
  }

  BeforeRemove(*component);

  // Disable the component so that OnActiveStatusChange is called before Cleanup
  // (unless the component was already inactive).
  component->SetEnabled(false);

  Cleanup(*component);

  components_.Remove(entity);

  if (components_.Empty()) {
    AfterLastRemoved();
  }
}

void BaseComponentPool::RemoveAll() noexcept {
  while (!components_.Empty()) {
    Remove(components_.BackEntity());
  }
}

void BaseComponentPool::CancelPending(utils::Entity entity) noexcept {
  auto setup_future_opt = GetSetupFuture(entity);
  if (setup_future_opt) {
    setup_future_opt->Cancel();
  }
}

Future<absl::Status> BaseComponentPool::MakeSetupFuture(
    Component& component, Future<absl::Status> future,
    bool should_enable_component) {
  utils::Entity entity = component.GetEntity();
  

  component.SetRunningAsyncSetupFlagInternal(true);

  Component* component_ptr = &component;

  auto setup_future = future.Then(
      [this, component_ptr, should_enable_component](absl::Status status) {
        // It's guaranteed that the component_ptr is still valid here. This is
        // because the first thing Remove does is call CancelPending, which will
        // cause this lambda to run prior to the component being destroyed if it
        // hasn't already run. Remove() also internally handles reentrancy.
        utils::Entity entity = component_ptr->GetEntity();
        weak_setup_futures_.erase(entity);
        component_ptr->SetRunningAsyncSetupFlagInternal(false);

        if (!status.ok()) {
          Remove(entity);
          return status;
        }

        PostSetup(*component_ptr, should_enable_component);

        return status;
      });

  if (!setup_future.Ready()) {
    weak_setup_futures_.emplace(entity, setup_future);
  }

  return setup_future;
}

absl::optional<Future<absl::Status>> BaseComponentPool::GetSetupFuture(
    utils::Entity entity) {
  auto it = weak_setup_futures_.find(entity);
  if (it != weak_setup_futures_.end()) {
    return it->second.Lock();
  }

  return absl::nullopt;
}

Invocable<void()> BaseComponentPool::Remember(utils::Entity e,
                                              Holdable holdable) {
  return rememberers_[e].Remember(std::move(holdable));
}

void BaseComponentPool::Forget(utils::Entity e) {
  // Extract it from the map and then let it go out of scope.
  // Don't erase directly to avoid potential re-entrancy issues if clearing the
  // Rememberer causes something to be destroyed that ends up calling Forget
  // again.

  // First, find the rememberer.
  auto itr = rememberers_.find(e);
  if (itr != rememberers_.end()) {
    // Move the rememberer out of the map, it will get destroyed when it goes
    // out of scope. This doesn't invalidate the iterator, since it just leaves
    // the Rememberer inside the map in an empty state.
    Rememberer rememberer = std::move(itr->second);

    // Erase the entry from the map before the Rememberer is destroyed, this
    // ensures that the we don't call into the map from within the erase call.
    rememberers_.erase(itr);
  }
}

}  // namespace imp
