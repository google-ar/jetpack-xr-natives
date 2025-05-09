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

#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/compiler.h"
#include "core/async/future.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"
#include "core/common/rememberer.h"
#include "core/ncsb/component.h"
#include "core/view/base_view.h"

namespace imp {

BaseComponentPool::ComponentVector::ComponentVector(BaseComponentPool* pool)
    : pool_(pool) {}

bool BaseComponentPool::ComponentVector::Empty() const {
  return components_.empty();
}

utils::Entity BaseComponentPool::ComponentVector::SwapAndPop(
    ComponentIndex instance) {
  utils::Entity swapped_entity;
  ComponentIndex last_instance = components_.size() - 1;
  if (instance != last_instance) {
    ComponentPtr& last_component = components_.at(last_instance);
    swapped_entity = last_component->GetEntity();
    std::swap(components_.at(instance), last_component);
  }

  components_.pop_back();
  return swapped_entity;
}

Component& BaseComponentPool::ComponentVector::AtRaw(ComponentIndex instance) {
  return *components_.at(instance);
}

const Component& BaseComponentPool::ComponentVector::AtRaw(
    ComponentIndex instance) const {
  return *components_.at(instance);
}

BaseComponentPool::BaseComponentPool(BaseView& view)
    : view_(view), components_(this) {}

BaseComponentPool::~BaseComponentPool() {
  // RemoveAll should be called prior to getting here.
  assert(components_.Empty());
}

bool BaseComponentPool::Has(utils::Entity entity) const noexcept {
  return entities_to_instances_.count(entity) > 0;
}

BaseComponentPool::ComponentIndex BaseComponentPool::Get(
    utils::Entity entity) const noexcept {
  return entities_to_instances_.at(entity);
}

Component* BaseComponentPool::Add(utils::Entity entity) noexcept {
  if (UTILS_UNLIKELY(Has(entity))) {
    Remove(entity);
  }

  if (components_.Empty()) {
    BeforeFirstAdded();
  }

  ComponentIndex instance = EmplaceBack();
  entities_to_instances_[entity] = instance;

  return &GetRawComponent(instance);
}

void BaseComponentPool::PostSetup(utils::Entity entity,
                                  bool should_enable_component) noexcept {
  Component& component = GetRawComponent(Get(entity));
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
  auto itr = entities_to_instances_.find(entity);
  if (itr == entities_to_instances_.end()) {
    return nullptr;
  }

  return &components_.AtRaw(itr->second);
}

utils::Entity BaseComponentPool::GetEntity(
    ComponentIndex instance) const noexcept {
  return components_.AtRaw(instance).GetEntity();
}

size_t BaseComponentPool::GetComponentCount() const noexcept {
  return entities_to_instances_.size();
}

void BaseComponentPool::Remove(utils::Entity entity) noexcept {
  CancelPending(entity);
  Forget(entity);

  auto itr = entities_to_instances_.find(entity);
  if (itr == entities_to_instances_.end()) {
    // Couldn't find instance...
    return;
  }

  ComponentIndex instance = itr->second;
  Component& component = GetRawComponent(instance);

  BeforeRemove(component);

  // Disable the component so that OnActiveStatusChange is called before Cleanup
  // (unless the component was already inactive).
  component.SetEnabled(false);

  Cleanup(instance);

  // It's possible that the Instance changed during Cleanup if another component
  // of this type was removed. Find it again.
  itr = entities_to_instances_.find(entity);
  if (itr == entities_to_instances_.end()) {
    return;
  }
  instance = itr->second;

  // Actually remove it now.
  entities_to_instances_.erase(itr);
  if (utils::Entity swapped_entity = components_.SwapAndPop(instance)) {
    // In this case, another entity was swapped and now uses instance, so we
    // must update the entities_to_instances_ map.
    entities_to_instances_[swapped_entity] = instance;
  }

  if (components_.Empty()) {
    AfterLastRemoved();
  }
}

void BaseComponentPool::RemoveAll() noexcept {
  while (!entities_to_instances_.empty()) {
    Remove(entities_to_instances_.begin()->first);
  }
}

bool BaseComponentPool::Pending(utils::Entity entity) const noexcept {
  return weak_setup_futures_.find(entity) != weak_setup_futures_.end();
}

void BaseComponentPool::CancelPending(utils::Entity entity) noexcept {
  auto setup_future_opt = GetSetupFuture(entity);
  if (setup_future_opt) {
    setup_future_opt->Cancel();
  }
}

Future<absl::Status> BaseComponentPool::MakeSetupFuture(
    utils::Entity entity, Future<absl::Status> future,
    bool should_enable_component) {
  assert(Has(entity));
  assert(weak_setup_futures_.find(entity) == weak_setup_futures_.end());

  auto setup_future =
      future.Then([this, entity, should_enable_component](absl::Status status) {
        weak_setup_futures_.erase(entity);

        if (!status.ok()) {
          Remove(entity);
          return status;
        }

        PostSetup(entity, should_enable_component);

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
