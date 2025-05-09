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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_SYSTEM_POOL_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_SYSTEM_POOL_H_

#include <memory>

#include "core/config.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_pool_with_updater.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Specialization of ComponentPool used for component types that declare a
// ComponentSystem.
//
// Used to store a component types system and call the systems lifecycle
// methods.
//
// See component_system.h for more details.
template <typename T>
class ComponentSystemPool : public ComponentPoolWithUpdater<T> {
 public:
  using TSystem = typename T::System;

  explicit ComponentSystemPool(BaseView& view, std::unique_ptr<TSystem> system)
      : ComponentPoolWithUpdater<T>(view), system_(std::move(system)) {}

  System* GetComponentSystem() override { return system_.get(); }

  void BeforeFirstAdded() noexcept override;
  void AfterAdd(Component& component) noexcept override;
  void BeforeRemove(Component& component) noexcept override;
  void AfterLastRemoved() noexcept override;
  void Update(const FrameTime& frame_time) noexcept override;

 private:
  std::unique_ptr<TSystem> system_;
};

template <typename T>
void ComponentSystemPool<T>::BeforeFirstAdded() noexcept {
  system_->BeforeFirstComponentAdded();
}

template <typename T>
void ComponentSystemPool<T>::AfterAdd(Component& component) noexcept {
#if IMP_RUNTIME(DEV)
  if constexpr (!component_traits::kShouldRunInEditMode<T>) {
    if (editor::IsInEditMode(BaseComponentPool::GetView().GetRegistry())) {
      return;
    }
  }
#endif

  system_->AfterComponentAdded(static_cast<T&>(component));
}

template <typename T>
void ComponentSystemPool<T>::BeforeRemove(Component& component) noexcept {
#if IMP_RUNTIME(DEV)
  if constexpr (!component_traits::kShouldRunInEditMode<T>) {
    if (editor::IsInEditMode(BaseComponentPool::GetView().GetRegistry())) {
      return;
    }
  }
#endif

  system_->BeforeComponentRemoved(static_cast<T&>(component));
}

template <typename T>
void ComponentSystemPool<T>::AfterLastRemoved() noexcept {
  system_->AfterLastComponentRemoved();
}

template <typename T>
void ComponentSystemPool<T>::Update(const FrameTime& frame_time) noexcept {
  // It isn't needed to check kShouldRunInEditMode here because
  // ComponentPoolWithUpdater already checks this, this method is only called if
  // it actually needs to run.

  system_->PreComponentsUpdated(frame_time);
  ComponentPoolWithUpdater<T>::Update(frame_time);
  system_->PostComponentsUpdated(frame_time);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_SYSTEM_POOL_H_
