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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_POOL_WITH_UPDATER_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_POOL_WITH_UPDATER_H_

#include "core/common/trace.h"
#include "core/config.h"
#include "core/ncsb/component_pool.h"
#include "core/ncsb/update_system.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

#if IMP_RUNTIME(DEV)
#include "core/editor/editor_info.h"
#endif

namespace imp {

// ComponentPool used for component types that implement an Update function.
//
// Also used by ComponentPoolWithSystem to receive updates even when the
// component has no Update function.
template <typename T>
class ComponentPoolWithUpdater : public ComponentPool<T> {
 public:
  ComponentPoolWithUpdater(BaseView& view)
      : ComponentPool<T>(view), view_(view), updater_(view, *this) {}

 protected:
  virtual void Update(const FrameTime& frame_time) noexcept;

 private:
  // Updater that uses the update info (phase, dependencies) of the component
  // type T.
  class Updater : public UpdateSystem::Updater<T> {
   public:
    Updater(BaseView& view, ComponentPoolWithUpdater<T>& pool);

    void Update(const FrameTime& frame_time) override;

#if IMP_RUNTIME(DEV)
    bool ShouldRunInEditMode() const override { return true; }
#endif

   private:
    ComponentPoolWithUpdater<T>& pool_;
  };

  void UpdateUsingAllocator(const FrameTime& frame_time) noexcept;
  void UpdateWithoutAllocator(const FrameTime& frame_time) noexcept;

  BaseView& view_;
  Updater updater_;
};

template <typename T>
void ComponentPoolWithUpdater<T>::Update(const FrameTime& frame_time) noexcept {
  if (this->allocator_) {
    UpdateUsingAllocator(frame_time);
  } else {
    UpdateWithoutAllocator(frame_time);
  }
}

template <typename T>
void ComponentPoolWithUpdater<T>::UpdateUsingAllocator(
    const FrameTime& frame_time) noexcept {
  // Logic intentionally duplicated from
  // ComponentPool::UpdateEachUsingAllocator because the compiler
  // optimizes the code better when the logic is explicitly inlined in this
  // function.
  // LINT.IfChange
  if constexpr (component_traits::kHasUpdateFunc<T>) {
    IMP_TRACE_NAME_TEMPLATED("Update", T);
    this->allocator_->ForEach([&frame_time, this](T* component) {
#if IMP_RUNTIME(DEV)
      // Skip components that should not be updated in editor draft/pause mode.
      if constexpr (!component_traits::kShouldRunInEditMode<T>) {
        if (editor::ShouldNotUpdate(view_.GetRegistry()) &&
            !component->IsEditorStaging()) {
          return;
        }
      }
#endif  // IMP_RUNTIME(DEV)
      if constexpr (T::kUpdateMode == T::UpdateMode::kAlwaysUpdate) {
        // Skip components where async setup is still running.
        if (!component->IsRunningAsyncSetup()) {
          component->Update(frame_time);
        }
      } else {
        // Component cannot be active until async setup is complete.
        if (component->IsActive()) {
          component->Update(frame_time);
        }
      }
    });
  }
  // LINT.ThenChange(//depot/google3/third_party/impress/core/ncsb/component_pool.h)
}

template <typename T>
void ComponentPoolWithUpdater<T>::UpdateWithoutAllocator(
    const FrameTime& frame_time) noexcept {
  // True if the editor is playing or the component is allowed to update in
  // editor draft/pause mode.
  // False means need to check for further exemptions to decide whether to
  // stop the updates.
  bool is_editor_playing_or_component_exempted = true;
#if IMP_RUNTIME(DEV)
  // Editor wants to stop updates and this type of component has not exemptions,
  // so we stop the updates.
  if constexpr (!component_traits::kShouldRunInEditMode<T>) {
    if (editor::ShouldNotUpdate(view_.GetRegistry())) {
      is_editor_playing_or_component_exempted = false;
    }
  }
#endif

  // kHasUpdateFunc check is needed because ComponentSystemWithPool uses
  // ComponentSystemWithUpdater to receive updates even when the component has
  // no Update function.
  if constexpr (component_traits::kHasUpdateFunc<T>) {
    IMP_TRACE_NAME_TEMPLATED("Update", T);
    if constexpr (T::kUpdateMode == T::UpdateMode::kAlwaysUpdate) {
      ComponentPool<T>::ForEach([&frame_time,
                                 &is_editor_playing_or_component_exempted,
                                 this](T* component) {
#if IMP_RUNTIME(DEV)
        if (!is_editor_playing_or_component_exempted &&
            !component->IsEditorStaging()) {
          return;
        }
#endif
        component->Update(frame_time);
      });
    } else {
      ComponentPool<T>::ForEach(
          [&frame_time,
           &is_editor_playing_or_component_exempted](T* component) {
            if (component->IsActive()) {
#if IMP_RUNTIME(DEV)
              if (!is_editor_playing_or_component_exempted &&
                  !component->IsEditorStaging()) {
                return;
              }
#endif
              component->Update(frame_time);
            }
          });
    }
  }
}

template <typename T>
ComponentPoolWithUpdater<T>::Updater::Updater(BaseView& view,
                                              ComponentPoolWithUpdater<T>& pool)
    : UpdateSystem::Updater<T>(view), pool_(pool) {}

template <typename T>
void ComponentPoolWithUpdater<T>::Updater::Update(const FrameTime& frame_time) {
  pool_.Update(frame_time);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_POOL_WITH_UPDATER_H_
