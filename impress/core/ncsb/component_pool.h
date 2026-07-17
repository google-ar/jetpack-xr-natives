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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_POOL_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_POOL_H_

#include <memory>
#include <optional>
#include <type_traits>
#include <utility>

#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/base_pool_allocator.h"
#include "core/common/pool_allocator.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_traits.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

#if IMP_RUNTIME(DEV)
#include "core/common/type_traits.h"
#include "core/editor/editor_info.h"
#endif  // IMP_RUNTIME(DEV)

namespace imp {

template <typename T>
class ComponentPool : public BaseComponentPool {
 public:
  explicit ComponentPool(BaseView& view);

  void NotifyActive(Component* component, bool active) noexcept override;

  void Deallocate(Component* c) override;

  template <typename Fn>
  void ForEach(Fn&& fn);

  template <typename Fn>
  void UpdateEach(Fn&& fn);

#if IMP_RUNTIME(DEV)
  bool IsExcludedFromEditor() const override;
  absl::string_view GetTypeName() const override;
  std::optional<HashValue> GetStateTypeUrlHash() const override;
  void DrawEditorUi(utils::Entity entity) override;

#endif  // IMP_RUNTIME(DEV)

 protected:
  Component* Emplace(NodeHandle node, ComponentKey* out_key) noexcept override;
  void Cleanup(Component& component) noexcept override;

  template <typename Fn>
  void ForEachUsingAllocator(Fn&& fn);

  template <typename Fn>
  void UpdateEachUsingAllocator(Fn&& fn);

  std::optional<PoolAllocator<T>> allocator_;
};

template <typename T>
ComponentPool<T>::ComponentPool(BaseView& view)
    : BaseComponentPool(view, nullptr, GetComponentTypeId<T>()) {
  if (*view.GetConfig().experimental_feature_flags->enable_pool_allocator) {
    allocator_.emplace();
    // Assign the allocator to the base class after it is created. After the
    // enable_pool_allocator flag is removed this can be passed via the
    // constructor instead. In the meantime, this is the simplest safe way to do
    // it given base constructors must be called prior to derived constructors.
    this->SetBaseAllocator(&allocator_.value());
  }
}

template <typename T>
void ComponentPool<T>::Deallocate(Component* c) {
  if (allocator_) {
    allocator_->Deallocate(static_cast<T*>(c));
  } else {
    delete static_cast<T*>(c);
  }
}

template <typename T>
void ComponentPool<T>::Cleanup(Component& component) noexcept {
#if IMP_RUNTIME(DEV)
  if constexpr (!component_traits::kShouldRunInEditMode<T>) {
    if (editor::IsInEditMode(GetView().GetRegistry())) {
      return;
    }
  }
#endif

  IMP_TRACE_TEMPLATED(T);

  static_cast<T&>(component).Cleanup();
}

template <typename T>
void ComponentPool<T>::NotifyActive(Component* component,
                                    bool active) noexcept {
  T* casted_component = static_cast<T*>(component);
  casted_component->SetActiveFlagInternal(active);

#if IMP_RUNTIME(DEV)
  if constexpr (!component_traits::kShouldRunInEditMode<T>) {
    if (editor::IsInEditMode(GetView().GetRegistry())) {
      return;
    }
  }
#endif

  IMP_TRACE_NAME_TEMPLATED("OnActiveStatusChanged", T);

  casted_component->OnActiveStatusChanged(active);
}

template <typename T>
Component* ComponentPool<T>::Emplace(NodeHandle node,
                                     ComponentKey* out_key) noexcept {
  if (allocator_) {
    using AllocateResult = typename PoolAllocator<T>::AllocateResult;
    AllocateResult result = allocator_->Allocate();
    *out_key = result.key;
    return result.ptr;
  } else {
    return new T();
  }
}

template <typename T>
template <typename Fn>
void ComponentPool<T>::ForEach(Fn&& fn) {
  if (allocator_) {
    ForEachUsingAllocator(std::forward<Fn>(fn));
  } else {
    GetComponents().template ForEach<T>(std::forward<Fn>(fn));
  }
}

template <typename T>
template <typename Fn>
void ComponentPool<T>::UpdateEach(Fn&& fn) {
  if (allocator_) {
    UpdateEachUsingAllocator(std::forward<Fn>(fn));
  } else {
    GetComponents().template UpdateEach<T>(std::forward<Fn>(fn));
  }
}

template <typename T>
template <typename Fn>
void ComponentPool<T>::ForEachUsingAllocator(Fn&& fn) {
  allocator_->ForEach([fn = std::forward<Fn>(fn)](T* component) mutable {
    if (!component->IsRunningAsyncSetup()) {
      fn(component);
    }
  });
}

template <typename T>
template <typename Fn>
void ComponentPool<T>::UpdateEachUsingAllocator(Fn&& fn) {
  // Logic intentionally duplicated from
  // ComponentPoolWithUpdater::UpdateUsingAllocator because the compiler
  // optimizes the code better when the logic is explicitly inlined in this
  // function.
  // LINT.IfChange
  allocator_->ForEach([fn = std::forward<Fn>(fn), this](T* component) {
#if IMP_RUNTIME(DEV)
    // Skip components that should not be updated in editor draft/pause mode.
    if constexpr (!component_traits::kShouldRunInEditMode<T>) {
      if (editor::ShouldNotUpdate(this->GetView().GetRegistry()) &&
          !component->IsEditorStaging()) {
        return;
      }
    }
#endif  // IMP_RUNTIME(DEV)
    if constexpr (T::kUpdateMode == T::UpdateMode::kAlwaysUpdate) {
      // Skip components where async setup is still running.
      if (!component->IsRunningAsyncSetup()) {
        fn(component);
      }
    } else {
      // Skip components where async setup is still running.
      if (component->IsActive()) {
        fn(component);
      }
    }
  });
  // LINT.ThenChange(//depot/google3/third_party/impress/core/ncsb/component_pool_with_updater.h)
}

#if IMP_RUNTIME(DEV)
template <typename T>
bool ComponentPool<T>::IsExcludedFromEditor() const {
  return T::kExcludeFromEditor;
}

template <typename T>
absl::string_view ComponentPool<T>::GetTypeName() const {
  return type_traits::kTypeName<T>;
}

template <typename T>
std::optional<HashValue> ComponentPool<T>::GetStateTypeUrlHash() const {
  if constexpr (component_traits::kIsIsfInfoDefined<T>) {
    return T::IsfInfo::kTypeUrlHash;
  }

  return std::nullopt;
}

template <typename T>
void ComponentPool<T>::DrawEditorUi(utils::Entity entity) {
  if (Component* component = TryGetRawComponentFromEntity(entity)) {
    static_cast<T*>(component)->DrawEditorUi();
  }
}

#endif  // IMP_RUNTIME(DEV)

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_POOL_H_
