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
#include <type_traits>

#include "core/common/trace.h"
#include "core/config.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_traits.h"
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

  template <typename Fn>
  void ForEach(Fn fn);

  template <typename Fn>
  void UpdateEach(Fn fn);

#if IMP_RUNTIME(DEV)
  bool IsExcludedFromEditor() const override;
  absl::string_view GetTypeName() const override;
  std::optional<HashValue> GetStateTypeUrlHash() const override;
  void DrawEditorUi(utils::Entity entity) override;

#endif  // IMP_RUNTIME(DEV)

 protected:
  ComponentIndex EmplaceBack() noexcept override;
  void Cleanup(ComponentIndex instance) noexcept override;
};

template <typename T>
ComponentPool<T>::ComponentPool(BaseView& view) : BaseComponentPool(view) {}

template <typename T>
void ComponentPool<T>::Cleanup(ComponentIndex instance) noexcept {
#if IMP_RUNTIME(DEV)
  if constexpr (!component_traits::kShouldRunInEditMode<T>) {
    if (editor::IsInEditMode(GetView().GetRegistry())) {
      return;
    }
  }
#endif

  IMP_TRACE_TEMPLATED(T);

  GetComponents().template At<T>(instance).Cleanup();
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
BaseComponentPool::ComponentIndex ComponentPool<T>::EmplaceBack() noexcept {
  return GetComponents().template EmplaceBack<T>();
}

template <typename T>
template <typename Fn>
void ComponentPool<T>::ForEach(Fn fn) {
  GetComponents().template ForEach<T>(std::move(fn));
}

template <typename T>
template <typename Fn>
void ComponentPool<T>::UpdateEach(Fn fn) {
  GetComponents().template UpdateEach<T>(std::move(fn));
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
  if (Has(entity)) {
    ComponentIndex instance = Get(entity);
    GetComponents().template At<T>(instance).DrawEditorUi();
  }
}

#endif  // IMP_RUNTIME(DEV)

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_POOL_H_
