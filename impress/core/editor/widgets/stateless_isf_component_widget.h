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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_STATELESS_ISF_COMPONENT_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_STATELESS_ISF_COMPONENT_WIDGET_H_

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/common/invocable.h"
#include "core/editor/function_command.h"
#include "core/editor/widgets/component_editor_helpers.h"
#include "core/editor/widgets/fallback_component_widget.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::editor {

// A widget that displays a component that includes Stateless IsfInfo.
template <typename T>
class StatelessIsfComponentWidget : public FallbackComponentWidget {
 public:
  explicit StatelessIsfComponentWidget(ComponentHandle<T> component);

  Invocable<void()> OnCloseButton() override;

 protected:
  ComponentHandle<T> GetComponent() const;

  const SceneMetadata::ComponentSource* GetBaseComponentSource() const override;

 private:
  ComponentHandle<T> component_;
};

template <typename T>

StatelessIsfComponentWidget<T>::StatelessIsfComponentWidget(
    ComponentHandle<T> component)
    : FallbackComponentWidget(
          component->GetNode(),
          component->GetNode()
              ->GetView()
              .GetComponentManager()
              .GetComponentPoolById(component->GetComponentId())),
      component_(component) {}

template <typename T>
ComponentHandle<T> StatelessIsfComponentWidget<T>::GetComponent() const {
  return component_;
}

template <typename T>
Invocable<void()> StatelessIsfComponentWidget<T>::OnCloseButton() {
  // If this component comes from a base isf file then it can't be removed.
  if (GetBaseComponentSource()) {
    return {};
  }

  return [this]() {
    NodeHandle node = GetComponent()->GetNode();
    GetCommandManager().template PerformCommand<FunctionCommand>(
        [node]() {
          if (node) {
            node->RemoveComponent<T>();
          }
        },
        [node]() -> absl::Status {
          if (node) {
            MP_RETURN_IF_ERROR(AddComponentSync<T>(node).status());
          }
          return absl::OkStatus();
        });
  };
}

template <typename T>
const SceneMetadata::ComponentSource*
StatelessIsfComponentWidget<T>::GetBaseComponentSource() const {
  ComponentHandle<SceneMetadata> metadata = GetMetadata();
  if (!metadata) {
    return nullptr;
  }

  return metadata->GetBaseComponentSource(T::IsfInfo::kTypeUrlHash);
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_STATELESS_ISF_COMPONENT_WIDGET_H_
