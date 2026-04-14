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

#ifndef THIRD_PARTY_ARCORE_AR_IMP_CORE_EDITOR_WIDGET_UI_SYSTEM_H
#define THIRD_PARTY_ARCORE_AR_IMP_CORE_EDITOR_WIDGET_UI_SYSTEM_H

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/strings/string_view.h"
#include "core/common/hash.h"
#include "core/editor/layout/layout_composer.h"
#include "core/editor/widget.h"
#include "core/editor/widget_layout_info.h"
#include "core/editor/widgets/window/window_configuration.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Manages all of the UI Widgets in the view and calls their ImGui rendering
// functions.
class WidgetUiSystem : public System {
 public:
  // A LayoutComposer may be passed in here to render ImGui UI in an Impress
  // Editor, platform-aware layout.
  //
  // If WidgetUiSystem is being used outside the Impress Editor and the Widgets
  // are unrelated to the Impress Editor, a LayoutComposer is not needed here.
  explicit WidgetUiSystem(
      BaseView* view, bool enabled = true,
      std::unique_ptr<LayoutComposer> layout_composer = nullptr);
  // Adds the ImGui UI Widget to be managed and have their rendering function
  // called by the system.
  template <typename T, typename... Args>
  T* AddWidget(const WidgetLayoutInfo& layout_info, Args&&... args);

  // Adds an existing widget to the system. Widget must be removed with the
  // RemoveWidget(Widget* widget) variant as the specific type is unknown.
  void AddWidget(const WidgetLayoutInfo& layout_info,
                 std::unique_ptr<Widget> widget);

  // Gets a widget of type T if one exists, otherwise returns nullptr.
  // If more than one widget of type T exist, it returns the first one that was
  // added.
  template <typename T>
  T* GetWidget();

  // Removes a widget of type T if one exists, otherwise does nothing.
  // If more than one widget of type T exist, it removes the first one that was
  // added.
  template <typename T>
  void RemoveWidget();

  // Removes the widget passed in if it exists, otherwise does nothing.
  void RemoveWidget(Widget* widget);

  // Returns true if the Widget UI is enabled.
  bool IsEnabled() const { return enabled_; }

  // Enables or disables all UI in the system.
  void SetEnabled(bool enabled);

  void SetLayoutComposer(std::unique_ptr<LayoutComposer> layout_composer);

  bool Is2DLargeScreenLayout() const;

 private:
  void ProcessPendingRemoves();
  void AddWindowConfiguration(absl::string_view name,
                              const WidgetLayoutInfo& layout_info);

  bool ShouldDrawWidget(const WidgetLayoutInfo& layout_info,
                        const Widget& widget) const;
  void UpdateSpatialUiCanvas();

  using WidgetEntry = std::pair<std::unique_ptr<Widget>, WidgetLayoutInfo>;

  // Handles widget drawing and platform-dependent layout logic.
  std::unique_ptr<LayoutComposer> layout_composer_;
  // Records the visibility of certain windows in the Impress editor. Currently
  // only used for the multiple windows layout.
  WindowConfiguration& window_configuration_;

  std::vector<WidgetEntry> widgets_;
  // Keep track of the types of the widgets to remove them from the widgets_
  // list if needed.
  std::vector<HashValue> type_hashes_;
  // Tracks which widgets are being removed this frame.
  absl::flat_hash_set<Widget*> pending_removes_;
  // Whether the UI widgets in the system are shown or not.
  bool enabled_;
};

template <typename T, typename... Args>
T* WidgetUiSystem::AddWidget(const WidgetLayoutInfo& layout_info,
                             Args&&... args) {
  std::unique_ptr<T> widget = std::make_unique<T>(std::forward<Args>(args)...);
  T* widget_ptr = widget.get();
  widgets_.push_back({std::move(widget), layout_info});
  type_hashes_.push_back(type_traits::kTypeHash<T>);

  AddWindowConfiguration(widget_ptr->GetName(), layout_info);
  return widget_ptr;
}

template <typename T>
T* WidgetUiSystem::GetWidget() {
  auto iter = std::find(type_hashes_.begin(), type_hashes_.end(),
                        type_traits::kTypeHash<T>);
  if (iter == type_hashes_.end()) return nullptr;
  size_t index = iter - type_hashes_.begin();
  return static_cast<T*>(widgets_.at(index).first.get());
}

template <typename T>
void WidgetUiSystem::RemoveWidget() {
  auto iter = std::find(type_hashes_.begin(), type_hashes_.end(),
                        type_traits::kTypeHash<T>);
  if (iter == type_hashes_.end()) return;
  size_t index = iter - type_hashes_.begin();
  pending_removes_.insert(widgets_.at(index).first.get());
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_ARCORE_AR_IMP_CORE_EDITOR_WIDGET_UI_SYSTEM_H
