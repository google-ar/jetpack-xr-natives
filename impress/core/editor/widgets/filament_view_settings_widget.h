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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FILAMENT_VIEW_SETTINGS_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FILAMENT_VIEW_SETTINGS_WIDGET_H_

#include <vector>

#include "absl/strings/string_view.h"
#include "core/editor/widget.h"
#include "core/editor/widgets/filament_view_settings.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Displays rendering settings of the current Filament View and allows editing.
class FilamentViewSettingsWidget : public editor::Widget {
 public:
  explicit FilamentViewSettingsWidget(BaseView& view);

  absl::string_view GetName() const override { return "Filament Settings"; }
  void DrawImGui() override;

 private:
  void DrawColorGradingUI();

  BaseView& view_;
  FilamentViewSettings view_settings_;

  // Persistent backing data for plots.
  std::vector<float> tone_map_plot_;
  std::vector<float> range_plot_;
  std::vector<float> curve_plot_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FILAMENT_VIEW_SETTINGS_WIDGET_H_
