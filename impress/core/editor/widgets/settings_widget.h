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

#ifndef THIRD_PARTY_IMPRESS_EDITOR_WIDGETS_SETTINGS_WIDGET_H_
#define THIRD_PARTY_IMPRESS_EDITOR_WIDGETS_SETTINGS_WIDGET_H_

#include "absl/strings/string_view.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/widget.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Shows a list of settings for the Impress editor.
// Some of these settings have no output for apps, like toggling the grid.

class SettingsWidget : public editor::Widget {
 public:
  explicit SettingsWidget(BaseView& view)
      : view_(view), editor_(*view_.GetRegistry().Get<editor::Editor>()) {}
  void DrawImGui() override;
  absl::string_view GetName() const override { return "Settings"; }

 private:
  BaseView& view_;
  editor::Editor& editor_;
  bool grid_enabled_ = true;
  bool skybox_enabled_ = true;
  bool show_bounds_enabled_ = false;
  bool show_all_colliders_enabled_ = false;
  bool show_all_origins_enabled_ = false;
  bool show_physics_visualizer_enabled_ = false;
  bool vertex_selection_enabled_ = false;
  bool load_mesh_data_on_cpu_enabled_ = false;
  bool bvh_mesh_collision_acceleration_enabled_ = false;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_EDITOR_WIDGETS_SETTINGS_WIDGET_H_
