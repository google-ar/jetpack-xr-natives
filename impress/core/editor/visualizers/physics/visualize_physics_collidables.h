/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_PHYSICS_VISUALIZE_PHYSICS_COLLIDABLES_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_PHYSICS_VISUALIZE_PHYSICS_COLLIDABLES_H_

#include "absl/strings/string_view.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/physics/physics_manager.h"
#include "core/view/base_view.h"

namespace imp::editor {
// Widget to visualize physics collidables.
class VisualizePhysicsCollidables : public Widget, public Rememberer {
 public:
  VisualizePhysicsCollidables(BaseView& view, bool use_view_dispatcher = false);

  void DrawImGui() override;

  absl::string_view GetName() const override {
    return "##Visualize Physics Collidables";
  }

 private:
  BaseView& view_;
  PhysicsManager* physics_manager_;
  bool show_all_physics_colliders_enabled_ = false;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_PHYSICS_VISUALIZE_PHYSICS_COLLIDABLES_H_
