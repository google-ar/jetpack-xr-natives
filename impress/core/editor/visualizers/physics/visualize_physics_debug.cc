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

#include "core/editor/visualizers/physics/visualize_physics_debug.h"

#include "core/common/log.h"
#include "core/config.h"
#include "core/editor/common/notifying_property.h"
#include "core/editor/editor.h"
#include "core/physics/physics_manager.h"
#include "core/view/base_view.h"

namespace imp::editor {
VisualizePhysicsDebug::VisualizePhysicsDebug(BaseView& view,
                                             bool use_view_dispatcher)
    : view_(view),
      physics_manager_(
          &view_.GetRegistry().GetOrCreate<PhysicsManager>(view_)) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();

  const auto set_enabled = [](bool enabled, bool /*old*/) {
    if (enabled) {
      IMP_LOG(imp::INFO) << "Visualizing all physics visualizers";
    } else {
      IMP_LOG(imp::INFO) << "Not visualizing any physics visualizers";
    }
  };

  show_physics_visualizer_binding_ = PropertyBinding<bool>(
      editor.GetSettings().show_physics_visualizer_enabled, set_enabled);
}

void VisualizePhysicsDebug::DrawImGui() {
#if IMP_RUNTIME(DEV)
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();

  if (*editor.GetSettings().show_physics_visualizer_enabled) {
    physics_manager_->DrawDebug();
  }
#endif
}

}  // namespace imp::editor
