// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/ncsb/update_system.h"

#include "core/common/trace.h"
#include "core/config.h"
#include "core/ncsb/component.h"
#include "core/ncsb/node.h"
#include "core/ncsb/system.h"
#include "core/ncsb/update_id.h"
#include "core/ncsb/update_phase.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

#if IMP_RUNTIME(DEV)
#include "core/editor/editor_info.h"
#endif

namespace imp {

UpdateSystem::UpdateSystem(BaseView* view) : System(view) {}

void UpdateSystem::Update(UpdatePhase phase, const FrameTime& frame_time) {
  IMP_TRACE();

  bool editor_stopped = false;
#if IMP_RUNTIME(DEV)
  if (editor::ShouldNotUpdate(GetView().GetRegistry())) {
    editor_stopped = true;
  }
#endif

  // First send the pre components update event.
  // Only send it before the early update phase.
  if (phase == UpdatePhase::kPreDefault) {
    IMP_TRACE_BLOCK("PreUpdateEvent");
    PreComponentsUpdateEvent pre_components_update_event(frame_time);
    GetView().GetDispatcher().Send(pre_components_update_event);
  }

  // Now update the components.
  {
    IMP_TRACE_BLOCK("UpdateComponents");
    UpdateGraph& update_graph = phases_[phase];
    update_graph.TraverseExtras(
        [&frame_time, &editor_stopped](BaseUpdater* updater) {
#if IMP_RUNTIME(DEV)
          // Editor can stop global updaters early, as all exceptions are for
          // updaters that are components.
          if (updater && !updater->ShouldRunInEditMode() && editor_stopped) {
            return;
          }
#endif
          // If the updater is null, that means no updater of the type
          // represented by update_id has been added to the UpdateSystem. That
          // can happen here if an updater was added with update
          // dependencies/dependees that haven't been added.
          if (updater) {
            updater->Update(frame_time);
          }
        });
  }

  // Finally, send the post components update event.
  // Only send it after the late update phase.
  if (phase == UpdatePhase::kPostDefault) {
    IMP_TRACE_BLOCK("PostUpdateEvent");
    PostComponentsUpdateEvent post_components_update_event(frame_time);
    GetView().GetDispatcher().Send(post_components_update_event);
  }
}

void UpdateSystem::RemoveUpdater(UpdatePhase phase, UpdateId update_id) {
  phases_[phase].SetExtra(update_id, nullptr);
}

}  // namespace imp
