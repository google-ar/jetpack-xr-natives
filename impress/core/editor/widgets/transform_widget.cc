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

#include "core/editor/widgets/transform_widget.h"

#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_system.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

namespace {
constexpr float kFarAway = 1e9f;
}

void TransformWidget::Setup() {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  Dispatcher& editor_dispatcher = editor.GetDispatcher();

  editor_dispatcher.Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        // We only support single selection for the transform widget.
        active_model_ = GetView()
                            .GetRegistry()
                            .Get<Editor>()
                            ->get()
                            .GetSingleSelectedNode();
        GetNode()->SetEnabled(active_model_ ? true : false);
      },
      this);
}

void TransformWidget::Update(const FrameTime& frame_time) {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  if (!editor.GetEditorRoot()->IsActive()) return;
  if (!active_model_) {
    // TODO: Remove this once we have a better way to hide the
    // transform widget.
    GetNode()->SetEnabled(false);
    return;
  }

  float3 camera_position = editor.GetCamera()->GetNode()->GetWorldPosition();
  float3 camera_to_target = active_model_->GetWorldPosition() - camera_position;
  GetNode()->SetWorldPosition(camera_position + normalize(camera_to_target));
  // TODO: Enable for local mode.
  // GetNode()->SetWorldRotation(active_model_->GetWorldRotation());
}

}  // namespace imp::editor
