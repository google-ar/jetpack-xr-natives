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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_RENDER_STATE_VALIDATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_RENDER_STATE_VALIDATOR_H_

#include "core/ncsb/update_phase.h"
#include "core/ncsb/update_system.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Utility to perform extra validation each frame that the state of the scene
// graph is valid for rendering.
//
// This can be used as early detection of issues
// prior to filament crashing on the render thread, and is useful for
// determining the root cause of issues by logging information about the state
// of the scene graph that led to the issue inside filament.
//
// NOTE: This does *NOT* catch all issues.
//
// Currently, it only detect when a MeshRenderer has an invalid filament
// texture assigned to it.
// TODO: Expand this to additional checks.
class RenderStateValidator
    : public UpdateSystem::Updater<RenderStateValidator> {
 public:
  // Update at the very end so that validation occurs after all other frame
  // logic is done.
  //
  // If there is other work being done during kEnd that must be validated, it
  // can specify RenderStateValidator in UpdateDependents to ensure the
  // validator happens afterwards.
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEnd;

  explicit RenderStateValidator(BaseView& view);

  void Update(const FrameTime& frame_time) override;

 private:
  BaseView& view_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_RENDER_STATE_VALIDATOR_H_
