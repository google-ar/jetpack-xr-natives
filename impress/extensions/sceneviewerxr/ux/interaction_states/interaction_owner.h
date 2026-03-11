/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_INTERACTION_OWNER_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_INTERACTION_OWNER_H_

#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "core/common/smooth.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"

namespace svxr {
namespace interaction_states {

class InteractionOwner {
 public:
  virtual ~InteractionOwner() = default;

  virtual InteractionMode& GetInteractionData() = 0;

  virtual imp::ComponentHandle<Footprint> GetFootprint() = 0;
  virtual imp::NodeHandle GetFootprintNode() = 0;
  virtual imp::NodeHandle GetModelNode() = 0;
  virtual imp::NodeHandle GetRigNode() = 0;

  virtual imp::float3 GetHeadPosition() = 0;

  virtual imp::Smooth<float>& GetModelLogScale() = 0;
  virtual float GetResetLogScale() = 0;
  virtual void ResetRigPosition() = 0;
  virtual void ToggleResetScaleType() = 0;

  virtual bool IsTalkbackEnabled() = 0;
  virtual bool IsIdleTimeoutEnabled() = 0;

  bool ReceiverIsModel(imp::NodeHandle receiver) {
    return receiver == GetModelNode();
  }

  bool ReceiverIsFootprint(imp::NodeHandle receiver) {
    return receiver == GetFootprintNode();
  }

  bool ReceiverInitiatesTranslation(imp::NodeHandle receiver) {
    return ReceiverIsFootprint(receiver);
  }
};

}  // namespace interaction_states
}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_INTERACTION_OWNER_H_
