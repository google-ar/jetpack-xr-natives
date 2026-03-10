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

#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"

#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"

namespace svxr {
namespace interaction_states {

namespace {

bool IsNodeOrDescendant(imp::NodeHandle potential_descendant,
                        imp::NodeHandle target) {
  if (!potential_descendant.IsValid() || !target.IsValid()) {
    return false;
  }
  imp::NodeHandle current = potential_descendant;
  while (current.IsValid()) {
    if (current == target) {
      return true;
    }
    current = current->GetParent();
  }
  return false;
}

}  // namespace

bool InteractionOwner::ReceiverIsModel(imp::NodeHandle receiver) {
  return IsNodeOrDescendant(receiver, GetModelNode());
}

bool InteractionOwner::ReceiverIsFootprint(imp::NodeHandle receiver) {
  return IsNodeOrDescendant(receiver, GetFootprintNode());
}

bool InteractionOwner::ReceiverInitiatesTranslation(imp::NodeHandle receiver) {
  return ReceiverIsFootprint(receiver);
}

}  // namespace interaction_states
}  // namespace svxr
