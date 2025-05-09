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

#ifndef THIRD_PARTY_SPLIT_ENGINE_SUBSPACE_EVENTS_H_
#define THIRD_PARTY_SPLIT_ENGINE_SUBSPACE_EVENTS_H_

#include <cstdint>
#include <string>

#include "core/ncsb/dispatcher/event.h"
#include "split_engine/subspace_root.h"

namespace android_xr {

// Event sent to request the creation of a subspace to the impress view
// The event contains the name of the subspace used as an identification token
// and the root node of the subspace that nodes in the subspace should be
// parented to.
struct OnSubspaceCreatedEvent : public imp::Event {
  explicit OnSubspaceCreatedEvent(std::string subspace_name,
                                  SubspaceRoot& subspace_root,
                                  uint32_t subspace_id)
      : subspace_name(subspace_name), subspace_root(subspace_root) {}

  std::string subspace_name;
  SubspaceRoot& subspace_root;
  uint32_t subspace_id;
};

// Event sent to signal the destruction of a subspace to the impress view
// The event contains the subspace id of the subspace that has been destroyed.
// This event is sent out immediately prior to the subspace node being
// destroyed.
struct OnSubspaceDestroyedEvent : public imp::Event {
  explicit OnSubspaceDestroyedEvent(uint32_t subspace_id)
      : subspace_id(subspace_id) {}
  uint32_t subspace_id;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_SUBSPACE_EVENTS_H_
