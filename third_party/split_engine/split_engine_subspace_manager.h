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

#ifndef THIRD_PARTY_SPLIT_ENGINE_SPLIT_ENGINE_SUBSPACE_MANAGER_H_
#define THIRD_PARTY_SPLIT_ENGINE_SPLIT_ENGINE_SUBSPACE_MANAGER_H_

#include <cstdint>
#include <string>

#include "absl/status/status.h"
#include "core/math/mat.h"
#include "split_engine/input/split_engine_input_event.h"
#include "split_engine/subspace_root.h"

namespace android_xr {

// An interface for managing subspaces which are registered by the Java/Kotlin
// side of the application, and accessed by the native side.
class SplitEngineSubspaceManager {
 public:
  virtual ~SplitEngineSubspaceManager() = default;

  virtual void DestroyAllSubspaces() = 0;

  // Returns the next available subspace ID. The returned ID is guaranteed to
  // be unique to be scoped to the lifetime of the App.
  virtual uint32_t GetNextSubspaceId() = 0;

  // Registers a subspace.
  virtual void RegisterSubspace(uint32_t subspace_id,
                                uint32_t existing_root_entity_id) = 0;

  // Registers the node to the identified subspace.
  virtual void CreateSubspace(uint32_t subspace_id,
                                      std::string app_name) = 0;

  // Destroys the subspace.
  virtual void DestroySubspace(uint32_t subspace_id) = 0;

  // Forwards the input event to the identified node under the subspace.
  virtual absl::Status ForwardInputEvent(
      uint32_t subspace_id, android_xr::SplitEngineInputEvent& input_event) = 0;

  // Forwards the subspace transforms to the subspace node.
  virtual void ForwardSubspaceTransform(
      uint32_t subspace_id, const imp::mat4f& subspace_transform) = 0;

  // Anchors the identified subspace to a specific location in space.
  virtual void UpdateSubspaceAnchor(
      uint32_t subspace_id, SubspaceRoot::AnchorType anchor_type) = 0;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_SPLIT_ENGINE_SUBSPACE_MANAGER_H_
