/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_NODE_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_NODE_MANAGER_H_

#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/math/transform.h"

namespace imp {

// Manages Impress node related operations for the ImpressApiView.
class NodeManager {
 public:
  virtual ~NodeManager() = default;

  // Any arbitrary integer is a valid playback channel ID with the one
  // exception of this special ID `kAllChannels`. This ID is a valid argument to
  // Stop(), Restart(), SetSpeedMultiplier(), IsLooping(), SetLooping() and
  // ConstrainAnimationTime() only.
  // For all other functions that accept a channel ID with type `int32_t`,
  // including any that do so indirectly via GltfAnimatorState::AnimOptions such
  // as Play(), this ID may not be used.
  constexpr static int32_t kAllChannels = -1;

  // Creates an Impress node and returns a corresponding entity ID.
  virtual int32_t CreateImpressNode() = 0;
  // Destroys an Impress node using its entity ID.
  virtual absl::Status DestroyImpressNode(int32_t node) = 0;
  // Sets the parent of an Impress node using the entity IDs of the child and
  // parent nodes.
  virtual absl::Status SetImpressNodeParent(int32_t child, int32_t parent) = 0;
  // Returns the entity ID of the parent of a node using its entity ID.
  virtual absl::StatusOr<int32_t> GetImpressNodeParent(int32_t node_id) = 0;
  // Returns the number of children for a specific node.
  virtual absl::StatusOr<int32_t> GetImpressNodeChildCount(int32_t node_id) = 0;
  // Returns the entity ID of a child at a specific index.
  virtual absl::StatusOr<int32_t> GetImpressNodeChildAt(int32_t node_id,
                                                        int32_t index) = 0;
  // Returns the name of an Impress node using its entity ID.
  virtual absl::StatusOr<absl::string_view> GetImpressNodeName(
      int32_t node_id) = 0;
  // Returns the local transform (TRS) of an Impress node using its entity ID.
  virtual absl::StatusOr<imp::Transform<float>> GetImpressNodeLocalTransform(
      int32_t node_id) = 0;
  // Sets the local transform (TRS) of an Impress node using its entity ID.
  virtual absl::Status SetImpressNodeLocalTransform(
      int32_t node_id, const imp::Transform<float>& transform) = 0;
  // Returns the transform (TRS) of an Impress node relative to a relative
  // Impress node.
  virtual absl::StatusOr<imp::Transform<float>> GetImpressNodeRelativeTransform(
      int32_t node_id, int32_t relative_node_id) = 0;
  // Sets the transform (TRS) of an Impress node relative to a relative
  // Impress node.
  virtual absl::Status SetImpressNodeRelativeTransform(
      int32_t node_id, int32_t relative_node_id,
      const imp::Transform<float>& transform) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_NODE_MANAGER_H_
