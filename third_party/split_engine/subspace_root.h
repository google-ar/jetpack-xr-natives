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

#ifndef THIRD_PARTY_SPLIT_ENGINE_SUBSPACE_ROOT_H_
#define THIRD_PARTY_SPLIT_ENGINE_SUBSPACE_ROOT_H_
#include <cstdint>
#include <optional>

#include "absl/status/status.h"
#include "core/common/rememberer.h"
#include "core/math/mat.h"
#include "core/ncsb/component.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace android_xr {

// A class to manage the Subspace root node.
// This class is responsible for exposing the Subspace root to split engine
// clients
// and for updating the position of the Subspaces relative to the
// Subspace node in CPM. In XROS, the anchor will be used for managing the
// positioning of the subspace.
// Anchoring is a optional feature, the Subspace root node will be used if no
// anchor is set.
class SubspaceRoot : public imp::Rememberer {
 public:
  explicit SubspaceRoot(imp::BaseView& view, imp::NodeHandle subspace_root)
      : view_(view), subspace_root_(subspace_root) {
    subspace_root_->SetLocalTrs(root_transform_);
    subspace_root_->AddComponent<Tag>();
  };

  ~SubspaceRoot() {
    if (subspace_anchor_) {
      view_.DestroyNode(*subspace_anchor_);
    }
    view_.DestroyNode(subspace_root_);
  }
  // LINT.IfChange(subspaceAnchor)
  enum class AnchorType : uint8_t {
    kTaskSpace = 0,
    kWorldSpace = 1,
  };
  // LINT.ThenChange(//depot/google3/third_party/split_engine/java/com/google/androidxr/splitengine/SubspaceNode.java:subspaceAnchor)

  // Component to mark the Subspace root node.
  class Tag : public imp::Component {};

  inline imp::NodeHandle GetNode() const {
    return subspace_anchor_ ? *subspace_anchor_ : subspace_root_;
  }

  // Attach the given node to the Subspace root node.
  void AttachToRoot(imp::NodeHandle node);

  // Updates the position of the anchor node relative to the Subspace node in
  // CPM.
  absl::Status UpdateAnchor(AnchorType anchor);

  // Updates the transform of the Subspace root node.
  absl::Status UpdateSubspaceTransform(const imp::mat4f& transform);

  // Returns the transform from world space to task space.
  imp::mat4f GetTaskFromWorldTransform() {
    return filament::math::details::matrix::inverse(root_transform_);
  }

  // Returns the transform from task space to world space.
  imp::mat4f GetWorldFromTaskTransform() { return root_transform_; }

 private:
  imp::BaseView& view_;
  imp::NodeHandle subspace_root_;
  imp::mat4f root_transform_ = {};  // Identity transform.
  AnchorType current_anchor_ = AnchorType::kTaskSpace;
  std::optional<imp::NodeHandle> subspace_anchor_ = std::nullopt;

  void UpdateAnchorTransform();
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_SUBSPACE_ROOT_H_
