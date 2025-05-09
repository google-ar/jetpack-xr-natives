// Copyright 2025 Google LLC
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

#include "split_engine/subspace_root.h"

#include <optional>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/math/mat.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"

namespace android_xr {

// Attach the given node to the Subspace root node.
void SubspaceRoot::AttachToRoot(imp::NodeHandle node) {
  if (subspace_anchor_) {
    node->SetParent(*subspace_anchor_);
  } else {
    node->SetParent(subspace_root_);
  }
};

// Updates the position of the anchor node relative to the Subspace node in
// CPM.
absl::Status SubspaceRoot::UpdateAnchor(AnchorType anchor) {
  if (current_anchor_ == anchor) {
    return absl::OkStatus();
  }
  current_anchor_ = anchor;
  switch (anchor) {
    case AnchorType::kTaskSpace:
      if (subspace_anchor_) {
        auto children = (*subspace_anchor_)->GetChildren();
        for (imp::NodeHandle child : children) {
          child->SetParent(subspace_root_);
        }
        view_.DestroyNode(*subspace_anchor_);
        subspace_anchor_ = std::nullopt;
      }
      break;
    case AnchorType::kWorldSpace:
      if (!subspace_anchor_) {
        auto children = subspace_root_->GetChildren();
        subspace_anchor_ = subspace_root_->CreateChildNode();
        for (auto& child : children) {
          child->SetParent(subspace_anchor_.value());
        }
        UpdateAnchorTransform();
      }
      break;
    default:
      return absl::InvalidArgumentError("Invalid anchor type");
  }
  return absl::OkStatus();
};

absl::Status SubspaceRoot::UpdateSubspaceTransform(
    const imp::mat4f& transform) {
  root_transform_ = transform;
  subspace_root_->SetLocalTrs(transform);
  if (current_anchor_ == AnchorType::kWorldSpace) {
    UpdateAnchorTransform();
  }
  return absl::OkStatus();
};

void SubspaceRoot::UpdateAnchorTransform() {
  if (current_anchor_ == AnchorType::kWorldSpace) {
    // If we are in World Space the optional variable subspace_anchor_ should
    // always be set.
    (*subspace_anchor_)
        ->SetLocalTrs(
            filament::math::details::matrix::inverse(root_transform_));
  }
}

}  // namespace android_xr
