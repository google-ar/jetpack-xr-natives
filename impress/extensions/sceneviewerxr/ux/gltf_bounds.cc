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

#include "extensions/sceneviewerxr/ux/gltf_bounds.h"

#include "core/common/log.h"
#include "absl/strings/match.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "core/common/filament_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/animation/animator_events.proto.imp.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"

namespace svxr {

void GltfBounds::Setup() {
  if (auto gltf_scene = GetNode()->GetComponent<imp::GltfScene>()) {
    // Toggled to true if any node is tagged as a bounds override.
    bool has_bounds_override = false;

    gltf_scene->ForAllNodes([this, &has_bounds_override](imp::NodeHandle node) {
      bool is_node_tagged_bounds_override =
          absl::StrContains(node->GetName(), "_boundsoverride");

      if (is_node_tagged_bounds_override || !has_bounds_override) {
        if (is_node_tagged_bounds_override && !has_bounds_override) {
          // Clears any previously added nodes that were added before knowing a
          // bounds override exists.
          bounds_nodes_.clear();
          has_bounds_override = true;
        }

        bounds_nodes_.push_back(node);
      }
    });
  }

  // Invalidates bounds whenever node's animation is advanced.
  GetNode()->Connect(
      [this](const imp::PlaybackUpdatedEvent&) mutable { UpdateBounds(); });

  UpdateBounds();
}

const imp::Box& GltfBounds::GetLocalBounds() const { return bounds_; }

void GltfBounds::UpdateBounds() {
  filament::RenderableManager& renderable_manager =
      imp::BaseView::GetSharedEngine()->getRenderableManager();
  bounds_ = imp::NilBounds();

  auto gltf_renderer = GetNode()->GetComponent<imp::GltfRenderer>();
  if (!gltf_renderer) {
    return;
  }

  imp::NodeHandle model_root = gltf_renderer->GetModelRoot();

  for (imp::NodeHandle node : bounds_nodes_) {
    if (!node.IsValid()) {
      IMP_LOG(imp::WARNING) << "attempting to get bounds of invalid node";
      continue;
    }

    if (filament::RenderableManager::Instance renderable_instance =
            renderable_manager.getInstance(node->GetEntity())) {
      imp::mat4f local_from_node = node->GetLocalTrs();
      for (imp::NodeHandle cursor = node->GetParent(); cursor != model_root;
           cursor = cursor->GetParent()) {
        local_from_node = cursor->GetLocalTrs() * local_from_node;
      }
      imp::Box local_bounds = imp::TransformBounds(
          renderable_manager.getAxisAlignedBoundingBox(renderable_instance),
          local_from_node);
      bounds_.unionSelf(local_bounds);
    }
  }

  Send(BoundsUpdated{});
}

}  // namespace svxr
