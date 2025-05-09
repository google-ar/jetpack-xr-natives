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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_CAMERA_CAMERA_TRANSFORMS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_CAMERA_CAMERA_TRANSFORMS_H_

#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/common/bit_flag.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/camera/camera_component.h"

namespace imp {

// TODO: Replace ViewTarget with NodeGroupHandle class later,
// when a generic NodeGroupHandle class has been implemented.
using ViewTarget =
    std::variant<NodeHandle, std::vector<NodeHandle>, absl::string_view>;

enum CameraHelperOptions : BitFlag {
  kNone = 0,
  kIncludeDisabled = (1 << 1),
  kIncludeDescendants = (1 << 2)
};

// Checks if the node has a valid renderable mesh with volume.
bool HasValidMesh(NodeHandle node, CameraHelperOptions = {});

// Preserving the camera's current rotation, moves the camera
// to a new position where it can see the target in its entirety.
absl::Status MoveIntoView(ComponentHandle<CameraComponent> camera,
                          const ViewTarget& target,
                          CameraHelperOptions = kNone);

// Filters and returns the nodes that are visible from the given camera.
// ViewTarget of NodeHandle - check if the node is visible.
// ViewTarget of NodeHandle array - retrieves visible nodes.
// ViewTarget of string - retrieves nodes of the given visibility group, if
// the visibility group is also found on the camera.
// Inactive nodes are included by default; can optionally exclude them.
std::vector<NodeHandle> FilterNodesViewableByCamera(
    ComponentHandle<CameraComponent> camera, const ViewTarget& target,
    CameraHelperOptions = kIncludeDisabled);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_CAMERA_CAMERA_TRANSFORMS_H_
