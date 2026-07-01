// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_DEGENERATE_TRIANGLE_RENDERER_FOR_DRIVER_BUG_WORKAROUND_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_DEGENERATE_TRIANGLE_RENDERER_FOR_DRIVER_BUG_WORKAROUND_H_

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/render/mesh_renderer.h"

namespace imp {

// A component to work around a Qualcomm Vulkan driver heuristic that switches
// from tiled rendering to direct mode if no standard geometry is rendered.
// It creates a child node containing a single degenerate triangle rendered
// with a blank material.
class DegenerateTriangleRendererForDriverBugWorkaround : public Component {
 public:
  DegenerateTriangleRendererForDriverBugWorkaround() = default;

  Future<absl::Status> Setup();
  void Cleanup();

 private:
  NodeHandle degenerate_triangle_node_;
  ComponentHandle<MeshRenderer> degenerate_triangle_renderer_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_DEGENERATE_TRIANGLE_RENDERER_FOR_DRIVER_BUG_WORKAROUND_H_
