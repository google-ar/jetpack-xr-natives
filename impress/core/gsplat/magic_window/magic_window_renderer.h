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

#ifndef THIRD_PARTY_IMPRESS_CORE_GSPLAT_MAGIC_WINDOW_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_GSPLAT_MAGIC_WINDOW_RENDERER_H_

#include <optional>

#include "absl/status/status.h"
#include "core/common/owned_ptr.h"
#include "core/gsplat/magic_window/magic_window_renderer_state.proto.imp.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "imp.h"
#include "split_engine/materials/gsplat_material_serializer.h"

namespace imp {

// Component to render a Gsplat scene through a Magic Window.
//
// The Gsplat scene should be set up to render to the render group specified by
// MagicWindowRendererState::render_group. This component will set up a texture
// pipeline to render the Gsplat scene to an offscreen texture, and provide the
// material that renders this offscreen texture.
class MagicWindowRenderer : public imp::Component {
 public:
  Future<absl::Status> Setup();
  void Update(const FrameTime& frame_time);

  // The `quad_state` is specified in the local space of the renderer node.
  void SetProjectionQuadState(
      const std::optional<MagicWindowQuadState>& quad_state);

  // Returns the material that renders the offscreen texture that contains the
  // Gsplat scene.
  BorrowedMaterialPtr GetMaterial() {
    return material_serializer_->GetMaterial();
  }

 private:
  // Updates the Magic Window specific parameters.
  void UpdateMaterialParameters();

  MagicWindowRendererState state_;
  // Serializes and propagates changes to the material used for the offscreen
  // texture.
  OwnedPtr<android_xr::GsplatMaterialSerializer> material_serializer_;

 public:
  using IsfInfo = IsfInfo<&MagicWindowRenderer::state_>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GSPLAT_MAGIC_WINDOW_RENDERER_H_
