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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_GSPLAT_PRECOMPUTE_TEXTURE_PIPELINE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_GSPLAT_PRECOMPUTE_TEXTURE_PIPELINE_H_

#include <optional>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/box.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/isf_info.h"
#include "core/render/mesh_renderer.h"
#include "core/render/texture.h"
#include "core/render_passes/texture_pipeline_renderer.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/resources/resource_definition.h"

namespace imp::split_engine {

// Manages textures containing computed data.
//
// Manages a mesh renderer and a TexturePipelineRenderer which will render
// an intermediate texture.
// See (broken link) for more details.
class PrecomputeTexturePipeline : public imp::Component {
 public:
  // Parameterless setup is not supported.
  imp::Future<absl::Status> Setup();
  // Setup is asynchronous to load materials.
  // After materials are loaded, this will be completed synchronously.
  imp::Future<absl::Status> Setup(
      resources::ResourceDefinition precompute_material_definition,
      std::optional<imp::Box> aabb_override = std::nullopt);
  void Cleanup();
  void OnActiveStatusChanged(bool is_active);
  BorrowedMaterialPtr BorrowMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;
  BorrowedTexturePtr BorrowTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  absl::Status ResizePassTexture(int pass_index, imp::uint2 texture_size);
  // Sets the camera for the precompute pass.
  absl::Status SetCamera(imp::ComponentHandle<imp::CameraComponent> camera);

  // Sets the projection quad for the precompute pass. This is required if the
  // Gsplat scene is rendered to a stereo offscreen texture to create Magic
  // Window look. We need to provide the projection quad to
  // TexturePipelineRenderer, so it uses the same projection matrices that are
  // going to be used for the actual stereo rendering.
  void SetProjectionQuad(
      const std::optional<imp::TexturePipelineRendererProjectionQuad>&
          projection_quad) {
    texture_pipeline_renderer_->SetProjectionQuad(projection_quad);
  }

  static constexpr absl::string_view kType =
      "split_engine.PrecomputeTexturePipeline";
  using IsfInfo = imp::StatelessIsfInfo<PrecomputeTexturePipeline, kType>;

  // Specifies TexturePipelineRenderer and MeshRenderer component won't be
  // removed until after this component. This is to give us control over
  // destroying these components ourselves to in Cleanup() to ensure that the
  // texture owned by the texture pipeline renderer (via the texture registry)
  // is not destroyed before the OwnedMaterial on this component which is
  // borrowing it.
  using CleanupDependents =
      imp::CleanupIds<imp::TexturePipelineRenderer, imp::MeshRenderer>;

 private:
  uint2 size_ = imp::kOne2;
  imp::ComponentHandle<imp::MeshRenderer> mesh_renderer_;
  imp::ComponentHandle<imp::TexturePipelineRenderer> texture_pipeline_renderer_;
  imp::OwnedMaterialPtr precompute_material_;
  imp::ComponentHandle<imp::CameraComponent> camera_override_;
};

}  // namespace imp::split_engine
#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_GSPLAT_PRECOMPUTE_TEXTURE_PIPELINE_H_
