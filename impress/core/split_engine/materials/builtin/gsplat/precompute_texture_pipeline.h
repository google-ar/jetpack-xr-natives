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

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/isf_info.h"
#include "core/render/texture.h"
#include "core/render_passes/texture_pipeline_renderer.h"
#include "core/view/framework/render/mesh_renderer.h"

namespace imp::split_engine {

// Manages textures containing computed data.
//
// Manages a mesh renderer and a TexturePipelineRenderer which will render
// an intermediate texture.
// See (broken link) for more details.
class PrecomputeTexturePipeline : public imp::Component {
 public:
  // Setup is asynchronous to load materials.
  // After materials are loaded, this will be completed synchronously.
  imp::Future<absl::Status> Setup();
  void Cleanup();
  BorrowedMaterialPtr BorrowMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;
  BorrowedTexturePtr BorrowTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  absl::Status ResizePassTexture(int pass_index, imp::uint2 texture_size);

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
};

}  // namespace imp::split_engine
#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_GSPLAT_PRECOMPUTE_TEXTURE_PIPELINE_H_
