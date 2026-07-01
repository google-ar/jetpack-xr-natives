/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_BLITTER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_BLITTER_H_

#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/filament/include/filament/View.h"
#include "filament/filament/include/filament/Viewport.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/view/base_view.h"

namespace imp {

// A utility class for performing full-screen blits using a material.
class Blitter {
 public:
  // Creates a Blitter asynchronously, waiting for the blit material to load.
  static Future<std::unique_ptr<Blitter>> Create(BaseView& view);

  ~Blitter();

  // Performs a blit using the provided material.
  // The material should have all necessary parameters (like source textures)
  // set. render_target: The target to render into. If null, renders to the
  // View's default target. viewport: The viewport to use. If null, uses the
  // target's full size.
  void Blit(BorrowedMaterialPtr material,
            filament::RenderTarget* render_target = nullptr,
            std::optional<filament::Viewport> viewport = std::nullopt);

  // Blits the contents of src into dest.
  // If dest is null, renders to the View's default target.
  // If dest_viewport is null, uses the dest's full resolution.
  absl::Status Blit(
      filament::RenderTarget* src, filament::RenderTarget* dest,
      std::optional<filament::Viewport> dest_viewport = std::nullopt);

 private:
  explicit Blitter(BaseView& view, OwnedMaterialPtr blit_material);

  void CreateVertexBuffer(filament::Engine& engine);
  void CreateIndexBuffer(filament::Engine& engine);

  BaseView& view_;
  filament::VertexBuffer* vb_ = nullptr;
  filament::IndexBuffer* ib_ = nullptr;

  utils::Entity camera_entity_;
  filament::Camera* camera_ = nullptr;

  filament::View* blit_view_ = nullptr;
  filament::Scene* blit_scene_ = nullptr;
  utils::Entity blit_quad_entity_;

  OwnedMaterialPtr blit_material_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_BLITTER_H_
