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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VIEWPORT_VIEWPORT_RENDER_TARGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VIEWPORT_VIEWPORT_RENDER_TARGET_H_

#include "core/math/vec.h"

namespace filament {
class Engine;
class RenderTarget;
class Texture;
}  // namespace filament

namespace imp::editor {

// Manages a Filament render target and its textures for rendering the viewport
// to a texture.
class ViewportRenderTarget {
 public:
  explicit ViewportRenderTarget(filament::Engine& engine, uint2 size);
  ~ViewportRenderTarget();

  ViewportRenderTarget(const ViewportRenderTarget&) = delete;
  ViewportRenderTarget& operator=(const ViewportRenderTarget&) = delete;
  ViewportRenderTarget(ViewportRenderTarget&&) = delete;
  ViewportRenderTarget& operator=(ViewportRenderTarget&&) = delete;

  // Resizes the render target and its textures.
  void SetSize(uint2 size);
  // Returns the size of the render target.
  uint2 GetSize() const { return size_; }

  // Returns the render target.
  filament::RenderTarget* GetRenderTarget() const { return render_target_; }
  // Returns the frame buffer texture.
  filament::Texture* GetColorTexture() const { return color_texture_; }

 private:
  void DestroyRenderTarget();

  filament::Engine& engine_;
  filament::RenderTarget* render_target_ = nullptr;
  filament::Texture* color_texture_ = nullptr;
  filament::Texture* depth_texture_ = nullptr;
  uint2 size_ = {0, 0};
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VIEWPORT_VIEWPORT_RENDER_TARGET_H_
