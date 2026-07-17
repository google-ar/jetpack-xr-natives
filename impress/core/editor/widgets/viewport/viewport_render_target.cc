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

#include "core/editor/widgets/viewport/viewport_render_target.h"

#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/math/vec.h"

namespace imp::editor {

ViewportRenderTarget::ViewportRenderTarget(filament::Engine& engine, uint2 size)
    : engine_(engine) {
  SetSize(size);
}

ViewportRenderTarget::~ViewportRenderTarget() { DestroyRenderTarget(); }

void ViewportRenderTarget::SetSize(uint2 size) {
  if (size.x <= 0 || size.y <= 0) return;

  if (size_ == size) return;

  size_ = size;

  DestroyRenderTarget();

  color_texture_ = filament::Texture::Builder()
                       .width(size.x)
                       .height(size.y)
                       .levels(1)
                       .sampler(filament::Texture::Sampler::SAMPLER_2D)
                       .format(filament::Texture::InternalFormat::RGBA8)
                       .usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                              filament::Texture::Usage::SAMPLEABLE)
                       .build(engine_);

  depth_texture_ = filament::Texture::Builder()
                       .width(size.x)
                       .height(size.y)
                       .levels(1)
                       .sampler(filament::Texture::Sampler::SAMPLER_2D)
                       .format(filament::Texture::InternalFormat::DEPTH32F)
                       .usage(filament::Texture::Usage::DEPTH_ATTACHMENT)
                       .build(engine_);

  render_target_ = filament::RenderTarget::Builder()
                       .texture(filament::RenderTarget::AttachmentPoint::COLOR,
                                color_texture_)
                       .texture(filament::RenderTarget::AttachmentPoint::DEPTH,
                                depth_texture_)
                       .build(engine_);
}

void ViewportRenderTarget::DestroyRenderTarget() {
  if (render_target_) {
    engine_.destroy(render_target_);
    render_target_ = nullptr;
  }
  if (color_texture_) {
    engine_.destroy(color_texture_);
    color_texture_ = nullptr;
  }
  if (depth_texture_) {
    engine_.destroy(depth_texture_);
    depth_texture_ = nullptr;
  }
}

}  // namespace imp::editor
