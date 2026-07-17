/*
 * Copyright (C) 2025 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "core/window/filagui_imgui_renderer.h"

#include <cstdint>
#include <functional>
#include <memory>

#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/View.h"
#include "filament/libs/filagui/include/filagui/ImGuiHelper.h"
#include "filament/libs/utils/include/utils/Path.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"

namespace imp::window {

FilaguiImGuiRenderer::FilaguiImGuiRenderer(filament::View* view,
                                           BaseView& base_view,
                                           const utils::Path& fontPath,
                                           ImGuiContext* imgui_context)
    : engine_(base_view.GetHost()->GetEngine()),
      view_(view),
      helper_(std::make_unique<filagui::ImGuiHelper>(engine_, view_, fontPath,
                                                     imgui_context)),
      base_view_(base_view) {}

FilaguiImGuiRenderer::~FilaguiImGuiRenderer() { helper_.reset(); };

void FilaguiImGuiRenderer::SetRenderTargetDisplaySize(int width, int height,
                                                      float scale_x,
                                                      float scale_y,
                                                      bool flip_vertical) {
  helper_->setDisplaySize(width, height, scale_x, scale_y, flip_vertical);
}

void FilaguiImGuiRenderer::Initialize(float2 texture_resolution) {
  // Create and register a texture to act as a canvas for the ImGui UI.
  // create with initial default size, since there will be a request to resize
  // it my the spatial ui canvas component.
  texture_size_ = texture_resolution;
  texture_ =
      OwnedOrBorrowedTexturePtr(base_view_.GetTextureFactory().CreateTexture(
          TextureFactory::TextureCreationSettings{
              .width = static_cast<uint32_t>(texture_size_.x),
              .height = static_cast<uint32_t>(texture_size_.y),
              .format = filament::Texture::InternalFormat::RGBA8,
              .usage = filament::Texture::Usage::COLOR_ATTACHMENT |
                       filament::Texture::Usage::SAMPLEABLE,
          }));
}

void FilaguiImGuiRenderer::RenderImGui(float timeStepInSeconds,
                                       std::function<void()> render_imgui_fn) {
  helper_->render(
      timeStepInSeconds,
      [&render_imgui_fn](filament::Engine* engine, filament::View* view) {
        if (render_imgui_fn) {
          render_imgui_fn();
        }
      });
}

}  // namespace imp::window
