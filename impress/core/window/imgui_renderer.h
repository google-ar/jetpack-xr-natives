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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_IMGUI_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_IMGUI_RENDERER_H_

#include <functional>

#include "core/common/owned_ptr.h"
#include "core/math/vec.h"

namespace imp {
class Texture;
}  // namespace imp

namespace imp::window {

// An interface that provide a generic reference to ImGuiHelper and
// OpenGLImGuiHelper classes
class ImGuiRenderer {
 public:
  virtual ~ImGuiRenderer() = default;

  // Informs ImGui of the current display size, as well as a scaling factor when
  // scissoring.
  virtual void SetRenderTargetDisplaySize(int width, int height, float scale_x,
                                          float scale_y,
                                          bool flip_vertical) = 0;

  // High-level utility method that takes a callback for creating all ImGui
  // windows and widgets. Clients are responsible for rendering the View. This
  // should be called on every frame, regardless of whether the Renderer wants
  // to skip or not.
  virtual void RenderImGui(float timeStepInSeconds,
                           std::function<void()> render_imgui_fn) = 0;

  // Initializes the ImGuiRenderer.
  virtual void Initialize(float2 texture_resolution) = 0;

  // Returns true if the ImGuiRenderer is ready to render and the texture has
  // been initialized and is ready to be rendered to.
  virtual bool IsReady() = 0;

  virtual imp::BorrowedPtr<imp::Texture> GetTexture() = 0;

  // Registers a callback to be called when the ImGuiRenderer is ready to
  // render.
  virtual void RegisterCallback(std::function<void()> callback) = 0;

  // Returns the GUI renderer interface.
  virtual ImGuiRenderer* GetImGuiRenderer() = 0;

  virtual uint2 GetTextureSize() const = 0;
};

}  // namespace imp::window

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_IMGUI_RENDERER_H_
