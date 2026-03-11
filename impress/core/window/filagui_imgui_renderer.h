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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAGUI_IMGUI_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAGUI_IMGUI_RENDERER_H_

#include <filagui/ImGuiHelper.h>
#include <filament/Engine.h>
#include <filament/View.h>
#include <utils/Path.h>

#include <functional>
#include <memory>

#include "absl/status/statusor.h"
#include "core/view/base_view.h"
#include "core/window/imgui_renderer.h"

struct ImGuiContext;

namespace imp::window {

// A wrapper around filagui::ImGuiHelper that provides an interface
// similar to OpenGLImGuiHelper.
class FilaguiImGuiRenderer : public ImGuiRenderer {
 public:
  // The constructor creates its own Scene and places it in the given View.
  FilaguiImGuiRenderer(filament::View* view, BaseView& base_view,
                       const utils::Path& fontPath,
                       // This is unused but kept for interface compatibility.
                       ImGuiContext* imgui_context = nullptr);

  ~FilaguiImGuiRenderer() override;

  // Informs ImGui of the current display size, as well as a scaling factor when
  // scissoring.
  void SetDisplaySize(int width, int height, float scale_x, float scale_y,
                      bool flip_vertical = false) override;

  // High-level utility method that takes a callback for creating all ImGui
  // windows and widgets. Clients are responsible for rendering the View. This
  // should be called on every frame, regardless of whether the Renderer wants
  // to skip or not.
  void RenderImGui(float timeStepInSeconds,
                   std::function<void()> render_imgui_fn) override;

  filament::Engine* engine_ = nullptr;
  filament::View* view_ = nullptr;

 private:
  std::unique_ptr<filagui::ImGuiHelper> helper_;
};

}  // namespace imp::window

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAGUI_IMGUI_RENDERER_H_
