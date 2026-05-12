/*
 * Copyright (C) 2015 The Android Open Source Project
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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_OPEN_GL_IMGUI_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_OPEN_GL_IMGUI_HELPER_H_

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/Material.h>
#include <filament/Texture.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <jni.h>
#include <utils/Path.h>

#include <functional>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "dear_imgui/backends/imgui_impl_android.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/window/imgui_renderer.h"
#include "split_engine/materials/texture_external_material.h"

struct ImGuiContext;

namespace imp::window {

// Translates ImGui's draw commands into Filament primitives, textures, vertex
// buffers, etc. Creates a UI-specific Scene object and populates it with a
// Renderable. Does not handle event processing; clients can simply call
// ImGui::GetIO() directly and set the mouse state. Uses OpenGL and renders on
// to an android external texture.
class OpenGLImGuiRenderer : public ImGuiRenderer {
 public:
  // The constructor creates its own Scene and places it in the given View.
  OpenGLImGuiRenderer(BaseView& base_view, const utils::Path& fontPath,
                      ImGuiContext* imgui_context = nullptr);
  ~OpenGLImGuiRenderer() override;

  // Informs ImGui of the current display size, as well as a scaling factor when
  // scissoring.
  void SetRenderTargetDisplaySize(int width, int height, float scale_x,
                                  float scale_y,
                                  bool flip_vertical = false) override;

  // High-level utility method that takes a callback for creating all ImGui
  // windows and widgets. Clients are responsible for rendering the View. This
  // should be called on every frame, regardless of whether the Renderer wants
  // to skip or not.
  void RenderImGui(float timeStepInSeconds,
                   std::function<void()> render_imgui_fn) override;

  void SetTextureBufferSize(int width, int height);

  // Initializes the ImGuiRenderer. Initializes the surface_future_ and
  // initializes the OGL context and ImGui. Class holder a pointer to the
  // AndroidExternalTextureSurface and can access the state from the IsReady()
  // call.
  void Initialize(float2 texture_resolution) override;

  // Indiicates whether the imgui renderer is ready to render. This is true
  // when the surface_future_ is ready and the OGL is initialized.
  bool IsReady() override {
    return surface_future_.Ready() && is_ogl_initialized_;
  }

  // returns a pointer to the ImGuiRenderer interface.
  ImGuiRenderer* GetImGuiRenderer() override { return this; }

  void RegisterCallback(std::function<void()> callback) override {
    notify_texture_ready_callback_ = callback;
  }

  absl::Status PreRender();

  uint2 GetTextureSize() const override { return texture_size_; };

  BorrowedTexturePtr GetTexture() override { return texture_.Borrow(); }

 private:
  absl::Status InitializeOGL();

  // Callback to be called when the ImGuiRenderer (OpenGL) is done initializing
  // and is ready to render.
  std::function<void()> notify_texture_ready_callback_ = nullptr;

  utils::Path m_settings_path_;
  bool flip_vertical_ = false;

  EGLDisplay egl_display_ = EGL_NO_DISPLAY;
  EGLContext egl_context_ = EGL_NO_CONTEXT;
  EGLSurface egl_surface_ = EGL_NO_SURFACE;

  ImGuiContext* imgui_context_ = nullptr;
  OwnedOrBorrowedTexturePtr texture_;
  std::unique_ptr<AndroidExternalTextureSurface> texture_surface_ptr_;
  BaseView& base_view_;
  JNIEnv* env_ = nullptr;
  ANativeWindow* window_;

  std::unique_ptr<android_xr::TextureExternalMaterial> material_;
  imp::NodeHandle node_;
  Future<std::unique_ptr<imp::AndroidExternalTextureSurface>> surface_future_;

  bool is_ogl_initialized_ = false;
  float2 texture_size_ = {1, 1};
};

}  // namespace imp::window

#endif /* THIRD_PARTY_IMPRESS_CORE_WINDOW_OPEN_GL_IMGUI_HELPER_H_ */
