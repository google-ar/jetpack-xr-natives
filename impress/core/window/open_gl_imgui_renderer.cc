/*
 * Copyright (C) 2018 The Android Open Source Project
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

#include "core/window/open_gl_imgui_renderer.h"

#include <android/native_window.h>
#include <android/native_window_jni.h>

#include <functional>
#include <memory>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "dear_imgui/backends/imgui_impl_android.h"
#include "dear_imgui/backends/imgui_impl_opengl3.h"
#include "dear_imgui/imgui.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/View.h"
#include "filament/libs/utils/include/utils/Path.h"
#include "core/async/future.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/texture.h"
#include "core/view/framework/view.h"
#include "split_engine/materials/texture_external_material.h"

namespace imp::window {

constexpr ImVec4 kClearColor = {0.0f, 0.0f, 0.0f, 1.0f};

OpenGLImGuiRenderer::OpenGLImGuiRenderer(BaseView& base_view,
                                         const utils::Path& fontPath,
                                         ImGuiContext* imgui_context)
    : imgui_context_(imgui_context), base_view_(base_view) {
  ImGuiIO& io = ImGui::GetIO();
  m_settings_path_.setPath(
      utils::Path::getUserSettingsDirectory() +
      utils::Path(
          std::string(".") +
          utils::Path::getCurrentExecutable().getNameWithoutExtension()) +
      utils::Path("imgui_settings.ini"));
  m_settings_path_.getParent().mkdirRecursive();
  io.IniFilename = m_settings_path_.c_str();
  if (fontPath.isFile()) {
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 16.0f);
  }
  ImGui::StyleColorsDark();
}

OpenGLImGuiRenderer::~OpenGLImGuiRenderer() {
  ImGui::DestroyContext(imgui_context_);
  imgui_context_ = nullptr;
}

void OpenGLImGuiRenderer::Initialize(float2 texture_resolution) {
  texture_size_ = texture_resolution;

  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(texture_resolution.x, texture_resolution.y);
  io.DisplayFramebufferScale.x = 1.0f;
  io.DisplayFramebufferScale.y = 1.0f;
  flip_vertical_ = false;

  surface_future_ = imp::AndroidExternalTextureSurface::CreateAsync(
      base_view_, ContentSecurityLevel::kNone,
      imp::kAndroidExternalTextureSurfaceConfigMono);
  surface_future_
      .Then([this, texture_resolution](
                absl::StatusOr<
                    std::unique_ptr<imp::AndroidExternalTextureSurface>>
                    surface) {
        if (!surface.ok()) {
          return absl::InternalError(
              "Failed to create AndroidExternalTextureSurface");
        }
        texture_surface_ptr_ = *std::move(surface);

        absl::Status status_resize = texture_surface_ptr_->SetDefaultBufferSize(
            int2(texture_resolution.x, texture_resolution.y));
        if (!status_resize.ok()) {
          return absl::InternalError("Failed to set default buffer size: ");
        }

        texture_ = texture_surface_ptr_->BorrowTexture();
        if (!texture_) {
          return absl::InternalError(
              "Failed to get texture from AndroidExternalTextureSurface");
        }
        env_ = base_view_.GetContext().GetJniEnv();
        jobject surface_reference =
            texture_surface_ptr_->GetSurface()->Reference();
        if (surface_reference == nullptr) {
          return absl::InternalError("Failed to get surface reference");
        }
        window_ = ANativeWindow_fromSurface(env_, surface_reference);
        ANativeWindow_acquire(window_);
        if (!window_) {
          return absl::InternalError(
              "Failed to get ANativeWindow from AndroidExternalTextureSurface");
        }

        absl::Status status_ogl = InitializeOGL();
        if (!status_ogl.ok()) {
          return absl::InternalError(
              "Failed to initialize OGL: trying to execute "
              "AndroidExternalTextureSurfaceCleanup");
        }

        return absl::OkStatus();
      })
      .KeptBy(&base_view_);
}

// resets the windows display size
void OpenGLImGuiRenderer::SetRenderTargetDisplaySize(int width, int height,
                                                     float scale_x,
                                                     float scale_y,
                                                     bool flip_vertical) {
  SetTextureBufferSize(width, height);

  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(width, height);
  io.DisplayFramebufferScale.x = scale_x;
  io.DisplayFramebufferScale.y = scale_y;
  flip_vertical_ = flip_vertical;
}

// run the main render loop and do lazy initialization of the OGL context, AET,
// and the ImGui context
void OpenGLImGuiRenderer::RenderImGui(float timeStepInSeconds,
                                      std::function<void()> render_imgui_fn) {
  // wait until the OGL context is initialized before rendering
  if (!is_ogl_initialized_) {
    return;
  }

  if (imgui_context_ == nullptr) {
    IMP_LOG(imp::ERROR) << "OpelGLImGuiHelper::RenderImGui: mImGuiContext is null";
    return;
  }
  ImGui::SetCurrentContext(imgui_context_);
  ImGuiIO& io = ImGui::GetIO();
  io.DeltaTime = timeStepInSeconds;

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplAndroid_NewFrame();
  ImGui::NewFrame();

  if (render_imgui_fn != nullptr) {
    render_imgui_fn();
  }
  // Let ImGui build up its draw data.
  ImGui::Render();
  glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
  glClearColor(kClearColor.x * kClearColor.w, kClearColor.y * kClearColor.w,
               kClearColor.z * kClearColor.w, kClearColor.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  eglSwapBuffers(egl_display_, egl_surface_);
  // noop if not android
}

// Initialize the OGL context and ImGui.
absl::Status OpenGLImGuiRenderer::InitializeOGL() {
  egl_display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  if (egl_display_ == EGL_NO_DISPLAY) {
    IMP_LOG(imp::ERROR) << "eglGetDisplay(EGL_DEFAULT_DISPLAY) returned "
                  "EGL_NO_DISPLAY";
    return absl::InternalError(
        "eglGetDisplay(EGL_DEFAULT_DISPLAY) returned "
        "EGL_NO_DISPLAY");
  }
  if (eglInitialize(egl_display_, 0, 0) != EGL_TRUE) {
    IMP_LOG(imp::ERROR) << "eglInitialize() returned with an error";
    return absl::InternalError("eglInitialize() returned with an error");
  }
  const EGLint egl_attributes[] = {EGL_BLUE_SIZE,    8,
                                   EGL_GREEN_SIZE,   8,
                                   EGL_RED_SIZE,     8,
                                   EGL_DEPTH_SIZE,   24,
                                   EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
                                   EGL_NONE};
  EGLint num_configs = 0;
  if (eglChooseConfig(egl_display_, egl_attributes, nullptr, 0, &num_configs) !=
      EGL_TRUE) {
    IMP_LOG(imp::ERROR) << "eglChooseConfig() returned with an error";
    return absl::InternalError("eglChooseConfig() returned with an error");
  }
  // Get the first matching config
  EGLConfig egl_config;
  eglChooseConfig(egl_display_, egl_attributes, &egl_config, 1, &num_configs);
  EGLint egl_format;
  eglGetConfigAttrib(egl_display_, egl_config, EGL_NATIVE_VISUAL_ID,
                     &egl_format);
  ANativeWindow_setBuffersGeometry(window_, 0, 0, egl_format);
  const EGLint egl_context_attributes[] = {EGL_CONTEXT_CLIENT_VERSION, 3,
                                           EGL_NONE};
  egl_context_ = eglCreateContext(egl_display_, egl_config, EGL_NO_CONTEXT,
                                  egl_context_attributes);
  if (egl_context_ == EGL_NO_CONTEXT) {
    IMP_LOG(imp::ERROR) << "eglCreateContext() returned EGL_NO_CONTEXT";
    return absl::InternalError("eglCreateContext() returned EGL_NO_CONTEXT");
  }
  egl_surface_ =
      eglCreateWindowSurface(egl_display_, egl_config, window_, nullptr);
  if (egl_surface_ == EGL_NO_SURFACE) {
    IMP_LOG(imp::ERROR) << "eglCreateWindowSurface() returned EGL_NO_SURFACE";
    return absl::InternalError(
        "eglCreateWindowSurface() returned EGL_NO_SURFACE");
  }
  eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_);

  ImGui::StyleColorsDark();

  ImGui_ImplAndroid_Init(window_);

  ImGui_ImplOpenGL3_Init("#version 300 es");
  ImGuiIO& io = ImGui::GetIO();
  ImFontConfig font_cfg;
  font_cfg.SizePixels = 22.0f;
  io.Fonts->AddFontDefault(&font_cfg);
  ImGui::GetStyle().ScaleAllSizes(3.0f);
  io.Fonts->Build();

  if (window_) {
    if (notify_texture_ready_callback_ != nullptr) {
      notify_texture_ready_callback_();
    }
    is_ogl_initialized_ = true;
  }

  return absl::OkStatus();
}

void OpenGLImGuiRenderer::SetTextureBufferSize(int width, int height) {
  if (!texture_surface_ptr_) {
    return;
  }
  absl::Status status_resize =
      texture_surface_ptr_->SetDefaultBufferSize(imp::int2(width, height));
  if (!status_resize.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to set default buffer size: " << status_resize;
    return;
  }
}

}  // namespace imp::window
