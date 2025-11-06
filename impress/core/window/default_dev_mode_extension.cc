// Copyright 2024 Google LLC
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

#include "core/window/default_dev_mode_extension.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/Viewport.h"
#include "filament/libs/filagui/include/filagui/ImGuiHelper.h"
#include "core/common/debug_draw.h"
#include "core/common/platform_helpers.h"
#include "core/common/resource_helpers.h"
#include "core/common/typed_vector.h"
#include "core/input/key_codes.h"
#include "core/math/vec.h"
#include "core/window/clipboard/clipboard_handler.h"
#include "core/window/filament_host.h"
#include "core/window/filament_host_input.h"
#include "core/window/filament_view.h"
#include "core/window/imp_dev_resources.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::window {

namespace {
constexpr float kWheelTravelMultiplier = 0.2f;
// ImGui asserts that the frame delta must be > 0, so this is used as a minimum.
constexpr float kMinimumImGuiFrameDelta = 0.001f;
constexpr absl::string_view kGoogleSansMediumName = "GoogleSans-Medium-18";

struct ImGuiMouseInputProcessor {
  ImGuiIO& io;

  absl::Status operator()(const detail::PointerDown& down) {
    io.MouseDown[down.id] = true;
    return absl::OkStatus();
  }

  absl::Status operator()(const detail::PointerMove& move) {
    io.MousePos = ImVec2(move.position.x, move.position.y);
    return absl::OkStatus();
  }

  absl::Status operator()(const detail::PointerUp& up) {
    io.MouseDown[up.id] = false;
    return absl::OkStatus();
  }

  absl::Status operator()(const detail::Wheel& wheel) {
    io.MouseWheel +=
        (static_cast<float>(wheel.travel.y)) * kWheelTravelMultiplier;
    io.MouseWheelH +=
        (static_cast<float>(wheel.travel.x)) * kWheelTravelMultiplier;
    return absl::OkStatus();
  }
};
}  // namespace

ImFont* DefaultDevModeExtension::LoadFont(const BufferAccess& font_data,
                                          const char* font_name, int size) {
  ImFontConfig font_config;
  font_config.FontDataOwnedByAtlas = false;
  font_config.OversampleH = 3;
  font_config.OversampleV = 3;
  snprintf(font_config.Name, sizeof(font_config.Name), "%s", font_name);
  ImGuiIO& io = ImGui::GetIO();
  return io.Fonts->AddFontFromMemoryTTF(
      const_cast<char*>(reinterpret_cast<const char*>(font_data.Data())),
      font_data.Size(), size, &font_config);
}

absl::Status DefaultDevModeExtension::Setup(FilamentHost* host) {
  host_ = host;
  filament::Engine* engine = host->GetEngine();
  filament::Scene* scene = host->GetScene();

  MP_RETURN_IF_ERROR(ui_view_.Setup(engine, "ui"));
  ui_view_.Get()->setPostProcessingEnabled(false);
  ui_view_.Get()->setShadowingEnabled(false);
  ImGuiContext* imgui_context = ImGui::CreateContext();
  // Resource setup.
  RegisterPackagedResources(imp_dev_resources_create());

  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->Clear();

  BufferAccess font_data;
  MP_RETURN_IF_ERROR(LoadPackagedFile("googlesans_medium.ttf", &font_data));

  if (fonts_.Append<FontId>(LoadFont(font_data, kGoogleSansMediumName.data(),
                                     18)) != kDefaultFont) {
    return absl::InternalError(
        absl::StrFormat("Font error loading %s", kGoogleSansMediumName));
  }

  RegisterExtraFonts(fonts_);

  io.FontDefault = fonts_[kDefaultFont];

  imgui_helper_ = std::make_unique<filagui::ImGuiHelper>(engine, ui_view_.Get(),
                                                         "", imgui_context);

  io.MousePos = ImVec2(-FLT_MAX, -FLT_MAX);
  io.MouseDown[0] = false;
  io.MouseDown[1] = false;
  io.MouseDown[2] = false;

  debug_draw_ = std::make_unique<debug_draw::Fixture>(
      engine, scene, custom_debug_draw_material_);

  return absl::OkStatus();
}

void DefaultDevModeExtension::Cleanup() {
  imgui_helper_.reset();
  debug_draw_.reset();
  if (host_) {
    ui_view_.Cleanup(host_->GetEngine());
  }
  if (render_target_) {
    host_->GetEngine()->destroy(render_target_);
  }
}

bool DefaultDevModeExtension::TryConsumeMouseInput(
    const detail::MouseInput& latest_input) {
  if (!IsEnabled()) {
    return false;
  }
  // TODO: Remove once all uses are satisfied by
  // imp::PointerInputHandler.
  // Don't consume input if rendering to a texture.
  if (imgui_helper_ && !render_target_texture_) {
    ImGuiMouseInputProcessor processor{ImGui::GetIO()};
    absl::visit(processor, latest_input).IgnoreError();
    if (ImGui::GetIO().WantCaptureMouse) return true;
  }
  return false;
}

void DefaultDevModeExtension::PreRender(absl::Duration previous_vsync,
                                        absl::Duration next_vsync, bool force) {
  if (!IsEnabled()) {
    return;
  }
  // We pump the imgui helper regardless of the result of beginFrame; this is
  // for the sake of imgui efficiency, as cached widgets are kept alive.
  absl::Duration delta_time =
      next_vsync - (last_vsync_ ? *last_vsync_ : previous_vsync);
  if (imgui_helper_ && (delta_time != absl::ZeroDuration() || force)) {
    float delta_time_seconds = std::clamp(
        static_cast<float>(absl::ToDoubleSeconds(delta_time)),
        kMinimumImGuiFrameDelta, std::numeric_limits<float>::infinity());
    imgui_helper_->render(delta_time_seconds, [this](filament::Engine* engine,
                                                     filament::View* view) {
      // Processes submitted commands.
      ImGuiRender();
      ProcessImGuiCommands();
      if (absl::Status status = host_->GetState()->UiRender(host_);
          !status.ok()) {
        IMP_LOG(imp::ERROR) << status;
      }
      // Process all submitted debug geometry.
      debug_draw_->Advance();
    });
    last_vsync_ = next_vsync;
  }
}

void DefaultDevModeExtension::RenderDevModeUI() {
  if (absl::Status status = host_->GetState()->UiRender(host_); !status.ok()) {
    IMP_LOG(imp::ERROR) << "Error rendering UI: " << status;
  }
}

void DefaultDevModeExtension::ApplyTextureRenderTarget(
    filament::Texture* texture) {
  render_target_texture_ = texture;
  // Update the size to that of the texture.
  UpdateCameraAndViewport(cached_screen_size_, cached_subpixel_ratio_);
  if (!texture) {
    ui_view_.Get()->setRenderTarget(nullptr);
    if (render_target_) {
      host_->GetEngine()->destroy(render_target_);
    }
    render_target_ = nullptr;
    return;
  }
  filament::RenderTarget::Builder render_target_builder;
  render_target_builder.texture(filament::RenderTarget::AttachmentPoint::COLOR,
                                texture);
  render_target_ = render_target_builder.build(*host_->GetEngine());
  ui_view_.Get()->setRenderTarget(render_target_);
  imgui_helper_->setDisplaySize(texture->getWidth(), texture->getHeight());
}

void DefaultDevModeExtension::OffscreenRender() {
  if (!IsEnabled()) {
    return;
  }
  // Rendering happens in Render() if no render target is being used.
  if (!render_target_texture_) {
    return;
  }
  if (imgui_helper_) {
    host_->GetRenderer()->render(ui_view_.Get());
  }
}

void DefaultDevModeExtension::Render() {
  if (!IsEnabled()) {
    return;
  }
  // Rendering happens in OffscreenRender() if a render target is being used.
  if (render_target_texture_) {
    return;
  }
  if (imgui_helper_) {
    host_->GetRenderer()->render(ui_view_.Get());
  }
}

void DefaultDevModeExtension::UpdateCameraAndViewport(uint2 screen_size,
                                                      float2 subpixel_ratio) {
  // Caching window size allows the screen-space Editor to be restored.
  cached_subpixel_ratio_ = subpixel_ratio;
  cached_screen_size_ = screen_size;
  if (render_target_texture_) {
    screen_size = {render_target_texture_->getWidth(),
                   render_target_texture_->getHeight()};
    subpixel_ratio = kOne2;
  }

  if (!std::min(screen_size.x, screen_size.y)) {
    IMP_LOG(imp::INFO) << "Skipping UpdateCameraAndViewport due to invalid bounds";
    return;
  }
  uint2 virtual_size = uint2{screen_size / subpixel_ratio};
  const auto viewport = filament::Viewport{0, 0, screen_size.x, screen_size.y};
  ui_view_.Get()->setViewport(viewport);
  ui_view_.GetViewCamera()->setProjection(filament::Camera::Projection::ORTHO,
                                          0.0, virtual_size.x, virtual_size.y,
                                          0.0, 0.0, 1.0);
  imgui_helper_->setDisplaySize(virtual_size.x, virtual_size.y,
                                subpixel_ratio.x, subpixel_ratio.y);
}

void DefaultDevModeExtension::QueueImGuiCommandBlock(ImGuiCommand cmd) {
  pending_imgui_commands_.push_back(std::move(cmd));
}

ImFont* DefaultDevModeExtension::GetFont(FontId font) {
  return fonts_.IsValid(font) ? fonts_[font] : nullptr;
}

void DefaultDevModeExtension::SetCustomDebugDrawMaterial(
    filament::Material* custom_debug_draw_material) {
  custom_debug_draw_material_ = custom_debug_draw_material;

  // If called after setup, recreate the debug draw fixture.
  if (debug_draw_) {
    filament::Engine* engine = host_->GetEngine();
    filament::Scene* scene = host_->GetScene();
    debug_draw_ = std::make_unique<debug_draw::Fixture>(
        engine, scene, custom_debug_draw_material_);
  }
}

void DefaultDevModeExtension::ProcessImGuiCommands() {
  while (!pending_imgui_commands_.empty()) {
    std::vector<ImGuiCommand> local_commands;
    std::swap(local_commands, pending_imgui_commands_);
    for (auto& pending_imgui_command : local_commands) {
      pending_imgui_command();
    }
  }
}

void DefaultDevModeExtension::OnClipboardHandlerChanged(
    ClipboardHandler* clipboard_handler) {
  ImGuiIO& io = ImGui::GetIO();
  if (clipboard_handler) {
    // If we do have a custom ClipboardHandler for this platform, we will
    // override the default ImGui clipboard function pointers.
    // See google3/third_party/azure_kinect/extern/imgui/src/imgui.cpp for the
    // default handler implementation (GetClipboardTextFn_DefaultImpl and
    // SetClipboardTextFn_DefaultImpl).
    //
    // ClipboardUserData will be passed to SetClipboardTextFn and
    // GetClipboardTextFn as void* user_data.
    io.ClipboardUserData = clipboard_handler;
    io.SetClipboardTextFn = [](void* user_data, const char* text) {
      ClipboardHandler* clipboard_handler =
          static_cast<ClipboardHandler*>(user_data);
      clipboard_handler->SetClipboardText(text);
    };
    io.GetClipboardTextFn = [](void* user_data) {
      ClipboardHandler* clipboard_handler =
          static_cast<ClipboardHandler*>(user_data);
      return clipboard_handler->GetClipboardText().data();
    };
  }
}

void DefaultDevModeExtension::SetEnabled(bool is_enabled) {
  is_enabled_ = is_enabled;
}

bool DefaultDevModeExtension::IsEnabled() { return is_enabled_; }

}  // namespace imp::window
