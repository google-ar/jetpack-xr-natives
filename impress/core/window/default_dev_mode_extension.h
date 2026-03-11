/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_DEV_MODE_EXTENSION_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_DEV_MODE_EXTENSION_IMPL_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "dear_imgui/imgui.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/libs/filagui/include/filagui/ImGuiHelper.h"
#include "core/common/debug_draw.h"
#include "core/common/typed_id.h"
#include "core/common/typed_vector.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"
#include "core/window/clipboard/clipboard_handler.h"
#include "core/window/filament_host.h"
#include "core/window/filament_view.h"
#include "core/window/imgui_renderer.h"

struct ImFont;

namespace imp::window {

// Isolates code that runs under --define=IMP_DEV_RUNTIME=1 and wraps it up.
// It manages the lifecycle for dear_imgui, debug geometry, and a secondary
// filament view used for rendering the former.
class DefaultDevModeExtension : public FilamentHost::DevModeExtension {
 public:
  using FontId = TypedId<ImFont*, int>;
  // Roboto, 18 pts
  static constexpr FontId kDefaultFont = FontId(0);
  // RobotoMono, 16 pts
  static constexpr FontId kMonoFont = FontId(1);
  // Get a specific font pointer, or NULL for invalid FontId's

  explicit DefaultDevModeExtension(BaseView& view) : base_view_(view) {}
  ~DefaultDevModeExtension() override = default;
  // Loads packaged fonts into ImGui and sets up its bindings to filament.
  absl::Status Setup(FilamentHost& host) override;
  // Called after Setup() to allow for delayed initialization after the view is
  // ready.
  absl::Status PostSetup() override;
  // Destroys all created resources.
  void Cleanup() override;
  // Examines a mouse event and returns true if it was consumed by ImGui.
  bool TryConsumeMouseInput(
      const FilamentHost::MouseInput& latest_input) override;
  // Applies a texture render target to the internal filament view.
  void ApplyTextureRenderTarget(filament::Texture* texture) override;
  // Returns true if the ImGui-specific view has a render target.
  bool HasRenderTarget() override { return render_target_ != nullptr; }
  // If the ImGui-specific view has a render target, the view will be submitted
  // for rendering here instead of in Render().
  void OffscreenRender() override;
  // Schedules all ImGui work for this frame.
  void PreRender(absl::Duration previous_vsync, absl::Duration next_vsync,
                 bool force) override;
  // Submits the ImGui-specific view for rendering.
  void Render() override;
  // Informs ImGui of swap chain resize events.
  void UpdateCameraAndViewport(uint2 screen_size,
                               float2 subpixel_ratio) override;
  // Posts a single-use closure to be executed during the next ImGui render.
  void QueueImGuiCommandBlock(ImGuiCommand cmd) override;
  void RenderDevModeUI() override;
  // Get a specific font pointer, or NULL for invalid FontId's
  ImFont* GetFont(FontId font);
  virtual void RegisterExtraFonts(TypedVector<ImFont*>& fonts) {}
  virtual void ImGuiRender() {}
  // Gets called when FilamentHost::SetClipboardHandler gets called.
  void OnClipboardHandlerChanged(ClipboardHandler* clipboard_handler) override;
  void SetEnabled(bool is_enabled) override;
  bool IsEnabled() override;

 protected:
  ImFont* LoadFont(const BufferAccess& font_data, const char* font_name,
                   int size);

  FilamentHost* GetHost() { return host_; }
  filament::View* GetFilamentView() { return ui_view_.Get(); }

  ImGuiRenderer* GetImGuiRenderer() { return imgui_renderer_.get(); }
  void SetCustomDebugDrawMaterial(
      filament::Material* custom_debug_draw_material);

  BaseView& base_view_;
  ImGuiContext* imgui_context_ = nullptr;

 private:
  void ProcessImGuiCommands();

  // Pointer to the host that owns this dev mode extension.
  FilamentHost* host_ = nullptr;

  // Optional view for developer UI.
  detail::FilamentView ui_view_;
  // Optional support for developer UI.

  std::unique_ptr<ImGuiRenderer> imgui_renderer_;
  std::vector<ImGuiCommand> pending_imgui_commands_;
  // Optional support for debug geometry.
  std::unique_ptr<debug_draw::Fixture> debug_draw_;
  TypedVector<ImFont*> fonts_;
  std::optional<absl::Duration> last_vsync_;
  filament::Material* custom_debug_draw_material_ = nullptr;
  // The texture currently being rendered to.
  filament::Texture* render_target_texture_ = nullptr;
  filament::RenderTarget* render_target_ = nullptr;
  // Saved window sizes so we can properly switch back to screen-space Editor.
  uint2 cached_screen_size_;
  float2 cached_subpixel_ratio_;
  bool is_enabled_ = true;
};

}  // namespace imp::window

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_DEV_MODE_EXTENSION_IMPL_H_
