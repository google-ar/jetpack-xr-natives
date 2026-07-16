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

#include "core/editor/remote_editor/remote_editor_renderer.h"

#include <cstdint>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/View.h"
#include "filament/filament/include/filament/Viewport.h"
#include "filament/libs/utils/include/utils/EntityManager.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/editor/components/world_space_editor_ui_assets.h"
#include "core/math/vec.h"
#include "core/render/primitive_shape_renderer.h"
#include "core/render/primitive_shape_renderer_state.proto.imp.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/utils/frame_time.h"
#include "core/window/filament_host.h"

namespace imp::editor {

namespace {

constexpr int kRenderTargetUpdateDelayMs = 33;

}  // namespace

RemoteEditorRenderer::RemoteEditorRenderer(BaseView& view)
    : UpdateSystem::Updater<RemoteEditorRenderer>(view), view_(view) {}

RemoteEditorRenderer::~RemoteEditorRenderer() { ClearRenderTargetWindow(); }

void RemoteEditorRenderer::ClearRenderTargetWindow() {
  view_.GetHost()->TryGetExtension()->ApplyTextureRenderTarget(nullptr);
  window_generation_++;
  filament::Engine* engine = view_.GetHost()->GetEngine();
  if (ui_quad_node_) {
    view_.DestroyNode(ui_quad_node_);
  }
  if (remote_view_) {
    engine->destroy(static_cast<const filament::View*>(remote_view_));
    remote_view_ = nullptr;
  }
  if (ui_scene_) {
    engine->destroy(static_cast<const filament::Scene*>(ui_scene_));
    ui_scene_ = nullptr;
  }
  if (ui_camera_) {
    engine->destroy(ui_camera_->getEntity());
    ui_camera_ = nullptr;
  }
  if (remote_swap_chain_) {
    engine->destroy(
        static_cast<const filament::SwapChain*>(remote_swap_chain_));
    remote_swap_chain_ = nullptr;
  }
  ui_material_.Reset();
  remote_window_ = nullptr;
}

void RemoteEditorRenderer::Update(const FrameTime& frame_time) {
  time_since_last_update_ += frame_time.GetDeltaTime();
  if (time_since_last_update_ <
      absl::Milliseconds(kRenderTargetUpdateDelayMs)) {
    return;
  }

  if (!remote_swap_chain_) return;

  filament::Renderer* renderer = view_.GetHost()->GetRenderer();

  if (renderer && remote_view_) {
    if (renderer->beginFrame(remote_swap_chain_)) {
      renderer->render(remote_view_);
      renderer->endFrame();
    }
  }
  // By using modulo instead of subtraction or reset, we carry over any
  // remainder time into the next frame's calculation to maintain an accurate
  // average framerate, but we discard excessive accumulated time during lag
  // spikes to prevent bursting (rendering on every frame to catch up).
  time_since_last_update_ %= absl::Milliseconds(kRenderTargetUpdateDelayMs);
}

void RemoteEditorRenderer::SetRenderTargetWindow(void* native_window, int width,
                                                 int height) {
  if (!native_window) {
    IMP_LOG(imp::ERROR) << "remote_window must be non-null.";
    return;
  }
  if (width <= 0 || height <= 0) {
    IMP_LOG(imp::ERROR) << "Ignoring invalid dimensions with valid window.";
    return;
  }

  const bool texture_needs_recreation = !texture_ ||
                                        texture_->GetSize().x != width ||
                                        texture_->GetSize().y != height;
  const bool resources_need_recreation =
      remote_window_ != native_window || texture_needs_recreation;

  if (!resources_need_recreation) {
    IMP_LOG(imp::WARNING)
        << "SetRenderTargetWindow: Window and texture size are unchanged; "
           "skipping resource recreation.";
    return;
  }

  ClearRenderTargetWindow();

  if (texture_needs_recreation) {
    texture_ = view_.GetTextureFactory().CreateTexture(
        width, height, filament::Texture::InternalFormat::RGBA8,
        filament::Texture::Usage::COLOR_ATTACHMENT |
            filament::Texture::Usage::SAMPLEABLE);
  }

  view_.GetHost()->TryGetExtension()->ApplyTextureRenderTarget(
      texture_->GetTexture());

  remote_window_ = native_window;

  filament::Engine* engine = view_.GetHost()->GetEngine();
  remote_swap_chain_ = engine->createSwapChain(native_window);

  ui_scene_ = engine->createScene();
  remote_view_ = engine->createView();
  remote_view_->setScene(ui_scene_);
  remote_view_->setViewport(
      {0, 0, static_cast<uint32_t>(width), static_cast<uint32_t>(height)});
  remote_view_->setPostProcessingEnabled(true);

  // Disable options that cause artifacts between rendered frames. The artifacts
  // result in less compression when sent over the network.
  remote_view_->setAntiAliasing(filament::View::AntiAliasing::NONE);
  remote_view_->setDithering(filament::View::Dithering::NONE);

  ui_camera_ = engine->createCamera(utils::EntityManager::get().create());
  remote_view_->setCamera(ui_camera_);
  ui_camera_->setProjection(filament::Camera::Projection::ORTHO, 0.0, 1.0, 0.0,
                            1.0, 0.0, 1.0);

  ui_quad_node_ = view_.CreateNode();
  ui_quad_node_->SetLocalPosition({0.5f, 0.5f, 0.0f});
  ui_scene_->addEntity(ui_quad_node_->GetEntity());

  PrimitiveShapeRendererState state;
  state.primitive.mesh =
      PrimitiveShapeRendererState::QuadMesh{.size = float2{1.0f, 1.0f}};
  state.channel = 0;
  state.frustrum_culling_mode = PrimitiveShapeRendererState::DISABLED;
  ui_quad_node_->AddComponentWithState<PrimitiveShapeRenderer>(state).KeptBy(
      this);

  int generation = window_generation_;

  // The specified material below causes imgui to render on our
  // offscreen texture.
  view_.GetAssetManager()
      .LoadMaterial(world_space_editor_ui_assets::kSpatialUiCanvasTextureCmat)
      .Then([this,
             generation](absl::StatusOr<AssetPtr<MaterialAsset>> material_asset)
                -> Future<absl::Status> {
        if (generation != window_generation_) {
          IMP_LOG(imp::WARNING) << "Material load skipped: Window generation changed.";
          return absl::OkStatus();
        }

        if (!material_asset.ok()) {
          return material_asset.status();
        }
        ui_material_ =
            view_.GetMaterialFactory().CreateMaterial(*material_asset);

        if (!ui_material_) {
          return absl::InternalError("Failed to create material for output UI");
        }

        if (texture_) {
          ui_material_->SetParameter("globalBaseColor", texture_.Borrow());
        }
        ui_material_->SetParameter("cornerUv", float2{0, 1});
        ui_material_->SetParameter("sizeUv", float2{1, -1});

        return ui_quad_node_->GetComponent<PrimitiveShapeRenderer>()
            ->SetMaterial(ui_material_.Borrow());
      })
      .KeptBy(this);
}

}  // namespace imp::editor
