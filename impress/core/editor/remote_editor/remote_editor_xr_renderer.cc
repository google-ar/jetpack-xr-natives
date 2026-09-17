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

#include "core/editor/remote_editor/remote_editor_xr_renderer.h"

#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "core/config.h"
#include "core/editor/remote_editor/base_remote_editor_renderer.h"

#if IMP_PLATFORM(ANDROID)
#include <android/native_window.h>
#endif

#include "core/common/log.h"
#include "absl/time/time.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/editor/remote_editor/remote_editor_info.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"
#include "core/window/filament_host.h"

namespace imp::editor {

namespace {
struct ActiveWindowContent {
  const ActiveWindowInfo& active_window_info;
  std::vector<uint8_t>* readback_buffer;
  uint32_t width = 0;
  uint32_t height = 0;
};
}  // namespace

RemoteEditorXrRenderer::RemoteEditorXrRenderer(
    BaseView& view, const ActiveWindowInfo& active_window_info)
    : UpdateSystem::Updater<RemoteEditorXrRenderer>(view),
      view_(view),
      active_window_info_(active_window_info) {}

RemoteEditorXrRenderer::~RemoteEditorXrRenderer() { ClearRenderTargetWindow(); }

void RemoteEditorXrRenderer::ClearRenderTargetWindow() {
  // Protect against dangling pointers during asynchronous application teardown,
  // where FilamentHost or DevModeExtension may already be destroyed.
  if (view_.GetHost() && view_.GetHost()->TryGetExtension()) {
    window::FilamentHost::DevModeExtension* extension =
        view_.GetHost()->TryGetExtension();
    if (!texture_ ||
        extension->GetRenderTargetTexture() == texture_->GetTexture()) {
      extension->ApplyTextureRenderTarget(nullptr);
    }
  }
  texture_.Reset();
  remote_window_ = nullptr;
}

bool RemoteEditorXrRenderer::IsReady() const {
  return texture_ && texture_->GetSize().x > 0 && texture_->GetSize().y > 0;
}

void RemoteEditorXrRenderer::Update(const FrameTime& frame_time) {
  time_since_last_update_ += frame_time.GetDeltaTime();
}

void RemoteEditorXrRenderer::SetRenderTargetWindow(void* native_window,
                                                   int width, int height) {
  if (!native_window) {
    IMP_LOG(imp::ERROR) << "remote_window must be non-null.";
    return;
  }
  if (width <= 0 || height <= 0) {
    IMP_LOG(imp::ERROR) << "Ignoring invalid dimensions with valid window.";
    return;
  }

  if (remote_window_ == native_window && texture_ &&
      texture_->GetSize().x == width && texture_->GetSize().y == height) {
    return;
  }

  if (!view_.GetHost() || !view_.GetHost()->TryGetExtension()) {
    IMP_LOG(imp::ERROR) << "SetRenderTargetWindow skipped: view_.GetHost() or "
                  "TryGetExtension() is null.";
    return;
  }

  bool texture_needs_recreation = !texture_ || texture_->GetSize().x != width ||
                                  texture_->GetSize().y != height;

  if (texture_needs_recreation) {
    texture_.Reset();
    texture_ = view_.GetTextureFactory().CreateTexture(
        imp::TextureFactory::TextureCreationSettings{
            .width = static_cast<uint32_t>(width),
            .height = static_cast<uint32_t>(height),
            .format = filament::Texture::InternalFormat::RGBA8,
            .usage = filament::Texture::Usage::COLOR_ATTACHMENT |
                     filament::Texture::Usage::SAMPLEABLE |
                     filament::Texture::Usage::BLIT_SRC,
        });
    if (!texture_ || !texture_->GetTexture()) {
      IMP_LOG(imp::ERROR) << "SetRenderTargetWindow: CreateTexture failed for size "
                 << width << "x" << height;
      return;
    }
  }

  view_.GetHost()->TryGetExtension()->ApplyTextureRenderTarget(
      texture_ ? texture_->GetTexture() : nullptr);

  remote_window_ = native_window;
}

void RemoteEditorXrRenderer::ReadPixelsIfNeeded(
    filament::Renderer* renderer, filament::RenderTarget* render_target) {
  if (!remote_window_ || !texture_ || !render_target || !renderer) {
    return;
  }

  if (time_since_last_update_ <
      absl::Milliseconds(RemoteEditorInfo::RemoteEditorStreamingConfig::
                             kDefaultUiStreamingUpdateDelayMs)) {
    return;
  }
  time_since_last_update_ %=
      absl::Milliseconds(RemoteEditorInfo::RemoteEditorStreamingConfig::
                             kDefaultUiStreamingUpdateDelayMs);

  uint32_t width = texture_->GetSize().x;
  uint32_t height = texture_->GetSize().y;
  size_t buffer_size = width * height * 4;

  if (readback_buffer_.size() != buffer_size) {
    readback_buffer_.resize(buffer_size);
  }

  // Track the in-flight readback under lock before scheduling readPixels so
  // RevokeWindowAndWait() can reliably wait for completion during teardown.
  {
    absl::MutexLock lock(active_window_info_.lock);
    if (!active_window_info_.active_window_ptr_ ||
        active_window_info_.in_flight_readbacks > 0) {
      return;
    }
    active_window_info_.in_flight_readbacks++;
  }

  uint8_t* buffer_ptr = readback_buffer_.data();
  ActiveWindowContent* user_data = new ActiveWindowContent{
      .active_window_info = active_window_info_,
      .readback_buffer = &readback_buffer_,
      .width = width,
      .height = height,
  };

  filament::backend::PixelBufferDescriptor pb_desc(
      buffer_ptr, buffer_size, filament::backend::PixelDataFormat::RGBA,
      filament::backend::PixelDataType::UBYTE,
      [](void* buffer, size_t size, void* user) {
        ActiveWindowContent* user_data =
            static_cast<ActiveWindowContent*>(user);
#if IMP_PLATFORM(ANDROID)
        void* native_window = nullptr;
        {
          absl::MutexLock lock(user_data->active_window_info.lock);
          native_window = user_data->active_window_info.active_window_ptr_;
        }
        if (native_window) {
          ANativeWindow* window = static_cast<ANativeWindow*>(native_window);
          ANativeWindow_Buffer nw_buffer;
          int lock_res = ANativeWindow_lock(window, &nw_buffer, nullptr);
          if (lock_res == 0) {
            uint8_t* dst = static_cast<uint8_t*>(nw_buffer.bits);
            const uint8_t* src = static_cast<const uint8_t*>(buffer);
            uint32_t copy_width =
                std::min((uint32_t)nw_buffer.width, user_data->width);
            int copy_height =
                std::min((int)nw_buffer.height, (int)user_data->height);
            for (int y = 0; y < copy_height; ++y) {
              memcpy(dst + y * nw_buffer.stride * 4,
                     src + y * user_data->width * 4, copy_width * 4);
            }
            ANativeWindow_unlockAndPost(window);
          }
        }
#endif
        {
          absl::MutexLock lock(user_data->active_window_info.lock);
          user_data->active_window_info.in_flight_readbacks--;
          if (user_data->active_window_info.in_flight_readbacks == 0) {
            user_data->active_window_info.readbacks_done.SignalAll();
          }
        }
        delete user_data;
      },
      user_data);

  renderer->readPixels(render_target, 0, 0, width, height, std::move(pb_desc));
}

}  // namespace imp::editor
