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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_WRITER_FRAME_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_WRITER_FRAME_H_

#import <CoreVideo/CoreVideo.h>

#include <condition_variable>
#include <mutex>

#include "core/common/log.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Fence.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "filament/filament/include/filament/Texture.h"

namespace imp {
namespace video {

// The start code prefix for H.264 NAL units.
constexpr uint8_t kStartCode[] = {0x00, 0x00, 0x00, 0x01};

// A lightweight structure to hold a pixel buffer and
// synchronization structures for writing a video frame.
class VideoFrame {
 public:
  VideoFrame(filament::Engine& engine, CVPixelBufferRef pxbuffer,
             double timestamp)
      : engine_(engine), pxbuffer_(pxbuffer), timestamp_(timestamp) {
    CVPixelBufferRetain(pxbuffer);
    bool needs_resize = false;
    if (width_ != CVPixelBufferGetWidth(pxbuffer) ||
        height_ != CVPixelBufferGetHeight(pxbuffer)) {
      width_ = CVPixelBufferGetWidth(pxbuffer);
      height_ = CVPixelBufferGetHeight(pxbuffer);
      needs_resize = true;
    }
    if (needs_resize) {
      if (texture_) {
        engine_.destroy(texture_);
        texture_ = nullptr;
      }
      if (render_target_) {
        engine_.destroy(render_target_);
        render_target_ = nullptr;
      }
      if (swap_chain_) {
        engine_.destroy(swap_chain_);
        swap_chain_ = nullptr;
      }
    }
    if (!texture_)
      texture_ = filament::Texture::Builder()
                     .width(width_)
                     .height(height_)
                     .levels(1)
                     .format(filament::Texture::InternalFormat::RGBA8)
                     .usage(filament::Texture::Usage::COLOR_ATTACHMENT)
                     .sampler(filament::Texture::Sampler::SAMPLER_2D)
                     .external()
                     .build(engine);
    if (texture_) {
      texture_->setExternalImage(engine, (void*)pxbuffer_);
    }
    if (!render_target_)
      render_target_ =
          filament::RenderTarget::Builder()
              .texture(filament::RenderTarget::AttachmentPoint::COLOR, texture_)
              .build(engine);
  }

  ~VideoFrame() {
    if (fence_) {
      engine_.destroy(fence_);
    }
    if (swap_chain_) {
      engine_.destroy(swap_chain_);
    }
    engine_.destroy(render_target_);
    engine_.destroy(texture_);
    CFRelease(pxbuffer_);
  }

  // Move-only semantics.
  VideoFrame(const VideoFrame& src) = delete;
  void operator=(const VideoFrame& src) = delete;

  void SetFence(filament::Fence* fence) {
    if (fence_) engine_.destroy(fence_);
    fence_ = fence;
  }

  // Blocks until all GPU commands for this frame have been completed.
  void WaitOnFence() {
    if (fence_) {
      fence_->wait();
    }
  }

  filament::Texture* GetTexture() const { return texture_; }
  filament::RenderTarget* GetRenderTarget() const { return render_target_; }
  filament::SwapChain* GetSwapChain() {
    if (!swap_chain_) {
      swap_chain_ = engine_.createSwapChain(
          (void*)pxbuffer_, filament::SwapChain::CONFIG_APPLE_CVPIXELBUFFER |
                                filament::SwapChain::CONFIG_READABLE);
    }
    return swap_chain_;
  }
  CVPixelBufferRef GetPixelBuffer() const { return pxbuffer_; }
  double GetTimestamp() const { return timestamp_; }

 private:
  filament::Engine& engine_;
  filament::Texture* texture_ = nullptr;
  filament::RenderTarget* render_target_ = nullptr;
  filament::SwapChain* swap_chain_ = nullptr;
  filament::Fence* fence_ = nullptr;
  CVPixelBufferRef pxbuffer_;
  double timestamp_;
  uint64_t width_ = 0;
  uint64_t height_ = 0;
};

}  // namespace video
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_WRITER_FRAME_H_
