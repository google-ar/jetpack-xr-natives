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

#include "filament/filament/backend/include/backend/CallbackHandler.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/SwapChain.h"

namespace imp {
namespace video {

// The start code prefix for H.264 NAL units.
constexpr uint8_t kStartCode[] = {0x00, 0x00, 0x00, 0x01};

namespace {
// A CallbackHandler that simply calls the callback on the same thread that
// calls post. When passing a custom CallbackHandler to
// setFrameCompletedCallback, this is guaranteed to not be the main Filament
// thread.
class FrameCompletedCallbackHandler
    : public filament::backend::CallbackHandler {
 public:
  void post(void* user,
            filament::backend::CallbackHandler::Callback callback) override {
    callback(user);
  };

  ~FrameCompletedCallbackHandler() override = default;
};
}  // namespace

// A lightweight structure to hold a pixel buffer, swap chain, and
// synchronization structures for writing a video frame.
class VideoFrame {
 public:
  VideoFrame(filament::Engine& engine, CVPixelBufferRef pxbuffer,
             double timestamp)
      : engine_(engine), pxbuffer_(pxbuffer), timestamp_(timestamp) {
    static FrameCompletedCallbackHandler filamentCallbackHandler;
    swap_chain_ = engine.createSwapChain(
        (void*)pxbuffer_, filament::SwapChain::CONFIG_APPLE_CVPIXELBUFFER |
                              filament::SwapChain::CONFIG_READABLE);
    swap_chain_->setFrameCompletedCallback(
        &filamentCallbackHandler, [&](filament::SwapChain* swapchain) {
          // Because we're passing a custom CallbackHandler, this
          // lambda is guaranteed to be called on a thread that
          // isn't the main Filament thread.
          std::unique_lock<std::mutex> lock(mutex_);
          finished_ = true;
          cond_.notify_one();
        });
  }

  ~VideoFrame() {
    engine_.destroy(swap_chain_);
    CFRelease(pxbuffer_);
  }

  // Move-only semantics.
  VideoFrame(const VideoFrame& src) = delete;
  void operator=(const VideoFrame& src) = delete;

  // Blocks until all GPU commands for this frame have been completed.
  void WaitOnFence() {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [&] { return finished_; });
  }

  filament::SwapChain* GetSwapChain() const { return swap_chain_; }
  CVPixelBufferRef GetPixelBuffer() const { return pxbuffer_; }
  double GetTimestamp() const { return timestamp_; }

 private:
  filament::Engine& engine_;
  filament::SwapChain* swap_chain_ = nullptr;
  bool finished_ = false;
  std::mutex mutex_;
  std::condition_variable cond_;
  CVPixelBufferRef pxbuffer_;
  double timestamp_;
};

}  // namespace video
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_WRITER_FRAME_H_
