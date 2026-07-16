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

#include <functional>
#include <memory>
#include <vector>

#include "core/common/log.h"
#include "third_party/absl/memory/memory.h"
#include "third_party/absl/status/status.h"
#include "third_party/absl/strings/string_view.h"
#include "third_party/absl/synchronization/mutex.h"
#include "third_party/absl/time/time.h"
#include "third_party/absl/types/optional.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Viewport.h"
#include "core/common/platform_helpers.h"
#include "core/ui_stream/web_server.h"
#include "core/video/video_writer.h"
#include "core/video/video_writer_frame.h"
#include "core/window/filament_host.h"

#import <VideoToolbox/VideoToolbox.h>

namespace imp {
namespace video {
namespace {

using imp::ui_stream::WebInputEvent;
using imp::ui_stream::WebServer;

// A VideoWriter that handles writing a video file on iOS devices.
class WebServerVideoStreamWriterIos : public VideoWriter {
 public:
  WebServerVideoStreamWriterIos() {}

  ~WebServerVideoStreamWriterIos() override {
    if (pixel_buffer_pool_) {
      CFRelease(pixel_buffer_pool_);
    }
  }

  // Callback for VTCompressionSession. This is called on a thread managed by
  // VideoToolbox when the compression session has finished encoding a frame.
  // We convert the sample buffer to a web socket message, and send it to the
  // web server.
  static void CompressionCallback(void* outputCallbackRefCon, void* sourceFrameRefCon,
                                  OSStatus status, VTEncodeInfoFlags infoFlags,
                                  CMSampleBufferRef sampleBuffer) {
    if (status != 0 || sampleBuffer == nullptr) {
      return;
    }
    WebServerVideoStreamWriterIos* writer =
        static_cast<WebServerVideoStreamWriterIos*>(outputCallbackRefCon);
    if (!writer || !writer->web_server_) {
      return;
    }

    std::vector<char> output_buffer;

    bool is_keyframe = false;
    CFArrayRef attachments = CMSampleBufferGetSampleAttachmentsArray(sampleBuffer, true);
    if (attachments && CFArrayGetCount(attachments)) {
      CFDictionaryRef attachment = (CFDictionaryRef)CFArrayGetValueAtIndex(attachments, 0);
      CFBooleanRef depends_on_others =
          (CFBooleanRef)CFDictionaryGetValue(attachment, kCMSampleAttachmentKey_DependsOnOthers);
      is_keyframe = (depends_on_others == kCFBooleanFalse);
    }

    if (is_keyframe) {
      CMFormatDescriptionRef format = CMSampleBufferGetFormatDescription(sampleBuffer);
      const uint8_t* param_set;
      size_t param_size;
      size_t param_count;
      // SPS (0)
      if (CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 0, &param_set, &param_size,
                                                             &param_count, 0) == noErr) {
        output_buffer.insert(output_buffer.end(), kStartCode, kStartCode + 4);
        output_buffer.insert(output_buffer.end(), param_set, param_set + param_size);
      }
      // PPS (1)
      if (CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 1, &param_set, &param_size,
                                                             &param_count, 0) == noErr) {
        output_buffer.insert(output_buffer.end(), kStartCode, kStartCode + 4);
        output_buffer.insert(output_buffer.end(), param_set, param_set + param_size);
      }
    }

    CMBlockBufferRef block_buffer = CMSampleBufferGetDataBuffer(sampleBuffer);
    size_t total_length;
    char* data_ptr;
    if (CMBlockBufferGetDataPointer(block_buffer, 0, nullptr, &total_length, &data_ptr) ==
        kCMBlockBufferNoErr) {
      size_t offset = 0;
      const int header_len = 4;
      while (offset < total_length - header_len) {
        uint32_t nal_len;
        memcpy(&nal_len, data_ptr + offset, header_len);
        nal_len = CFSwapInt32BigToHost(nal_len);

        output_buffer.insert(output_buffer.end(), kStartCode, kStartCode + 4);
        output_buffer.insert(output_buffer.end(), data_ptr + offset + header_len,
                             data_ptr + offset + header_len + nal_len);

        offset += header_len + nal_len;
      }
    }

    if (!output_buffer.empty()) {
      writer->web_server_->BroadcastMessage(output_buffer.data(), output_buffer.size());
    }
  }

  absl::Status Open(uint2 dimensions, absl::string_view filename) override {
    absl::MutexLock lock(&mutex_);

    dimensions_ = dimensions;

    // Check to ensure we don't already have a recording active.
    if (open_) {
      return absl::UnavailableError("This writer is already open.");
    }

    if (dimensions_.x == 0 || dimensions_.y == 0) {
      return absl::InvalidArgumentError("Invalid dimensions (0)");
    }

    VTCompressionSessionCreate(nullptr, dimensions.x, dimensions.y, kCMVideoCodecType_H264, nullptr,
                               nullptr, 0, CompressionCallback, /*outputRefCon=*/this,
                               &compression_session_);
    VTSessionSetProperty(compression_session_, kVTCompressionPropertyKey_RealTime, kCFBooleanTrue);

    NSDictionary* ioSurfaceProps = [NSDictionary dictionary];
    NSDictionary<NSString*, id>* pixel_buffer_attributes = [NSDictionary
        dictionaryWithObjectsAndKeys:[NSNumber numberWithUnsignedInt:kCVPixelFormatType_32BGRA],
                                     kCVPixelBufferPixelFormatTypeKey,
                                     [NSNumber numberWithUnsignedInt:dimensions_.x],
                                     kCVPixelBufferWidthKey,
                                     [NSNumber numberWithUnsignedInt:dimensions_.y],
                                     kCVPixelBufferHeightKey, ioSurfaceProps,
                                     kCVPixelBufferIOSurfacePropertiesKey,
                                     [NSNumber numberWithBool:YES],
                                     kCVPixelBufferMetalCompatibilityKey, nil];

    CVReturn status = CVPixelBufferPoolCreate(kCFAllocatorDefault, NULL,
                                              (__bridge CFDictionaryRef)pixel_buffer_attributes,
                                              &pixel_buffer_pool_);
    if (status != kCVReturnSuccess) {
      return absl::InternalError("Failed to create pixel buffer pool!");
    }
    web_server_ = std::make_unique<WebServer>();
    MP_RETURN_IF_ERROR(web_server_->Start());
    open_ = true;
    is_ready_ = true;
    return absl::OkStatus();
  }

  void CaptureFrame(window::FilamentHost* filament_host) override {
    absl::MutexLock lock(&mutex_);

    // Do not write more frames if the video writer is not open.
    if (!open_) {
      return;
    }

    if (needs_resize_) {
      // Wait for pending frames to finish before dropping them.
      for (const auto& frame : video_frames_) {
        frame->WaitOnFence();
      }
      // Drop pending frames as they have the wrong dimensions.
      video_frames_.clear();

      if (pixel_buffer_pool_) {
        CFRelease(pixel_buffer_pool_);
        pixel_buffer_pool_ = nullptr;
      }
      if (compression_session_) {
        VTCompressionSessionInvalidate(compression_session_);
        CFRelease(compression_session_);
        compression_session_ = nullptr;
      }
      VTCompressionSessionCreate(nullptr, dimensions_.x, dimensions_.y, kCMVideoCodecType_H264,
                                 nullptr, nullptr, 0, CompressionCallback, /*outputRefCon=*/this,
                                 &compression_session_);
      VTSessionSetProperty(compression_session_, kVTCompressionPropertyKey_RealTime,
                           kCFBooleanTrue);
      NSDictionary* ioSurfaceProps = [NSDictionary dictionary];
      NSDictionary<NSString*, id>* pixel_buffer_attributes = [NSDictionary
          dictionaryWithObjectsAndKeys:[NSNumber numberWithUnsignedInt:kCVPixelFormatType_32BGRA],
                                       kCVPixelBufferPixelFormatTypeKey,
                                       [NSNumber numberWithUnsignedInt:dimensions_.x],
                                       kCVPixelBufferWidthKey,
                                       [NSNumber numberWithUnsignedInt:dimensions_.y],
                                       kCVPixelBufferHeightKey, ioSurfaceProps,
                                       kCVPixelBufferIOSurfacePropertiesKey,
                                       [NSNumber numberWithBool:YES],
                                       kCVPixelBufferMetalCompatibilityKey, nil];
      CVPixelBufferPoolCreate(kCFAllocatorDefault, NULL,
                              (__bridge CFDictionaryRef)pixel_buffer_attributes,
                              &pixel_buffer_pool_);
      needs_resize_ = false;
    }

    absl::StatusOr<std::unique_ptr<VideoFrame>> video_frame = CreateVideoFrame(filament_host);
    if (!video_frame.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to create video frame!";
      return;
    }
    video_frames_.push_back(std::move(video_frame.value()));
  }

  void WriteFrame() override {
    if (video_frames_.size() < kMinFramesToCapture) {
      return;
    }

    WriteNextFrame();
  }

  Future<absl::Status> Close() override {
    absl::MutexLock lock(&mutex_);

    // Ensure we do not try to write any more frames.
    open_ = false;
    is_ready_ = false;

    if (web_server_) {
      web_server_->Stop();
      web_server_.reset();
    }
    while (!video_frames_.empty()) {
      WriteNextFrame();
    }
    return Future<absl::Status>(absl::OkStatus());
  }

  bool IsReady() const override { return is_ready_; }

void ProcessInput(window::FilamentHost* host) override {
    if (!web_server_) {
      return;
    }

    std::vector<WebInputEvent> inputs = web_server_->PopInputs();
    if (inputs.empty()) {
      return;
    }
    float2 ratio = host->GetSubpixelRatio();
    for (const auto& event : inputs) {
      if (static_cast<int>(event.type) == static_cast<int>(WebInputEvent::Type::kResize)) {
        if (ratio.x <= 0 || ratio.y <= 0) {
          continue;
        }
        int width = static_cast<int>(event.x * ratio.x);
        int height = static_cast<int>(event.y * ratio.y);
        if (width > 0 && height > 0) {
          absl::MutexLock lock(&mutex_);
          if (width != dimensions_.x || height != dimensions_.y) {
            dimensions_ = {width, height};
            needs_resize_ = true;
            if (auto dev_mode_extension = host->TryGetExtension()) {
              dev_mode_extension->UpdateCameraAndViewport(dimensions_, ratio);
            }
          }
        }
        continue;
      }

      int px = static_cast<int>(event.x * dimensions_.x / ratio.x);
      int py = static_cast<int>(event.y * dimensions_.y / ratio.y);

      switch (static_cast<int>(event.type)) {
        case static_cast<int>(WebInputEvent::Type::kMove):
          host->QueueMouseInput(window::detail::PointerMove({px, py}, {0, 0})).IgnoreError();
          break;
        case static_cast<int>(WebInputEvent::Type::kDown):
          host->QueueMouseInput(window::detail::PointerMove({px, py}, {0, 0})).IgnoreError();
          host->QueueMouseInput(window::detail::PointerDown{static_cast<int>(event.param1)})
              .IgnoreError();
          break;
        case static_cast<int>(WebInputEvent::Type::kUp):
          host->QueueMouseInput(window::detail::PointerMove({px, py}, {0, 0})).IgnoreError();
          host->QueueMouseInput(window::detail::PointerUp{static_cast<int>(event.param1)})
              .IgnoreError();
          break;
        case static_cast<int>(WebInputEvent::Type::kWheel):
          host->QueueMouseInput(window::detail::Wheel{{static_cast<int>(event.param1),
                                                       static_cast<int>(event.param2)}})
              .IgnoreError();
          break;
      }
    }
  }

 private:
  // Gets a CVPixelBufferRef from the pool. Must be released with
  // CFRelease(buffer).
  absl::StatusOr<CVPixelBufferRef> GetPixelBuffer() {
    CVPixelBufferRef pxbuffer;
    CVPixelBufferPoolRef pool;
    pool = pixel_buffer_pool_;
    // Create a pixel buffer to get the stride and for writing frames.
    CVReturn status = CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &pxbuffer);
    if (status != kCVReturnSuccess || !pxbuffer) {
      return absl::InternalError("Failed to create pixel buffer!");
    }
    return pxbuffer;
  }

  // Creates a VideoFrame object for the current frame.
  absl::StatusOr<std::unique_ptr<VideoFrame>> CreateVideoFrame(
      window::FilamentHost* filament_host) {
    absl::StatusOr<CVPixelBufferRef> pxbuffer_or = GetPixelBuffer();
    MP_RETURN_IF_ERROR(pxbuffer_or.status());
    // Calculate the time of this frame.
    if (!initial_time_.has_value()) {
      initial_time_ = absl::Now();
    }
    double seconds = absl::ToDoubleSeconds(absl::Now() - initial_time_.value());
    auto video_frame =
        std::make_unique<VideoFrame>(*filament_host->GetEngine(), pxbuffer_or.value(), seconds);
    filament::Renderer* renderer = filament_host->GetRenderer();
    window::detail::FilamentView* ui_view = filament_host->TryGetExtension()->GetUiView();

    if (renderer && ui_view) {
      bool beginFrameSuccess = renderer->beginFrame(video_frame->GetSwapChain());
      if (beginFrameSuccess) {
        filament_host->TryGetExtension()->Render();
      }
      renderer->endFrame();
    }
    return video_frame;
  }

  // Writes the given VideoFrame to the movie file.
  void WriteFrame(std::unique_ptr<VideoFrame> video_frame) {
    video_frame->WaitOnFence();

    VTCompressionSessionEncodeFrame(
        compression_session_, video_frame->GetPixelBuffer(),
        CMTimeMakeWithSeconds(video_frame->GetTimestamp(), kWebSocketFrameTimeGranularity),
        kCMTimeInvalid, nil, nil, nil);
  }

  // Writes the next frame of video to the movie file.
  void WriteNextFrame() {
    if (video_frames_.empty()) {
      return;
    }
    std::unique_ptr<VideoFrame> video_frame = std::move(video_frames_[0]);
    video_frames_.erase(video_frames_.begin());
    WriteFrame(std::move(video_frame));
  }

  filament::math::int2 dimensions_;

  std::vector<std::unique_ptr<VideoFrame>> video_frames_;

  // Timing data.
  absl::Mutex mutex_;
  absl::optional<absl::Time> initial_time_ = absl::nullopt;
  bool open_ = false;
  bool needs_resize_ = false;

  // Websocket data.
  bool is_ready_ = false;
  std::unique_ptr<WebServer> web_server_;
  VTCompressionSessionRef compression_session_;
  CVPixelBufferPoolRef pixel_buffer_pool_ = nullptr;

  // The minimum number of frames to have captured before starting to write.
  inline static constexpr int kMinFramesToCapture = 3;
  // This value is needed by iOS API for adding frames to the video and
  // represents 1/n seconds.
  inline static constexpr int kWebSocketFrameTimeGranularity = 1000000000;
};

}  // namespace
}  // namespace video
}  // namespace imp

namespace imp {
namespace video {

std::unique_ptr<VideoWriter> CreateVideoStreamWriterIos() {
  return std::make_unique<WebServerVideoStreamWriterIos>();
}

}  // namespace video
}  // namespace imp
