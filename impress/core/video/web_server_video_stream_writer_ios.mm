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

#include "core/video/web_server_video_stream_writer_ios.h"

#include <functional>
#include <memory>
#include <vector>

#import <Foundation/Foundation.h>
#import <VideoToolbox/VideoToolbox.h>
#include "core/common/log.h"
#include "third_party/absl/memory/memory.h"
#include "third_party/absl/status/status.h"
#include "third_party/absl/status/statusor.h"
#include "third_party/absl/strings/escaping.h"
#include "third_party/absl/strings/str_format.h"
#include "third_party/absl/strings/string_view.h"
#include "third_party/absl/strings/substitute.h"
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

namespace imp {
namespace video {
namespace {

constexpr int kMinFramesToCapture = 3;

}  // namespace

using imp::ui_stream::WebInputEvent;
using imp::ui_stream::WebServer;

WebServerVideoStreamWriterIos::WebServerVideoStreamWriterIos() {}

WebServerVideoStreamWriterIos::~WebServerVideoStreamWriterIos() { static_cast<void>(Close()); }

// Callback for VTCompressionSession. This is called on a thread managed by
// VideoToolbox when the compression session has finished encoding a frame.
void WebServerVideoStreamWriterIos::CompressionCallback(void* outputCallbackRefCon,
                                                        void* sourceFrameRefCon, OSStatus status,
                                                        VTEncodeInfoFlags infoFlags,
                                                        CMSampleBufferRef sampleBuffer) {
  WebServerVideoStreamWriterIos* writer =
      static_cast<WebServerVideoStreamWriterIos*>(outputCallbackRefCon);
  if (!writer) return;

  if (status != 0 || sampleBuffer == nullptr) {
    return;
  }

  std::string json_config;
  std::string json_resize;
  bool needs_config = false;
  bool is_keyframe = false;
  std::vector<char> output_buffer;

  {
    absl::MutexLock lock(&writer->mutex_);
    if (!writer->web_server_) return;

    CMFormatDescriptionRef format = CMSampleBufferGetFormatDescription(sampleBuffer);
    CMVideoDimensions frame_dims = CMVideoFormatDescriptionGetDimensions(format);
    if (frame_dims.width != writer->dimensions_.x || frame_dims.height != writer->dimensions_.y) {
      return;
    }
    if (!writer->sent_config_) {
      writer->PrepareConfig(format, &json_config, &json_resize);
      writer->sent_config_ = true;
      needs_config = true;
    }

    CFArrayRef attachments = CMSampleBufferGetSampleAttachmentsArray(sampleBuffer, true);
    if (attachments && CFArrayGetCount(attachments)) {
      CFDictionaryRef attachment = (CFDictionaryRef)CFArrayGetValueAtIndex(attachments, 0);
      CFBooleanRef depends_on_others =
          (CFBooleanRef)CFDictionaryGetValue(attachment, kCMSampleAttachmentKey_DependsOnOthers);
      is_keyframe = (depends_on_others == kCFBooleanFalse);
    }
    output_buffer.push_back(is_keyframe ? 1 : 0);
    CMBlockBufferRef block_buffer = CMSampleBufferGetDataBuffer(sampleBuffer);
    if (block_buffer) {
      size_t total_length = CMBlockBufferGetDataLength(block_buffer);
      size_t offset = output_buffer.size();
      output_buffer.resize(offset + total_length);
      CMBlockBufferCopyDataBytes(block_buffer, 0, total_length, &output_buffer[offset]);
    }
  }

  if (needs_config) {
    writer->web_server_->BroadcastTextMessage(json_config);
    writer->web_server_->BroadcastTextMessage(json_resize);
  }
  if (output_buffer.size() > 1) {
    writer->web_server_->BroadcastMessage(output_buffer.data(), output_buffer.size());
  }
}

// Opens the web server and initializes the compression session.
absl::Status WebServerVideoStreamWriterIos::Open(uint2 dimensions, absl::string_view filename) {
  absl::MutexLock lock(&mutex_);
  if (dimensions.x > 0 && dimensions.y > 0) {
    dimensions_ = {static_cast<int>(dimensions.x), static_cast<int>(dimensions.y)};
  } else {
    dimensions_ = kDefaultDimensions;
  }

  if (open_) return absl::UnavailableError("Already open.");

  if (!CreateCompressionSession(dimensions_)) {
    return absl::InternalError("Failed to create compression session");
  }

  web_server_ = std::make_unique<WebServer>();
  {
    absl::MutexLock lock(&callback_mutex_);
    // set web server port before starting
    web_server_->SetPorts(http_port_, ui_stream_port_, script_port_);
  }
  MP_RETURN_IF_ERROR(web_server_->Start());
  {
    absl::MutexLock lock(&callback_mutex_);
    if (client_connected_callback_) {
      web_server_->SetOnConnectionChangedCallback(client_connected_callback_);
    }
    if (on_script_message_callback_) {
      web_server_->SetOnScriptMessageCallback(on_script_message_callback_);
    }
    web_server_->SetOnWebSocketConnectedCallback([this]() {
      absl::MutexLock lock(&mutex_);
      is_ready_ = true;
      sent_config_ = false;
      force_key_frame_ = true;
      if (web_server_) web_server_->BroadcastTextMessage("{\"type\": \"ready\"}");
    });
  }
  open_ = true;
  return absl::OkStatus();
}

// Captures a frame from the filament host and adds it to the video frames queue. If the video
// frame is not the same size as the compression session, the compression session will be
// recreated.
void WebServerVideoStreamWriterIos::CaptureFrame(window::FilamentHost* filament_host) {
  VTCompressionSessionRef old_session = nullptr;
  CVPixelBufferPoolRef old_pool = nullptr;
  std::vector<std::unique_ptr<VideoFrame>> frames_to_wait_on;
  bool just_resized = false;
  {
    absl::MutexLock lock(&mutex_);
    if (!open_ || !is_ready_) return;
    if (needs_resize_) {
      frames_to_wait_on = std::move(video_frames_);
      video_frames_.clear();
      old_pool = pixel_buffer_pool_;
      pixel_buffer_pool_ = nullptr;
      old_session = compression_session_;
      compression_session_ = nullptr;
      if (CreateCompressionSession(dimensions_)) {
        needs_resize_ = false;
        just_resized = true;
      }
      sent_config_ = false;
      force_key_frame_ = true;
    }
  }

  if (old_session) {
    VTCompressionSessionInvalidate(old_session);
    CFRelease(old_session);
  }
  if (old_pool) {
    CFRelease(old_pool);
  }

  filament::math::int2 capture_dims;
  {
    absl::MutexLock lock(&mutex_);
    if (!open_ || !is_ready_ || needs_resize_) return;
    capture_dims = dimensions_;
  }

  absl::StatusOr<std::unique_ptr<VideoFrame>> video_frame_or =
      CreateVideoFrame(filament_host, capture_dims);
  if (video_frame_or.ok()) {
    absl::MutexLock lock(&mutex_);
    if (open_ && is_ready_ && !needs_resize_) {
      video_frames_.push_back(std::move(video_frame_or.value()));
    }
  }
}

// Processes and encodes a frame.
void WebServerVideoStreamWriterIos::WriteFrame() {
  std::unique_ptr<VideoFrame> frame_to_write;
  VTCompressionSessionRef session = nullptr;
  {
    absl::MutexLock lock(&mutex_);
    if (!open_ || !is_ready_) return;
    if (video_frames_.size() < kMinFramesToCapture) return;
    frame_to_write = std::move(video_frames_.front());
    video_frames_.erase(video_frames_.begin());
    session = compression_session_;
    if (session) CFRetain(session);
  }
  if (frame_to_write && session) {
    ProcessAndEncodeFrame(std::move(frame_to_write), session);
    CFRelease(session);
  }
}

// Closes the web server and releases all resources.
Future<absl::Status> WebServerVideoStreamWriterIos::Close() {
  std::shared_ptr<WebServer> server_to_stop;
  VTCompressionSessionRef session_to_release = nullptr;
  CVPixelBufferPoolRef pool_to_release = nullptr;
  filament::Renderer* renderer_to_destroy = nullptr;
  {
    absl::MutexLock lock(&mutex_);
    if (!open_) return Future<absl::Status>(absl::OkStatus());
    open_ = false;
    is_ready_ = false;
    server_to_stop = std::move(web_server_);
    session_to_release = compression_session_;
    compression_session_ = nullptr;
    pool_to_release = pixel_buffer_pool_;
    pixel_buffer_pool_ = nullptr;
    renderer_to_destroy = renderer_;
    renderer_ = nullptr;
    video_frames_.clear();
  }
  if (session_to_release) {
    VTCompressionSessionInvalidate(session_to_release);
    CFRelease(session_to_release);
  }
  if (server_to_stop) server_to_stop->Stop();
  if (pool_to_release) CFRelease(pool_to_release);
  if (renderer_to_destroy && engine_) engine_->destroy(renderer_to_destroy);
  return Future<absl::Status>(absl::OkStatus());
}

// Returns true if the web server is ready to capture frames.
bool WebServerVideoStreamWriterIos::IsReady() const { return is_ready_; }

uint2 WebServerVideoStreamWriterIos::GetDimensions() const {
  absl::MutexLock lock(&mutex_);
  return {static_cast<uint32_t>(dimensions_.x), static_cast<uint32_t>(dimensions_.y)};
}

// Processes input events from the web server and updates the dimensions and viewport if
// necessary.
void WebServerVideoStreamWriterIos::ProcessInput(window::FilamentHost* host) {
  if (!web_server_) return;
  std::vector<WebInputEvent> inputs = web_server_->PopInputs();
  for (const auto& event : inputs) {
    int width = event.width & ~1;
    int height = event.height & ~1;
    if (width > 0 && height > 0) {
      absl::MutexLock lock(&mutex_);
      if (width != dimensions_.x || height != dimensions_.y || force_next_resize_) {
        dimensions_ = {width, height};
        needs_resize_ = true;
        force_next_resize_ = false;
        sent_config_ = false;
        host->EnsureNextRenderCompletes();
        if (host->TryGetExtension()) {
          host->TryGetExtension()->UpdateCameraAndViewport({(uint32_t)width, (uint32_t)height},
                                                           {1.0f, 1.0f});
        }
      }
    }
  }
}

// Sets the callback to be called when the web server connects or disconnects from a client.
void WebServerVideoStreamWriterIos::SetOnClientConnectedCallback(
    std::function<void(bool connected)> callback) {
  auto wrapped_callback = [this, callback](bool connected) {
    {
      absl::MutexLock lock(&mutex_);
      if (connected) {
        SetRemoteScreenMode();
      } else {
        is_ready_ = false;
        // video_frames_.clear() moved to thread-safe locations (Close/CaptureFrame)
        needs_resize_ = true;
        sent_config_ = false;
      }
    }
    if (callback) callback(connected);
  };
  {
    absl::MutexLock lock(&callback_mutex_);
    client_connected_callback_ = wrapped_callback;
    if (web_server_) web_server_->SetOnConnectionChangedCallback(wrapped_callback);
  }
}

// Sets the callback to be called when the web server receives a script message.
void WebServerVideoStreamWriterIos::SetOnScriptMessageCallback(
    std::function<void(const void*, size_t)> callback) {
  {
    absl::MutexLock lock(&callback_mutex_);
    on_script_message_callback_ = std::move(callback);
  }
  {
    absl::MutexLock lock(&callback_mutex_);
    web_server_->SetOnScriptMessageCallback(on_script_message_callback_);
  }
}

// Broadcasts a script message to all connected clients.
void WebServerVideoStreamWriterIos::BroadcastScriptMessage(const std::string& message) {
  {
    absl::MutexLock lock(&mutex_);
    if (web_server_)
      web_server_->BroadcastTextMessage(message, WebServer::ListenerType::kScripting);
  }
}

// Sets the native screen mode.
void WebServerVideoStreamWriterIos::SetNativeScreenMode() { is_ready_ = false; }

void WebServerVideoStreamWriterIos::SetRemoteScreenMode() {
  is_ready_ = true;
  needs_resize_ = true;
  sent_config_ = false;
  force_next_resize_ = true;
  initial_time_ = absl::nullopt;
}

// Sets the ports for the web server.
void WebServerVideoStreamWriterIos::SetPorts(int http_port, int ui_stream_port, int script_port) {
  absl::MutexLock lock(&mutex_);
  http_port_ = http_port;
  ui_stream_port_ = ui_stream_port;
  script_port_ = script_port;
  if (web_server_) {
    web_server_->SetPorts(http_port_, ui_stream_port_, script_port_);
  }
}

absl::StatusOr<CVPixelBufferRef> WebServerVideoStreamWriterIos::GetPixelBuffer() {
  CVPixelBufferRef pxbuffer;
  CVPixelBufferPoolRef pool = pixel_buffer_pool_;
  if (!pool) return absl::InternalError("Pixel buffer pool is null");
  CVReturn res = CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &pxbuffer);
  if (res != kCVReturnSuccess || !pxbuffer)
    return absl::InternalError("Failed to create pixel buffer");
  return pxbuffer;
}

bool WebServerVideoStreamWriterIos::CreateCompressionSession(int2 dims) {
  if (compression_session_) VTCompressionSessionInvalidate(compression_session_);
  OSStatus res = VTCompressionSessionCreate(
      nullptr, dims.x, dims.y, kCMVideoCodecType_H264, nullptr, nullptr, 0,
      [](void* outputCallbackRefCon, void* sourceFrameRefCon, OSStatus status,
         VTEncodeInfoFlags infoFlags, CMSampleBufferRef sampleBuffer) {
        CompressionCallback(outputCallbackRefCon, sourceFrameRefCon, status, infoFlags,
                            sampleBuffer);
      },
      this, &compression_session_);
  if (res != noErr) return false;
  VTSessionSetProperty(compression_session_, kVTCompressionPropertyKey_RealTime, kCFBooleanTrue);
  VTSessionSetProperty(compression_session_, kVTCompressionPropertyKey_AllowFrameReordering,
                       kCFBooleanFalse);
  VTSessionSetProperty(compression_session_, kVTCompressionPropertyKey_ProfileLevel,
                       kVTProfileLevel_H264_Baseline_AutoLevel);
  if (pixel_buffer_pool_) CFRelease(pixel_buffer_pool_);
  NSDictionary* ioSurfaceProps = @{};
  NSDictionary* attrs = @{
    (id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA),
    (id)kCVPixelBufferWidthKey : @(dims.x),
    (id)kCVPixelBufferHeightKey : @(dims.y),
    (id)kCVPixelBufferIOSurfacePropertiesKey : ioSurfaceProps,
    (id)kCVPixelBufferMetalCompatibilityKey : @YES
  };
  CVPixelBufferPoolCreate(kCFAllocatorDefault, NULL, (__bridge CFDictionaryRef)attrs,
                          &pixel_buffer_pool_);
  return true;
}

absl::StatusOr<std::unique_ptr<VideoFrame>> WebServerVideoStreamWriterIos::CreateVideoFrame(
    window::FilamentHost* filament_host, int2 dims) {
  absl::StatusOr<CVPixelBufferRef> pxbuffer_or = GetPixelBuffer();
  MP_RETURN_IF_ERROR(pxbuffer_or.status());
  double seconds = 0;
  {
    absl::MutexLock lock(&mutex_);
    if (!initial_time_.has_value()) initial_time_ = absl::Now();
    seconds = absl::ToDoubleSeconds(absl::Now() - initial_time_.value());
  }
  std::unique_ptr<VideoFrame> video_frame =
      std::make_unique<VideoFrame>(*filament_host->GetEngine(), pxbuffer_or.value(), seconds);

  filament::Renderer* local_renderer = nullptr;
  {
    absl::MutexLock lock(&mutex_);
    if (!open_) return video_frame;
    if (!renderer_) {
      renderer_ = filament_host->GetEngine()->createRenderer();
      engine_ = filament_host->GetEngine();
    }
    local_renderer = renderer_;
  }

  if (!local_renderer) return video_frame;

  auto* extension = filament_host->TryGetExtension();
  if (!extension) return video_frame;
  window::detail::FilamentView* ui_view_wrapper = extension->GetUiView();
  if (!ui_view_wrapper || !ui_view_wrapper->Get()) return video_frame;

  filament::View* view = ui_view_wrapper->Get();
  if (local_renderer->beginFrame(video_frame->GetSwapChain())) {
    filament::Renderer::ClearOptions clear_options;
    clear_options.clearColor = {0.0f, 0.0f, 0.0f, 0.0f};
    clear_options.clear = true;
    local_renderer->setClearOptions(clear_options);
    local_renderer->render(view);
    local_renderer->endFrame();
  } else {
    return absl::InternalError("Failed to begin frame for video stream");
  }
  return video_frame;
}

void WebServerVideoStreamWriterIos::PrepareConfig(CMFormatDescriptionRef format,
                                                  std::string* out_json_config,
                                                  std::string* out_json_resize) {
  const uint8_t* param_set;
  size_t param_size;
  size_t param_count;
  if (CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 0, &param_set, &param_size,
                                                         &param_count, 0) != noErr)
    return;
  std::vector<uint8_t> sps(param_set, param_set + param_size);
  if (CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 1, &param_set, &param_size,
                                                         &param_count, 0) != noErr)
    return;
  std::vector<uint8_t> pps(param_set, param_set + param_size);
  size_t avcc_size = 1 + 3 + 1 + 1 + 2 + sps.size() + 1 + 2 + pps.size();
  std::vector<uint8_t> avcc(avcc_size);
  uint8_t* ptr = avcc.data();
  *ptr++ = 1;
  *ptr++ = sps[1];
  *ptr++ = sps[2];
  *ptr++ = sps[3];
  *ptr++ = 0xFF;
  *ptr++ = 0xE1;
  *ptr++ = (sps.size() >> 8) & 0xFF;
  *ptr++ = sps.size() & 0xFF;
  memcpy(ptr, sps.data(), sps.size());
  ptr += sps.size();
  *ptr++ = 1;
  *ptr++ = (pps.size() >> 8) & 0xFF;
  *ptr++ = pps.size() & 0xFF;
  memcpy(ptr, pps.data(), pps.size());
  std::string codec_str = absl::StrFormat("avc1.%02X%02X%02X", sps[1], sps[2], sps[3]);
  std::string b64_description;
  absl::Base64Escape(absl::string_view((char*)avcc.data(), avcc.size()), &b64_description);
  CMVideoDimensions dims = CMVideoFormatDescriptionGetDimensions(format);
  *out_json_config = absl::Substitute(
      "{\"type\": \"config\", \"codec\": \"$0\", \"codedWidth\": $1, \"codedHeight\": $2, "
      "\"description\": \"$3\", \"rotation\": 0, \"fps\": 30}",
      codec_str, (uint32_t)dims.width, (uint32_t)dims.height, b64_description);
  *out_json_resize = absl::Substitute("{\"type\": \"resize\", \"width\": $0, \"height\": $1}",
                                      (uint32_t)dims.width, (uint32_t)dims.height);
  codec_string_ = codec_str;
}

void WebServerVideoStreamWriterIos::ProcessAndEncodeFrame(std::unique_ptr<VideoFrame> video_frame,
                                                          VTCompressionSessionRef session) {
  if (!session) return;
  video_frame->WaitOnFence();
  NSDictionary* frame_props = nil;
  {
    absl::MutexLock lock(&mutex_);
    if (force_key_frame_) {
      frame_props = @{(__bridge NSString*)kVTEncodeFrameOptionKey_ForceKeyFrame : @YES};
      force_key_frame_ = false;
    }
  }
  OSStatus status = VTCompressionSessionEncodeFrame(
      session, video_frame->GetPixelBuffer(),
      CMTimeMakeWithSeconds(video_frame->GetTimestamp(), kWebSocketFrameTimeGranularity),
      kCMTimeInvalid, (__bridge CFDictionaryRef)frame_props, nil, nil);
  if (status != noErr) {
    IMP_LOG(imp::ERROR) << "Failed to encode frame: " << status;
  }
}

std::unique_ptr<WebServerVideoStreamWriterIos> CreateVideoStreamWriterIos() {
  return std::make_unique<WebServerVideoStreamWriterIos>();
}

}  // namespace video
}  // namespace imp
