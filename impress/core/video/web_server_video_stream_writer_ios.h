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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_WEB_SERVER_VIDEO_STREAM_WRITER_IOS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_WEB_SERVER_VIDEO_STREAM_WRITER_IOS_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Renderer.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ui_stream/web_server.h"
#include "core/video/video_writer.h"
#include "core/video/video_writer_frame.h"
#include "core/window/filament_host.h"

#ifdef __OBJC__
#import <CoreMedia/CoreMedia.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>
#import <VideoToolbox/VideoToolbox.h>
#else
// Opaque types mapping for Apple platform types under pure C++ compilation
// pipelines
typedef struct OpaqueVTCompressionSession* VTCompressionSessionRef;
typedef struct __CVPixelBufferPool* CVPixelBufferPoolRef;
typedef struct __CVBuffer* CVPixelBufferRef;
typedef struct opaqueCMSampleBuffer* CMSampleBufferRef;
typedef const struct __CFDictionary* CFDictionaryRef;
typedef const struct __CFArray* CFArrayRef;
typedef struct opaqueCMFormatDescription* CMFormatDescriptionRef;
typedef int OSStatus;
#endif

namespace imp {
namespace video {

// A VideoWriter that handles streaming a video via a web server on iOS devices.
class WebServerVideoStreamWriterIos : public VideoWriter {
 public:
  WebServerVideoStreamWriterIos();
  ~WebServerVideoStreamWriterIos() override;

  // --- VideoWriter overrides ---
  absl::Status Open(uint2 dimensions, absl::string_view filename = "") override;
  void CaptureFrame(window::FilamentHost* filament_host) override;
  void WriteFrame() override;
  Future<absl::Status> Close() override;
  bool IsReady() const override;
  uint2 GetDimensions() const override;

  // --- Streaming specific APIs ---
  // Sets the ports to use for the web server, if applicable.
  void SetPorts(int http_port, int ui_stream_port, int script_port);

  // Process input events from the web client.
  void ProcessInput(window::FilamentHost* host);

  // Sets a callback to be called when a client connects or disconnects.
  void SetOnClientConnectedCallback(
      std::function<void(bool connected)> callback);

  // Sets a callback to be called when a scripting message is received.
  void SetOnScriptMessageCallback(
      std::function<void(const void*, size_t)> callback);

  // Broadcasts a scripting message to the web client.
  void BroadcastScriptMessage(const std::string& message);

  void SetNativeScreenMode();
  void SetRemoteScreenMode();

 private:
  static void CompressionCallback(void* outputCallbackRefCon,
                                  void* sourceFrameRefCon, OSStatus status,
                                  unsigned int infoFlags,
                                  CMSampleBufferRef sampleBuffer);

  absl::StatusOr<CVPixelBufferRef> GetPixelBuffer();
  bool CreateCompressionSession(filament::math::int2 dims);
  absl::StatusOr<std::unique_ptr<VideoFrame>> CreateVideoFrame(
      window::FilamentHost* filament_host, filament::math::int2 dims);
  void PrepareConfig(CMFormatDescriptionRef format,
                     std::string* out_json_config,
                     std::string* out_json_resize);
  void ProcessAndEncodeFrame(std::unique_ptr<VideoFrame> video_frame,
                             VTCompressionSessionRef session);

  filament::math::int2 dimensions_ = {1920, 1080};
  std::vector<std::unique_ptr<VideoFrame>> video_frames_;
  mutable absl::Mutex mutex_;
  absl::optional<absl::Time> initial_time_ = absl::Now();
  bool open_ = false;
  bool needs_resize_ = false;
  bool is_ready_ = false;
  bool force_key_frame_ = false;
  bool sent_config_ = false;
  bool force_next_resize_ = false;
  std::string codec_string_;
  std::unique_ptr<ui_stream::WebServer> web_server_;
  VTCompressionSessionRef compression_session_ = nullptr;
  CVPixelBufferPoolRef pixel_buffer_pool_ = nullptr;
  filament::Renderer* renderer_ = nullptr;
  filament::Engine* engine_ = nullptr;
  absl::Mutex callback_mutex_;
  std::function<void(bool)> client_connected_callback_;
  std::function<void(const void*, size_t)> on_script_message_callback_;
  int http_port_ = 0;
  int ui_stream_port_ = 0;
  int script_port_ = 0;
  inline static constexpr int kWebSocketFrameTimeGranularity = 100000;
  inline static constexpr filament::math::int2 kDefaultDimensions = {1920,
                                                                     1080};
};

// Creates a video writer that streams video via a web server on iOS.
std::unique_ptr<WebServerVideoStreamWriterIos> CreateVideoStreamWriterIos();

}  // namespace video
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_WEB_SERVER_VIDEO_STREAM_WRITER_IOS_H_
