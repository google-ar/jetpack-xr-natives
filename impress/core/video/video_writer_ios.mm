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

#include "core/common/log.h"
#include "filament/filament/backend/include/backend/CallbackHandler.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Fence.h"
#include "filament/filament/include/filament/Viewport.h"
#import "core/video/video_writer.h"

#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>

#include "third_party/absl/memory/memory.h"
#include "core/common/platform_helpers.h"
#include "core/media/media_asset.h"

#import <AVFoundation/AVFoundation.h>
#import <Accelerate/Accelerate.h>
#import <CoreMedia/CMTime.h>
#import <Photos/PHAssetChangeRequest.h>
#import <Photos/PHPhotoLibrary.h>

// An interface for capturing iOS audio samples in C++ code.
struct AudioSampleHandler {
  virtual ~AudioSampleHandler() {};
  virtual void OnNewAudioSample(CMSampleBufferRef sample_buffer) = 0;
};

// An objective-c delegate that forwards audio samples to the given AudioSampleHandler.
@interface IMPAudioSampleBufferDelegate : NSObject <AVCaptureAudioDataOutputSampleBufferDelegate>

@property AudioSampleHandler *audioSampleHandler;

/* Use the designated initializer instead. */
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

/**
 * @param engine provides access to the filament rendering engine.
 */
- (instancetype)initWithHandler:(AudioSampleHandler*)audioSampleHandler
    NS_DESIGNATED_INITIALIZER;

@end

@implementation IMPAudioSampleBufferDelegate

- (instancetype)initWithHandler:(AudioSampleHandler*)audioSampleHandler {
  self = [super init];

  if (self) {
    _audioSampleHandler = audioSampleHandler;
  }

  return self;
}

#pragma mark - AVCaptureAudioDataOutputSampleBufferDelegate Methods

- (void)captureOutput:(AVCaptureOutput *)captureOutput
    didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
           fromConnection:(AVCaptureConnection *)connection {
  if (_audioSampleHandler) {
    _audioSampleHandler->OnNewAudioSample(sampleBuffer);
  }
}

@end

namespace imp {
namespace video {
namespace {

// A CallbackHandler that simply calls the callback on the same thread that calls post. When passing
// a custom CallbackHandler to setFrameCompletedCallback, this is guaranteed to not be the main
// Filament thread.
class FrameCompletedCallbackHandler : public filament::backend::CallbackHandler {
 public:
  void post(void* user, filament::backend::CallbackHandler::Callback callback) override {
    callback(user);
  };

  ~FrameCompletedCallbackHandler() override = default;
};

// A lightweight structure to hold a pixel buffer, swap chain, and synchronization structures for
// writing a video frame.
class VideoFrame {
 public:
  VideoFrame(filament::Engine& engine, CVPixelBufferRef pxbuffer, double timestamp)
      : engine_(engine), pxbuffer_(pxbuffer), timestamp_(timestamp) {
    static FrameCompletedCallbackHandler filamentCallbackHandler;
    swap_chain_ =
        engine.createSwapChain((void*)pxbuffer_, filament::SwapChain::CONFIG_APPLE_CVPIXELBUFFER |
                                                     filament::SwapChain::CONFIG_READABLE);
    swap_chain_->setFrameCompletedCallback(&filamentCallbackHandler,
                                           [&](filament::SwapChain* swapchain) {
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

// A VideoWriter that handles writing a video file on iOS devices.
class VideoWriterIos : public VideoWriter, AudioSampleHandler {
 public:
  explicit VideoWriterIos(bool record_microphone_audio)
      : record_microphone_audio_(record_microphone_audio) {}

  absl::Status Open(uint2 dimensions, absl::string_view filename) override {
    absl::MutexLock lock(&mutex_);

    dimensions_ = dimensions;

    // Check to ensure we don't already have a recording active.
    if (open_) {
      return absl::UnavailableError("This writer is already open.");
    }

    NSString* file_url = GetVideoUrl(filename);

    NSFileManager* fileManager = [NSFileManager defaultManager];
    // Delete any file that already exists at the path.
    [fileManager removeItemAtPath:file_url error:nil];

    // Create an AVAssetWriter to handle writing the video file.
    NSError* writer_error;
    writer_ = [[AVAssetWriter alloc] initWithURL:[NSURL fileURLWithPath:file_url]
                                              fileType:AVFileTypeAppleM4V
                                                 error:&writer_error];
    if (writer_error != nil) {
      return absl::InvalidArgumentError("Failed to create video file for filename: " +
                                        std::string(filename));
    }

    //// VIDEO RECORDING ////

    // Create an AVAssetWriterInput object for encoding the video.
    NSDictionary<NSString*, id>* video_settings = [NSDictionary
        dictionaryWithObjectsAndKeys:AVVideoCodecH264, AVVideoCodecKey,
                                     [NSNumber numberWithInt:dimensions_.x], AVVideoWidthKey,
                                     [NSNumber numberWithInt:dimensions_.y], AVVideoHeightKey, nil];

    video_writer_ = [AVAssetWriterInput assetWriterInputWithMediaType:AVMediaTypeVideo
                                                      outputSettings:video_settings];
    // Must set expectsMediaDataInRealTime so that it will optimize for throughput.
    video_writer_.expectsMediaDataInRealTime = YES;

    [writer_ addInput:video_writer_];

    // Create adapter for pixel buffers.
    NSDictionary<NSString*, id>* source_attributes = [NSDictionary
        dictionaryWithObjectsAndKeys:[NSNumber numberWithInt:kCVPixelFormatType_32BGRA],
                                     kCVPixelBufferPixelFormatTypeKey,
                                     [NSNumber numberWithInt:dimensions_.x], kCVPixelBufferWidthKey,
                                     [NSNumber numberWithInt:dimensions_.y],
                                     kCVPixelBufferHeightKey, nil];

    adaptor_ = [AVAssetWriterInputPixelBufferAdaptor
        assetWriterInputPixelBufferAdaptorWithAssetWriterInput:video_writer_
                                   sourcePixelBufferAttributes:source_attributes];

    //// AUDIO RECORDING ////
    if (record_microphone_audio_) {
      // Create an audio input for capturing the microphone audio.
      audio_capture_ = [[AVCaptureSession alloc] init];
      audio_device_ = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeAudio];
      NSError* audio_input_error;
      audio_input_ = [AVCaptureDeviceInput deviceInputWithDevice:audio_device_
                                                           error:&audio_input_error];
      if (audio_input_error != nil) {
        return absl::InvalidArgumentError("Failed to initialize audio input device!");
      }

      // Create an audio output for sending microphone audio samples to the handler.
      audio_output_ = [[AVCaptureAudioDataOutput alloc] init];

      // Create the audio capture session and attach the input and output.
      audio_capture_ = [[AVCaptureSession alloc] init];
      [audio_capture_ addInput:audio_input_];
      [audio_capture_ addOutput:audio_output_];
      audio_capture_.sessionPreset = AVCaptureSessionPresetHigh;
#if IMP_PLATFORM(IOS)
      [[AVAudioSession sharedInstance] setCategory:AVAudioSessionCategoryPlayAndRecord
                                       withOptions:AVAudioSessionCategoryOptionMixWithOthers
                                             error:nil];
// TODO: Find an alternative to AVAudioSession for MacOS, and then rename this file
// so it can be use for both iOS and macos.
#endif

      // Route the incoming microphone audio samples to the handler method.
      audio_sample_buffer_delegate_ = [[IMPAudioSampleBufferDelegate alloc] initWithHandler:this];
      dispatch_queue_t queue = dispatch_queue_create("AudioSampleQueue", nullptr);
      [audio_output_ setSampleBufferDelegate:audio_sample_buffer_delegate_ queue:queue];

      // Create an AVAssetWriterInput for writing the audio samples to the video file.
      AudioChannelLayout acl;
      bzero(&acl, sizeof(acl));
      acl.mChannelLayoutTag = kAudioChannelLayoutTag_Mono;
      NSDictionary<NSString*, id>* audio_settings = [NSDictionary
          dictionaryWithObjectsAndKeys:[NSNumber numberWithInt:kAudioFormatAppleLossless],
                                       AVFormatIDKey, [NSNumber numberWithInt:16],
                                       AVEncoderBitDepthHintKey, [NSNumber numberWithFloat:44100.0],
                                       AVSampleRateKey, [NSNumber numberWithInt:1],
                                       AVNumberOfChannelsKey,
                                       [NSData dataWithBytes:&acl length:sizeof(acl)],
                                       AVChannelLayoutKey, nil];

      audio_writer_ = [AVAssetWriterInput assetWriterInputWithMediaType:AVMediaTypeAudio
                                                         outputSettings:audio_settings];
      audio_writer_.expectsMediaDataInRealTime = YES;
      [writer_ addInput:audio_writer_];
    }

    // Start writing the video file.
    [writer_ startWriting];
    [writer_ startSessionAtSourceTime:kCMTimeZero];
    open_ = true;

    // Start the microphone capture session.
    [audio_capture_ startRunning];

    return absl::OkStatus();
  }

  void CaptureFrame(window::FilamentHost* filament_host) override {
    absl::MutexLock lock(&mutex_);

    // Do not write more frames if the video writer is not open.
    if (!open_) {
      return;
    }

    // Have to ensure the asset writer is ready before we write a frame, otherwise we must skip it.
    if (!adaptor_.assetWriterInput.readyForMoreMediaData) {
      IMP_LOG(imp::ERROR) << "Skipping frame - assert writer not ready for more data!";
      return;
    }

    absl::StatusOr<std::unique_ptr<VideoFrame>> video_frame_or = CreateVideoFrame(filament_host);
    if (!video_frame_or.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to create video frame!";
      return;
    }
    video_frames_.push_back(std::move(video_frame_or.value()));
  }

  void WriteFrame() override {
    if (video_frames_.size() < 3) {
      return;
    }

    WriteNextFrame();
  }

  Future<absl::Status> Close() override {
    absl::MutexLock lock(&mutex_);

    // Ensure we do not try to write any more frames.
    open_ = false;

    // Stop the audio capture session.
    [audio_capture_ stopRunning];
    [audio_writer_ markAsFinished];

    while (!video_frames_.empty()) {
      WriteNextFrame();
    }

    // Asynchronously finish writing the video and resolve the Future.
    [video_writer_ markAsFinished];
    Future<absl::Status> result;
    [writer_ finishWritingWithCompletionHandler:^{
      result.Return(absl::OkStatus());
    }];
    return result;
  }

  void OnNewAudioSample(CMSampleBufferRef sample_buffer) override {
    absl::MutexLock lock(&mutex_);
    if (!open_) {
      return;
    }

    // Have to ensure the asset writer is ready before we write a frame, otherwise we must skip it.
    if (!audio_writer_.readyForMoreMediaData) {
      IMP_LOG(imp::ERROR) << "Skipping frame - assert writer not ready for more data!";
      return;
    }

    // Get the reported time of the sample so we can adjust to be relative to the video start time.
    CMTime ts = CMSampleBufferGetPresentationTimeStamp(sample_buffer);
    double seconds_reported = CMTimeGetSeconds(ts);
    if (!initial_time_audio_.has_value()) {
      // This is the first sample, so that becomes the zero point.
      initial_time_audio_ = seconds_reported;
    }

    // The adjusted time of the sample relative to the start of the video.
    double seconds = seconds_reported - initial_time_audio_.value();

    // Due to Apple API limitations, the sample must be copied in order to be retimed.
    CMSampleTimingInfo sampleTimingInfo;
    sampleTimingInfo.duration = CMSampleBufferGetOutputDuration(sample_buffer);
    sampleTimingInfo.presentationTimeStamp = CMTimeMakeWithSeconds(seconds, kFrameTimeGranularity);
    sampleTimingInfo.decodeTimeStamp = kCMTimeInvalid;
    OSStatus status;
    CMSampleBufferRef sample_buffer_copy;
    status = CMSampleBufferCreateCopyWithNewTiming(kCFAllocatorDefault,
                                                   sample_buffer,
                                                   1,
                                                   &sampleTimingInfo,
                                                   &sample_buffer_copy);

    // Append the sample to the video.
    if (![audio_writer_ appendSampleBuffer:sample_buffer_copy]) {
      IMP_LOG(imp::ERROR) << "Unable to write to audio input";
    }
    // Release the copied buffer.
    CFRelease(sample_buffer_copy);
  }

 private:
  // Gets the final URL to write (including folder path) as a NSString.
  NSString* GetVideoUrl(absl::string_view filename) {
    // TODO: Uncomment to write to tmp directory instead.
    // NSString* filename_nsstring = [NSTemporaryDirectory()
    //     stringByAppendingPathComponent:[NSString stringWithUTF8String:filename.c_str()]];
    NSURL* docs_url =
        [[[NSFileManager defaultManager] URLsForDirectory:NSDocumentDirectory
                                                inDomains:NSUserDomainMask] lastObject];
    NSString* filename_nsstring = [NSString stringWithUTF8String:std::string(filename).c_str()];

    return [docs_url.path stringByAppendingPathComponent:filename_nsstring];
  }

  // Gets a CVPixelBufferRef from the pool. Must be released with CFRelease(buffer).
  absl::StatusOr<CVPixelBufferRef> GetPixelBuffer() {
    CVPixelBufferRef pxbuffer;
    // Create a pixel buffer to get the stride and for writing frames.
    CVReturn status = CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault,
                                                         adaptor_.pixelBufferPool, &pxbuffer);
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
    filament_host->CopyFrame(
        video_frame->GetSwapChain(), filament::Viewport(0, 0, dimensions_.x, dimensions_.y),
        filament::Viewport(0, 0, dimensions_.x, dimensions_.y), filament::Renderer::COMMIT);
    return video_frame;
  }

  // Writes the given VideoFrame to the movie file.
  void WriteFrame(std::unique_ptr<VideoFrame> video_frame) {
    video_frame->WaitOnFence();
    BOOL result = [adaptor_ appendPixelBuffer:video_frame->GetPixelBuffer()
                         withPresentationTime:CMTimeMakeWithSeconds(video_frame->GetTimestamp(),
                                                                    kFrameTimeGranularity)];
    if (!result) {
      IMP_LOG(imp::ERROR) << "Failed to write!";
      if (writer_.status == AVAssetWriterStatusFailed) {
        NSLog(@"Writer.status: AVAssetWriterStatusFailed, error: %@", writer_.error);
      }
    }
  }

  // Writes the next frame of video to the movie file.
  void WriteNextFrame() {
    std::unique_ptr<VideoFrame> video_frame = std::move(video_frames_[0]);
    video_frames_.erase(video_frames_.begin());
    WriteFrame(std::move(video_frame));
  }

  NSString* filename_;
  filament::math::int2 dimensions_;
  AVAssetWriter* writer_;

  // Video data.
  AVAssetWriterInput* video_writer_;
  AVAssetWriterInputPixelBufferAdaptor* adaptor_;
  std::vector<std::unique_ptr<VideoFrame>> video_frames_;

  // Audio data.
  AVCaptureSession* audio_capture_;
  AVCaptureAudioDataOutput* audio_output_;
  AVCaptureDevice* audio_device_;
  AVCaptureDeviceInput* audio_input_;
  AVAssetWriterInput* audio_writer_;
  IMPAudioSampleBufferDelegate* audio_sample_buffer_delegate_;

  // Timing data.
  absl::Mutex mutex_;
  absl::optional<absl::Time> initial_time_ = absl::nullopt;
  absl::optional<double> initial_time_audio_ = absl::nullopt;
  bool open_ = false;
  bool record_microphone_audio_;

  // This value is needed by iOS API for adding frames to the video and represents 1/n seconds.
  inline static constexpr int kFrameTimeGranularity = 120;
};

}  // namespace

std::unique_ptr<VideoWriter> VideoWriter::CreateVideoWriter(bool record_microphone_audio) {
  return std::make_unique<VideoWriterIos>(record_microphone_audio);
}

}  // namespace video
}  // namespace imp
