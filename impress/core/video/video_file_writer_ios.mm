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
#include "core/video/video_writer.h"
#include "core/video/video_writer_frame.h"
#include "core/window/filament_host.h"

#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CMTime.h>

// An interface for capturing iOS audio samples in C++ code.
struct AudioSampleHandler {
  virtual ~AudioSampleHandler() {};
  virtual void OnNewAudioSample(CMSampleBufferRef sample_buffer) = 0;
};

// An objective-c delegate that forwards audio samples to the given AudioSampleHandler.
@interface IMPAudioSampleBufferDelegate : NSObject <AVCaptureAudioDataOutputSampleBufferDelegate>

@property AudioSampleHandler* audioSampleHandler;

/* Use the designated initializer instead. */
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

/**
 * @param engine provides access to the filament rendering engine.
 */
- (instancetype)initWithHandler:(AudioSampleHandler*)audioSampleHandler NS_DESIGNATED_INITIALIZER;

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

- (void)captureOutput:(AVCaptureOutput*)captureOutput
    didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
           fromConnection:(AVCaptureConnection*)connection {
  if (_audioSampleHandler) {
    _audioSampleHandler->OnNewAudioSample(sampleBuffer);
  }
}

@end

namespace imp {
namespace video {
namespace {

// A VideoWriter that handles writing a video file on iOS devices.
class VideoFileWriterIos : public VideoWriter, AudioSampleHandler {
 public:
  explicit VideoFileWriterIos(bool record_microphone_audio)
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
    is_ready_ = true;

    // Start the microphone capture session.
    if (record_microphone_audio_) {
      [audio_capture_ startRunning];
    }

    return absl::OkStatus();
  }

  void CaptureFrame(window::FilamentHost* filament_host) override {
    absl::MutexLock lock(&mutex_);

    // Do not write more frames if the video writer is not open.
    if (!open_) {
      return;
    }

    // Have to ensure the asset writer is ready before we write a frame, otherwise we must skip
    // it.
    if (!adaptor_.assetWriterInput.readyForMoreMediaData) {
      IMP_LOG(imp::ERROR) << "Skipping frame - assert writer not ready for more data!";
      return;
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

    // Stop the audio capture session.
    if (record_microphone_audio_) {
      [audio_capture_ stopRunning];
      [audio_writer_ markAsFinished];
    }

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

  bool IsReady() const override { return is_ready_; }

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
    status = CMSampleBufferCreateCopyWithNewTiming(kCFAllocatorDefault, sample_buffer, 1,
                                                   &sampleTimingInfo, &sample_buffer_copy);

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
    CVPixelBufferPoolRef pool;
    pool = adaptor_.pixelBufferPool;
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
    if (video_frames_.empty()) {
      return;
    }
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
  bool is_ready_ = false;
  bool record_microphone_audio_;

  // The minimum number of frames to have captured before starting to write.
  inline static constexpr int kMinFramesToCapture = 3;
  // This value is needed by iOS API for adding frames to the video and represents 1/n seconds.
  inline static constexpr int kFrameTimeGranularity = 120;
};

}  // namespace
}  // namespace video
}  // namespace imp

namespace imp {
namespace video {

std::unique_ptr<VideoWriter> CreateVideoFileWriterIos(bool record_microphone_audio) {
  return std::make_unique<VideoFileWriterIos>(record_microphone_audio);
}

}  // namespace video
}  // namespace imp
