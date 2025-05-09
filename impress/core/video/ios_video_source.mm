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

#include <functional>
#include <memory>
#include <utility>
#include "core/common/log.h"

#include "third_party/absl/memory/memory.h"
#include "third_party/absl/status/status.h"

#include "core/common/platform_helpers.h"
#include "core/media/media_asset.h"
#include "core/render/texture_factory.h"
#include "core/video/video_color_space.h"
#include "core/video/video_source.h"

#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CMTime.h>
#import <Foundation/Foundation.h>

@class VideoDelegate;

namespace imp::video {

using ::absl::Seconds;
using ::absl::Status;
using ::absl::StatusOr;
using ::imp::media::MediaAsset;
using ::imp::media::MediaSource;

/**
 * iOS implementation of VideoSource, which wraps an AVPlayer that allows playback of a video asset.
 */
class IOSVideoSource : public VideoSource {
 public:
  explicit IOSVideoSource(BaseView* view);

  // TODO: Refactor MediaSource to remove Load and LoadSync with MediaAsset* since it
  // is not used by every derived class.
  Future<Status> Load(const MediaAsset* media_asset) override;
  Status LoadSync(const MediaAsset* media_asset) override;

  /** Loads the asset at the given URL asynchronously. */
  Future<Status> Load(absl::string_view url);
  /** Loads the asset at the given URL synchronously. This is currently only used for tests.*/
  Status LoadSync(absl::string_view url);

  Status Play() override;

  Status Pause() override;

  Status Stop() override;

  Status SetPlaybackSpeed(float speed) override;

  Status SeekTo(float seconds, SeekType seek_type) override;

  Status SetLoopCount(int loop) override;

  Status SetVolume(float volume) override;

  StatusOr<absl::Duration> GetDuration() const override;

  StatusOr<absl::Duration> GetPlaybackTime() const override;

  StatusOr<int> GetLoopCount() const override;

  MediaSource::State GetState() const override;

  uint2 GetVideoSize() const override;

  VideoColorSpace GetColorSpace() const override;
  MediaStereoMode GetStereoMode() const override;

  void SetOnPlaybackCompleteCallback(std::function<void()> callback) override;

  void SetOnSeekCompleteCallback(std::function<void()> callback) override;

  void SetOnBufferingCallback(std::function<void(BufferingState)> callback) override;

  absl::StatusOr<Texture*> CreateVideoTexture() override;

  absl::StatusOr<BorrowedTexturePtr> BorrowVideoTextureImpl(SmallSourceLocation loc) override;

  /**
   * Updates texture with a CVPixelBufferRef image for the current item time. For iOS video source,
   * this needs to be called on every frame.
   */
  void UpdateVideoTexture(filament::Texture* texture, absl::Duration frame_delta) override;
  /** Callback when playback completes. */
  void OnPlaybackComplete();
  /** Callback when currently playing item reaches the end. */
  void OnPlayerItemDidReachEnd(NSNotification* notification);
  /** Callback when player item is loaded and ready to play. */
  void OnPlayerItemReady();

 private:
  VideoDelegate* delegate_;
  AVPlayerItemVideoOutput* output_;
  AVPlayer* player_;
  MediaSource::State state_;
  // Loop count for SetLoopCount/GetLoopCount.
  int loop_count_ = 0;
  // Current loop number. Resets when SetLoopCount is called.
  int current_loop_ = 0;

  // Callback when AVPlayer finishes loading asset.
  std::function<void()> on_ready_callback_;
  // When playback has completed or Stop() is called,
  // on_playback_complete_callback is called to signal to the caller that the
  // player is now unused.
  std::function<void()> on_playback_complete_callback_;

  OwnedTexturePtr texture_;

  Future<Status> LoadImpl(absl::string_view url, bool async);
  void AddVideoOutputForPlayerItem(AVPlayerItem* playerItem);
};

}  // namespace imp::video

/**
 * Delegate for VideoSource to listen for changes in player item status and when it reaches the
 * end of playback. This class is owned by VideoSource.
 */
@interface VideoDelegate : NSObject {
  /** Parent pointer to the IOSVideoSource that owns this VideoDelegate. */
  imp::video::IOSVideoSource* _videoSource;
}
- (id)initWithVideoSource:(imp::video::IOSVideoSource*)videoSource;
/** Adds observer for AVPlayerItemDidPlayToEndTimeNotification on the player item. */
- (void)addPlayToEndObserverForPlayerItem:(AVPlayerItem*)item;
/** Adds observer for status changes on the player item. */
- (void)addStatusObserverForPlayerItem:(AVPlayerItem*)item;
/** Removes observer for status changes on the player item. */
- (void)removeStatusObserverForPlayerItem:(AVPlayerItem*)item;

- (void)playerItemDidReachEnd:(NSNotification*)notification;

- (void)observeValueForKeyPath:(NSString*)keyPath
                      ofObject:(id)object
                        change:(NSDictionary<NSKeyValueChangeKey, id>*)change
                       context:(void*)context;

@end

@implementation VideoDelegate
- (id)initWithVideoSource:(imp::video::IOSVideoSource*)videoSource {
  self = [super init];
  if (self) {
    _videoSource = videoSource;
  }
  return self;
}

- (void)addPlayToEndObserverForPlayerItem:(AVPlayerItem*)item {
  [[NSNotificationCenter defaultCenter] addObserver:self
                                           selector:@selector(playerItemDidReachEnd:)
                                               name:AVPlayerItemDidPlayToEndTimeNotification
                                             object:item];
}

- (void)addStatusObserverForPlayerItem:(AVPlayerItem*)item {
  [item addObserver:self forKeyPath:@"status" options:NSKeyValueObservingOptionNew context:nil];
}

- (void)removeStatusObserverForPlayerItem:(AVPlayerItem*)item {
  [item removeObserver:self forKeyPath:@"status"];
}

- (void)playerItemDidReachEnd:(NSNotification*)notification {
  _videoSource->OnPlayerItemDidReachEnd(notification);
}

- (void)observeValueForKeyPath:(NSString*)keyPath
                      ofObject:(id)object
                        change:(NSDictionary<NSKeyValueChangeKey, id>*)change
                       context:(void*)context {
  if ([keyPath isEqualToString:@"status"]) {
    _videoSource->OnPlayerItemReady();
  }
}
@end

namespace imp::video {

IOSVideoSource::IOSVideoSource(BaseView* view) : VideoSource(view) {
  delegate_ = [[VideoDelegate alloc] initWithVideoSource:this];
}

absl::StatusOr<Texture*> IOSVideoSource::CreateVideoTexture() {
  texture_ = view_->GetTextureFactory().CreateExternalTexture();
  return &(*texture_);
}

absl::StatusOr<BorrowedTexturePtr> IOSVideoSource::BorrowVideoTextureImpl(SmallSourceLocation loc) {
  if (!texture_) {
    texture_ = view_->GetTextureFactory().CreateExternalTexture();
  }
  return texture_.Borrow(loc);
}

void IOSVideoSource::UpdateVideoTexture(filament::Texture* texture, absl::Duration frame_delta) {
  if (player_ == nil || player_.currentItem.status != AVPlayerItemStatusReadyToPlay) {
    return;
  }

  CMTime currentTime = player_.currentItem.currentTime;

  if (!output_ || player_.currentItem.outputs.count == 0 ||
      player_.outputObscuredDueToInsufficientExternalProtection ||
      ![output_ hasNewPixelBufferForItemTime:currentTime]) {
    return;
  }

  // Filament takes ownership of the CVPixelBuffer here, retaining and releasing it the next time
  // setExternalImage is called, when the texture is destroyed, or when the swap chain is
  // destroyed.
  CVPixelBufferRef buffer = [output_ copyPixelBufferForItemTime:currentTime itemTimeForDisplay:nil];
  texture->setExternalImage(*engine_, buffer);
}

// TODO: Refactor MediaSource to remove Load and LoadSync with MediaAsset* since it is
//  not used by every derived class.
Future<Status> IOSVideoSource::Load(const MediaAsset* media_asset) {
  IMP_LOG(imp::FATAL) << "IOSVideoSource should call Load with a URL string.";
  return Future<Status>(absl::InternalError("IOSVideoSource error loading video."));
}

Status IOSVideoSource::LoadSync(const MediaAsset* media_asset) {
  IMP_LOG(imp::FATAL) << "IOSVideoSource should call Load with a URL string.";
  return absl::InternalError("IOSVideoSource error loading video.");
}

Future<Status> IOSVideoSource::Load(absl::string_view url) { return LoadImpl(url, true); }

Status IOSVideoSource::LoadSync(absl::string_view url) { return LoadImpl(url, false).Get(); }

Future<Status> IOSVideoSource::LoadImpl(absl::string_view url, bool async) {
  NSString* url_string = [[NSString alloc] initWithBytes:url.data()
                                                  length:url.size()
                                                encoding:NSUTF8StringEncoding];
  player_ = [AVPlayer playerWithURL:[NSURL fileURLWithPath:url_string]];
  [delegate_ addPlayToEndObserverForPlayerItem:player_.currentItem];

  Future<Status> on_ready_result;
  if (async) {
    [delegate_ addStatusObserverForPlayerItem:player_.currentItem];
    on_ready_callback_ = [this, on_ready_result]() {
      [delegate_ removeStatusObserverForPlayerItem:player_.currentItem];
      if (player_.currentItem.status == AVPlayerItemStatusReadyToPlay) {
        AddVideoOutputForPlayerItem(player_.currentItem);
        on_ready_result.Return(absl::OkStatus());
      } else {
        on_ready_result.Return(absl::InternalError("AVPlayer failed to load"));
      }
    };
  } else {
    AddVideoOutputForPlayerItem(player_.currentItem);
    on_ready_result.Return(absl::OkStatus());
  }
  return on_ready_result;
}

void IOSVideoSource::AddVideoOutputForPlayerItem(AVPlayerItem* playerItem) {
  NSDictionary<NSString*, id>* pixelBufferAttributes = @{
    (id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_420YpCbCr8BiPlanarFullRange),
  };
  output_ = [[AVPlayerItemVideoOutput alloc] initWithPixelBufferAttributes:pixelBufferAttributes];
  output_.suppressesPlayerRendering = YES;
  [playerItem addOutput:output_];
}

void IOSVideoSource::OnPlayerItemDidReachEnd(NSNotification* notification) {
  if (current_loop_ == loop_count_) {
    [player_ pause];

    OnPlaybackComplete();
  } else {
    current_loop_ += 1;
    [player_ seekToTime:CMTimeMakeWithSeconds(0, 1)];
    [player_ play];
  }
}

Status IOSVideoSource::Play() {
  [player_ play];
  state_ = State::kPlaying;
  return absl::OkStatus();
}

Status IOSVideoSource::Pause() {
  [player_ pause];
  state_ = State::kReady;
  return absl::OkStatus();
}

Status IOSVideoSource::Stop() {
  [player_ pause];
  state_ = State::kStopped;
  OnPlaybackComplete();
  return absl::OkStatus();
}

Status IOSVideoSource::SetPlaybackSpeed(float speed) {
  return absl::UnimplementedError("Not implemented");
}

Status IOSVideoSource::SeekTo(float seconds, SeekType seek_type) {
  switch (seek_type) {
    case SeekType::QUICK:
      [player_ seekToTime:CMTimeMakeWithSeconds(seconds, 1 /* preferredTimescale */)];
      break;
    case SeekType::PRECISE:
      [player_ seekToTime:CMTimeMakeWithSeconds(seconds, 1 /* preferredTimescale */)
          toleranceBefore:kCMTimeZero
           toleranceAfter:kCMTimeZero];
      break;
  }

  return absl::OkStatus();
}

Status IOSVideoSource::SetLoopCount(int loop) {
  loop_count_ = loop;
  current_loop_ = 0;
  return absl::OkStatus();
}

Status IOSVideoSource::SetVolume(float volume) {
  player_.volume = volume;
  return absl::OkStatus();
}

StatusOr<absl::Duration> IOSVideoSource::GetDuration() const {
  auto time = player_.currentItem.duration;
  if (CMTIME_IS_INVALID(time)) {
    return absl::InternalError("Invalid AVPlayerItem duration time.");
  }
  return Seconds(CMTimeGetSeconds(player_.currentItem.duration));
}

StatusOr<absl::Duration> IOSVideoSource::GetPlaybackTime() const {
  auto time = player_.currentItem.currentTime;
  if (CMTIME_IS_INVALID(time)) {
    return absl::InternalError("Invalid AVPlayerItem playback time.");
  }
  return Seconds(CMTimeGetSeconds(time));
}

StatusOr<int> IOSVideoSource::GetLoopCount() const { return loop_count_; }

MediaSource::State IOSVideoSource::GetState() const { return state_; }

uint2 IOSVideoSource::GetVideoSize() const {
  CGSize size = player_.currentItem.presentationSize;
  return {size.width, size.height};
}

VideoColorSpace IOSVideoSource::GetColorSpace() const { return VideoColorSpace(); }

MediaStereoMode IOSVideoSource::GetStereoMode() const {
  // TODO: Implement stereo mode retrieval for iOSVideoSource.
  return MediaStereoMode::kUnknown;
}

void IOSVideoSource::SetOnPlaybackCompleteCallback(std::function<void()> callback) {
  on_playback_complete_callback_ = callback;
}

void IOSVideoSource::SetOnSeekCompleteCallback(std::function<void()> callback) {
  // Not implemented.
}

void IOSVideoSource::SetOnBufferingCallback(std::function<void(BufferingState)> callback) {
  // Not implemented.
}

void IOSVideoSource::OnPlaybackComplete() {
  state_ = MediaSource::State::kReady;
  if (on_playback_complete_callback_) {
    on_playback_complete_callback_();
  }
}

void IOSVideoSource::OnPlayerItemReady() {
  state_ = MediaSource::State::kReady;
  if (on_ready_callback_) {
    on_ready_callback_();
  }
}

// Use synchronous load implementation for tests.
#if IMP_PLATFORM(IOS_SIMULATOR)
Future<std::unique_ptr<VideoSource>> CreateVideoSource(BaseView& base_view,
                                                       absl::string_view asset_url) {
  auto video_source = std::make_unique<IOSVideoSource>(&base_view);
  auto status = video_source->LoadSync(asset_url);
  if (status.ok()) {
    return Future<std::unique_ptr<VideoSource>>(std::move(video_source));
  }
  return Future<std::unique_ptr<VideoSource>>(status);
}
#else
Future<std::unique_ptr<VideoSource>> CreateVideoSource(BaseView& base_view,
                                                       absl::string_view asset_url) {
  auto video_source = std::make_unique<IOSVideoSource>(&base_view);

  return video_source->Load(asset_url).Then(
      [media_source = std::move(video_source)](
          absl::Status status) mutable -> absl::StatusOr<std::unique_ptr<VideoSource>> {
        if (status.ok()) {
          return std::move(media_source);
        } else {
          return status;
        }
      });
}
#endif

}  // namespace imp::video
