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

#include "third_party/absl/status/status.h"
#include "third_party/absl/status/statusor.h"
#include "third_party/absl/strings/string_view.h"
#include "third_party/absl/time/time.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/media/media_asset.h"
#include "core/media/media_color_space.h"
#include "core/media/media_source.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/video/video_source.h"
#include "core/view/base_view.h"

#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CMTime.h>
#import <Foundation/Foundation.h>

@class VideoDelegate;

namespace imp::video {

using ::absl::Status;
using ::absl::StatusOr;
using ::imp::media::MediaAsset;

/**
 * MacOS implementation of VideoSource, which wraps an AVPlayer that allows playback of a video
 * asset.
 */
class MacOSVideoSource : public VideoSource {
 public:
  explicit MacOSVideoSource(BaseView* view);

  MacOSVideoSource(const MacOSVideoSource&) = delete;
  MacOSVideoSource& operator=(const MacOSVideoSource&) = delete;

  Future<Status> Load(const MediaAsset* audio_asset) override;
  Status LoadSync(const MediaAsset* audio_asset) override;
  /** Loads the asset at the given URL asynchronously. */
  Future<Status> Load(absl::string_view url);
  /** Loads the asset at the given URL synchronously. This is currently only used for tests.*/
  Status LoadSync(absl::string_view url);

  absl::StatusOr<Texture*> CreateVideoTexture() override;

  absl::StatusOr<BorrowedTexturePtr> BorrowVideoTextureImpl(SmallSourceLocation loc) override;

  void UpdateVideoTexture(filament::Texture* texture, absl::Duration frame_delta) override;

  Status Play() override;
  Status Pause() override;
  Status Stop() override;

  // Sets the speed of playback for a video, if it is currently playing.
  // Does nothing if the video is not active.
  Status SetPlaybackSpeed(float speed) override;
  Status SeekTo(float seconds, SeekType seek_type) override;
  Status SetLoopCount(int loop) override;
  Status SetVolume(float volume) override;

  StatusOr<absl::Duration> GetDuration() const override;
  StatusOr<absl::Duration> GetPlaybackTime() const override;
  StatusOr<int> GetLoopCount() const override;
  VideoSource::State GetState() const override;
  uint2 GetVideoSize() const override;
  MediaColorSpace GetColorSpace() const override;
  MediaStereoMode GetStereoMode() const override;

  void SetOnPlaybackCompleteCallback(std::function<void()> callback) override;
  void SetOnSeekCompleteCallback(std::function<void()> callback) override;
  void SetOnBufferingCallback(std::function<void(BufferingState)> callback) override;

  void OnPlaybackComplete();
  /** Callback when currently playing item reaches the end. */
  void OnPlayerItemDidReachEnd(NSNotification* notification);
  /** Callback when player item is loaded and ready to play. */
  void OnPlayerItemReady();

 private:
  VideoDelegate* delegate_;
  AVPlayerItemVideoOutput* output_;
  AVPlayer* player_;
  State state_;
  // Loop count for SetLoopCount/GetLoopCount.
  int loop_count_ = 0;
  // Current loop number. Resets when SetLoopCount is called.
  int current_loop_ = 0;
  std::function<void()> on_playback_complete_callback_;
  // Callback when AVPlayer finishes loading asset.
  std::function<void()> on_ready_callback_;

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
  /** Parent pointer to the MacOsVideoSource that owns this VideoDelegate. */
  imp::video::MacOSVideoSource* _videoSource;
}
- (id)initWithVideoSource:(imp::video::MacOSVideoSource*)videoSource;
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
- (id)initWithVideoSource:(imp::video::MacOSVideoSource*)videoSource {
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

// Checks the URL validity and type from the given url string.
StatusOr<NSURL*> CastToNSURL(NSString* url_string) {
  NSURL* url = [NSURL URLWithString:url_string];

  if ([url.scheme isEqualToString:@"https"]) {
    return url;
  } else if ([url.scheme isEqualToString:@"http"]) {
    return absl::InternalError("MacOSVideoSource::DetermineUrlType error: "
                               "Url must use https");
  }

  NSFileManager* file_manager = [NSFileManager defaultManager];
  NSURL* file_path = [NSURL fileURLWithPath:url.path];
  if ([file_manager fileExistsAtPath:file_path.path]) {
    return [NSURL URLWithString:file_path.absoluteString];
  }

  return absl::InternalError("MacOSVideoSource::DetermineUrlType error: "
                             "Schema of the given url string is invalid.");
}

MacOSVideoSource::MacOSVideoSource(BaseView* view) : VideoSource(view) {
  delegate_ = [[VideoDelegate alloc] initWithVideoSource:this];
}

absl::StatusOr<Texture*> MacOSVideoSource::CreateVideoTexture() {
  texture_ = view_->GetTextureFactory().CreateExternalTexture();
  return &(*texture_);
}

absl::StatusOr<BorrowedTexturePtr> MacOSVideoSource::BorrowVideoTextureImpl(
    SmallSourceLocation loc) {
  if (!texture_) {
    texture_ = view_->GetTextureFactory().CreateExternalTexture();
  }
  return texture_.Borrow(loc);
}

void MacOSVideoSource::UpdateVideoTexture(filament::Texture* texture, absl::Duration frame_delta) {
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

Future<Status> MacOSVideoSource::Load(const MediaAsset* audio_asset) {
  return Future<Status>(absl::InternalError("MacOSVideoSource::Load Error: "
                                            "Load should be called with a URL String."));
}

Status MacOSVideoSource::LoadSync(const MediaAsset* audio_asset) {
  return absl::InternalError("MacOSVideoSource::Load Error: "
                             "Load should be called with a URL String.");
}

Future<Status> MacOSVideoSource::Load(absl::string_view url) { return LoadImpl(url, true); }

Status MacOSVideoSource::LoadSync(absl::string_view url) { return LoadImpl(url, false).Get(); }

Future<Status> MacOSVideoSource::LoadImpl(absl::string_view url, bool async) {
  NSString* url_string = [[NSString alloc] initWithBytes:url.data()
                                                  length:url.size()
                                                encoding:NSUTF8StringEncoding];

  StatusOr<NSURL*> url_validity = CastToNSURL(url_string);
  if (url_validity.ok()) {
    player_ = [AVPlayer playerWithURL:url_validity.value()];
  } else {
    return Future<Status>(url_validity.status());
  }

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
        on_ready_result.Return(absl::InternalError("MacOSVideoSource::LoadImpl error: "
                                                   "AVPlayer failed to load."));
      }
    };
  } else {
    AddVideoOutputForPlayerItem(player_.currentItem);
    on_ready_result.Return(absl::OkStatus());
  }
  return on_ready_result;
}

void MacOSVideoSource::AddVideoOutputForPlayerItem(AVPlayerItem* playerItem) {
  NSDictionary<NSString*, id>* pixelBufferAttributes = @{
    (id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA),
  };
  output_ = [[AVPlayerItemVideoOutput alloc] initWithPixelBufferAttributes:pixelBufferAttributes];
  output_.suppressesPlayerRendering = YES;
  [playerItem addOutput:output_];
}

void MacOSVideoSource::OnPlayerItemDidReachEnd(NSNotification* notification) {
  if (current_loop_ == loop_count_) {
    [player_ pause];

    OnPlaybackComplete();
  } else {
    current_loop_ += 1;
    [player_ seekToTime:CMTimeMakeWithSeconds(0, 1)];
    [player_ play];
  }
}

Status MacOSVideoSource::Play() {
  [player_ play];
  state_ = State::kPlaying;
  return absl::OkStatus();
}

Status MacOSVideoSource::Pause() {
  [player_ pause];
  state_ = State::kReady;
  return absl::OkStatus();
}

Status MacOSVideoSource::Stop() {
  [player_ pause];
  state_ = State::kStopped;
  OnPlaybackComplete();
  return absl::OkStatus();
}

Status MacOSVideoSource::SetPlaybackSpeed(float speed) {
  player_.rate = speed;
  return absl::OkStatus();
}

Status MacOSVideoSource::SeekTo(const float seconds, SeekType seek_type) {
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

Status MacOSVideoSource::SetLoopCount(const int loop) {
  loop_count_ = loop;
  current_loop_ = 0;
  return absl::OkStatus();
}

Status MacOSVideoSource::SetVolume(const float volume) {
  player_.volume = volume;
  return absl::OkStatus();
}

StatusOr<absl::Duration> MacOSVideoSource::GetDuration() const {
  auto time = player_.currentItem.duration;
  if (CMTIME_IS_INVALID(time)) {
    return absl::InternalError("MacOSVideoSource::GetDuration error: "
                               "AVPlayerItem duration time is invalid value.");
  }
  return absl::Seconds(CMTimeGetSeconds(player_.currentItem.duration));
}

StatusOr<absl::Duration> MacOSVideoSource::GetPlaybackTime() const {
  auto time = player_.currentItem.currentTime;
  if (CMTIME_IS_INVALID(time)) {
    return absl::InternalError("MacOSVideoSource::GetDuration error: "
                               "AVPlayerItem playback time is invalid value.");
  }
  return absl::Seconds(CMTimeGetSeconds(time));
}

StatusOr<int> MacOSVideoSource::GetLoopCount() const { return loop_count_; }

VideoSource::State MacOSVideoSource::GetState() const { return state_; }

uint2 MacOSVideoSource::GetVideoSize() const {
  CGSize size = player_.currentItem.presentationSize;
  return {size.width, size.height};
}

MediaColorSpace MacOSVideoSource::GetColorSpace() const { return MediaColorSpace(); }

MediaStereoMode MacOSVideoSource::GetStereoMode() const {
  // TODO: Implement stereo mode retrieval for MacOSVideoSource.
  return MediaStereoMode::kUnknown;
}

void MacOSVideoSource::SetOnPlaybackCompleteCallback(std::function<void()> callback) {
  on_playback_complete_callback_ = callback;
}

void MacOSVideoSource::SetOnSeekCompleteCallback(std::function<void()> callback) {
  // Not implemented.
}

void MacOSVideoSource::SetOnBufferingCallback(std::function<void(BufferingState)> callback) {
  // Not implemented.
}

void MacOSVideoSource::OnPlaybackComplete() {
  state_ = VideoSource::State::kStopped;
  if (on_playback_complete_callback_) {
    on_playback_complete_callback_();
  }
}

void MacOSVideoSource::OnPlayerItemReady() {
  state_ = MediaSource::State::kReady;
  if (on_ready_callback_) {
    on_ready_callback_();
  }
}

Future<std::unique_ptr<VideoSource>> CreateVideoSource(BaseView& base_view,
                                                       absl::string_view asset_url) {
  auto video_source = std::make_unique<MacOSVideoSource>(&base_view);

  return video_source->Load(asset_url).Then(
      [media_source = std::move(video_source)](
          Status status) mutable -> StatusOr<std::unique_ptr<VideoSource>> {
        if (status.ok()) {
          return std::move(media_source);
        } else {
          return status;
        }
      });
}

}  // namespace imp::video
