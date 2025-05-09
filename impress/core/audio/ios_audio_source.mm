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

#include <cstring>
#include <functional>
#include <memory>

#include "third_party/absl/memory/memory.h"
#include "third_party/absl/synchronization/mutex.h"
#include "core/audio/audio_source.h"
#include "core/media/media_asset.h"
#include "core/view/framework/assets/asset_manager.h"

#import <AVFoundation/AVFoundation.h>

@class AudioDelegate;

namespace imp {
namespace audio {
namespace {

class IOSAudioSource : public AudioSource {
 public:
  explicit IOSAudioSource(BaseView* view);

  Future<absl::Status> Load(const MediaAsset* audio_asset) override;

  absl::Status LoadSync(const MediaAsset* audio_asset) override;

  absl::Status Play() override;

  absl::Status Pause() override;

  absl::Status Stop() override;

  absl::Status SetPlaybackSpeed(float speed) override;

  absl::Status SeekTo(float seconds, SeekType seek_type) override;

  absl::Status SetLoopCount(int loop) override;

  absl::Status SetVolume(float volume) override;

  absl::StatusOr<absl::Duration> GetDuration() const override;

  absl::StatusOr<absl::Duration> GetPlaybackTime() const override;

  absl::StatusOr<int> GetLoopCount() const override;

  AudioSource::State GetState() const override;

  void SetOnPlaybackCompleteCallback(std::function<void()> callback) override;

  void SetOnSeekCompleteCallback(std::function<void()> callback) override;

  void SetOnBufferingCallback(std::function<void(BufferingState)> callback) override;

  void OnPlaybackComplete();

 private:
  // TODO Improve on maintaining player state instead of mutexing everything.
  AVAudioPlayer* audio_player_ ABSL_GUARDED_BY(mu_);
  mutable absl::Mutex mu_;
  AudioSource::State state_ ABSL_GUARDED_BY(mu_);
  AudioDelegate* audio_delegate_;

  // When playback has completed or Stop() is called,
  // on_playback_complete_callback is called to signal to the caller that the
  // player is now unused.
  std::function<void()> on_playback_complete_callback_;
};

}  // namespace
}  // namespace audio
}  // namespace imp

// This interface declaration and implementation has to be made sandwiched here as there's a strict
// declaration order that has to be maintained here, and breaking it into multiple files may seem
// more confusing. Due to a circular dependency in IOSAudioSource and AudioDelegate, AudioDelegate
// declaration has to happen after IOSAudioSource declaration, IOSAudioSource definition has to
// happen after AudioDelegate declaration, and IOSAudioSource requires a forward declaration of
// AudioDelegate.
@interface AudioDelegate : NSObject <AVAudioPlayerDelegate> {
  // A pointer to the AudioSource class that wraps and initializes AVAudioPlayer and by extension
  // this delegate. The lifecycle of the delegate and the AVAudioPlayer is tied to the AudioSource,
  // which is will be managed by the user of the API.
  imp::audio::IOSAudioSource* _audioSource;
  // YES if we get an interruption notification while playing.
  BOOL _interruptedPlayback;
}
- (id)initWithAudioSource:(imp::audio::IOSAudioSource*)audioSource;
@end

@implementation AudioDelegate
- (id)initWithAudioSource:(imp::audio::IOSAudioSource*)audioSource {
  self = [super init];
  if (self) {
    _audioSource = audioSource;
    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(handleInterruptionNotification:)
                                                 name:AVAudioSessionInterruptionNotification
                                               object:nil];
  }
  return self;
}

- (void)audioPlayerDidFinishPlaying:(AVAudioPlayer*)player successfully:(BOOL)flag {
  if (_audioSource) {
    _audioSource->OnPlaybackComplete();
  }
}

- (void)handleInterruptionNotification:(NSNotification*)notification {
  NSNumber* typeNumber = notification.userInfo[AVAudioSessionInterruptionTypeKey];
  AVAudioSessionInterruptionType type =
      (AVAudioSessionInterruptionType)typeNumber.unsignedIntegerValue;
  switch (type) {
    case AVAudioSessionInterruptionTypeBegan: {
      if (_audioSource->GetState() == imp::audio::AudioSource::State::kPlaying) {
        _audioSource->Pause().IgnoreError();
        _interruptedPlayback = YES;
      }
      break;
    }
    case AVAudioSessionInterruptionTypeEnded: {
      if (_interruptedPlayback) {
        _audioSource->Play().IgnoreError();
        _interruptedPlayback = NO;
      }
      break;
    }
    default: {
      break;
    }
  }
}

@end

namespace imp {
namespace audio {
namespace {

IOSAudioSource::IOSAudioSource(BaseView* view) : AudioSource(view) {
  audio_delegate_ = [[AudioDelegate alloc] initWithAudioSource:this];
}

Future<absl::Status> IOSAudioSource::Load(const MediaAsset* audio_asset) {
  return Future<absl::Status>::Schedule(
      [this, audio_asset]() -> absl::Status { return LoadSync(audio_asset); },
      Executor::Type::kBackground);
}

absl::Status IOSAudioSource::LoadSync(const MediaAsset* audio_asset) {
  absl::MutexLock lock(&mu_);
  NSError* error;
  NSData* data = [NSData dataWithBytes:audio_asset->GetData() length:audio_asset->GetSize()];
  if (audio_player_) {
    [audio_player_ stop];
  }
  audio_player_ = [[AVAudioPlayer alloc] initWithData:data error:&error];
  if (audio_player_.delegate) {
    audio_player_.delegate = nil;
  }
  audio_player_.delegate = audio_delegate_;
  if (error) {
    return absl::InternalError(
        FormatString("Unable to create player: '%s'", [[error localizedDescription] UTF8String]));
  }
  [audio_player_ prepareToPlay];
  [[AVAudioSession sharedInstance] setCategory:AVAudioSessionCategorySoloAmbient error:&error];
  if (error) {
    return absl::InternalError(FormatString("Unable to set playback category: '%s'",
                                            [[error localizedDescription] UTF8String]));
  }
  [[AVAudioSession sharedInstance] setActive:YES error:&error];
  if (error) {
    return absl::InternalError(FormatString("Unable to set this app as the active player: '%s'",
                                            [[error localizedDescription] UTF8String]));
  }
  return absl::Status();
}

absl::Status IOSAudioSource::Play() {
  absl::MutexLock lock(&mu_);
  if ([audio_player_ play]) {
    state_ = State::kPlaying;
    return absl::Status();
  } else {
    return absl::InternalError("Unable to play audio from AVAudioPlayer");
  }
}

absl::Status IOSAudioSource::Pause() {
  absl::MutexLock lock(&mu_);
  [audio_player_ pause];
  state_ = State::kReady;
  return absl::Status();
}

absl::Status IOSAudioSource::Stop() {
  {
    absl::MutexLock lock(&mu_);
    [audio_player_ stop];
    state_ = State::kStopped;
  }
  OnPlaybackComplete();
  return absl::Status();
}

absl::Status IOSAudioSource::SetPlaybackSpeed(float speed) {
  return absl::UnimplementedError("Not implemented");
}

absl::Status IOSAudioSource::SeekTo(const float seconds, SeekType seek_type) {
  absl::MutexLock lock(&mu_);
  audio_player_.currentTime = seconds;
  return absl::Status();
}

absl::Status IOSAudioSource::SetLoopCount(const int loop) {
  absl::MutexLock lock(&mu_);
  audio_player_.numberOfLoops = loop;
  return absl::Status();
}

absl::Status IOSAudioSource::SetVolume(const float volume) {
  absl::MutexLock lock(&mu_);
  audio_player_.volume = volume;
  return absl::Status();
}

absl::StatusOr<absl::Duration> IOSAudioSource::GetDuration() const {
  absl::MutexLock lock(&mu_);
  return absl::Seconds(audio_player_.duration);
}

absl::StatusOr<absl::Duration> IOSAudioSource::GetPlaybackTime() const {
  absl::MutexLock lock(&mu_);
  return absl::Seconds(audio_player_.currentTime);
}

absl::StatusOr<int> IOSAudioSource::GetLoopCount() const {
  absl::MutexLock lock(&mu_);
  return audio_player_.numberOfLoops;
}

AudioSource::State IOSAudioSource::GetState() const {
  absl::MutexLock lock(&mu_);
  return state_;
}

void IOSAudioSource::SetOnPlaybackCompleteCallback(std::function<void()> callback) {
  on_playback_complete_callback_ = callback;
}

void IOSAudioSource::SetOnSeekCompleteCallback(std::function<void()> callback) {
  // Not implemented.
}

void IOSAudioSource::SetOnBufferingCallback(std::function<void(BufferingState)> callback) {
  // Not implemented.
}

void IOSAudioSource::OnPlaybackComplete() {
  {
    absl::MutexLock lock(&mu_);
    state_ = AudioSource::State::kReady;
  }
  if (on_playback_complete_callback_) {
    on_playback_complete_callback_();
  }
}

Future<std::unique_ptr<AudioSource>> CreateIOSAudioSource(
    BaseView& view, const AssetPtr<media::MediaAsset>& media_asset) {
  return Future<std::unique_ptr<AudioSource>>::Schedule(
      [&view, media_asset]() -> absl::StatusOr<std::unique_ptr<AudioSource>> {
        if (!media_asset) {
          return absl::InternalError("Invalid MediaAsset");
        }

        std::unique_ptr<AudioSource> audio_source = absl::make_unique<IOSAudioSource>(&view);
        MP_RETURN_IF_ERROR(audio_source->LoadSync(media_asset.Get()));
        return audio_source;
      },
      Executor::Type::kBackground);
}

}  // namespace

Future<std::unique_ptr<AudioSource>> CreateAudioSource(BaseView& view,
                                                       absl::string_view asset_url) {
  if (asset_url.empty()) {
    return Future<std::unique_ptr<AudioSource>>(
        absl::InvalidArgumentError("Empty asset_url cannot be loaded!"));
  }

  // TODO Switch to std::make_unique after updating Lint
  return view.GetAssetManager().LoadMedia(asset_url).Then(
      [&view](
          const AssetPtr<media::MediaAsset>& media_asset) -> Future<std::unique_ptr<AudioSource>> {
        return CreateIOSAudioSource(view, media_asset);
      });
}

Future<std::unique_ptr<AudioSource>> CreateAudioSource(BaseView& base_view, absl::Cord content) {
  std::optional<absl::string_view> flattened_string = content.TryFlat();
  Future<AssetPtr<MediaAsset>> media_asset;
  if (flattened_string) {
    media_asset = base_view.GetAssetManager().LoadAsset<MediaAsset>(content, /*asset_url=*/"");
  } else {
    media_asset =
        Future<absl::Cord>::Schedule(
            [contents = std::move(content)]() mutable {
              contents.Flatten();
              return std::move(contents);
            },
            Executor::Type::kBackground)
            .Then([&base_view](absl::Cord flattened_cord) mutable {
              return base_view.GetAssetManager().LoadAsset<MediaAsset>(std::move(flattened_cord),
                                                                       /*asset_url=*/"");
            });
  }

  return media_asset.Then([&base_view](const AssetPtr<media::MediaAsset>& media_asset) mutable
                          -> Future<std::unique_ptr<AudioSource>> {
    return CreateIOSAudioSource(base_view, media_asset);
  });
}

}  // namespace audio
}  // namespace imp
