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
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/assets/asset_ptr.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/audio/audio_source.h"
#include "core/common/registry.h"
#include "core/media/media_asset.h"
#include "core/media/media_source.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "proposed/delayed_future_scheduler.h"

namespace imp {
namespace audio {
namespace {

using media::MediaAsset;

class NoopAudioSource : public AudioSource {
 public:
  explicit NoopAudioSource(BaseView* view);

  NoopAudioSource(const NoopAudioSource&) = delete;
  NoopAudioSource& operator=(const NoopAudioSource&) = delete;

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

  void SetOnBufferingCallback(
      std::function<void(BufferingState)> callback) override;

  void OnPlaybackComplete();

 private:
  State state_ = AudioSource::State::kReady;
  std::function<void()> on_playback_complete_callback_;

  int loop_count_ = 0;

  Future<absl::Status> playback_future_;
  BaseView& view_;
};

NoopAudioSource::NoopAudioSource(BaseView* view)
    : AudioSource(view), view_(*view) {}

Future<absl::Status> NoopAudioSource::Load(const MediaAsset* audio_asset) {
  return Future<absl::Status>::Schedule(
      []() -> absl::Status { return absl::Status(); },
      Executor::Type::kForeground);
}

absl::Status NoopAudioSource::LoadSync(const MediaAsset* audio_asset) {
  return absl::Status();
}

absl::Status NoopAudioSource::Play() {
  state_ = State::kPlaying;
  DelayedFutureScheduler& scheduler =
      view_.GetRegistry().GetOrCreate<imp::DelayedFutureScheduler>(&view_);
  playback_future_ = scheduler.ScheduleDelayed<absl::Status>(
      internal::kDebugPlaybackDuration, [this]() { return Stop(); });

  return absl::Status();
}

absl::Status NoopAudioSource::Pause() {
  state_ = State::kReady;
  return absl::Status();
}

absl::Status NoopAudioSource::Stop() {
  state_ = State::kStopped;
  OnPlaybackComplete();
  return absl::Status();
}

absl::Status NoopAudioSource::SetPlaybackSpeed(float speed) {
  return absl::UnimplementedError("Not implemented");
}

absl::Status NoopAudioSource::SeekTo(const float seconds, SeekType seek_type) {
  return absl::Status();
}

absl::Status NoopAudioSource::SetLoopCount(const int loop) {
  loop_count_ = loop;
  return absl::Status();
}

absl::Status NoopAudioSource::SetVolume(const float volume) {
  return absl::Status();
}

absl::StatusOr<absl::Duration> NoopAudioSource::GetDuration() const {
  return absl::ZeroDuration();
}

absl::StatusOr<absl::Duration> NoopAudioSource::GetPlaybackTime() const {
  return absl::ZeroDuration();
}

absl::StatusOr<int> NoopAudioSource::GetLoopCount() const {
  return loop_count_;
}

AudioSource::State NoopAudioSource::GetState() const { return state_; }

void NoopAudioSource::SetOnPlaybackCompleteCallback(
    std::function<void()> callback) {
  on_playback_complete_callback_ = callback;
}

void NoopAudioSource::SetOnSeekCompleteCallback(
    std::function<void()> callback) {
  // Not implemented.
}

void NoopAudioSource::SetOnBufferingCallback(
    std::function<void(BufferingState)> callback) {
  // Not implemented.
}

void NoopAudioSource::OnPlaybackComplete() {
  state_ = AudioSource::State::kStopped;
  if (on_playback_complete_callback_) {
    on_playback_complete_callback_();
  }
}

}  // namespace

Future<std::unique_ptr<AudioSource>> CreateAudioSource(
    BaseView& view, absl::string_view asset_url) {
  if (asset_url.empty()) {
    return Future<std::unique_ptr<AudioSource>>(
        absl::InvalidArgumentError("Empty asset_url cannot be loaded!"));
  }
  return view.GetAssetManager().LoadMedia(asset_url).Then(
      [&view](const AssetPtr<MediaAsset>& media_asset) {
        std::unique_ptr<AudioSource> result =
            std::make_unique<NoopAudioSource>(&view);
        return result;
      });
}

Future<std::unique_ptr<AudioSource>> CreateAudioSource(BaseView& base_view,
                                                       absl::Cord content) {
  std::optional<absl::string_view> flattened_string = content.TryFlat();
  Future<AssetPtr<MediaAsset>> media_asset;
  if (flattened_string) {
    media_asset =
        base_view.GetAssetManager().LoadAsset<MediaAsset>(content, "");
  } else {
    media_asset =
        Future<absl::Cord>::Schedule(
            [contents = std::move(content)]() mutable {
              contents.Flatten();
              return std::move(contents);
            },
            Executor::Type::kBackground)
            .Then([&base_view](absl::Cord flattened_cord) mutable {
              return base_view.GetAssetManager().LoadAsset<MediaAsset>(
                  std::move(flattened_cord), "");
            });
  }
  return media_asset.Then(
      [&base_view](const AssetPtr<MediaAsset>& media_asset) {
        std::unique_ptr<AudioSource> result =
            std::make_unique<NoopAudioSource>(&base_view);
        return result;
      });
}

}  // namespace audio
}  // namespace imp
