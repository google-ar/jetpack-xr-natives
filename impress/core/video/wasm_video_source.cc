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

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/math/vec.h"
#include "core/media/media_asset.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/texture.h"
#include "core/video/video_source.h"
#include "core/view/base_view.h"

namespace imp::video {

using ::imp::media::MediaAsset;

constexpr absl::string_view kUnimplemented =
    "Wasm support for video does not exist.";

class WasmVideoSource : public VideoSource {
 public:
  explicit WasmVideoSource(BaseView* view);

  WasmVideoSource(const WasmVideoSource&) = delete;
  WasmVideoSource& operator=(const WasmVideoSource&) = delete;

  Future<absl::Status> Load(const MediaAsset* audio_asset) override;
  absl::Status LoadSync(const MediaAsset* audio_asset) override;

  absl::StatusOr<Texture*> CreateVideoTexture() override { return nullptr; }

  absl::StatusOr<BorrowedTexturePtr> BorrowVideoTextureImpl(
      SmallSourceLocation loc) override {
    return BorrowedTexturePtr();
  }

  void UpdateVideoTexture(filament::Texture* texture,
                          absl::Duration frame_delta) override {};

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

  VideoSource::State GetState() const override;

  uint2 GetVideoSize() const override;

  MediaColorSpace GetColorSpace() const override;
  MediaStereoMode GetStereoMode() const override;

  void SetOnPlaybackCompleteCallback(std::function<void()> callback) override;

  void SetOnSeekCompleteCallback(std::function<void()> callback) override;

  void SetOnBufferingCallback(
      std::function<void(BufferingState)> callback) override;

  void OnPlaybackComplete();

 private:
  State state_;
  int loop_count_;
  float volume_;
  std::function<void()> on_playback_complete_callback_;
};

WasmVideoSource::WasmVideoSource(BaseView* view)
    : VideoSource(view), state_(State::kReady) {}

Future<absl::Status> WasmVideoSource::Load(const MediaAsset* audio_asset) {
  return Future<absl::Status>(absl::UnimplementedError(kUnimplemented));
}

absl::Status WasmVideoSource::LoadSync(const MediaAsset* audio_asset) {
  return absl::UnimplementedError(kUnimplemented);
}

absl::Status WasmVideoSource::Play() {
  state_ = State::kPlaying;
  return absl::UnimplementedError(kUnimplemented);
}

absl::Status WasmVideoSource::Pause() {
  state_ = State::kReady;
  return absl::UnimplementedError(kUnimplemented);
}

absl::Status WasmVideoSource::Stop() {
  state_ = State::kStopped;
  OnPlaybackComplete();
  return absl::UnimplementedError(kUnimplemented);
}

absl::Status WasmVideoSource::SetPlaybackSpeed(float speed) {
  return absl::UnimplementedError("Not implemented");
}

absl::Status WasmVideoSource::SeekTo(const float seconds, SeekType seek_type) {
  return absl::UnimplementedError(kUnimplemented);
}

absl::Status WasmVideoSource::SetLoopCount(const int loop) {
  loop_count_ = loop;
  return absl::UnimplementedError(kUnimplemented);
}

absl::Status WasmVideoSource::SetVolume(const float volume) {
  volume_ = volume;
  return absl::UnimplementedError(kUnimplemented);
}

absl::StatusOr<absl::Duration> WasmVideoSource::GetDuration() const {
  return absl::ZeroDuration();
}

absl::StatusOr<absl::Duration> WasmVideoSource::GetPlaybackTime() const {
  return absl::ZeroDuration();
}

absl::StatusOr<int> WasmVideoSource::GetLoopCount() const {
  return loop_count_;
}

VideoSource::State WasmVideoSource::GetState() const { return state_; }

uint2 WasmVideoSource::GetVideoSize() const { return {0, 0}; }

MediaColorSpace WasmVideoSource::GetColorSpace() const {
  return MediaColorSpace();
}

MediaStereoMode WasmVideoSource::GetStereoMode() const {
  return MediaStereoMode::kUnknown;
}

void WasmVideoSource::SetOnPlaybackCompleteCallback(
    std::function<void()> callback) {
  on_playback_complete_callback_ = callback;
}

void WasmVideoSource::SetOnSeekCompleteCallback(
    std::function<void()> callback) {
  // Not implemented.
}

void WasmVideoSource::SetOnBufferingCallback(
    std::function<void(BufferingState)> callback) {
  // Not implemented.
}

void WasmVideoSource::OnPlaybackComplete() {
  state_ = VideoSource::State::kStopped;
  if (on_playback_complete_callback_) {
    on_playback_complete_callback_();
  }
}

Future<std::unique_ptr<VideoSource>> CreateVideoSource(
    BaseView& base_view, absl::string_view asset_url) {
  return Future<std::unique_ptr<VideoSource>>(
      std::make_unique<WasmVideoSource>(&base_view));
}

}  // namespace imp::video
