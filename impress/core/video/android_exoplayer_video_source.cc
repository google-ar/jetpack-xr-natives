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

#include "core/video/android_exoplayer_video_source.h"

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/math/vec.h"
#include "core/media/android/android_exoplayer.h"
#include "core/media/android/android_exoplayer_listener.h"
#include "core/media/media_asset.h"
#include "core/media/media_source.h"
#include "core/media/media_type.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/resources/resource_manager.h"
#include "core/video/video_color_space.h"
#include "core/video/video_source.h"
#include "core/view/base_view.h"
#include "util/task/status_builder.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::video {

absl::StatusOr<std::unique_ptr<AndroidExoPlayerVideoSource>>
AndroidExoPlayerVideoSource::Create(
    BaseView* view, ContentSecurityLevel security_level,
    absl::Span<const SurfaceViewType> view_types) {
  auto video_source =
      absl::WrapUnique(new AndroidExoPlayerVideoSource(view, security_level));
  MP_RETURN_IF_ERROR(video_source->CreateExternalTextureSurface(view_types));
  return video_source;
}

AndroidExoPlayerVideoSource::AndroidExoPlayerVideoSource(
    BaseView* view, ContentSecurityLevel security_level)
    : VideoSource(view), view_(*view), security_level_(security_level) {}

absl::Status AndroidExoPlayerVideoSource::CreateExternalTextureSurface(
    absl::Span<const SurfaceViewType> view_types) {
  MP_ASSIGN_OR_RETURN(surface_, AndroidExternalTextureSurface::Create(
                                 view_, security_level_, view_types));
  return absl::OkStatus();
}

absl::StatusOr<Texture*> AndroidExoPlayerVideoSource::CreateVideoTexture() {
  return surface_->GetTexture();
}

absl::StatusOr<BorrowedTexturePtr>
AndroidExoPlayerVideoSource::BorrowVideoTextureImpl(SmallSourceLocation loc) {
  return surface_->BorrowTexture(loc);
}

absl::StatusOr<RobinMap<SurfaceViewType, Texture*>>
AndroidExoPlayerVideoSource::CreateVideoTextures() {
  return surface_->GetTextures();
}

absl::StatusOr<RobinMap<SurfaceViewType, BorrowedTexturePtr>>
AndroidExoPlayerVideoSource::BorrowVideoTextures(SmallSourceLocation loc) {
  return surface_->BorrowTextures(loc);
}

void AndroidExoPlayerVideoSource::UpdateVideoTexture(
    filament::Texture* texture, absl::Duration frame_delta) {}

uint2 AndroidExoPlayerVideoSource::GetVideoSize() const {
  absl::MutexLock lock(&mu_);
  absl::StatusOr<int> width = exoplayer_->GetVideoWidth();
  absl::StatusOr<int> height = exoplayer_->GetVideoHeight();
  if (!width.ok() || !height.ok()) {
    IMP_LOG(imp::ERROR) << "Unable to get video size";
    return {0, 0};
  }
  return {*width, *height};
}

VideoColorSpace AndroidExoPlayerVideoSource::GetColorSpace() const {
  absl::MutexLock lock(&mu_);
  absl::StatusOr<VideoColorSpace> color_space = exoplayer_->GetColorSpace();
  if (!color_space.ok()) {
    IMP_LOG(imp::ERROR) << "Unable to get color space.";
    return VideoColorSpace();
  }
  return *color_space;
}

MediaStereoMode AndroidExoPlayerVideoSource::GetStereoMode() const {
  absl::MutexLock lock(&mu_);
  absl::StatusOr<MediaStereoMode> stereo_mode = exoplayer_->GetStereoMode();
  if (!stereo_mode.ok()) {
    IMP_LOG(imp::ERROR) << "Unable to get stereo mode.";
    return MediaStereoMode::kUnknown;
  }
  return *stereo_mode;
}

void AndroidExoPlayerVideoSource::SetUpExoPlayer() {
  absl::MutexLock lock(&mu_);
  exoplayer_->SetListener(exoplayer_listener_.get());
  exoplayer_->SetVideoSurface(surface_->GetSurface());
}

absl::Status AndroidExoPlayerVideoSource::Play() {
  absl::MutexLock lock(&mu_);
  bool result = exoplayer_->Play();
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to start playback");
  }
  state_ = MediaSource::State::kPlaying;
  return absl::Status();
}

absl::Status AndroidExoPlayerVideoSource::Pause() {
  absl::MutexLock lock(&mu_);
  bool result = exoplayer_->Pause();
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to pause playback");
  }
  state_ = MediaSource::State::kReady;
  return absl::Status();
}

absl::Status AndroidExoPlayerVideoSource::Stop() {
  absl::ReleasableMutexLock lock(&mu_);
  bool result = exoplayer_->Stop();
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to stop playback");
  }

  state_ = MediaSource::State::kStopped;
  lock.Release();
  OnPlaybackComplete();
  return absl::Status();
}

absl::Status AndroidExoPlayerVideoSource::SetPlaybackSpeed(float speed) {
  absl::MutexLock lock(&mu_);
  bool result = exoplayer_->SetPlaybackSpeed(speed);
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return util::InternalErrorBuilder()
           << "Unable to set playback speed to " << speed;
  }
  return absl::Status();
}

absl::Status AndroidExoPlayerVideoSource::SeekTo(
    float seconds, MediaSource::SeekType seek_type) {
  absl::MutexLock lock(&mu_);
  bool result = exoplayer_->SeekTo(seconds * 1000, seek_type);
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to seek media");
  }
  return absl::OkStatus();
}

absl::Status AndroidExoPlayerVideoSource::SetLoopCount(int loop) {
  if (loop > 0) {
    return absl::InternalError(
        "ExoPlayer only supports infinite looping or one-time playback");
  }

  absl::MutexLock lock(&mu_);
  // TODO: Retrieve Java error and pipe it here
  bool result = exoplayer_->SetLooping(loop < 0);
  if (!result) {
    return absl::InternalError("Unable to set loop mode");
  }
  return absl::OkStatus();
}

absl::Status AndroidExoPlayerVideoSource::SetVolume(float volume) {
  absl::MutexLock lock(&mu_);
  bool result = exoplayer_->SetVolume(volume);
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to set volume");
  }
  return absl::Status();
}

absl::StatusOr<absl::Duration> AndroidExoPlayerVideoSource::GetDuration()
    const {
  absl::MutexLock lock(&mu_);
  absl::StatusOr<int> duration_milliseconds = exoplayer_->GetDuration();
  // Returned value is negative if there is an error/exception
  if (!duration_milliseconds.ok()) {
    return duration_milliseconds.status();
  }
  return absl::Milliseconds(*duration_milliseconds);
}

absl::StatusOr<absl::Duration> AndroidExoPlayerVideoSource::GetPlaybackTime()
    const {
  absl::MutexLock lock(&mu_);
  absl::StatusOr<int> position_milliseconds = exoplayer_->GetCurrentPosition();
  // Returned value is negative if there is an error/exception
  if (!position_milliseconds.ok()) {
    return position_milliseconds.status();
  }
  return absl::Milliseconds(*position_milliseconds);
}

absl::StatusOr<int> AndroidExoPlayerVideoSource::GetLoopCount() const {
  absl::MutexLock lock(&mu_);
  absl::StatusOr<bool> result = exoplayer_->IsLooping();
  if (!result.ok()) {
    return result.status();
  }

  return *result ? MediaSource::kLoopIndefinite : MediaSource::kLoopSingleShot;
}

media::MediaSource::State AndroidExoPlayerVideoSource::GetState() const {
  absl::MutexLock lock(&mu_);
  return state_;
}

void AndroidExoPlayerVideoSource::SetOnPlaybackCompleteCallback(
    std::function<void()> callback) {
  on_playback_complete_callback_ = callback;
}

void AndroidExoPlayerVideoSource::SetOnSeekCompleteCallback(
    std::function<void()> callback) {
  on_seek_complete_callback_ = callback;
}

void AndroidExoPlayerVideoSource::SetOnBufferingCallback(
    std::function<void(MediaSource::BufferingState)> callback) {
  on_buffering_callback_ = callback;
}

void AndroidExoPlayerVideoSource::OnReady() {
  {
    absl::MutexLock lock(&mu_);
    state_ = MediaSource::State::kReady;
  }
  if (on_ready_callback_) {
    on_ready_callback_(absl::OkStatus());
  }
}

void AndroidExoPlayerVideoSource::OnPlaybackComplete() {
  {
    absl::MutexLock lock(&mu_);
    state_ = MediaSource::State::kReady;
  }
  if (on_playback_complete_callback_) {
    on_playback_complete_callback_();
  }
}

void AndroidExoPlayerVideoSource::OnSeekComplete() {
  if (on_seek_complete_callback_) {
    on_seek_complete_callback_();
  }
}

void AndroidExoPlayerVideoSource::OnBuffering(int buffering_state) {
  std::optional<MediaSource::BufferingState> buffering = std::nullopt;
  {
    absl::MutexLock lock(&mu_);
    if (buffering_state == exoplayer_->BufferingState()) {
      buffering = MediaSource::BufferingState::kBuffering;
    } else if (buffering_state == exoplayer_->BufferingDoneState()) {
      buffering = MediaSource::BufferingState::kBufferingDone;
    }
  }
  if (buffering.has_value() && on_buffering_callback_) {
    on_buffering_callback_(*buffering);
  }
}

absl::Status AndroidExoPlayerVideoSource::LoadSync(
    const media::MediaAsset* media_asset) {
  return absl::InternalError("Loading MediaAsset is not supported.");
}

Future<absl::Status> AndroidExoPlayerVideoSource::Load(
    const media::MediaAsset* media_asset) {
  return Future<absl::Status>(
      absl::InternalError("Loading MediaAsset is not supported."));
}

Future<absl::Status> AndroidExoPlayerVideoSource::Load(
    absl::string_view url, absl::string_view drm_license_url,
    absl::string_view drm_scheme_uuid) {
  exoplayer_listener_ =
      std::make_unique<media::ImpExoPlayerListener>(view_.GetContext(), this);

  Future<absl::Status> on_ready_result;
  on_ready_callback_ = [weak_on_ready_result = make_weak(on_ready_result)](
                           absl::Status status) mutable {
    absl::optional<Future<absl::Status>> on_ready_result =
        weak_on_ready_result.Lock();
    if (on_ready_result.has_value()) {
      on_ready_result->Return(status);
    }
  };

  {
    absl::MutexLock lock(&mu_);
    if (!exoplayer_) {
      exoplayer_ =
          std::make_unique<media::AndroidExoPlayer>(view_.GetContext());
    }
  }

  {
    absl::MutexLock lock(&mu_);
    if (drm_license_url.empty() || drm_scheme_uuid.empty()) {
      exoplayer_->SetMediaItem(std::string(url));
    } else {
      if (security_level_ == ContentSecurityLevel::kNone) {
        IMP_LOG(imp::ERROR) << "DRM content requires a protected security level. Media "
                      "item cannot be set.";
      } else {
        exoplayer_->SetProtectedMediaItem(std::string(url),
                                          std::string(drm_license_url),
                                          std::string(drm_scheme_uuid));
      }
    }
  }

  SetUpExoPlayer();
  Prepare();

  return on_ready_result;
}

void AndroidExoPlayerVideoSource::Prepare() {
  absl::MutexLock lock(&mu_);
  exoplayer_->Prepare();
}

Future<std::unique_ptr<video::VideoSource>> CreateVideoSource(
    BaseView& view, absl::string_view asset_url) {
  return {};
}

Future<std::unique_ptr<video::VideoSource>> CreateVideoSource(
    BaseView& view, absl::string_view asset_url,
    absl::string_view drm_license_url, absl::string_view drm_scheme_uuid) {
  IMP_LOG(imp::ERROR) << "Use ExoPlayerVideoSourceFactory to create a VideoSource.";
  return {};
}

}  // namespace imp::video
