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

#include "core/video/video_controller.h"

#include <iterator>
#include <memory>
#include <string>
#include <utility>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/media/media_source.h"
#include "core/media/media_type.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/node.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/texture.h"
#include "core/video/video_color_space.h"
#include "core/video/video_controller_state.proto.imp.h"
#include "core/video/video_source.h"
#include "core/video/video_source_factory.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

using State = ::imp::media::MediaSource::State;

VideoController::System::System(BaseView* view)
    : ComponentSystem<VideoController>(view) {}

void VideoController::System::SetCustomVideoSourceFactory(
    std::unique_ptr<video::VideoSourceFactory> factory) {
  VideoController::System::video_source_factory_ = std::move(factory);
}

video::VideoSourceFactory* VideoController::System::GetVideoSourceFactory() {
  return VideoController::System::video_source_factory_.get();
}

Future<absl::Status> VideoController::Setup() {
  // This version of Setup should only be called as part of an isf load.
  if (state_.asset.empty()) {
    return Future<absl::Status>(
        absl::FailedPreconditionError("Video asset has not been specified."));
  }
  std::string drm_license_url =
      !state_.drm_license_url.has_value() ? "" : state_.drm_license_url.value();
  std::string drm_scheme_uuid =
      !state_.drm_scheme_uuid.has_value() ? "" : state_.drm_scheme_uuid.value();
  return Setup(state_.asset, drm_license_url, drm_scheme_uuid,
               state_.multiview_mode);
}

Future<absl::Status> VideoController::Setup(
    const AssetDefinition& asset_definition) {
  return Setup(asset_definition.GetUrl());
}

Future<absl::Status> VideoController::Setup(
    absl::string_view asset_url, absl::string_view drm_license_url,
    absl::string_view drm_scheme_uuid,
    VideoControllerState::MultiviewMode multiview_mode) {
  std::string last_url = state_.asset;
  state_.asset = std::string(asset_url);

  // Start loading the new video before actually swapping the video source.
  video::VideoSourceFactory* video_source_factory =
      GetView()
          .GetComponentManager()
          .GetComponentSystem<VideoController>()
          .GetVideoSourceFactory();

  if (!drm_license_url.empty() || !drm_scheme_uuid.empty() ||
      (multiview_mode != VideoControllerState::UNSET &&
       multiview_mode != VideoControllerState::MONOSCOPIC)) {
    // DRM and multiview content require a custom VideoSourceFactory.
    if (video_source_factory == nullptr) {
      return Future<absl::Status>(absl::FailedPreconditionError(
          "Playing DRM content requires an ExoPlayerVideoSourceFactory."));
    }
  }

  absl::Span<const SurfaceViewType> view_types =
      kAndroidExternalTextureSurfaceConfigMono;
  if (multiview_mode == VideoControllerState::MULTIVIEW) {
    view_types = kAndroidExternalTextureSurfaceConfigStereo;
  }

  Future<absl::Status> video_source_future =
      (video_source_factory != nullptr
           ? video_source_factory->CreateVideoSource(
                 GetView(), state_.asset, drm_license_url, drm_scheme_uuid,
                 view_types)
           : video::CreateVideoSource(GetView(), state_.asset))
          .Then([this, last_url](
                    absl::StatusOr<std::unique_ptr<video::VideoSource>> status)
                    -> absl::Status {
            if (status.ok()) {
              return OnVideoLoaded(std::move(status.value()));
            }
            state_.asset = last_url;
            return status.status();
          });

  video_source_future_ = WeakFuture<absl::Status>(video_source_future);

  return video_source_future;
}

void VideoController::Cleanup() { CancelLoad(); }

absl::Status VideoController::OnVideoLoaded(
    std::unique_ptr<video::VideoSource> source) {
  // Stop old video if it has not been stopped already.
  // Note: this will trigger an OnPlaybackCompleted event.
  if (source_ && source_->GetState() != video::VideoSource::State::kStopped) {
    source_->Stop().IgnoreError();
  }

  // Release textures before assigning a new video source.
  textures_.clear();
  // Now that the new video source is loaded, switch the asset.
  source_ = std::move(source);
  video_color_space_ = source_->GetColorSpace();
  media_stereo_mode_ = source_->GetStereoMode();
// Set up textures for the video source.
#if IMP_PLATFORM(ANDROID) && \
    defined(IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER)
  MP_ASSIGN_OR_RETURN(textures_, source_->BorrowVideoTextures());
#else
  MP_ASSIGN_OR_RETURN(textures_[SurfaceViewType::kPrimaryView],
                   source_->BorrowVideoTexture());
#endif
  // Set up callbacks for the video source.
  source_->SetOnPlaybackCompleteCallback([this]() {
    GetNode()->Send(VideoController::PlaybackCompleteEvent(GetHandle(this)));
  });
  source_->SetOnSeekCompleteCallback([this]() {
    GetNode()->Send(VideoController::SeekCompleteEvent(GetHandle(this)));
  });
  source_->SetOnBufferingCallback([this](BufferingState buffering_state) {
    GetNode()->Send(
        VideoController::BufferingEvent(GetHandle(this), buffering_state));
  });

  // Send out an event signifying that the video has loaded.
  GetNode()->Send(VideoController::VideoLoadedEvent(GetHandle(this)));
  return absl::OkStatus();
}

void VideoController::Update(const imp::FrameTime& frame_time) {
  if (!source_ || !textures_[SurfaceViewType::kPrimaryView]) {
    return;
  }
  // Texture is updated for next video frame. If the video source supports
  // filament streaming, calling this is no-op.
  source_->UpdateVideoTexture(
      textures_[SurfaceViewType::kPrimaryView]->GetTexture(),
      frame_time.GetDeltaTime());
}

imp::Texture* VideoController::GetVideoTexture() const {
  if (!textures_.contains(SurfaceViewType::kPrimaryView)) return nullptr;
  return &(*textures_.at(SurfaceViewType::kPrimaryView));
}

imp::BorrowedTexturePtr VideoController::BorrowVideoTexture(
    SmallSourceLocation loc) const {
  if (!textures_.contains(SurfaceViewType::kPrimaryView)) {
    return BorrowedTexturePtr();
  }
  return textures_.at(SurfaceViewType::kPrimaryView).WithNewLocation(loc);
}

RobinMap<SurfaceViewType, imp::Texture*> VideoController::GetVideoTextures()
    const {
  RobinMap<SurfaceViewType, imp::Texture*> result;
  absl::c_transform(textures_, std::inserter(result, result.end()),
                    [](const auto& texture) {
                      return std::make_pair(texture.first, &(*texture.second));
                    });
  return result;
}

RobinMap<SurfaceViewType, imp::BorrowedTexturePtr>
VideoController::BorrowVideoTextures(SmallSourceLocation loc) const {
  RobinMap<SurfaceViewType, imp::BorrowedTexturePtr> result;
  absl::c_transform(textures_, std::inserter(result, result.end()),
                    [&loc](const auto& texture) {
                      return std::make_pair(
                          texture.first, texture.second.WithNewLocation(loc));
                    });
  return result;
}

MediaStereoMode VideoController::GetMediaStereoMode() const {
  return media_stereo_mode_;
}

video::VideoColorSpace VideoController::GetVideoColorSpace() const {
  return video_color_space_;
}

video::VideoColorSpace VideoController::GetSourceColorSpace() const {
  return source_->GetColorSpace();
}

Future<absl::Status> VideoController::LoadVideoAsset(
    absl::string_view asset_url, absl::string_view drm_license_url,
    absl::string_view drm_scheme_uuid,
    VideoControllerState::MultiviewMode multiview_mode) {
  // If this is the same asset, there is no need to reload.
  // Simply rewind and send out a loaded event.
  if (asset_url == state_.asset && source_ &&
      source_->GetState() == media::MediaSource::State::kReady) {
    if (source_->SeekTo(0, SeekType::QUICK).ok()) {
      GetNode()->Send(VideoController::VideoLoadedEvent(GetHandle(this)));
      return Future<absl::Status>(absl::OkStatus());
    }
  }
  // Cancel any current loading jobs and call setup.
  CancelLoad();
  return Setup(asset_url, drm_license_url, drm_scheme_uuid, multiview_mode);
}

void VideoController::CancelLoad() {
  absl::optional<Future<absl::Status>> video_source_future_option =
      video_source_future_.Lock();
  if (video_source_future_option) {
    video_source_future_option->Cancel();
    video_source_future_ = {};
  }
}

absl::Status VideoController::Play() {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->Play();
}

absl::Status VideoController::Pause() {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->Pause();
}

absl::Status VideoController::Stop() {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->Stop();
}

absl::Status VideoController::SetPlaybackSpeed(float speed) {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->SetPlaybackSpeed(speed);
}

absl::Status VideoController::SeekTo(float seconds, SeekType seek_type) {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->SeekTo(seconds, seek_type);
}

absl::Status VideoController::SetLoopCount(int loop) {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->SetLoopCount(loop);
}

absl::Status VideoController::SetVolume(float volume) {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->SetVolume(volume);
}

absl::StatusOr<absl::Duration> VideoController::GetDuration() const {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->GetDuration();
}

absl::StatusOr<absl::Duration> VideoController::GetPlaybackTime() const {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->GetPlaybackTime();
}

absl::StatusOr<int> VideoController::GetLoopCount() const {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->GetLoopCount();
}

absl::StatusOr<State> VideoController::GetVideoState() const {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->GetState();
}

absl::StatusOr<uint2> VideoController::GetVideoSize() const {
  if (!source_) {
    return absl::InternalError("VideoController has no video loaded.");
  }
  return source_->GetVideoSize();
}

absl::string_view VideoController::GetAssetUrl() const { return state_.asset; }

void VideoController::OnActiveStatusChanged(bool active) {
  if (was_active_ == active) {
    return;
  }

  if (source_) {
    if (was_active_) {
      state_when_active_ = source_->GetState();
    }

    if (state_when_active_ == State::kPlaying) {
      if (!active) {
        (void)source_->Pause();
      } else {
        (void)source_->Play();
      }
    }
  }

  was_active_ = active;
}

#if IMP_RUNTIME(DEV)
void VideoController::DrawEditorUi() {
  if (helper_ == nullptr) {
    helper_ = std::make_unique<VideoControllerWidgetHelper>(GetHandle(this));
  }
  helper_->DrawVideoControllerUi();
}
#endif  // IMP_RUNTIME(DEV)

}  // namespace imp
