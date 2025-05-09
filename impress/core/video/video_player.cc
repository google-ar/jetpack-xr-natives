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

#include "core/video/video_player.h"

#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/media/media_source.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/node.h"
#include "core/render/texture.h"
#include "core/video/video_player_state.proto.imp.h"
#include "core/video/video_source_factory.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/render/material.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

using State = ::imp::media::MediaSource::State;

constexpr absl::string_view kVideoTextureMaterialParameter = "videoTexture";

VideoPlayer::System::System(BaseView* view)
    : ComponentSystem<VideoPlayer>(view) {}

void VideoPlayer::System::SetCustomVideoSourceFactory(
    std::unique_ptr<video::VideoSourceFactory> factory) {
  VideoPlayer::System::video_source_factory_ = std::move(factory);
}

video::VideoSourceFactory* VideoPlayer::System::GetVideoSourceFactory() {
  return VideoPlayer::System::video_source_factory_.get();
}

Future<absl::Status> VideoPlayer::Setup() {
  // This version of Setup should only be called as part of an isf load.
  if (state_.asset.empty()) {
    return Future<absl::Status>(
        absl::FailedPreconditionError("Video asset has not been specified."));
  }

  return Setup(state_.asset, state_.mode);
}

Future<absl::Status> VideoPlayer::Setup(
    const AssetDefinition& asset_definition, VideoPlayerState::QuadMode mode,
    const imp::AssetDefinition& video_material) {
  return Setup(asset_definition.GetUrl(), mode, video_material);
}

Future<absl::Status> VideoPlayer::Setup(
    absl::string_view asset_url, VideoPlayerState::QuadMode mode,
    const imp::AssetDefinition& video_material) {
  state_.asset = std::string(asset_url);
  state_.mode = mode;

  // Start loading the material.
  Future<AssetPtr<MaterialAsset>> material_asset_future =
      GetView().GetAssetManager().LoadMaterial(video_material);

  // Start loading the video.
  video::VideoSourceFactory* video_source_factory =
      GetView()
          .GetComponentManager()
          .GetComponentSystem<VideoPlayer>()
          .GetVideoSourceFactory();
  Future<std::unique_ptr<video::VideoSource>> video_source_future =
      video_source_factory != nullptr
          ? video_source_factory->CreateVideoSource(GetView(), state_.asset)
          : video::CreateVideoSource(GetView(), state_.asset);

  // Wait until both the material & the video are loaded and then perform some
  // final setup work.
  return material_asset_future.Merge(video_source_future)
      .Then([this](std::tuple<AssetPtr<MaterialAsset>,
                              std::unique_ptr<video::VideoSource>>
                       tuple) {
        video_material_ = std::get<0>(tuple);
        source_ = std::move(std::get<1>(tuple));
        return OnMaterialAndVideoLoaded();
      });
}

absl::Status VideoPlayer::OnMaterialAndVideoLoaded() {
  // Set up the player's VideoSource, TexturePtr, and its playback
  // completion callback.
  MP_ASSIGN_OR_RETURN(texture_ptr_, source_->CreateVideoTexture());
  source_->SetOnPlaybackCompleteCallback([this]() {
    GetNode()->Send(VideoPlayer::PlaybackCompleteEvent(GetHandle(this)));
  });
  source_->SetOnSeekCompleteCallback([this]() {
    GetNode()->Send(VideoPlayer::SeekCompleteEvent(GetHandle(this)));
  });
  source_->SetOnBufferingCallback([this](BufferingState buffering_state) {
    GetNode()->Send(
        VideoPlayer::BufferingEvent(GetHandle(this), buffering_state));
  });

  // Add a VideoPlayer quad into the scene if kCreateQuad is
  // passed.
  if (state_.mode == VideoPlayerState::QuadMode::CREATE) {
    ComponentHandle<MeshRenderer> mesh_renderer =
        GetNode()->AddComponent<MeshRenderer>();
    // Create a quad with UV origin at top-left vertex to be consistent
    // with glb model mesh UV. Video material has flipUV as false and thus
    // will use the original UV.
    mesh_renderer->SetMesh(
        GetView().GetMeshFactory().CreateQuad({.flip_uv = true}));
    SetVideoMaterial(mesh_renderer);
    auto size = source_->GetVideoSize();
    if (size.x > 0 && size.y > 0) {
      float ratio = static_cast<float>(size.x) / static_cast<float>(size.y);
      GetNode()->SetLocalScale({ratio, 1, 1});
    }
  }
  return absl::OkStatus();
}

void VideoPlayer::SetVideoMaterial(
    ComponentHandle<MeshRenderer> mesh_renderer) {
  auto material_ptr = CreateVideoMaterial();
  mesh_renderer->SetMaterial(std::move(material_ptr));
}

void VideoPlayer::Update(const imp::FrameTime& frame_time) {
  if (!source_ || !texture_ptr_) {
    return;
  }
  // Texture is updated for next video frame. If the video source supports
  // filament streaming, calling this is no-op.
  source_->UpdateVideoTexture(texture_ptr_->GetTexture(),
                              frame_time.GetDeltaTime());
}

imp::MaterialPtr VideoPlayer::CreateVideoMaterial() const {
  auto material_ptr =
      GetView().GetMaterialFactory().CreateMaterial(video_material_);
  const std::string material_param(kVideoTextureMaterialParameter);
  material_ptr->SetParameter(material_param, GetVideoTexture());
  return material_ptr;
}

imp::Texture* VideoPlayer::GetVideoTexture() const { return texture_ptr_; }

Future<absl::Status> VideoPlayer::LoadVideoAsset(absl::string_view asset_url) {
  // Start loading the new video before actually swapping the video source.
  video::VideoSourceFactory* video_source_factory =
      GetView()
          .GetComponentManager()
          .GetComponentSystem<VideoPlayer>()
          .GetVideoSourceFactory();
  Future<std::unique_ptr<video::VideoSource>> video_source_future =
      video_source_factory != nullptr
          ? video_source_factory->CreateVideoSource(GetView(), asset_url)
          : video::CreateVideoSource(GetView(), asset_url);

  // Wait until the new video source is loaded and then perform some
  // final setup work.
  return video_source_future.Then(
      [this,
       asset_url](std::unique_ptr<video::VideoSource> source) -> absl::Status {
        source_->Stop().IgnoreError();

        // Now that the new video source is loaded, switch the asset.
        state_.asset = std::string(asset_url);
        source_ = std::move(source);
        return OnMaterialAndVideoLoaded();
      });
}

absl::Status VideoPlayer::Play() {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->Play();
}

absl::Status VideoPlayer::Pause() {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->Pause();
}

absl::Status VideoPlayer::Stop() {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->Stop();
}

absl::Status VideoPlayer::SetPlaybackSpeed(float speed) {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->SetPlaybackSpeed(speed);
}

absl::Status VideoPlayer::SeekTo(float seconds, SeekType seek_type) {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->SeekTo(seconds, seek_type);
}

absl::Status VideoPlayer::SetLoopCount(int loop) {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->SetLoopCount(loop);
}

absl::Status VideoPlayer::SetVolume(float volume) {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->SetVolume(volume);
}

absl::StatusOr<absl::Duration> VideoPlayer::GetDuration() const {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->GetDuration();
}

absl::StatusOr<absl::Duration> VideoPlayer::GetPlaybackTime() const {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->GetPlaybackTime();
}

absl::StatusOr<int> VideoPlayer::GetLoopCount() const {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->GetLoopCount();
}

absl::StatusOr<State> VideoPlayer::GetPlayerState() const {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->GetState();
}

absl::StatusOr<uint2> VideoPlayer::GetVideoSize() const {
  if (!source_) {
    return absl::InternalError("VideoPlayer has no video loaded.");
  }
  return source_->GetVideoSize();
}

absl::string_view VideoPlayer::GetAssetUrl() const { return state_.asset; }

void VideoPlayer::OnActiveStatusChanged(bool active) {
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
void VideoPlayer::DrawEditorUi() {
  if (helper_ == nullptr) {
    helper_ = std::make_unique<VideoPlayerWidgetHelper>(GetHandle(this));
  }
  helper_->DrawVideoPlayerUi();
}
#endif  // IMP_RUNTIME(DEV)

}  // namespace imp
