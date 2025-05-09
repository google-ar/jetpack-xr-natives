/*
 * Copyright 2024 Google LLC
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

#include "core/ncsb/component_system.h"
#include "core/view/framework/render/mesh_renderer.h"
#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_VIDEO_VIDEO_PLAYER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_VIDEO_VIDEO_PLAYER_H_

#include <memory>

#include "absl/base/attributes.h"
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
#include "core/ncsb/component_traits.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/isf_info.h"
#include "core/render/texture.h"
#include "core/video/data/video_assets.h"
#include "core/video/video_player_state.proto.imp.h"
#include "core/video/video_source.h"
#include "core/video/video_source_factory.h"
#include "core/view/framework/render/material.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/frame_time.h"
#if IMP_RUNTIME(DEV)
#include "core/video/video_player_widget_helper.h"
#endif  // IMP_RUNTIME(DEV)

namespace imp {

ABSL_DEPRECATED("Use VideoController or BasicVideoViewer instead.")
class VideoPlayer : public Component {
 public:
  using SeekType = ::imp::media::MediaSource::SeekType;
  using State = ::imp::media::MediaSource::State;
  using BufferingState = ::imp::media::MediaSource::BufferingState;

  struct PlaybackCompleteEvent : public Event {
    PlaybackCompleteEvent() = default;
    explicit PlaybackCompleteEvent(ComponentHandle<VideoPlayer> player)
        : player(player) {}

    ComponentHandle<VideoPlayer> player;
  };

  struct SeekCompleteEvent : public Event {
    SeekCompleteEvent() = default;
    explicit SeekCompleteEvent(ComponentHandle<VideoPlayer> player)
        : player(player) {}

    ComponentHandle<VideoPlayer> player;
  };

  struct BufferingEvent : public Event {
    BufferingEvent() = default;
    explicit BufferingEvent(ComponentHandle<VideoPlayer> player,
                            BufferingState state)
        : player(player), state(state) {}

    ComponentHandle<VideoPlayer> player;
    BufferingState state;
  };

  // ComponentSystem which gives VideoPlayer components access to a custom
  // VideoSourceFactory.
  // Example usage:
  // GetView()->GetComponentManager()
  //   .GetComponentSystem<VideoPlayer>()
  //   .SetCustomVideoSourceFactory(std::make_unique<FooVideoSourceFactory>());
  class System : public ComponentSystem<VideoPlayer> {
   public:
    explicit System(BaseView* view);

    void SetCustomVideoSourceFactory(
        std::unique_ptr<video::VideoSourceFactory> factory);
    video::VideoSourceFactory* GetVideoSourceFactory();

   private:
    std::unique_ptr<video::VideoSourceFactory> video_source_factory_;
  };

  Future<absl::Status> Setup();
  Future<absl::Status> Setup(
      const AssetDefinition& asset_definition, VideoPlayerState::QuadMode mode,
      const imp::AssetDefinition& video_material = video::kVideoMaterialCmat);
  Future<absl::Status> Setup(
      absl::string_view asset_url, VideoPlayerState::QuadMode mode,
      const imp::AssetDefinition& video_material = video::kVideoMaterialCmat);

  void Update(const imp::FrameTime& frame_time);

  void OnActiveStatusChanged(bool active);

  // Returns a pointer to a material containing the video texture.
  imp::MaterialPtr CreateVideoMaterial() const;

  // Returns a pointer to the video imp::Texture. This is owned by the
  // VideoPlayer and will be destroyed when the VideoPlayer is destroyed.
  imp::Texture* GetVideoTexture() const;

  // Starts playback of the video, or resumes from the point it was
  // previously paused. Returns an error if the action fails or if the player
  // has not been fully initialized. Note that this can only be called on the
  // foreground thread.
  absl::Status Play();

  // Pauses playback of the video. Returns an error if the action fails
  // or if the player has not been fully initialized. Note that this can only be
  // called on the foreground thread.
  absl::Status Pause();

  // Stops playback of the video. Returns an error if the action fails or
  // if the player has not been fully initialized. Note that this can only be
  // called on the foreground thread.
  absl::Status Stop();

  // Sets the playback speed factor of the video. 1.0 means normal speed.
  // Returns an error if the action fails or if the player has not been fully
  // initialized.
  absl::Status SetPlaybackSpeed(float speed);

  // Loads a new video from |asset_url| and after it is loaded, unload the
  // current video.
  Future<absl::Status> LoadVideoAsset(absl::string_view asset_url);

  // Seeks to the specific time in seconds in the video. Playback state
  // is preserved while seeking. Returns an error if the action fails or if the
  // player has not been fully initialized. Note that this can only be called on
  // the foreground thread.
  absl::Status SeekTo(float seconds, SeekType seek_type = SeekType::QUICK);

  // Sets the looping behaviour of the track. Set to 0 for no looping, and any
  // positive integer for the number of times it will be looped, and to -1 for
  // looping indefinitely. Returns an error if the action fails or if the player
  // has not been fully initialized. Note that this can only be called on the
  // foreground thread.
  absl::Status SetLoopCount(int loop);

  // Sets the source volume of playback for the video in range [0, 1].
  // Returns an error if the action fails or if the player has not been fully
  // initialized. Note that this can only be called on the foreground thread.
  absl::Status SetVolume(float volume);

  // Returns the total playback length, in seconds, of the video attached
  // to this player. Returns an error if the action fails or if the player has
  // not been fully initialized. Note that this can only be called on the
  // foreground thread.
  absl::StatusOr<absl::Duration> GetDuration() const;

  // Returns the current timestamp, in seconds, of the playback of the attached
  // video. Returns an error if the action fails or if the player has not
  // been fully initialized. Note that this can only be called on the foreground
  // thread.
  absl::StatusOr<absl::Duration> GetPlaybackTime() const;

  // Returns the looping behavior of the current playback. Returns an error if
  // the action fails or if the player has not been fully initialized. Note that
  // this can only be called on the foreground thread.
  absl::StatusOr<int> GetLoopCount() const;

  // Returns if the video is currently set to playing. Returns an error
  // if the action fails or if the player has not been fully initialized.
  absl::StatusOr<State> GetPlayerState() const;

  // Returns the {width, height} of the video, in points/dps. Returns {0, 0} if
  // the video is not ready to play when this is called. Returns an error if
  // the player has been fully initialized.
  absl::StatusOr<uint2> GetVideoSize() const;

  absl::string_view GetAssetUrl() const;

 private:
  std::unique_ptr<video::VideoSource> source_ = nullptr;
  Texture* texture_ptr_ = nullptr;
  State state_when_active_ = State::kReady;
  bool was_active_ = true;

  AssetPtr<MaterialAsset> video_material_;

  VideoPlayerState state_;

  absl::Status OnMaterialAndVideoLoaded();
  void SetVideoMaterial(ComponentHandle<MeshRenderer> mesh_renderer);

 public:
  using IsfInfo = IsfInfo<&VideoPlayer::state_>;

#if IMP_RUNTIME(DEV)
  void DrawEditorUi();
  std::unique_ptr<VideoPlayerWidgetHelper> helper_;
#endif  // IMP_RUNTIME(DEV)
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_VIDEO_VIDEO_PLAYER_H_
