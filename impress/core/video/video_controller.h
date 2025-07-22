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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_CONTROLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_CONTROLLER_H_

#include <memory>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_source.h"
#include "core/media/media_type.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/component_traits.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/isf_info.h"
#include "core/render/android/android_defines.h"
#include "core/render/texture.h"
#include "core/video/video_controller_state.proto.imp.h"
#include "core/video/video_source.h"
#include "core/video/video_source_factory.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/frame_time.h"

#if IMP_RUNTIME(DEV)
#include "core/video/video_controller_widget_helper.h"
#endif  // IMP_RUNTIME(DEV)

namespace imp {

// Provides video controls and playback events,
// as well as access to video texture.
class VideoController : public Component {
 public:
  using SeekType = ::imp::media::MediaSource::SeekType;
  using State = ::imp::media::MediaSource::State;
  using BufferingState = ::imp::media::MediaSource::BufferingState;

  struct VideoLoadedEvent : public Event {
    VideoLoadedEvent() = default;
    explicit VideoLoadedEvent(ComponentHandle<VideoController> video_controller)
        : video_controller(video_controller) {}

    ComponentHandle<VideoController> video_controller;
  };

  struct PlaybackCompleteEvent : public Event {
    PlaybackCompleteEvent() = default;
    explicit PlaybackCompleteEvent(
        ComponentHandle<VideoController> video_controller)
        : video_controller(video_controller) {}

    ComponentHandle<VideoController> video_controller;
  };

  struct SeekCompleteEvent : public Event {
    SeekCompleteEvent() = default;
    explicit SeekCompleteEvent(
        ComponentHandle<VideoController> video_controller)
        : video_controller(video_controller) {}

    ComponentHandle<VideoController> video_controller;
  };

  struct BufferingEvent : public Event {
    BufferingEvent() = default;
    explicit BufferingEvent(ComponentHandle<VideoController> video_controller,
                            BufferingState state)
        : video_controller(video_controller), state(state) {}

    ComponentHandle<VideoController> video_controller;
    BufferingState state;
  };

  // ComponentSystem which gives VideoController components access to a custom
  // VideoSourceFactory.
  // Example usage:
  // GetView()->GetComponentManager()
  //   .GetComponentSystem<VideoController>()
  //   .SetCustomVideoSourceFactory(std::make_unique<FooVideoSourceFactory>());
  class System : public ComponentSystem<VideoController> {
   public:
    explicit System(BaseView* view);

    void SetCustomVideoSourceFactory(
        std::unique_ptr<video::VideoSourceFactory> factory);
    video::VideoSourceFactory* GetVideoSourceFactory();

   private:
    std::unique_ptr<video::VideoSourceFactory> video_source_factory_;
  };

  Future<absl::Status> Setup();
  Future<absl::Status> Setup(const AssetDefinition& asset_definition);
  Future<absl::Status> Setup(absl::string_view asset_url,
                             absl::string_view drm_license_url = "",
                             absl::string_view drm_scheme_uuid = "",
                             VideoControllerState::MultiviewMode
                                 multiview_mode = VideoControllerState::UNSET);

  void Cleanup();

  void Update(const imp::FrameTime& frame_time);

  void OnActiveStatusChanged(bool active);

  // Returns a pointer to the video imp::Texture. This is owned by the
  // VideoController and will be destroyed when the VideoController is
  // destroyed.
  ABSL_DEPRECATED("Use BorrowVideoTexture instead.")
  imp::Texture* GetVideoTexture() const;

  // Borrows the video texture. This is owned by the VideoController and
  // will be destroyed when the VideoController is destroyed.
  imp::BorrowedTexturePtr BorrowVideoTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Returns a pointer to the imp::Texture for each view or depth. This is owned
  // by the VideoController and will be destroyed when the VideoController is
  // destroyed.
  ABSL_DEPRECATED("Use BorrowVideoTextures instead.")
  RobinMap<SurfaceViewType, imp::Texture*> GetVideoTextures() const;

  RobinMap<SurfaceViewType, BorrowedTexturePtr> BorrowVideoTextures(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Returns the stereo mode of the video.
  MediaStereoMode GetMediaStereoMode() const;

  // Returns the color space of the video. This is queried from ExoPlayer.
  MediaColorSpace GetVideoColorSpace() const;

  // Returns the color space of the surface. This is queried from the
  // AHardwareBuffer.
  MediaColorSpace GetSourceColorSpace() const;

  // Starts playback of the video, or resumes from the point it was
  // previously paused. Returns an error if the action fails or if the
  // video controller has not been fully initialized. Note that this can
  // only be called on the foreground thread.
  absl::Status Play();

  // Pauses playback of the video. Returns an error if the action fails
  // or if the video controller has not been fully initialized. Note that
  // this can only be called on the foreground thread.
  absl::Status Pause();

  // Stops playback of the video. Returns an error if the action fails or
  // if the video controller  has not been fully initialized. Note that this
  // can only be called on the foreground thread.
  absl::Status Stop();

  // Sets the playback speed factor of the video. 1.0 means normal speed.
  // Returns an error if the action fails or if the video controller has not
  //  been fully initialized.
  absl::Status SetPlaybackSpeed(float speed);

  // Loads a new video from |asset_url| and after it is loaded, unload the
  // current video.
  Future<absl::Status> LoadVideoAsset(
      absl::string_view asset_url, absl::string_view drm_license_url = "",
      absl::string_view drm_scheme_uuid = "",
      VideoControllerState::MultiviewMode multiview_mode =
          VideoControllerState::UNSET);

  // If a video is currently being loaded, cancels the future
  // that is currently doing the load operation.
  void CancelLoad();

  // Seeks to the specific time in seconds in the video. Playback state
  // is preserved while seeking. Returns an error if the action fails or if the
  // video controller has not been fully initialized. Note that this can only
  // be called on the foreground thread.
  absl::Status SeekTo(float seconds, SeekType seek_type = SeekType::QUICK);

  // Sets the looping behaviour of the track. Set to 0 for no looping, and any
  // positive integer for the number of times it will be looped, and to -1 for
  // looping indefinitely. Returns an error if the action fails or if the
  // video controller has not been fully initialized. Note that this can only
  // be called on the foreground thread.
  absl::Status SetLoopCount(int loop);

  // Sets the source volume of playback for the video in range [0, 1].
  // Returns an error if the action fails or if the video controller has not
  // been fully initialized. Note that this can only be called on the
  // foreground thread.
  absl::Status SetVolume(float volume);

  // Returns the total playback length, in seconds, of the video attached
  // to this video controller. Returns an error if the action fails or if the
  // video controller has not been fully initialized. Note that this can only
  // be called on the foreground thread.
  absl::StatusOr<absl::Duration> GetDuration() const;

  // Returns the current timestamp, in seconds, of the playback of the attached
  // video. Returns an error if the action fails or if the video controller
  // has not been fully initialized. Note that this can only be called on the
  // foreground thread.
  absl::StatusOr<absl::Duration> GetPlaybackTime() const;

  // Returns the looping behavior of the current playback. Returns an error if
  // the action fails or if the video controller has not been fully
  // initialized. Note that this can only be called on the foreground thread.
  absl::StatusOr<int> GetLoopCount() const;

  // Returns if the video is currently set to playing. Returns an error
  // if the action fails or if the video controller has not been
  // fully initialized.
  absl::StatusOr<State> GetVideoState() const;

  // Returns the {width, height} of the video, in points/dps. Returns {0, 0} if
  // the video is not ready to play when this is called. Returns an error if
  // the video controller has been fully initialized.
  absl::StatusOr<uint2> GetVideoSize() const;

  absl::string_view GetAssetUrl() const;

 private:
  std::unique_ptr<video::VideoSource> source_ = nullptr;
  State state_when_active_ = State::kReady;
  bool was_active_ = true;

  // Handles the loading and post load work on video requests.
  WeakFuture<absl::Status> video_source_future_ = Future<absl::Status>();

  RobinMap<SurfaceViewType, BorrowedTexturePtr> textures_;

  VideoControllerState state_;

  MediaStereoMode media_stereo_mode_ = MediaStereoMode::kUnknown;

  MediaColorSpace media_color_space_;

  absl::Status OnVideoLoaded(std::unique_ptr<video::VideoSource> source);

 public:
  using IsfInfo = IsfInfo<&VideoController::state_>;

#if IMP_RUNTIME(DEV)
  void DrawEditorUi();
  std::unique_ptr<VideoControllerWidgetHelper> helper_;
#endif  // IMP_RUNTIME(DEV)
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_CONTROLLER_H_
