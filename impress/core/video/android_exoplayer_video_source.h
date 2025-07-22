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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_ANDROID_EXOPLAYER_VIDEO_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_ANDROID_EXOPLAYER_VIDEO_SOURCE_H_

#include <sys/types.h>

#include <functional>
#include <memory>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/math/vec.h"
#include "core/media/android/android_exoplayer.h"
#include "core/media/android/android_exoplayer_listener.h"
#include "core/media/media_asset.h"
#include "core/media/media_color_space.h"
#include "core/media/media_source.h"
#include "core/media/media_type.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/texture.h"
#include "core/video/video_source.h"
#include "core/view/base_view.h"

namespace imp::video {

// VideoSource that uses the ExoPlayer instead of the Android MediaPlayer.
class AndroidExoPlayerVideoSource : public VideoSource,
                                    public media::AndroidExoPlayerListener {
 public:
  static absl::StatusOr<std::unique_ptr<AndroidExoPlayerVideoSource>> Create(
      BaseView* view,
      ContentSecurityLevel security_level = ContentSecurityLevel::kNone,
      absl::Span<const SurfaceViewType> view_types =
          kAndroidExternalTextureSurfaceConfigMono);

  AndroidExoPlayerVideoSource(const AndroidExoPlayerVideoSource&) = delete;
  AndroidExoPlayerVideoSource& operator=(const AndroidExoPlayerVideoSource&) =
      delete;

  absl::StatusOr<Texture*> CreateVideoTexture() override;
  absl::StatusOr<BorrowedTexturePtr> BorrowVideoTextureImpl(
      SmallSourceLocation loc) override;
  absl::StatusOr<RobinMap<SurfaceViewType, Texture*>> CreateVideoTextures()
      override;
  absl::StatusOr<RobinMap<SurfaceViewType, BorrowedTexturePtr>>
  BorrowVideoTextures(SmallSourceLocation loc) override;

  void UpdateVideoTexture(filament::Texture* texture,
                          absl::Duration frame_delta) override;

  uint2 GetVideoSize() const override;

  MediaColorSpace GetColorSpace() const override;
  MediaStereoMode GetStereoMode() const override;
  ContentSecurityLevel GetSecurityLevel() const { return security_level_; }

  void SetUpExoPlayer();

  // VideoSource overrides
  absl::Status Play() override;
  absl::Status Pause() override;
  absl::Status Stop() override;
  absl::Status SetPlaybackSpeed(float speed) override;
  absl::Status SeekTo(float seconds, MediaSource::SeekType seek_type) override;
  absl::Status SetLoopCount(int loop) override;
  absl::Status SetVolume(float volume) override;
  absl::StatusOr<absl::Duration> GetDuration() const override;
  absl::StatusOr<absl::Duration> GetPlaybackTime() const override;
  absl::StatusOr<int> GetLoopCount() const override;
  MediaSource::State GetState() const override;
  void SetOnPlaybackCompleteCallback(std::function<void()> callback) override;
  void SetOnSeekCompleteCallback(std::function<void()> callback) override;
  void SetOnBufferingCallback(
      std::function<void(MediaSource::BufferingState)> callback) override;
  absl::Status LoadSync(const media::MediaAsset* media_asset) override;
  Future<absl::Status> Load(const media::MediaAsset* media_asset) override;

  // AndroidExoPlayerListener overrides
  void OnReady() override;
  void OnPlaybackComplete() override;
  void OnSeekComplete() override;
  void OnBuffering(int buffering_state) override;

  Future<absl::Status> Load(absl::string_view url,
                            absl::string_view drm_license_url = "",
                            absl::string_view drm_scheme_uuid = "");

 private:
  explicit AndroidExoPlayerVideoSource(BaseView* view,
                                       ContentSecurityLevel security_level);
  absl::Status CreateExternalTextureSurface(
      absl::Span<const SurfaceViewType> view_types);

  mutable absl::Mutex mu_;
  MediaSource::State state_ ABSL_GUARDED_BY(mu_);
  std::unique_ptr<media::AndroidExoPlayer> exoplayer_ ABSL_GUARDED_BY(mu_) =
      nullptr;
  BaseView& view_;

  ContentSecurityLevel security_level_;
  std::unique_ptr<AndroidExternalTextureSurface> surface_;

  // Callback when MediaPlayer finishes loading asset.
  std::function<void(absl::Status)> on_ready_callback_;
  // When playback has completed or Stop() is called,
  // on_playback_complete_callback is called to signal to the caller that the
  // player is now unused.
  std::function<void()> on_playback_complete_callback_;
  // When seek has completed, on_seek_complete_callback is called to signal to
  // the caller that the player is now ready.
  std::function<void()> on_seek_complete_callback_;
  std::function<void(MediaSource::BufferingState)> on_buffering_callback_;

  std::unique_ptr<media::ImpExoPlayerListener> exoplayer_listener_;

  void Prepare();
};

}  // namespace imp::video

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_ANDROID_EXOPLAYER_VIDEO_SOURCE_H_
