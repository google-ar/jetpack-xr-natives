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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_BASE_VIDEO_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_BASE_VIDEO_SOURCE_H_

#include <memory>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_source.h"
#include "core/media/media_type.h"
#include "core/render/android/android_defines.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"

namespace imp::video {

class VideoSource : public media::MediaSource {
 public:
  template <typename... Args>
  explicit VideoSource(BaseView* view, Args... args)
      : media::MediaSource(std::forward<Args>(args)...),
        view_(view),
        engine_(view->GetHost()->GetEngine()) {}

  // Returns a new texture for the video.
  ABSL_DEPRECATED("Use BorrowVideoTexture instead.")
  virtual absl::StatusOr<Texture*> CreateVideoTexture() = 0;

  virtual absl::StatusOr<BorrowedTexturePtr> BorrowVideoTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current()) {
    return BorrowVideoTextureImpl(loc);
  }

  // Returns a new texture for each view of the video. This method is overridden
  // by AndroidExoPlayerVideoSource to support MVHEVC playback.
  ABSL_DEPRECATED("Use BorrowVideoTextures instead.")
  virtual absl::StatusOr<RobinMap<SurfaceViewType, Texture*>>
  CreateVideoTextures() {
    return absl::UnimplementedError("CreateVideoTextures is not implemented.");
  }

  virtual absl::StatusOr<RobinMap<SurfaceViewType, BorrowedTexturePtr>>
  BorrowVideoTextures(
      SmallSourceLocation loc = SmallSourceLocation::Current()) {
    return absl::UnimplementedError("BorrowVideoTextures is not implemented.");
  }

  // Updates the video texture for video playback. This should be called on
  // every frame update to display the next video frame. This method is not
  // required for Android, as CreateVideoTexture() returns a texture with an
  // external stream. Multiview playback is exclusive to Android, so
  // CreateVideoTextures() does not need a corresponding UpdateVideoTextures()
  // method.
  virtual void UpdateVideoTexture(filament::Texture* texture,
                                  absl::Duration frame_delta) = 0;

  // Returns the {width, height} of the video in points/dps.
  virtual uint2 GetVideoSize() const = 0;

  // Returns the color space information of the video, including the standard,
  // transfer function, range, luma and chroma bitdepths, and the maximum
  // content light level.
  virtual MediaColorSpace GetColorSpace() const = 0;

  virtual MediaStereoMode GetStereoMode() const = 0;

 protected:
  virtual absl::StatusOr<BorrowedTexturePtr> BorrowVideoTextureImpl(
      SmallSourceLocation loc) = 0;

  BaseView* view_;
  filament::Engine* engine_;
};

Future<std::unique_ptr<VideoSource>> CreateVideoSource(
    BaseView& base_view, absl::string_view asset_url);

}  // namespace imp::video

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_BASE_VIDEO_SOURCE_H_
