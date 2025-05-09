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

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#include <memory>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/media/android/android_media_source.h"
#include "core/media/media_asset.h"
#include "core/media/media_type.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/texture.h"
#include "core/video/video_color_space.h"
#include "core/video/video_source.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::video {

class AndroidVideoSource : public media::AndroidMediaSource<VideoSource> {
 public:
  static absl::StatusOr<std::unique_ptr<AndroidVideoSource>> Create(
      BaseView* view) {
    auto video_source = absl::WrapUnique(new AndroidVideoSource(view));
    MP_RETURN_IF_ERROR(video_source->CreateExternalTextureSurface());
    return video_source;
  }

  AndroidVideoSource(const AndroidVideoSource&) = delete;
  AndroidVideoSource& operator=(const AndroidVideoSource&) = delete;

  absl::StatusOr<Texture*> CreateVideoTexture() override {
    absl::MutexLock lock(&mu_);
    media_player_ptr_->SetSurface(surface_->GetSurface());
    return surface_->GetTexture();
  }

  absl::StatusOr<BorrowedTexturePtr> BorrowVideoTextureImpl(
      SmallSourceLocation loc) override {
    absl::MutexLock lock(&mu_);
    media_player_ptr_->SetSurface(surface_->GetSurface());
    return surface_->BorrowTexture(loc);
  }

  void UpdateVideoTexture(filament::Texture* texture,
                          absl::Duration frame_delta) override {}

  uint2 GetVideoSize() const override {
    absl::MutexLock lock(&mu_);
    return {media_player_ptr_->GetVideoWidth(),
            media_player_ptr_->GetVideoHeight()};
  }

  VideoColorSpace GetColorSpace() const override { return VideoColorSpace(); }

  MediaStereoMode GetStereoMode() const override {
    // AndroidMediaPlayer does not support stereo mode.
    return MediaStereoMode::kUnknown;
  }

  void SetUpMediaPlayer() override { AndroidMediaSource::SetUpMediaPlayer(); }

 private:
  AndroidVideoSource(BaseView* view)
      : media::AndroidMediaSource<VideoSource>(view, view->GetContext()) {}

  absl::Status CreateExternalTextureSurface() {
    MP_ASSIGN_OR_RETURN(surface_, AndroidExternalTextureSurface::Create(*view_));
    return absl::OkStatus();
  }

  std::unique_ptr<AndroidExternalTextureSurface> surface_;
};

// The default implementation of CreateVideoSource uses an async implementation
// of Load, where Android returns a callback on the main thread. The main
// thread is also used for testing on Linux, so the default implementation
// results in deadlock. We use the synchronous implementation instead when
// compiling for test purposes.
#if IMP_PLATFORM(ANDROID)
Future<std::unique_ptr<VideoSource>> CreateVideoSource(
    BaseView& view, absl::string_view asset_url) {
  // If the url is remote instead of embedded then load it directly via the
  // MediaSource instead of first downloading the file as a MediaAsset. This
  // allows the video to be streamed so playback can begin before download
  // completes.
  if (resources::ResourceManager::IsRemoteUrl(asset_url)) {
    absl::StatusOr<std::unique_ptr<AndroidVideoSource>> video_source =
        AndroidVideoSource::Create(&view);
    if (!video_source.ok()) {
      return Future<std::unique_ptr<VideoSource>>(video_source.status());
    }
    // Because the closure may move the video_source unique_ptr before Load()
    // gets called we should call Load() directly on the pointer itself rather.
    AndroidVideoSource* src = video_source->get();
    return src->Load(asset_url).Then(
        [source = *std::move(video_source)](absl::Status status) mutable {
          if (!status.ok()) {
            // Allows the video source to return its own error.
            return Future<std::unique_ptr<VideoSource>>(status);
          }
          return Future<std::unique_ptr<VideoSource>>(std::move(source));
        });
  }

  return view.GetAssetManager().LoadMedia(asset_url).Then(
      [&view](const AssetPtr<media::MediaAsset>& media_asset) mutable
          -> Future<std::unique_ptr<VideoSource>> {
        if (!media_asset) {
          return Future<std::unique_ptr<VideoSource>>(
              absl::InternalError("Invalid MediaAsset"));
        }

        absl::StatusOr<std::unique_ptr<AndroidVideoSource>> video_source =
            AndroidVideoSource::Create(&view);
        if (!video_source.ok()) {
          return Future<std::unique_ptr<VideoSource>>(video_source.status());
        }
        return (*video_source)
            ->Load(media_asset.Get())
            .Then([source =
                       *std::move(video_source)](absl::Status status) mutable {
              if (!status.ok()) {
                // Allows the video source to return its own error.
                return Future<std::unique_ptr<VideoSource>>(status);
              }
              return Future<std::unique_ptr<VideoSource>>(std::move(source));
            });
      });
}
#else
Future<std::unique_ptr<VideoSource>> CreateVideoSource(
    BaseView& view, absl::string_view asset_url) {
  return view.GetAssetManager().LoadMedia(asset_url).Then(
      [&view](const AssetPtr<media::MediaAsset>& media_asset) mutable
          -> absl::StatusOr<std::unique_ptr<VideoSource>> {
        if (!media_asset) {
          return absl::InternalError("Invalid MediaAsset");
        }

        MP_ASSIGN_OR_RETURN(std::unique_ptr<AndroidVideoSource> video_source,
                         AndroidVideoSource::Create(&view));
        absl::Status status = video_source->LoadSync(media_asset.Get());
        if (!status.ok()) {
          return status;
        } else {
          return video_source;
        }
      });
}
#endif

}  // namespace imp::video
