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

#include "core/video/exoplayer_video_source_factory.h"

#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/async/future.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/resources/resource_manager.h"
#include "core/video/android_exoplayer_video_source.h"
#include "core/video/video_source.h"
#include "core/view/base_view.h"

namespace imp::video {

Future<std::unique_ptr<video::VideoSource>>
ExoPlayerVideoSourceFactory::CreateVideoSource(BaseView& view,
                                               absl::string_view asset_url) {
  return CreateVideoSource(view, asset_url, "", "",
                           kAndroidExternalTextureSurfaceConfigMono);
}

Future<std::unique_ptr<video::VideoSource>>
ExoPlayerVideoSourceFactory::CreateVideoSource(
    BaseView& view, absl::string_view asset_url,
    absl::string_view drm_license_url, absl::string_view drm_scheme_uuid,
    absl::Span<const SurfaceViewType> view_types) {
  ContentSecurityLevel security_level = ContentSecurityLevel::kNone;
  if (!drm_license_url.empty() && !drm_scheme_uuid.empty()) {
    security_level = ContentSecurityLevel::kProtected;
  }
  // If the url is remote instead of embedded then load it directly via
  // the MediaSource instead of first downloading the file as a
  // MediaAsset. This allows the video to be streamed so playback can
  // begin before download completes.
  if (resources::ResourceManager::IsRemoteUrl(asset_url)) {
    absl::StatusOr<std::unique_ptr<AndroidExoPlayerVideoSource>> video_source =
        AndroidExoPlayerVideoSource::Create(&view, security_level, view_types);
    if (!video_source.ok()) {
      return Future<std::unique_ptr<VideoSource>>(video_source.status());
    }
    // Because the closure may move the video_source unique_ptr before Load()
    // gets called we should call Load() directly on the pointer itself rather.
    AndroidExoPlayerVideoSource* src = video_source->get();
    return src->Load(asset_url, drm_license_url, drm_scheme_uuid)
        .Then([source = *std::move(video_source)](absl::Status status) mutable {
          if (!status.ok()) {
            // Allows the video source to return its own error.
            return Future<std::unique_ptr<VideoSource>>(status);
          }
          return Future<std::unique_ptr<VideoSource>>(std::move(source));
        });
  }

  return Future<std::unique_ptr<VideoSource>>(absl::UnimplementedError(
      "Creating ExoPlayerVideoSource from embedded url is not supported."));
}

}  // namespace imp::video
