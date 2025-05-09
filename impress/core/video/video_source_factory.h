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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_SOURCE_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_SOURCE_FACTORY_H_

#include <memory>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/async/future.h"
#include "core/render/android/android_defines.h"
#include "core/video/video_source.h"
#include "core/view/base_view.h"

namespace imp::video {

// Factory class that creates VideoSources that may differ from the platform
// default.
class VideoSourceFactory {
 public:
  virtual ~VideoSourceFactory() = default;

  virtual Future<std::unique_ptr<video::VideoSource>> CreateVideoSource(
      BaseView& view, absl::string_view asset_url) = 0;

  virtual Future<std::unique_ptr<video::VideoSource>> CreateVideoSource(
      BaseView& view, absl::string_view asset_url,
      absl::string_view drm_license_url, absl::string_view drm_scheme_uuid,
      absl::Span<const SurfaceViewType> view_types) {
    if (!drm_license_url.empty() && !drm_scheme_uuid.empty()) {
      IMP_LOG(imp::ERROR) << "DRM content is not supported by this VideoSourceFactory. "
                    "Falling back to using a non-DRM VideoSource instead.";
    }
    if (view_types.length() > 1) {
      IMP_LOG(imp::ERROR)
          << "Multiview is not supported by this VideoSourceFactory. "
             "Falling back to using a non-multiview VideoSource instead.";
    }
    return CreateVideoSource(view, asset_url);
  }
};

}  // namespace imp::video

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_SOURCE_FACTORY_H_
