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

#include <jni.h>

#include <memory>
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/match.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/audio/audio_source.h"
#include "core/common/trace.h"
#include "core/media/android/android_media_source.h"
#include "core/media/media_asset.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace audio {

// Prefix for URLs that reference an asset on the Android filesystem.
constexpr absl::string_view kFilePrefix = "file://";

class AndroidAudioSource : public media::AndroidMediaSource<AudioSource> {
 public:
  explicit AndroidAudioSource(BaseView* view)
      : media::AndroidMediaSource<AudioSource>(view, view->GetContext()) {}

  AndroidAudioSource(const AndroidAudioSource&) = delete;
  AndroidAudioSource& operator=(const AndroidAudioSource&) = delete;
};

namespace {
Future<std::unique_ptr<AudioSource>> CreateAndroidAudioSource(
    BaseView& view, const AssetPtr<media::MediaAsset>& media_asset) {
  auto android_media_source = std::make_unique<AndroidAudioSource>(&view);
  AndroidAudioSource* android_media_source_ptr = android_media_source.get();
  return android_media_source_ptr->Load(media_asset.Get())
      .Then([android_media_source =
                 std::move(android_media_source)](absl::Status status) mutable
            -> absl::StatusOr<std::unique_ptr<AudioSource>> {
        MP_RETURN_IF_ERROR(status);
        return std::move(android_media_source);
      });
}

Future<std::unique_ptr<AudioSource>> CreateAndroidAudioSource(
    BaseView& view, absl::string_view asset_url) {
  auto android_media_source = std::make_unique<AndroidAudioSource>(&view);
  AndroidAudioSource* android_media_source_ptr = android_media_source.get();
  return android_media_source_ptr->Load(asset_url).Then(
      [android_media_source =
           std::move(android_media_source)](absl::Status status) mutable
      -> absl::StatusOr<std::unique_ptr<AudioSource>> {
        MP_RETURN_IF_ERROR(status);
        return std::move(android_media_source);
      });
}
}  // namespace

Future<std::unique_ptr<AudioSource>> CreateAudioSource(
    BaseView& view, absl::string_view asset_url) {
  IMP_TRACE();
  if (asset_url.empty()) {
    return Future<std::unique_ptr<AudioSource>>(
        absl::InvalidArgumentError("Empty asset_url cannot be loaded!"));
  }

  // Load the audio asset directly if its a local file.
  if (absl::StartsWith(asset_url, kFilePrefix)) {
    return view.GetAssetManager().LoadMedia(asset_url).Then(
        [&view](const AssetPtr<media::MediaAsset>& media_asset) mutable {
          IMP_TRACE_BLOCK("Then");
          return CreateAndroidAudioSource(view, media_asset);
        });
  }

  // Stream the audio asset directly using the Android OS's MediaPlayer
  // if its a remote file.
  return CreateAndroidAudioSource(view, asset_url);
}

Future<std::unique_ptr<AudioSource>> CreateAudioSource(BaseView& view,
                                                       absl::Cord content) {
  IMP_TRACE();

  std::optional<absl::string_view> flattened_string = content.TryFlat();
  Future<AssetPtr<MediaAsset>> media_asset;
  if (flattened_string) {
    media_asset =
        view.GetAssetManager().LoadAsset<MediaAsset>(content, /*asset_url=*/"");
  } else {
    media_asset = Future<absl::Cord>::Schedule(
                      [contents = std::move(content)]() mutable {
                        contents.Flatten();
                        return std::move(contents);
                      },
                      Executor::Type::kBackground)
                      .Then([&view](absl::Cord flattened_cord) mutable {
                        return view.GetAssetManager().LoadAsset<MediaAsset>(
                            std::move(flattened_cord), /*asset_url=*/"");
                      });
  }

  return media_asset.Then(
      [&view](const AssetPtr<media::MediaAsset>& media_asset) mutable {
        IMP_TRACE_BLOCK("Then");
        return CreateAndroidAudioSource(view, media_asset);
      });
}

}  // namespace audio
}  // namespace imp
