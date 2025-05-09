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

#ifndef THIRD_PARTY_IMPRESS_CORE_AUDIO_AUDIO_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_AUDIO_AUDIO_SOURCE_H_

#include <memory>

#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/async/future.h"
#include "core/media/media_source.h"
#include "core/view/base_view.h"

namespace imp {
namespace audio {

class AudioSource : public media::MediaSource {
 public:
  template <typename... Args>
  explicit AudioSource(BaseView* view, Args... args)
      : media::MediaSource(std::forward<Args>(args)...) {}
};

Future<std::unique_ptr<AudioSource>> CreateAudioSource(
    BaseView& view, absl::string_view asset_url);

Future<std::unique_ptr<AudioSource>> CreateAudioSource(BaseView& view,
                                                       absl::Cord content);

namespace internal {
constexpr absl::Duration kDebugPlaybackDuration = absl::Seconds(1);
}  // namespace internal

}  // namespace audio
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AUDIO_AUDIO_SOURCE_H_
