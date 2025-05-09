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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_PLAYER_WIDGET_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_PLAYER_WIDGET_HELPER_H_

#include <string>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "core/media/media_source.h"
#include "core/ncsb/component_handle.h"

namespace imp {

// Forward declare VideoPlayer to prevent circular dependency
class VideoPlayer;

// Helper class to control and display more information on the VideoPlayer
// component
ABSL_DEPRECATED("This widget helper is deprecated along with VideoPlayer.")
class VideoPlayerWidgetHelper {
 public:
  explicit VideoPlayerWidgetHelper(ComponentHandle<VideoPlayer> component);

  void DrawVideoPlayerUi();
  void ShowPlaybackControlsRow();
  void ShowPlaybackSeekRow(absl::Duration current_duration,
                           absl::Duration total_duration);
  void ShowPlaybackLoopRow();
  void ShowPlaybackVolumeRow();
  void ShowPlaybackSpeedRow();
  void ShowPlaybackInformationRow();

  void CheckError(absl::Status status);
  void MaybeShowErrorRow();

 private:
  ComponentHandle<VideoPlayer> component_;
  float volume_;
  media::MediaSource::SeekType seek_type_;
  float playback_speed_;
  // Error message to display if applicable
  std::string error_str_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_PLAYER_WIDGET_HELPER_H_
