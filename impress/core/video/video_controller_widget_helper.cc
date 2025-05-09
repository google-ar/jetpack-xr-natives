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

#include "core/video/video_controller_widget_helper.h"

#include <inttypes.h>

#include "absl/status/statusor.h"
#include "dear_imgui/imgui.h"
#include "core/ncsb/component_handle.h"
#include "core/video/video_controller.h"

namespace imp {

// Used as a multiplier to the animation options buttons item rect max. This
// helps determine if the next button can be on the same line (not cut off by
// the window).
constexpr float kItemRectWrapMultiplier = 1.5f;

VideoControllerWidgetHelper::VideoControllerWidgetHelper(
    ComponentHandle<VideoController> component)
    : component_(component) {
  volume_ = 1.0f;
  seek_type_ = media::MediaSource::SeekType::QUICK;
  playback_speed_ = 1.0f;
  error_str_ = "";
}

void VideoControllerWidgetHelper::DrawVideoControllerUi() {
  MaybeShowErrorRow();
  ShowPlaybackControlsRow();
  ShowPlaybackSpeedRow();

  absl::StatusOr<absl::Duration> current_duration =
      component_->GetPlaybackTime();
  absl::StatusOr<absl::Duration> total_duration = component_->GetDuration();
  if (current_duration.ok() && total_duration.ok()) {
    ShowPlaybackSeekRow(*current_duration, *total_duration);
  } else if (!current_duration.ok() && !total_duration.ok()) {
    error_str_ = "Could not get current nor total playback duration";
  } else if (!current_duration.ok()) {
    error_str_ = "Could not get current playback duration";
  } else {
    error_str_ = "Could not get total playback duration";
  }
  ShowPlaybackLoopRow();
  ShowPlaybackVolumeRow();
  ShowPlaybackInformationRow();
}

void VideoControllerWidgetHelper::ShowPlaybackControlsRow() {
  float max_window_size_x =
      ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
  ImGuiStyle& style = ImGui::GetStyle();

  if (ImGui::Button("Play")) {
    CheckError(component_->Play());
  }

  float next_button_position =
      ImGui::GetItemRectMax().x * kItemRectWrapMultiplier + style.ItemSpacing.x;
  if (next_button_position < max_window_size_x) ImGui::SameLine();
  if (ImGui::Button("Pause")) {
    CheckError(component_->Pause());
  }

  next_button_position =
      ImGui::GetItemRectMax().x * kItemRectWrapMultiplier + style.ItemSpacing.x;
  if (next_button_position < max_window_size_x) ImGui::SameLine();
  if (ImGui::Button("Stop")) {
    CheckError(component_->Stop());
  }
}

void VideoControllerWidgetHelper::ShowPlaybackSeekRow(
    absl::Duration current_duration, absl::Duration total_duration) {
  RobinMap<media::MediaSource::SeekType, std::string> seek_map;
  seek_map[media::MediaSource::SeekType::QUICK] = "Quick";
  seek_map[media::MediaSource::SeekType::PRECISE] = "Precise";
  if (ImGui::BeginCombo("Seek Type", seek_map[seek_type_].c_str())) {
    if (ImGui::Selectable(seek_map[media::MediaSource::SeekType::QUICK].c_str(),
                          seek_type_ == media::MediaSource::SeekType::QUICK)) {
      seek_type_ = media::MediaSource::SeekType::QUICK;
    }
    if (ImGui::Selectable(
            seek_map[media::MediaSource::SeekType::PRECISE].c_str(),
            seek_type_ == media::MediaSource::SeekType::PRECISE)) {
      seek_type_ = media::MediaSource::SeekType::PRECISE;
    }
    ImGui::EndCombo();
  }

  int64_t min_sec = 0;
  int64_t max_sec = (total_duration) / absl::Seconds(1);
  int64_t curr_elapsed_sec = (current_duration / absl::Seconds(1));
  if (ImGui::SliderScalar("Playback Seeker", ImGuiDataType_::ImGuiDataType_S64,
                          &curr_elapsed_sec, &min_sec, &max_sec, "", 0)) {
    CheckError(component_->SeekTo(curr_elapsed_sec, seek_type_));
  }

  int64_t curr_elapsed_min_disp = current_duration / absl::Minutes(1);
  int64_t curr_elapsed_sec_disp = curr_elapsed_sec % 60;
  ImGui::Text("Playback: %" PRId64 ":%02" PRId64 "/ %" PRId64 ":%02" PRId64,
              curr_elapsed_min_disp, curr_elapsed_sec_disp,
              (total_duration) / absl::Minutes(1),
              ((total_duration) / absl::Seconds(1)) % 60);
}

void VideoControllerWidgetHelper::ShowPlaybackLoopRow() {
  int loop_count = component_->GetLoopCount().value_or(0);
  if (ImGui::BeginCombo("Loop Count",
                        loop_count == -1
                            ? "Repeat"
                            : absl::StrFormat("%d", loop_count).c_str())) {
    if (ImGui::Selectable("Repeat", loop_count == -1)) {
      loop_count = -1;
    }
    for (int i = 0; i <= 5; i++) {
      if (ImGui::Selectable(absl::StrFormat("%d", i).c_str(),
                            loop_count == i)) {
        loop_count = i;
      }
    }
    ImGui::EndCombo();
    CheckError(component_->SetLoopCount(loop_count));
  }
}

void VideoControllerWidgetHelper::ShowPlaybackVolumeRow() {
  if (ImGui::SliderFloat("Volume", &volume_, 0.0f, 1.0f, "%.1f")) {
    CheckError(component_->SetVolume(volume_));
  }
}

void VideoControllerWidgetHelper::ShowPlaybackSpeedRow() {
  ImGui::Text("Playback Speed");
  if (ImGui::BeginCombo("Playback Speed",
                        absl::StrFormat("%.2fx", playback_speed_).c_str())) {
    for (int i = 1; i <= 8; ++i) {
      float speed = 0.25f * i;
      if (ImGui::Selectable(absl::StrFormat("%.2fx", speed).c_str(),
                            playback_speed_ == speed)) {
        playback_speed_ = speed;
      }
    }
    ImGui::EndCombo();
    CheckError(component_->SetPlaybackSpeed(playback_speed_));
  }
}

void VideoControllerWidgetHelper::ShowPlaybackInformationRow() {
  absl::StatusOr<VideoController::State> controller_state =
      component_->GetVideoState();
  absl::string_view controller_state_str = "Unknown";
  if (controller_state.ok()) {
    switch (*controller_state) {
      case media::MediaSource::State::kReady:
        controller_state_str = "Ready";
        break;
      case media::MediaSource::State::kPlaying:
        controller_state_str = "Playing";
        break;
      case media::MediaSource::State::kStopped:
        controller_state_str = "Stopped";
        break;
    }
  } else {
    error_str_ = "Failed to get current controller state";
  }
  ImGui::Text(
      absl::StrFormat("Controller State: %s", controller_state_str.data())
          .c_str());

  absl::StatusOr<uint2> video_size = component_->GetVideoSize();
  if (video_size.ok()) {
    ImGui::Text("Video Size: %u x %u", video_size->x, video_size->y);
  } else {
    error_str_ = "Failed to get video size";
  }
}

void VideoControllerWidgetHelper::CheckError(absl::Status status) {
  if (!status.ok()) {
    error_str_ = status.ToString();
  }
}

void VideoControllerWidgetHelper::MaybeShowErrorRow() {
  if (!error_str_.empty()) {
    ImGui::Text("Last error:\n%s", error_str_.c_str());
  }
}

}  // namespace imp
