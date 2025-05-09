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

#include "core/audio/audio_player.h"

#include <cassert>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/audio/audio_source.h"
#include "core/common/trace.h"
#include "core/media/media_source.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/view/utils/asset.h"
#include "core/view/view_events.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace {
constexpr int kDefaultLoopCount = 0;
constexpr float kDefaultVolume = 1.0f;
}  // namespace

AudioPlayer::AudioPlayer() = default;

Future<absl::Status> AudioPlayer::Setup() {
  // This version of Setup should only be called as part of an ISF load.
  if (state_.asset.empty()) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("Asset URL is empty."));
  }
  return Setup(state_.asset);
}

Future<Status> AudioPlayer::Setup(
    const AssetDefinition& audio_asset_definition) {
  return Setup(audio_asset_definition.GetUrl());
}

Future<Status> AudioPlayer::Setup(absl::string_view audio_asset_url) {
  IMP_TRACE();
  return audio::CreateAudioSource(GetNode()->GetView(), audio_asset_url)
      .Then([handle = GetHandle(this),
             this](std::unique_ptr<AudioSource> source) mutable -> Status {
        IMP_TRACE_BLOCK("Then");
        return SetupInternal(std::move(source));
      });
}

Future<Status> AudioPlayer::Setup(absl::Cord content) {
  IMP_TRACE();

  if (!state_.asset.empty()) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "Cannot create AudioPlayer from absl::Cord while asset URL is "
        "present."));
  }

  return audio::CreateAudioSource(GetNode()->GetView(), std::move(content))
      .Then([handle = GetHandle(this),
             this](std::unique_ptr<AudioSource> source) mutable -> Status {
        IMP_TRACE_BLOCK("Then");
        return SetupInternal(std::move(source));
      });
}

Status AudioPlayer::Setup(std::unique_ptr<AudioSource> source) {
  return SetupInternal(std::move(source));
}

Status AudioPlayer::SetupInternal(std::unique_ptr<AudioSource> source) {
  ComponentHandle<AudioPlayer> handle = GetHandle(this);
  audio_source_ = std::move(source);

  GetNode()->GetView().GetDispatcher().Connect(
      [handle](const ViewPausedEvent& event) mutable {
        handle->OnComponentStateChanged(handle->component_active_, true);
      },
      this);

  GetNode()->GetView().GetDispatcher().Connect(
      [handle](const ViewResumedEvent& event) mutable {
        handle->OnComponentStateChanged(handle->component_active_, false);
      },
      this);

  auto on_complete_fn = [foreground_executor = Executor::ForegroundExecutor(),
                         handle]() {
    if (Executor::CurrentExecutor() == foreground_executor) {
      handle->GetNode()->Send(AudioPlayer::PlaybackCompleteEvent(handle));
    } else {
      foreground_executor->ScheduleInvocable([handle]() {
        // Check the handle in case the component was destroyed in the
        // time it took for the task to be executed.
        if (handle) {
          handle->GetNode()->Send(AudioPlayer::PlaybackCompleteEvent(handle));
        }
      });
    }
  };

  MP_RETURN_IF_ERROR(SetLoopCount(state_.loop_count.value_or(kDefaultLoopCount)));
  MP_RETURN_IF_ERROR(SetVolume(state_.volume.value_or(kDefaultVolume)));
  if (state_.auto_play && *state_.auto_play) {
    MP_RETURN_IF_ERROR(Play());
  }

  handle->audio_source_->SetOnPlaybackCompleteCallback(on_complete_fn);

  return absl::OkStatus();
}

Status AudioPlayer::Play() {
  if (!audio_source_) {
    return absl::InternalError(
        "AudioPlayer has no sound loaded. Call LoadSound.");
  }

  return audio_source_->Play();
}

Status AudioPlayer::Pause() {
  if (!audio_source_) {
    return absl::InternalError(
        "AudioPlayer has no sound loaded. Call LoadSound.");
  }

  return audio_source_->Pause();
}

Status AudioPlayer::Stop() {
  if (!audio_source_) {
    return absl::InternalError(
        "AudioPlayer has no sound loaded. Call LoadSound.");
  }

  return audio_source_->Stop();
}

Status AudioPlayer::SeekTo(const float seconds) {
  if (!audio_source_) {
    return absl::InternalError(
        "AudioPlayer has no sound loaded. Call LoadSound.");
  }

  return audio_source_->SeekTo(seconds, media::MediaSource::SeekType::QUICK);
}

Status AudioPlayer::SetLoopCount(const int loop) {
  if (!audio_source_) {
    return absl::InternalError(
        "AudioPlayer has to have a sound loaded to do this. Call LoadSound.");
  }

  return audio_source_->SetLoopCount(loop);
}

Status AudioPlayer::SetVolume(const float volume) {
  if (!audio_source_) {
    return absl::InternalError(
        "AudioPlayer has to have a sound loaded to do this. Call LoadSound.");
  }

  return audio_source_->SetVolume(volume);
}

StatusOr<absl::Duration> AudioPlayer::GetDuration() const {
  if (!audio_source_) {
    return absl::InternalError(
        "AudioPlayer has no sound loaded. Call LoadSound.");
  }

  return audio_source_->GetDuration();
}

StatusOr<absl::Duration> AudioPlayer::GetPlaybackTime() const {
  if (!audio_source_) {
    return absl::InternalError(
        "AudioPlayer has no sound loaded. Call LoadSound.");
  }

  return audio_source_->GetPlaybackTime();
}

StatusOr<int> AudioPlayer::GetLoopCount() const {
  if (!audio_source_) {
    return absl::InternalError(
        "AudioPlayer has to have a sound loaded to do this. Call LoadSound.");
  }

  return audio_source_->GetLoopCount();
}

StatusOr<State> AudioPlayer::GetPlayerState() const {
  if (!audio_source_) {
    return absl::InternalError(
        "AudioPlayer has no sound loaded. Call LoadSound.");
  }

  return audio_source_->GetState();
}

void AudioPlayer::OnActiveStatusChanged(bool active) {
  OnComponentStateChanged(active, component_paused_);
}

void AudioPlayer::OnComponentStateChanged(bool active, bool pause) {
  if (!audio_source_) {
    return;
  }

  // If we're pausing while the component is active, or deactivating while the
  // component is unpaused, we want to save the current playback state.
  if ((pause && !component_paused_ && component_active_) ||
      (!active && component_active_ && !component_paused_)) {
    saved_state_ = audio_source_->GetState();
  }

  // The only playback state we want to do anything about is if it was
  // originally playing
  if (saved_state_ == audio::AudioSource::State::kPlaying) {
    if ((!active || pause) &&
        audio_source_->GetState() == audio::AudioSource::State::kPlaying) {
      (void)audio_source_->Pause();
    } else if ((active && !component_active_ && !component_paused_) ||
               (!pause && component_paused_ && component_active_)) {
      (void)audio_source_->Play();
    }
  }

  component_paused_ = pause;
  component_active_ = active;
}

absl::string_view AudioPlayer::GetAssetUrl() const { return state_.asset; }

}  // namespace imp
