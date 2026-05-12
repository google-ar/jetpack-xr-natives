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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_SOURCE_H_

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/context.h"
#include "core/media/android/android_media_data_source.h"
#include "core/media/android/android_media_listener.h"
#include "core/media/android/android_media_player.h"
#include "core/media/media_asset.h"
#include "core/media/media_source.h"
#include "core/view/base_view.h"

namespace imp::media {

template <typename T>
class AndroidMediaSource : public T, public AndroidMediaListener {
 public:
  explicit AndroidMediaSource(BaseView* view, const Context& context);

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

  void OnPlaybackComplete() override;

  void OnSeekComplete() override;

  void OnReady() override;

  bool OnError(int what, int extra) override;

  bool OnInfo(int what, int extra) override;

  absl::Status LoadSync(const MediaAsset* media_asset) override;

  Future<absl::Status> Load(const MediaAsset* media_asset) override;

  Future<absl::Status> Load(absl::string_view url);

 protected:
  virtual void SetUpMediaPlayer();

  mutable absl::Mutex mu_;
  MediaSource::State state_ ABSL_GUARDED_BY(mu_);
  std::unique_ptr<AndroidMediaPlayer> media_player_ptr_ ABSL_GUARDED_BY(mu_) =
      nullptr;

 private:
  const Context& context_;
  // Contains the success/failure status of the loaded media.
  absl::Status result_status_;
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

  std::unique_ptr<OnCompletionListener> on_completion_listener_ptr_ = nullptr;
  std::unique_ptr<OnSeekCompleteListener> on_seek_complete_listener_ptr_ =
      nullptr;
  std::unique_ptr<OnPreparedListener> on_prepared_listener_ptr_ = nullptr;
  std::unique_ptr<OnErrorListener> on_error_listener_ptr_ = nullptr;
  std::unique_ptr<OnInfoListener> on_info_listener_ptr_ = nullptr;
  std::unique_ptr<AndroidMediaDataSource> media_data_src_ptr_ = nullptr;

  Future<absl::Status> LoadImpl(
      absl::variant<const MediaAsset*, std::string> asset, bool async = true);
  void PrepareAsync();
  void Prepare();
};

template <typename T>
AndroidMediaSource<T>::AndroidMediaSource(BaseView* view,
                                          const Context& context)
    : T(view), context_(context) {}

template <typename T>
absl::Status AndroidMediaSource<T>::Play() {
  absl::MutexLock lock(mu_);
  bool result = media_player_ptr_->Start();
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to start playback");
  }
  state_ = MediaSource::State::kPlaying;
  return absl::Status();
}

template <typename T>
absl::Status AndroidMediaSource<T>::Pause() {
  absl::MutexLock lock(mu_);
  bool result = media_player_ptr_->Pause();
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to pause playback");
  }
  state_ = MediaSource::State::kReady;
  return absl::Status();
}

template <typename T>
absl::Status AndroidMediaSource<T>::Stop() {
  absl::ReleasableMutexLock lock(mu_);
  bool result = media_player_ptr_->Stop();
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to stop playback");
  }

  state_ = MediaSource::State::kStopped;
  lock.Release();
  OnPlaybackComplete();
  return absl::Status();
}

template <typename T>
absl::Status AndroidMediaSource<T>::SetPlaybackSpeed(const float speed) {
  absl::MutexLock lock(mu_);
  bool result = media_player_ptr_->SetPlaybackSpeed(speed);
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError(
        absl::StrFormat("Unable to set playback speed to %f", speed));
  }
  return absl::Status();
}

template <typename T>
absl::Status AndroidMediaSource<T>::SeekTo(
    const float seconds, const MediaSource::SeekType seek_type) {
  absl::MutexLock lock(mu_);
  bool result = media_player_ptr_->SeekTo(seconds * 1000, seek_type);
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to seek media");
  }
  return absl::Status();
}

template <typename T>
absl::Status AndroidMediaSource<T>::SetLoopCount(const int loop) {
  if (loop > 0) {
    return absl::InternalError(
        "Android Media Player only supports infinite looping or one-time "
        "playback");
  }

  absl::MutexLock lock(mu_);
  bool result = media_player_ptr_->SetLooping(loop < 0);
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to set loop state");
  }
  return absl::Status();
}

template <typename T>
absl::Status AndroidMediaSource<T>::SetVolume(const float volume) {
  absl::MutexLock lock(mu_);
  bool result = media_player_ptr_->SetVolume(volume, volume);
  if (!result) {
    // TODO: Retrieve Java error and pipe it here
    return absl::InternalError("Unable to set volume");
  }
  return absl::Status();
}

template <typename T>
absl::StatusOr<absl::Duration> AndroidMediaSource<T>::GetDuration() const {
  absl::MutexLock lock(mu_);
  int duration_msec = media_player_ptr_->GetDuration();
  // Returned value is negative if there is an error/exception
  if (duration_msec < 0) {
    return absl::InternalError("Unable to retrieve duration");
  }
  return absl::Milliseconds(duration_msec);
}

template <typename T>
absl::StatusOr<absl::Duration> AndroidMediaSource<T>::GetPlaybackTime() const {
  absl::MutexLock lock(mu_);
  int position_msec = media_player_ptr_->GetCurrentPosition();
  // Returned value is negative if there is an error/exception
  if (position_msec < 0) {
    return absl::InternalError("Unable to retrieve playback time");
  }
  return absl::Milliseconds(position_msec);
}

template <typename T>
absl::StatusOr<int> AndroidMediaSource<T>::GetLoopCount() const {
  absl::MutexLock lock(mu_);
  if (media_player_ptr_->IsLooping()) {
    return MediaSource::kLoopIndefinite;
  }
  return MediaSource::kLoopSingleShot;
}

template <typename T>
MediaSource::State AndroidMediaSource<T>::GetState() const {
  absl::MutexLock lock(mu_);
  return state_;
}

template <typename T>
void AndroidMediaSource<T>::SetOnPlaybackCompleteCallback(
    std::function<void()> callback) {
  on_playback_complete_callback_ = callback;
}

template <typename T>
void AndroidMediaSource<T>::SetOnSeekCompleteCallback(
    std::function<void()> callback) {
  on_seek_complete_callback_ = callback;
}

template <typename T>
void AndroidMediaSource<T>::SetOnBufferingCallback(
    std::function<void(MediaSource::BufferingState)> callback) {
  on_buffering_callback_ = callback;
}

template <typename T>
void AndroidMediaSource<T>::OnPlaybackComplete() {
  {
    absl::MutexLock lock(mu_);
    state_ = MediaSource::State::kReady;
  }
  if (on_playback_complete_callback_) {
    on_playback_complete_callback_();
  }
}

template <typename T>
void AndroidMediaSource<T>::OnSeekComplete() {
  if (on_seek_complete_callback_) {
    on_seek_complete_callback_();
  }
}

template <typename T>
void AndroidMediaSource<T>::OnReady() {
  {
    absl::MutexLock lock(mu_);
    state_ = MediaSource::State::kReady;
  }
  if (on_ready_callback_) {
    on_ready_callback_(result_status_);
  }
}

// Builds an error string from a media error.
std::string MediaErrorToString(AndroidMediaPlayer* player, int what, int extra);

template <typename T>
bool AndroidMediaSource<T>::OnError(int what, int extra) {
  {
    absl::MutexLock lock(mu_);
    std::string error =
        MediaErrorToString(media_player_ptr_.get(), what, extra);
    result_status_ = absl::InternalError(error);
  }

  // Completes the waiting future and informs of the error result.
  if (on_ready_callback_) {
    on_ready_callback_(result_status_);
  }
  // Returns false indicating 'error unhandled'
  // see android.media.MediaPlayer.OnErrorListener.
  return false;
}

template <typename T>
bool AndroidMediaSource<T>::OnInfo(int what, int extra) {
  std::optional<MediaSource::BufferingState> buffering = std::nullopt;
  {
    absl::MutexLock lock(mu_);
    if (what == media_player_ptr_->MediaInfoBufferingStart()) {
      buffering = MediaSource::BufferingState::kBuffering;
    } else if (what == media_player_ptr_->MediaInfoBufferingEnd()) {
      buffering = MediaSource::BufferingState::kBufferingDone;
    }
  }
  if (buffering.has_value() && on_buffering_callback_) {
    on_buffering_callback_(*buffering);
  }
  // Returns false indicating 'info unhandled'
  // see android.media.MediaPlayer.OnInfoListener.
  return false;
}

template <typename T>
absl::Status AndroidMediaSource<T>::LoadSync(const MediaAsset* media_asset) {
  return LoadImpl(media_asset, false).Get();
}

template <typename T>
Future<absl::Status> AndroidMediaSource<T>::Load(
    const MediaAsset* media_asset) {
  return LoadImpl(media_asset);
}

template <typename T>
Future<absl::Status> AndroidMediaSource<T>::Load(absl::string_view url) {
  return LoadImpl(std::string(url));
}

template <typename T>
void AndroidMediaSource<T>::SetUpMediaPlayer() {
  absl::MutexLock lock(mu_);
  media_player_ptr_->SetOnCompletionListener(on_completion_listener_ptr_.get());
  media_player_ptr_->SetOnSeekCompleteListener(
      on_seek_complete_listener_ptr_.get());
  media_player_ptr_->SetOnPreparedListener(on_prepared_listener_ptr_.get());
  media_player_ptr_->SetOnErrorListener(on_error_listener_ptr_.get());
  media_player_ptr_->SetOnInfoListener(on_info_listener_ptr_.get());
}

template <typename T>
Future<absl::Status> AndroidMediaSource<T>::LoadImpl(
    absl::variant<const MediaAsset*, std::string> asset, bool async) {
  // Resets result status.
  result_status_ = absl::OkStatus();

  on_completion_listener_ptr_ =
      std::make_unique<OnCompletionListener>(context_, this);
  on_seek_complete_listener_ptr_ =
      std::make_unique<OnSeekCompleteListener>(context_, this);
  on_prepared_listener_ptr_ =
      std::make_unique<OnPreparedListener>(context_, this);
  on_error_listener_ptr_ = std::make_unique<OnErrorListener>(context_, this);
  on_info_listener_ptr_ = std::make_unique<OnInfoListener>(context_, this);
  if (absl::holds_alternative<const MediaAsset*>(asset)) {
    media_data_src_ptr_ = std::make_unique<AndroidMediaDataSource>(
        context_, absl::get<const MediaAsset*>(asset));
  }

  Future<absl::Status> on_ready_result;
  on_ready_callback_ = [weak_on_ready_result = make_weak(on_ready_result)](
                           absl::Status status) mutable {
    absl::optional<Future<absl::Status>> on_ready_result =
        weak_on_ready_result.Lock();
    if (on_ready_result.has_value()) {
      on_ready_result->Return(status);
    }
  };

  {
    absl::MutexLock lock(mu_);
    if (!media_player_ptr_) {
      media_player_ptr_ = std::make_unique<AndroidMediaPlayer>(context_);
    }
  }

  if (async) {
    on_ready_result.DependsOn(
        Future<absl::Status>::Schedule(
            // On a background thread, call SetDataSource.
            [this, asset]() {
              {
                absl::MutexLock lock(mu_);
                if (absl::holds_alternative<const MediaAsset*>(asset)) {
                  media_player_ptr_->SetDataSource(media_data_src_ptr_.get());
                } else {
                  media_player_ptr_->SetDataSource(
                      absl::get<std::string>(asset));
                }
              }
              return absl::OkStatus();
            },
            Executor::Type::kBackground)
            .Then([this]() {
              // After setting the data source, finalize
              // things on the main thread.
              SetUpMediaPlayer();
              PrepareAsync();
              return absl::OkStatus();
            }));
  } else {
    {
      absl::MutexLock lock(mu_);
      if (absl::holds_alternative<const MediaAsset*>(asset)) {
        media_player_ptr_->SetDataSource(media_data_src_ptr_.get());
      } else {
        media_player_ptr_->SetDataSource(absl::get<std::string>(asset));
      }
    }
    SetUpMediaPlayer();
    Prepare();
    OnReady();
  }

  return on_ready_result;
}

template <typename T>
void AndroidMediaSource<T>::PrepareAsync() {
  absl::MutexLock lock(mu_);
  media_player_ptr_->PrepareAsync();
}

template <typename T>
void AndroidMediaSource<T>::Prepare() {
  absl::MutexLock lock(mu_);
  media_player_ptr_->Prepare();
}

}  // namespace imp::media

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_SOURCE_H_
