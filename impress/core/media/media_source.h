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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_SOURCE_H_

#include <functional>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/async/future.h"
#include "core/media/media_asset.h"

namespace imp::media {

// A thin wrapper around a platform-specific media player implementation, and
// manages the lifecycle of the media player.
// Each MediaSource plays only one audio/video, which is loaded into it via an
// MediaAsset object.
class MediaSource {
 public:
  static constexpr int kLoopIndefinite = -1;
  static constexpr int kLoopSingleShot = 0;

  enum class State {
    // Player is ready to play and no playback is currently ongoing.
    kReady,
    // Playback is currently ongoing.
    kPlaying,
    // Player has been stopped and media object has been unloaded. Note that
    // in this state, Load has to be called before the player can be used
    // again.
    kStopped
  };

  enum class BufferingState {
    // Video is buffering and playback cannot proceed.
    kBuffering,
    // Video buffering is done and playback can resume.
    kBufferingDone
  };

  MediaSource() = default;
  MediaSource(const MediaSource&) = delete;
  MediaSource& operator=(const MediaSource&) = delete;

  virtual ~MediaSource() = default;

  // Loads the media asset for playback.
  virtual Future<absl::Status> Load(const MediaAsset* media_asset) = 0;
  virtual absl::Status LoadSync(const MediaAsset* media_asset) = 0;

  // Starts playback of the media stream, or resumes from the point it was
  // previously paused. Returns an error if the platform-specific media player
  // is not yet ready.
  virtual absl::Status Play() = 0;

  // Pauses playback of the media stream. Returns an error if the
  // platform-specific media player is not yet ready.
  virtual absl::Status Pause() = 0;

  // Stops playback of the media stream, and unloads all associated media
  // objects. Returns an error if the platform-specific media player is not yet
  // ready.
  virtual absl::Status Stop() = 0;

  // Sets the playback speed factor of the video. 1.0 means normal speed.
  // Returns an error if the platform-specific media player is not yet
  // ready.
  virtual absl::Status SetPlaybackSpeed(float speed) = 0;

  enum class SeekType {
    // Quickly seek to a place close to the given timestamp. The implementation
    // is free to sync a bit before or after the given timestamp for efficiency.
    QUICK,
    // Seek to the exact provided time. This may be slower than QUICK.
    PRECISE,
  };

  // Seeks to the specific time in seconds in the media stream.
  // Playback state is preserved while seeking. Returns an error if the
  // platform-specific media player is not yet ready.
  virtual absl::Status SeekTo(float seconds, SeekType seek_type) = 0;

  // Sets the looping behaviour of the track. Set to 0 for no looping, and any
  // positive integer for the number of times it will be looped, and to -1 for
  // looping indefinitely. Returns an error if the platform-specific media
  // player is not yet ready.
  virtual absl::Status SetLoopCount(int loop) = 0;

  // Sets the source volume of playback for the media stream in range [0, 1].
  // Returns an error if the platform-specific media player is not yet ready.
  virtual absl::Status SetVolume(float volume) = 0;

  // Returns the total playback length, in seconds, of the media object attached
  // to this player. Returns -1 if the platform-specific media player is not yet
  // ready.
  virtual absl::StatusOr<absl::Duration> GetDuration() const = 0;

  // Returns the current timestamp, in seconds, of the playback of the attached
  // media object. Returns -1 if the platform-specific media player is not yet
  // ready.
  virtual absl::StatusOr<absl::Duration> GetPlaybackTime() const = 0;

  // Returns the looping behavior of the current playback. Returns 0 if the
  // platform-specific media player is not yet ready.
  virtual absl::StatusOr<int> GetLoopCount() const = 0;

  // Returns the current state of the player.
  virtual MediaSource::State GetState() const = 0;

  // Sets a callback to signal to the caller that playback has completed or
  // has been stopped.
  virtual void SetOnPlaybackCompleteCallback(
      std::function<void()> callback) = 0;

  // Sets a callback to signal to the caller that seek has completed.
  virtual void SetOnSeekCompleteCallback(std::function<void()> callback) = 0;

  // Sets a callback to signal to the caller when the video is buffering (or
  // done buffering).
  virtual void SetOnBufferingCallback(
      std::function<void(BufferingState)> callback) = 0;
};
}  // namespace imp::media

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_SOURCE_H_
