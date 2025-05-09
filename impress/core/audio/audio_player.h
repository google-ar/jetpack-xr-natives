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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_AUDIO_AUDIO_PLAYER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_AUDIO_AUDIO_PLAYER_H_

#include <memory>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/async/future.h"
#include "core/audio/audio_player_state.proto.imp.h"
#include "core/audio/audio_source.h"
#include "core/media/media_source.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/isf_info.h"
#include "core/view/async/future.h"
#include "core/view/utils/asset.h"

namespace imp {
using ::absl::Status;
using ::absl::StatusOr;
using ::imp::audio::AudioSource;
using State = ::imp::media::MediaSource::State;

// AudioPlayer is a component used to perform playback of loaded audio assets.
// Each AudioPlayer is meant to play just one audio definition, and can perform
// common playback functions once the asset is loaded.
//
// To use AudioPlayer, it must first be loaded either by providing a valid
// AssetDefinition when adding the AudioPlayer to a node or when calling
// LoadSound on the AudioPlayer, returning a Future that, when ready, allows
// calling of any of the playback functions.
//
// When playback has completed, a PlaybackCompleteEvent will be emitted to the
// Node that the AudioPlayer is attached to, and listening to the event will
// be a matter of connecting to the node and listening for the specific event.
//
// Example:
// AssetDefinition audio_definition("https://www.google.com/audio.mp3");
// node_->AddComponent<AudioPlayer>(audio_definition)
//      .Then([](const StatusOr<ComponentHandle<AudioPlayer>>& statusor) {
//        if (statusor.ok()) {
//          auto player = *statusor;
//          if (player->Play().ok()) {
//          player->GetNode()->Connect(
//              [](const PlaybackCompleteEvent& event) {
//                IMP_LOG(imp::INFO) << "Playback complete!";
//              });
//          }
//        }
//      })
//      .KeptBy(this);
// TODO Add tests
class AudioPlayer : public Component {
 public:
  struct PlaybackCompleteEvent : public Event {
    PlaybackCompleteEvent() {}
    explicit PlaybackCompleteEvent(ComponentHandle<AudioPlayer> player)
        : player(player) {}

    ComponentHandle<AudioPlayer> player;
  };

  static constexpr int kLoopSingleShot = 0;
  static constexpr int kLoopIndefinite = -1;

  AudioPlayer();

  Future<Status> Setup();
  Future<Status> Setup(const AssetDefinition& audio_asset_definition);
  Future<Status> Setup(absl::string_view audio_asset_url);
  Future<Status> Setup(absl::Cord content);
  Status Setup(std::unique_ptr<AudioSource> audio_source);

  // Starts playback of the audio stream, or resumes from the point it was
  // previously paused. Returns an error if the action fails or if the player
  // has not been fully initialized. Note that this can only be called on the
  // foreground thread.
  Status Play();

  // Pauses playback of the audio stream. Returns an error if the action fails
  // or if the player has not been fully initialized. Note that this can only be
  // called on the foreground thread.
  Status Pause();

  // Stops playback of the audio stream. Returns an error if the action fails or
  // if the player has not been fully initialized. Note that this can only be
  // called on the foreground thread.
  Status Stop();

  // Seeks to the specific time in seconds in the audio stream. Playback state
  // is preserved while seeking. Returns an error if the action fails or if the
  // player has not been fully initialized. Note that this can only be called on
  // the foreground thread.
  Status SeekTo(float seconds);

  // Sets the looping behaviour of the track. Set to 0 for no looping, and any
  // positive integer for the number of times it will be looped, and to -1 for
  // looping indefinitely. Returns an error if the action fails or if the player
  // has not been fully initialized. Note that this can only be called on the
  // foreground thread.
  Status SetLoopCount(int loop);

  // Sets the source volume of playback for the audio stream in range [0, 1].
  // Returns an error if the action fails or if the player has not been fully
  // initialized. Note that this can only be called on the foreground thread.
  Status SetVolume(float volume);

  // Returns the total playback length, in seconds, of the sound object attached
  // to this player. Returns an error if the action fails or if the player has
  // not been fully initialized. Note that this can only be called on the
  // foreground thread.
  StatusOr<absl::Duration> GetDuration() const;

  // Returns the current timestamp, in seconds, of the playback of the attached
  // sound object. Returns an error if the action fails or if the player has not
  // been fully initialized. Note that this can only be called on the foreground
  // thread.
  StatusOr<absl::Duration> GetPlaybackTime() const;

  // Returns the looping behavior of the current playback. Returns an error if
  // the action fails or if the player has not been fully initialized. Note that
  // this can only be called on the foreground thread.
  StatusOr<int> GetLoopCount() const;

  // Returns if the audio stream is currently set to playing. Returns an error
  // if the action fails or if the player has not been fully initialized.
  StatusOr<State> GetPlayerState() const;

  absl::string_view GetAssetUrl() const;

  void OnActiveStatusChanged(bool active);

 protected:
  std::unique_ptr<AudioSource> audio_source_ = nullptr;

 private:
  Status SetupInternal(std::unique_ptr<AudioSource> source);
  void OnComponentStateChanged(bool active, bool paused);
  State saved_state_ = State::kReady;
  AudioPlayerState state_;

  bool component_paused_ = false;
  bool component_active_ = true;

 public:
  using IsfInfo = IsfInfo<&AudioPlayer::state_>;
  static constexpr bool kRunInEditMode = true;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_AUDIO_AUDIO_PLAYER_H_
