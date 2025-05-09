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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_GLTF_AUDIO_EXTENSION_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_GLTF_AUDIO_EXTENSION_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/async/future.h"
#include "core/audio/audio_player.h"
#include "core/model/model_data.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_extension.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

// GltfAudioExtension is the Impress implementation of the KHR_audio extension
// spec
// (https://github.com/omigroup/gltf-extensions/blob/main/extensions/2.0/KHR_audio/README.md).
//
// Like other GltfExtensions, GltfAudioExtension requires GltfRenderer with
// valid model data to be present on the same node. During Setup(),
// GltfAudioExension will read from the model data and populate the AudioPlayers
// accordingly.
//
// At a high level, GltfAudioExtension will create AudioPlayers from the audio
// information specified in the spec, either from a uri or a buffer view. If
// both are specified, the buffer view will be loaded.
class GltfAudioExtension : public GltfExtension {
 public:
  // Handle to the AudioPlayers created to play an emitter.
  // This can be used to look up AudioPlayers.
  using EmitterPlayerHandle = uint32_t;

  // Maximum number of emitter players allowed.
  static constexpr uint32_t kMaxEmitterPlayers = 256;

  void Cleanup();

  // Returns the AudioPlayers created by this GltfAudioExtension.
  std::vector<ComponentHandle<AudioPlayer>> GetAudioPlayers() const;

  // Invoked via SFINAE to test if this extension is valid for the gltf
  // renderer passed in.
  static bool IsValidFor(ComponentHandle<GltfRenderer> gltf_renderer);

  // Returns true if the handle is valid. False otherwise.
  bool IsEmitterPlayerHandleValid(EmitterPlayerHandle handle) const;

  // Plays the emitter player by the emitter index in gltf.
  //
  // TODO: Add support for reusing stopped EmitterPlayers instead
  // of recreating them.
  absl::StatusOr<EmitterPlayerHandle> PlayEmitter(uint16_t emitter_index);

  // Returns a future that becomes ready when the EmitterPlayer has finished
  // setting up its AudioPlayers. If the handle is invalid, an error will be
  // returned.
  Future<absl::Status> GetEmitterPlayerSetupFuture(
      EmitterPlayerHandle handle) const;

  // Removes the EmitterPlayer, including removing all the AudioPlayers and
  // all the nodes created for the EmitterPlayer.
  void RemoveEmitterPlayer(EmitterPlayerHandle handle);

  // This serves as a global multiplier to the volume of all audio players
  // created through audio extension.
  absl::Status SetVolume(float volume);

  // TODO : Add support for stopping emitter players by emitter
  // indices.

 private:
  // EmitterPlayer is a collection of AudioPlayers for playing an emitter.
  // TODO: Split this into a separate file and add proper testing.
  struct EmitterPlayer {
    struct AudioPlayerData {
      ComponentHandle<AudioPlayer> audio_player;
      float raw_volume = 1.0f;
      bool auto_play = false;
    };

    ~EmitterPlayer() {
      for (const NodeHandle& created_node : created_nodes) {
        if (created_node) {
          created_node->GetView().DestroyNode(created_node);
        }
      }
    }

    Future<absl::Status> setup_future;
    Future<absl::Status> play_future;
    std::vector<AudioPlayerData> audio_player_data;
    std::vector<NodeHandle> created_nodes;
  };

  Future<absl::Status> SetupInternal(
      ComponentHandle<GltfRenderer> gltf_renderer) override;

  // Creates and configures AudioPlayers for audio emitters in all nodes.
  Future<absl::Status> SetupNodes(const model::ModelData& model_data);
  // Creates and configures AudioPlayers for audio emitters in the gltf scene.
  Future<absl::Status> SetupScene(const model::ModelData& model_data);

  // Creates the emitter player by the AudioEmitterId.
  // Creates new AudioPlayers and returns a future to the EmitterPlayerHandle.
  // If no emitter is found at `emitter_id`, an error will be returned.
  // If `stop_on_create` is true, the AudioPlayers will be stopped on creation.
  // Otherwise, they will be played according to their `auto_play` settings.
  //
  // TODO: Add support for registering a callback when the playback
  // finishes.
  absl::StatusOr<EmitterPlayerHandle> CreateEmitterPlayerById(
      NodeHandle parent_node, model::ModelData::AudioEmitterId emitter_id,
      bool force_auto_play = false);

  // Creates the emitter player by the emitter index in gltf.
  // Creates new AudioPlayers and returns a future to the EmitterPlayerHandle.
  // If no emitter is found at `emitter_index`, an error will be returned.
  // If `force_auto_play` is true, the AudioPlayers will be played regardless of
  // their `auto_play` settings. Otherwise, they will be played according to
  // their `auto_play` settings.
  //
  // TODO: Cleanup players/nodes once the emitter finishes playing.
  absl::StatusOr<EmitterPlayerHandle> CreateEmitterPlayerByIndex(
      NodeHandle parent_node, uint16_t emitter_index,
      bool force_auto_play = false);

  EmitterPlayer* GetEmitterPlayer(EmitterPlayerHandle handle) const;

  EmitterPlayerHandle GenerateEmitterPlayerHandle();

  Future<EmitterPlayer::AudioPlayerData> AddAudioPlayerToNode(
      NodeHandle node,
      const model::ModelData::AudioEmitterData& audio_emitter_data,
      const model::ModelData::AudioSourceData& audio_source_data,
      const model::ModelData::AudioData& audio_data);

  absl::Status PlayAudioPlayer(
      EmitterPlayer::AudioPlayerData& audio_player_data);

  ComponentHandle<GltfRenderer> gltf_renderer_;

  NodeHandle scene_emitter_root_node_;
  std::optional<float> volume_;

  // Lookup table for EmitterPlayers.
  tsl::robin_map<EmitterPlayerHandle, std::unique_ptr<EmitterPlayer>>
      emitter_player_lookup_;
  EmitterPlayerHandle last_emitter_player_handle_ = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_GLTF_AUDIO_EXTENSION_H_
