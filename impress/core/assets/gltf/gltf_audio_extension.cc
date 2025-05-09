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

#include "core/assets/gltf/gltf_audio_extension.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/audio/audio_player.h"
#include "core/audio/audio_player_state.proto.imp.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_vector.h"
#include "core/model/model_data.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

using model::ModelData;

Future<absl::Status> GltfAudioExtension::SetupInternal(
    ComponentHandle<GltfRenderer> gltf_renderer) {
  if (!gltf_renderer) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("Invalid GltfRenderer handle."));
  }

  gltf_renderer_ = gltf_renderer;

  const ModelData& model_data = gltf_renderer->GetGltfAsset()->GetModelData();

  return SetupScene(model_data).Combine(SetupNodes(model_data));
}

void GltfAudioExtension::Cleanup() {
  emitter_player_lookup_.clear();

  GetView().DestroyNode(scene_emitter_root_node_);
}

std::vector<ComponentHandle<AudioPlayer>> GltfAudioExtension::GetAudioPlayers()
    const {
  std::vector<ComponentHandle<AudioPlayer>> audio_players;
  for (const auto& [handle, emitter_player] : emitter_player_lookup_) {
    if (emitter_player) {
      for (const EmitterPlayer::AudioPlayerData& audio_player_data :
           emitter_player->audio_player_data) {
        audio_players.push_back(audio_player_data.audio_player);
      }
    }
  }

  return audio_players;
}

absl::Status GltfAudioExtension::SetVolume(float volume) {
  volume_ = std::optional(volume);
  for (ComponentHandle<AudioPlayer>& audio_player : GetAudioPlayers()) {
    MP_RETURN_IF_ERROR(audio_player->SetVolume(volume));
  }

  return absl::OkStatus();
}

Future<absl::Status> GltfAudioExtension::SetupNodes(
    const ModelData& model_data) {
  Future<absl::Status> setup_future = Future<absl::Status>(absl::OkStatus());
  const TypedSetVector<ModelData::EntityData>& entities = model_data.Entities();
  for (auto self : entities.Ids<ModelData::EntityId>()) {
    const auto entity_data = entities[self];
    NodeHandle node =
        GetNode()->GetComponent<GltfScene>()->GetOrCreateNodeFromBone(
            entity_data.bone);

    model::ModelData::AudioEmitterId audio_emitter_id =
        entity_data.audio_emitter;

    if (model_data.AudioEmitters().IsValid(audio_emitter_id)) {
      absl::StatusOr<EmitterPlayerHandle> handle =
          CreateEmitterPlayerById(node, audio_emitter_id);
      if (!handle.ok()) {
        return Future<absl::Status>(handle.status());
      }
      const EmitterPlayer* emitter_player = GetEmitterPlayer(*handle);
      setup_future = setup_future.Combine(emitter_player->setup_future);
    }
  }

  return setup_future;
}

Future<absl::Status> GltfAudioExtension::SetupScene(
    const ModelData& model_data) {
  Future<absl::Status> setup_future = Future<absl::Status>(absl::OkStatus());

  // Sets up scene audio emitters.
  if (!model_data.SceneAudioEmitters().empty()) {
    scene_emitter_root_node_ = GetNode()->CreateChildNode();
    scene_emitter_root_node_->SetName("SceneAudioEmitters");

    for (ModelData::AudioEmitterId scene_audio_emitter_id :
         model_data.SceneAudioEmitters()) {
      absl::StatusOr<EmitterPlayerHandle> handle = CreateEmitterPlayerById(
          scene_emitter_root_node_, scene_audio_emitter_id);
      if (!handle.ok()) {
        return Future<absl::Status>(handle.status());
      }
      const EmitterPlayer* emitter_player = GetEmitterPlayer(*handle);
      setup_future = setup_future.Combine(emitter_player->setup_future);
    }
  }

  return setup_future;
}

bool GltfAudioExtension::IsValidFor(
    ComponentHandle<GltfRenderer> gltf_renderer) {
  const ModelData& model_data = gltf_renderer->GetGltfAsset()->GetModelData();
  return !model_data.Audios().empty();
}

GltfAudioExtension::EmitterPlayer* GltfAudioExtension::GetEmitterPlayer(
    EmitterPlayerHandle handle) const {
  auto it = emitter_player_lookup_.find(handle);
  if (it != emitter_player_lookup_.end()) {
    return it->second.get();
  }
  return nullptr;
}

Future<GltfAudioExtension::EmitterPlayer::AudioPlayerData>
GltfAudioExtension::AddAudioPlayerToNode(
    NodeHandle node, const ModelData::AudioEmitterData& audio_emitter_data,
    const ModelData::AudioSourceData& audio_source_data,
    const ModelData::AudioData& audio_data) {
  float raw_volume = audio_emitter_data.gain * audio_source_data.gain;
  bool auto_play = audio_source_data.auto_play;

  return node
      ->AddComponentWithState<AudioPlayer>(
          AudioPlayerState{
              // Always false as it needs to wait for all other audio players to
              // be fully set up.
              .auto_play = false,
              // Setting the loop count to be -1 so that it loops indefinitely.
              .loop_count = audio_source_data.loop ? -1 : 0},
          audio_data.data)
      .Then([raw_volume, auto_play](ComponentHandle<AudioPlayer> audio_player)
                -> EmitterPlayer::AudioPlayerData {
        return EmitterPlayer::AudioPlayerData{
            .audio_player = audio_player,
            .raw_volume = raw_volume,
            .auto_play = auto_play,
        };
      });
}

absl::StatusOr<GltfAudioExtension::EmitterPlayerHandle>
GltfAudioExtension::CreateEmitterPlayerByIndex(NodeHandle parent_node,
                                               uint16_t emitter_index,
                                               bool force_auto_play) {
  return CreateEmitterPlayerById(
      parent_node, ModelData::AudioEmitterId(emitter_index), force_auto_play);
}

absl::StatusOr<GltfAudioExtension::EmitterPlayerHandle>
GltfAudioExtension::CreateEmitterPlayerById(
    NodeHandle parent_node, ModelData::AudioEmitterId emitter_id,
    bool force_auto_play) {
  if (emitter_player_lookup_.size() >= kMaxEmitterPlayers) {
    return absl::ResourceExhaustedError(absl::StrFormat(
        "Total number of audio players exceeds maximum allowed number: %d",
        kMaxEmitterPlayers));
  }

  const ModelData& model_data = gltf_renderer_->GetGltfAsset()->GetModelData();

  if (!model_data.AudioEmitters().IsValid(emitter_id)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Invalid AudioEmitterId %u", static_cast<uint16_t>(emitter_id)));
  }

  const model::ModelData::AudioEmitterData& audio_emitter_data =
      model_data.AudioEmitters()[emitter_id];

  // Check if the data if valid.
  for (model::ModelData::AudioSourceId audio_source_id :
       audio_emitter_data.audio_sources) {
    if (!model_data.AudioSources().IsValid(audio_source_id)) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Invalid AudioSourceId %u", static_cast<uint16_t>(audio_source_id)));
    }

    model::ModelData::AudioSourceData audio_source_data =
        model_data.AudioSources()[audio_source_id];

    if (!model_data.Audios().IsValid(audio_source_data.audio)) {
      return absl::InvalidArgumentError(
          absl::StrFormat("Invalid AudioId %u",
                          static_cast<uint16_t>(audio_source_data.audio)));
    }
  }

  std::unique_ptr<EmitterPlayer> emitter_player =
      std::make_unique<EmitterPlayer>();
  emitter_player->setup_future = Future<absl::Status>(absl::OkStatus());

  NodeHandle emitter_player_root_node = parent_node->CreateChildNode();
  emitter_player->created_nodes.push_back(emitter_player_root_node);

  emitter_player_root_node->SetName(
      absl::StrFormat("EmitterPlayer_%s", audio_emitter_data.name));

  for (model::ModelData::AudioSourceId audio_source_id :
       audio_emitter_data.audio_sources) {
    model::ModelData::AudioSourceData audio_source_data =
        model_data.AudioSources()[audio_source_id];

    const ModelData::AudioData& audio_data =
        model_data.Audios()[audio_source_data.audio];

    NodeHandle audio_player_node = emitter_player_root_node->CreateChildNode();
    emitter_player->created_nodes.push_back(emitter_player_root_node);

    audio_player_node->SetName(
        absl::StrFormat("AudioPlayer_%s", audio_source_data.name));
    Future<absl::Status> add_player_future =
        AddAudioPlayerToNode(audio_player_node, audio_emitter_data,
                             audio_source_data, audio_data)
            .Then([emitter_player = emitter_player.get()](
                      EmitterPlayer::AudioPlayerData data) {
              emitter_player->audio_player_data.push_back(data);
            });

    emitter_player->setup_future =
        emitter_player->setup_future.Combine(add_player_future);
  }

  EmitterPlayerHandle handle = GenerateEmitterPlayerHandle();

  emitter_player->setup_future = emitter_player->setup_future.Then(
      [this, force_auto_play,
       emitter_player = emitter_player.get()]() -> absl::Status {
        // Handles auto play once all players are fully set up.
        for (EmitterPlayer::AudioPlayerData& audio_player_data :
             emitter_player->audio_player_data) {
          if (force_auto_play || audio_player_data.auto_play) {
            MP_RETURN_IF_ERROR(PlayAudioPlayer(audio_player_data));
          }
        }
        return absl::OkStatus();
      });

  auto [_, inserted] =
      emitter_player_lookup_.insert({handle, std::move(emitter_player)});
  if (!inserted) {
    return absl::InternalError("Failed to create emitter audio players.");
  }

  return handle;
}

absl::Status GltfAudioExtension::PlayAudioPlayer(
    EmitterPlayer::AudioPlayerData& audio_player_data) {
  MP_RETURN_IF_ERROR(audio_player_data.audio_player->SetVolume(
      audio_player_data.raw_volume * volume_.value_or(1.0f)));

  return audio_player_data.audio_player->Play();
}

absl::StatusOr<GltfAudioExtension::EmitterPlayerHandle>
GltfAudioExtension::PlayEmitter(uint16_t emitter_index) {
  MP_ASSIGN_OR_RETURN(
      EmitterPlayerHandle handle,
      CreateEmitterPlayerByIndex(gltf_renderer_->GetNode(), emitter_index,
                                 /*force_auto_play=*/true));

  EmitterPlayer* emitter_player = GetEmitterPlayer(handle);
  if (emitter_player == nullptr) {
    return absl::InternalError(absl::StrFormat(
        "Failed to get EmitterPlayer for emitter index %u", emitter_index));
  }

  emitter_player->play_future = emitter_player->setup_future.Then(
      [this, emitter_player, handle]() -> Future<absl::Status> {
        Future<absl::Status> play_future =
            Future<absl::Status>(absl::OkStatus());

        for (EmitterPlayer::AudioPlayerData& audio_player_data :
             emitter_player->audio_player_data) {
          NodeHandle audio_player_node =
              audio_player_data.audio_player->GetNode();
          Future<absl::Status> sub_play_future;
          audio_player_node->Connect(
              [sub_play_future](
                  const AudioPlayer::PlaybackCompleteEvent& event) {
                sub_play_future.Return(absl::OkStatus());
              });
          play_future = play_future.Combine(sub_play_future);
        }

        return play_future.Then(
            [this, handle]() { RemoveEmitterPlayer(handle); });
      });

  return handle;
}

bool GltfAudioExtension::IsEmitterPlayerHandleValid(
    EmitterPlayerHandle handle) const {
  return GetEmitterPlayer(handle) != nullptr;
}

Future<absl::Status> GltfAudioExtension::GetEmitterPlayerSetupFuture(
    EmitterPlayerHandle handle) const {
  const EmitterPlayer* emitter_player = GetEmitterPlayer(handle);
  if (emitter_player) {
    return emitter_player->setup_future;
  }
  return Future<absl::Status>(absl::InvalidArgumentError(absl::StrFormat(
      "Invalid EmitterPlayerHandle %u", static_cast<uint16_t>(handle))));
}

void GltfAudioExtension::RemoveEmitterPlayer(EmitterPlayerHandle handle) {
  auto it = emitter_player_lookup_.find(handle);
  if (it == emitter_player_lookup_.end()) {
    return;
  }
  emitter_player_lookup_.erase(it);
}

GltfAudioExtension::EmitterPlayerHandle
GltfAudioExtension::GenerateEmitterPlayerHandle() {
  return last_emitter_player_handle_++;
}

}  // namespace imp
