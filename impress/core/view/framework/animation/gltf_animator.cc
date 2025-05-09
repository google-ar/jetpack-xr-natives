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

#include "core/view/framework/animation/gltf_animator.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/animation/gltf_animation.h"
#include "core/animation/light_punctual_animation.h"
#include "core/animation/material_animation.h"
#include "core/animation/texture_transform_animation.h"
#include "core/assets/asset_ptr.h"
#include "core/common/invocable.h"
#include "core/common/paired_span.h"
#include "core/common/robin_map.h"
#include "core/common/trace.h"
#include "core/common/typed_id.h"
#include "core/config.h"
#include "core/material_library/generic_material_constants.h"
#include "core/material_library/material_param_value.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/transform.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/system.h"
#include "core/render/base_renderable_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/animation/animator_events.proto.imp.h"
#include "core/view/framework/animation/gltf_animator_state.proto.imp.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/lighting/light_component.h"
#include "core/view/framework/render/material.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"
#if IMP_RUNTIME(DEV)
#include "dear_imgui/imgui.h"
#endif  // IMP_RUNTIME(DEV)

namespace imp {
namespace {
using AnimatedMaterialParameter =
    animation::MaterialAnimation::AnimatedMaterialParameter;
using LightParameter = animation::LightPunctualAnimation::LightParameter;
using AnimatedLightParameter =
    animation::LightPunctualAnimation::AnimatedLightParameter;
using TextureTransformParameter =
    animation::TextureTransformAnimation::TextureTransformParameter;
using AnimatedTextureTransformParameter =
    animation::TextureTransformAnimation::AnimatedTextureTransformParameter;

// Used to provide the current transform of a bone during animation so that it
// can be supplied to fill in missing channels instead of using a default value.
class MissingTransformProvider
    : public animation::GltfAnimation::MissingTransformProvider {
 public:
  MissingTransformProvider(ComponentHandle<GltfScene> gltf_scene)
      : gltf_scene_(gltf_scene) {}

  Transform<float> GetMissingTransform(model::BoneId bone) override {
    return gltf_scene_->GetLocalTransformFromBone(bone);
  }

  ComponentHandle<GltfScene> gltf_scene_;
};

}  // namespace

GltfAnimator::System::System(BaseView* view) : ComponentSystem(view) {}

void GltfAnimator::System::PreComponentsUpdated(const FrameTime& frame_time) {
  IMP_TRACE();

  // Since we will be setting a lot of transforms in a localized hierarchy,
  // delay transform matrix propagation until we are done. We do one transaction
  // for every animator because each transaction has a performance cost.
  bool is_transaction_open = false;
  filament::Engine* engine = BaseView::GetSharedEngine();
  filament::TransformManager& tm = engine->getTransformManager();

  // Pause sending animation events while the transform manager has an open
  // transaction and instead queue the events. This is because while the
  // transaction is open, getting the world transform of any node impacted by
  // the transaction will produce incorrect results so we must wait to send the
  // events until after we close the transaction so that if a user accesses the
  // world transform of an updated node during an event it is correct.
  are_events_paused_ = true;

  GetComponentManager().UpdateEach<GltfAnimator>(
      [&frame_time, &is_transaction_open, &tm](GltfAnimator* animator) {
        if (!is_transaction_open && animator->IsPlaying()) {
          is_transaction_open = true;
          tm.openLocalTransformTransaction();
        }

        animator->AdvanceAnimationPlayback(frame_time.GetDeltaTime());
      });

  if (is_transaction_open) {
    tm.commitLocalTransformTransaction();
  }

  // Resume sending events and dispatch the events that were queued while
  // advancing animation playback.
  are_events_paused_ = false;
  DispatchQueuedEvents();
}

void GltfAnimator::System::SendOrQueuePlaybackUpdatedEvents(
    PlaybackUpdatedEvent ev, std::vector<NodeHandle> targets) {
  ev.SetPropagationMode(Event::PropagationMode::kNone);
  if (!are_events_paused_) {
    for (NodeHandle target : targets) {
      if (target) {
        target->Send(ev);
      }
    }
  } else {
    playback_updated_event_queue_.push_back(
        std::make_pair(ev, std::move(targets)));
  }
}

void GltfAnimator::System::SendOrQueuePlaybackLoopedEvent(
    PlaybackLoopedEvent ev, NodeHandle target) {
  if (!are_events_paused_) {
    if (target) {
      target->Send(ev);
    }
  } else {
    playback_looped_event_queue_.push_back(std::make_pair(ev, target));
  }
}

void GltfAnimator::System::SendOrQueuePlaybackEndedEvent(PlaybackEndedEvent ev,
                                                         NodeHandle target) {
  if (!are_events_paused_) {
    if (target) {
      target->Send(ev);
    }
  } else {
    playback_ended_event_queue_.push_back(std::make_pair(ev, target));
  }
}

void GltfAnimator::System::DispatchQueuedEvents() {
  if (are_events_paused_) {
    return;
  }

  for (const QueuedPlaybackUpdatedEvent& queued_updated_event :
       playback_updated_event_queue_) {
    for (NodeHandle target : queued_updated_event.second) {
      if (target) {
        target->Send(queued_updated_event.first);
      }
    }
  }
  playback_updated_event_queue_.clear();

  for (const QueuedPlaybackLoopedEvent& queued_looped_event :
       playback_looped_event_queue_) {
    if (queued_looped_event.second) {
      queued_looped_event.second->Send(queued_looped_event.first);
    }
  }
  playback_looped_event_queue_.clear();

  for (const QueuedPlaybackEndedEvent& queued_ended_event :
       playback_ended_event_queue_) {
    if (queued_ended_event.second) {
      queued_ended_event.second->Send(queued_ended_event.first);
    }
  }
  playback_ended_event_queue_.clear();
}

GltfAnimator::GltfAnimator() = default;

void GltfAnimator::Setup() {
  if (state_.starting_animation) {
    Setup(*state_.starting_animation);
  }
}

void GltfAnimator::Setup(const AssetPtr<GltfAsset>& gltf_asset) {
  SetTargetGltfAsset(gltf_asset);
}

void GltfAnimator::Setup(const PlayCommand& play_command,
                         const AssetPtr<GltfAsset>& gltf_asset) {
  Play(play_command, gltf_asset);
}

void GltfAnimator::OnActiveStatusChanged(bool is_active) {
  if (is_active) {
    AdvanceAnimationPlayback(absl::ZeroDuration());
  }
}

absl::Status GltfAnimator::CanPlay(const PlayCommand& play_command) const {
  int32_t anim_index;
  const animation::GltfAnimation* anim;
  return CanPlay(play_command, anim_index, anim);
}

void GltfAnimator::Play(const PlayCommand& play_command,
                        const AssetPtr<GltfAsset>& gltf_asset) {
  absl::Status result = PlaySafely(play_command, gltf_asset);
  if (!result.ok()) {
    IMP_LOG(imp::FATAL) << result;
  }
}

void GltfAnimator::Play(absl::string_view anim_name,
                        GltfAnimatorState::AnimOptions options) {
  PlayCommand play_command{.animation = std::string(anim_name),
                           .options = options};
  Play(play_command);
}

absl::Status GltfAnimator::PlaySafely(const PlayCommand& play_command,
                                      const AssetPtr<GltfAsset>& gltf_asset) {
  if (gltf_asset) {
    SetTargetGltfAsset(gltf_asset);
  }
  int32_t anim_index;
  const animation::GltfAnimation* anim;
  absl::Status status = CanPlay(play_command, anim_index, anim);
  if (status.ok()) {
    PlayAnim(anim_index, anim, play_command.options);
  }
  return status;
}

void GltfAnimator::Restart(PlaybackChannelId channel_id) {
  if (IsAllChannelsId(channel_id)) {
    if (playback_channel_lookup_.empty()) {
      PlayCommand play_command;
      play_command.options = GltfAnimatorState::AnimOptions();
      Play(play_command, target_gltf_asset_);
    } else {
      for (auto channel_id : playback_channel_lookup_) {
        PlaybackChannel& channel = playback_channels_[channel_id.second];
        channel.anim_playback.t = channel.anim_playback.anim->FirstT();
      }
      AdvanceAnimationPlayback(absl::ZeroDuration());
    }
  } else {
    auto channel = GetPlaybackChannel(channel_id);
    if (channel) {
      channel->anim_playback.t = channel->anim_playback.anim->FirstT();
      AdvanceAnimationPlayback(*channel, absl::ZeroDuration(),
                               GetNode()->GetComponent<GltfScene>());
    }
  }
}

void GltfAnimator::PlayAnim(int32_t anim_index,
                            const animation::GltfAnimation* anim,
                            const Options& options) {
  PlaybackChannelId channel_id = options.playback_channel;
  if (IsAllChannelsId(channel_id)) {
    IMP_LOG(imp::ERROR) << "Invalid playback channel ID.";
    return;
  }

  absl::Duration start_duration =
      clamp(absl::Seconds(options.start_time_seconds), absl::ZeroDuration(),
            anim->Duration());

  auto channel = GetPlaybackChannel(channel_id);
  if (channel) {
    if (options.blend_time_seconds) {
      RobinMap<BoneId, BlendingAnim::BlendingBoneTarget> blending_bone_targets;

      const animation::GltfAnimation::BoneTargetSpan blend_out_targets =
          channel->anim_playback.anim->TransformTargets();
      for (animation::GltfAnimation::BoneTargetId target_id :
           blend_out_targets.Ids<animation::GltfAnimation::BoneTargetId>()) {
        blending_bone_targets[blend_out_targets[target_id].bone].out_id =
            target_id;
      }

      const animation::GltfAnimation::BoneTargetSpan blend_in_targets =
          anim->TransformTargets();
      for (animation::GltfAnimation::BoneTargetId target_id :
           blend_in_targets.Ids<animation::GltfAnimation::BoneTargetId>()) {
        blending_bone_targets[blend_in_targets[target_id].bone].in_id =
            target_id;
      }

      channel->blend_anim.emplace(BlendingAnim{
          channel->anim_playback, std::move(blending_bone_targets),
          absl::Seconds(options.blend_time_seconds), absl::ZeroDuration()});
    } else {
      channel->blend_anim.reset();
    }

    channel->anim_playback = GltfAnimPlayback{
        anim_index,
        anim,
        anim->CreateCursor(),
        anim->FirstT() + start_duration,
        options.speed_multiplier ? options.speed_multiplier : 1.0f,
        options.looping,
        0};
    channel->active = true;
    channel->persist = options.persist_channel;
  } else {
    PlaybackChannel new_channel{
        .anim_playback =
            GltfAnimPlayback{
                anim_index, anim, anim->CreateCursor(),
                anim->FirstT() + start_duration,
                options.speed_multiplier ? options.speed_multiplier : 1.0f,
                options.looping, 0},
        .blend_anim = {},
        .active = true,
        .persist = options.persist_channel};

    // Save the new channel into the first unused slot in the vector. If there
    // are none then add a new slot.
    auto new_channel_iter = std::find_if(
        playback_channels_.begin(), playback_channels_.end(),
        [](const PlaybackChannel& channel) { return !channel.active; });
    if (new_channel_iter != playback_channels_.end()) {
      *new_channel_iter = std::move(new_channel);
    } else {
      playback_channels_.emplace_back(std::move(new_channel));
      new_channel_iter = playback_channels_.end() - 1;
    }
    // Record the index of the new channel into the ChannelId lookup map.
    int new_channel_index = new_channel_iter - playback_channels_.begin();
    playback_channel_lookup_.insert({channel_id, new_channel_index});
  }

  PlaybackStartedEvent ev;
  ev.animation_index = anim_index;
  GetNode()->Send(ev);

  AdvanceAnimationPlayback(absl::ZeroDuration());
}

NodeHandle GltfAnimator::GetOrCreateNode(absl::string_view name) {
  return GetNode()->GetComponent<GltfScene>()->GetOrCreateNode(name);
}

NodeHandle GltfAnimator::GetRoot() const {
  return GetNode()->GetComponent<GltfScene>()->GetRoot();
}

void GltfAnimator::Stop(PlaybackChannelId channel_id) {
  auto send_channel_stopped_event = [&](PlaybackChannel& channel) {
    PlaybackEndedEvent ev;
    ev.cause = PlaybackEndedEvent::STOPPED;
    ev.animation_index = channel.anim_playback.anim_index;
    GetView()
        .GetComponentManager()
        .GetComponentSystem<GltfAnimator>()
        .SendOrQueuePlaybackEndedEvent(ev, GetNode());
  };
  if (IsAllChannelsId(channel_id)) {
    for (auto it = playback_channel_lookup_.begin();
         it != playback_channel_lookup_.end();) {
      auto* channel = &playback_channels_[it->second];
      if (channel->active) {
        send_channel_stopped_event(*channel);
        channel->active = false;
      }
      if (!channel->persist) {
        it = playback_channel_lookup_.erase(it);
      } else {
        ++it;
      }
    }
    if (playback_channel_lookup_.empty()) {
      playback_channels_.clear();
    }
  } else {
    auto channel = GetPlaybackChannel(channel_id);
    if (channel && channel->active) {
      send_channel_stopped_event(*channel);
      channel->active = false;
      if (!channel->persist) {
        playback_channel_lookup_.erase(channel_id);
      }
    }
  }
}

bool GltfAnimator::IsPlaying(
    std::optional<PlaybackChannelId> channel_id) const {
  if (!IsActive()) {
    return false;
  }

  if (!channel_id) {
    for (auto channel_id : playback_channel_lookup_) {
      if (playback_channels_[channel_id.second].active) {
        return true;
      }
    }
    return false;
  }

  auto channel = GetPlaybackChannel(*channel_id);
  return channel && channel->active;
}

absl::Duration GltfAnimator::GetNextPlaybackTime(
    absl::Duration frame_time_delta, GltfAnimPlayback& playback) {
  if (frame_time_delta == absl::ZeroDuration()) {
    return playback.t;
  }
  absl::Duration next_playback_t =
      playback.t + frame_time_delta * playback.speed_multiplier;
  if (elapsed_time_provider_) {
    absl::StatusOr<absl::Duration> provided_time_or = elapsed_time_provider_();
    if (provided_time_or.ok()) {
      absl::Duration provided_time = provided_time_or.value();
      provided_time %= playback.anim->Duration();
      next_playback_t = playback.anim->FirstT() + provided_time;
    }
  }
  return next_playback_t;
}

void GltfAnimator::AdvanceAnimationPlayback(absl::Duration delta_time) {
  if (!IsPlaying()) {
    return;
  }

  ComponentHandle<GltfScene> gltf_scene = GetNode()->GetComponent<GltfScene>();
  if (gltf_scene) {
    std::vector<PlaybackChannelLookup::iterator> channels_to_erase;
    for (auto channel_id = playback_channel_lookup_.begin();
         channel_id != playback_channel_lookup_.end(); ++channel_id) {
      PlaybackChannel& channel = playback_channels_[channel_id->second];
      if (!channel.active) {
        continue;
      }
      bool animation_ended =
          AdvanceAnimationPlayback(channel, delta_time, gltf_scene);
      if (animation_ended) {
        channel.active = false;
        if (!channel.persist) {
          channels_to_erase.push_back(channel_id);
        }
      }
    }
    for (auto iter : channels_to_erase) {
      playback_channel_lookup_.erase(iter);
    }
  }
}

bool GltfAnimator::AdvanceAnimationPlayback(
    PlaybackChannel& channel, absl::Duration delta_time,
    ComponentHandle<GltfScene> gltf_scene) {
  GltfAnimPlayback& playback = channel.anim_playback;

  playback.t = GetNextPlaybackTime(delta_time, playback);
  float curr_time_seconds =
      static_cast<float>(absl::ToDoubleSeconds(playback.t));

  // Animate all morph target animations.
  animation::GltfAnimation::BoneTargetLookup<std::array<float, 256>>
      morph_target_anims = playback.anim->EvaluateMorphTargetAnimations(
          curr_time_seconds, &playback.cursor.weights);
  const animation::GltfAnimation::BoneTargetSpan& morph_target_anim_targets =
      playback.anim->MorphTargetAnimationTargets();

  PlaybackUpdatedEvent ev;
  ev.animation_index = playback.anim_index;
  std::vector<NodeHandle> updated_event_targets;

  for (animation::GltfAnimation::BoneTargetId id :
       morph_target_anim_targets
           .Ids<animation::GltfAnimation::BoneTargetId>()) {
    const animation::GltfAnimation::BoneTarget& target =
        morph_target_anim_targets[id];
    if (NodeHandle node = gltf_scene->GetNodeFromBone(target.bone);
        utils::Entity entity = node->GetEntity()) {
      BaseRenderableManager& renderable_manager =
          GetView().GetRenderableManager();
      renderable_manager.SetMorphWeights(renderable_manager.GetInstance(entity),
                                         morph_target_anims[id].data(),
                                         morph_target_anims[id].size());

      // Send playback updated event to nodes with morph target curves.
      updated_event_targets.push_back(node);
    } else {
      IMP_LOG(imp::ERROR) << "No node found for morph target animation";
    }
  }

  if (channel.blend_anim.has_value()) {
    AdvanceBlendAnimation(channel, delta_time);
  } else {
    // Animate all t/r/s animations.
    MissingTransformProvider missing_transform_provider(gltf_scene);
    animation::GltfAnimation::BoneTargetLookup<Trsf> transforms =
        playback.anim->EvaluateTransform(curr_time_seconds,
                                         &playback.cursor.trs,
                                         &missing_transform_provider);

    const animation::GltfAnimation::BoneTargetSpan& bone_targets =
        playback.anim->TransformTargets();
    for (animation::GltfAnimation::BoneTargetId id :
         bone_targets.Ids<animation::GltfAnimation::BoneTargetId>()) {
      const animation::GltfAnimation::BoneTarget& target = bone_targets[id];
      gltf_scene->SetLocalTransformFromBone(target.bone, transforms[id]);

      if (NodeHandle node = gltf_scene->GetNodeFromBone(target.bone)) {
        // Send playback updated event to nodes with t/r/s curves.
        updated_event_targets.push_back(node);
      }
    }
  }

  if (absl::Status status = ApplyMaterialAnimation(channel); !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to apply material animation: " << status;
  }
  ApplyLightPunctualAnimation(channel);

  // Send playback updated event to self.
  updated_event_targets.push_back(GetNode());
  auto& animator_system =
      GetView().GetComponentManager().GetComponentSystem<GltfAnimator>();
  animator_system.SendOrQueuePlaybackUpdatedEvents(
      ev, std::move(updated_event_targets));

  bool animation_ended = false;
  bool crossed_endpoint =
      playback.anim->SanitizeT(playback.looping, &playback.t);

  if (crossed_endpoint) {
    if (playback.looping) {
      // Animation looped.
      PlaybackLoopedEvent ev;
      ev.loop_count = ++playback.loop_count;
      ev.animation_index = playback.anim_index;
      animator_system.SendOrQueuePlaybackLoopedEvent(ev, GetNode());
    } else {
      // Animation Ended.
      PlaybackEndedEvent ev;
      ev.cause = PlaybackEndedEvent::COMPLETED;
      ev.animation_index = channel.anim_playback.anim_index;
      animator_system.SendOrQueuePlaybackEndedEvent(ev, GetNode());
      animation_ended = true;
    }
  }

  return animation_ended;
}

absl::Status GltfAnimator::ApplyMaterialAnimation(PlaybackChannel& channel) {
  ComponentHandle<GltfRenderer> gltf_renderer =
      GetNode()->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::OkStatus();
  }

  GltfAnimPlayback& playback = channel.anim_playback;
  float curr_time_seconds =
      static_cast<float>(absl::ToDoubleSeconds(playback.t));

  animation::GltfAnimation::MaterialTargetLookup<
      animation::MaterialAnimation::MaterialParameter>
      material_parameters = playback.anim->EvaluateMaterialParameters(
          curr_time_seconds, &playback.cursor.material_parameters);
  const animation::GltfAnimation::MaterialTargetSpan& material_targets =
      playback.anim->MaterialAnimationTargets();

  for (animation::GltfAnimation::MaterialTargetId id :
       material_targets.Ids<animation::GltfAnimation::MaterialTargetId>()) {
    const animation::GltfAnimation::MaterialTarget& target =
        material_targets[id];
    uint16_t target_index = static_cast<uint16_t>(target);
    MP_ASSIGN_OR_RETURN(
        GenericMaterial * material,
        GetNode()->GetComponent<GltfRenderer>()->GetGenericMaterialByIndex(
            target_index));

    auto& flag = material_parameters[id].animated_material_parameter;
    if (flag & AnimatedMaterialParameter::kBaseColorFactor) {
      material->SetBaseColorFactor(material_parameters[id].base_color_factor);
    }
    if (flag & AnimatedMaterialParameter::kMetallicFactor) {
      material->SetMetallicFactor(material_parameters[id].metallic_factor);
    }
    if (flag & AnimatedMaterialParameter::kRoughnessFactor) {
      material->SetRoughnessFactor(material_parameters[id].roughness_factor);
    }
    if (flag & AnimatedMaterialParameter::kAlphaCutoff) {
      material->SetAlphaCutoff(material_parameters[id].alpha_cutoff);
    }
    if (flag & AnimatedMaterialParameter::kEmissiveFactor) {
      material->SetEmissiveFactor(material_parameters[id].emissive_factor);
    }
    if (flag & AnimatedMaterialParameter::kNormalTextureScale) {
      material->SetNormalScale(material_parameters[id].normal_texture_scale);
    }
    if (flag & AnimatedMaterialParameter::kOcclusionTextureStrength) {
      material->SetAmbientOcclusionStrength(
          material_parameters[id].occlusion_texture_strength);
    }
    if (flag & AnimatedMaterialParameter::kTransmission) {
      material->SetTransmissionFactor(material_parameters[id].transmission);
    }
    if (flag & AnimatedMaterialParameter::kIor) {
      material->SetIndexOfRefraction(material_parameters[id].ior);
    }

    MP_RETURN_IF_ERROR(ApplyTextureTransformAnimation(
        material_parameters[id].texture_transform_parameters, material));
  }
  return absl::OkStatus();
}

absl::Status GltfAnimator::ApplyTextureTransformAnimation(
    TextureTransformParameters& texture_transform_parameters,
    GenericMaterial* material) const {
  if (texture_transform_parameters.empty()) {
    return absl::OkStatus();
  }

  for (TextureTransformParameter& tt_param : texture_transform_parameters) {
    Invocable<mat3f(const mat3f&)> calculate_uv_transform =
        [&tt_param](const mat3f& current_uv_transform) {
          float2 offset, scale;
          float rotation;
          UvTransformFromMatrix(/*input*/ current_uv_transform,
                                /*output*/ offset, /*output*/ rotation,
                                /*output*/ scale);

          auto& texture_flag = tt_param.animated_texture_transform_parameter;
          if (texture_flag &
              AnimatedTextureTransformParameter::kTextureTransformOffset) {
            offset = tt_param.texture_transform_offset;
          }
          if (texture_flag &
              AnimatedTextureTransformParameter::kTextureTransformRotation) {
            rotation = tt_param.texture_transform_rotation;
          }
          if (texture_flag &
              AnimatedTextureTransformParameter::kTextureTransformScale) {
            scale = tt_param.texture_transform_scale;
          }
          return MatrixFromUvTransform(offset, rotation, scale);
        };
    switch (tt_param.texture_target) {
      case animation::TexturableParameters::kBaseColorTexture: {
        mat3f uv_transform = calculate_uv_transform(
            material->GetBaseColorTexture().uv_transform);
        MP_RETURN_IF_ERROR(material->SetBaseColorUvTransform(uv_transform));
        break;
      }
      case animation::TexturableParameters::kMetallicRoughnessTexture: {
        mat3f uv_transform = calculate_uv_transform(
            material->GetMetallicRoughnessTexture().uv_transform);
        MP_RETURN_IF_ERROR(
            material->SetMetallicRoughnessUvTransform(uv_transform));
        break;
      }
      case animation::TexturableParameters::kNormalTexture: {
        mat3f uv_transform =
            calculate_uv_transform(material->GetNormalTexture().uv_transform);
        MP_RETURN_IF_ERROR(material->SetNormalUvTransform(uv_transform));
        break;
      }
      case animation::TexturableParameters::kOcclusionTexture: {
        return absl::UnimplementedError(
            "Failed to set UV transform for occlusion channel:");
        break;
      }
      case animation::TexturableParameters::kEmissiveTexture: {
        mat3f uv_transform =
            calculate_uv_transform(material->GetEmissiveTexture().uv_transform);
        MP_RETURN_IF_ERROR(material->SetEmissiveUvTransform(uv_transform));
        break;
      }
      case animation::TexturableParameters::kClearcoatTexture: {
        return absl::UnimplementedError(
            "Failed to set UV transform for clearcoat channel");
        break;
      }
      case animation::TexturableParameters::kClearcoatNormalTexture: {
        return absl::UnimplementedError(
            "Failed to set UV transform for clearcoat normal channel");
        break;
      }
      case animation::TexturableParameters::kClearcoatRoughnessTexture: {
        return absl::UnimplementedError(
            "Failed to set UV transform for clearcoat roughness channel");
        break;
      }
      case animation::TexturableParameters::kSheenColorTexture: {
        return absl::UnimplementedError(
            "Failed to set UV transform for sheen color channel");
        break;
      }
      case animation::TexturableParameters::kSheenColorRoughnessTexture: {
        return absl::UnimplementedError(
            "Failed to set UV transform for sheen color roughness channel");
        break;
      }
      case animation::TexturableParameters::kIorTexture: {
        return absl::UnimplementedError(
            "Failed to set UV transform for ior channel");
        break;
      }
      case animation::TexturableParameters::kTransmissionTexture: {
        mat3f uv_transform = calculate_uv_transform(
            material->GetTransmissionTexture().uv_transform);
        MP_RETURN_IF_ERROR(material->SetTransmissionUvTransform(uv_transform));
        break;
      }
      default:
        return absl::InvalidArgumentError(
            absl::StrCat("No UV transform found for texture target: ",
                         static_cast<int>(tt_param.texture_target)));
        break;
    }
  }
  return absl::OkStatus();
}

void GltfAnimator::ApplyLightPunctualAnimation(PlaybackChannel& channel) {
  ComponentHandle<GltfRenderer> gltf_renderer =
      GetNode()->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return;
  }

  GltfAnimPlayback& playback = channel.anim_playback;
  float curr_time_seconds =
      static_cast<float>(absl::ToDoubleSeconds(playback.t));

  animation::GltfAnimation::LightTargetLookup<LightParameter> light_parameters =
      playback.anim->EvaluateLightParameters(curr_time_seconds,
                                             &playback.cursor.light_punctuals);
  const animation::GltfAnimation::LightTargetSpan& light_targets =
      playback.anim->LightPunctualAnimationTargets();
  for (animation::GltfAnimation::LightTargetId id :
       light_targets.Ids<animation::GltfAnimation::LightTargetId>()) {
    const animation::GltfAnimation::LightTarget& target = light_targets[id];
    ComponentHandle<LightComponent> light_component =
        gltf_renderer->GetLightComponentById(target);
    if (!light_component) {
      IMP_LOG(imp::WARNING) << "No light component found";
    } else {
      auto& flag = light_parameters[id].animated_light_parameter;
      bool is_updated = false;
      if (flag & AnimatedLightParameter::kColor) {
        light_component->SetColor(light_parameters[id].color);
        is_updated = true;
      }
      if (flag & AnimatedLightParameter::kIntensity) {
        light_component->SetIntensity(light_parameters[id].intensity);
        is_updated = true;
      }
      if (flag & AnimatedLightParameter::kRange) {
        light_component->SetFalloff(light_parameters[id].range);
        is_updated = true;
      }
      if (flag & AnimatedLightParameter::kSpotInnerConeAngle ||
          (flag & AnimatedLightParameter::kSpotOuterConeAngle)) {
        float2 spot_cone = light_component->GetSpotCone();
        if (flag & AnimatedLightParameter::kSpotInnerConeAngle) {
          spot_cone.x = light_parameters[id].spot_inner_cone_angle;
        }
        if (flag & AnimatedLightParameter::kSpotOuterConeAngle) {
          spot_cone.y = light_parameters[id].spot_outer_cone_angle;
        }
        light_component->SetSpotCone(spot_cone);
        is_updated = true;
      }
      if (is_updated) {
        light_component->OnIsfStateChanged();
      }
    }
  }
}

void GltfAnimator::AdvanceBlendAnimation(GltfAnimator::PlaybackChannel& channel,
                                         absl::Duration delta_time) {
  if (!channel.blend_anim.has_value()) {
    IMP_LOG(imp::FATAL) << "AdvanceBlendAnimation called without a current BlendingAnim";
  }
  GltfAnimPlayback& blend_out_anim = channel.blend_anim->blend_out_anim;

  blend_out_anim.t = GetNextPlaybackTime(delta_time, blend_out_anim);
  channel.blend_anim->elapsed_seconds += delta_time;

  ComponentHandle<GltfScene> gltf_scene = GetNode()->GetComponent<GltfScene>();
  if (!gltf_scene) {
    return;
  }

  const imp::GltfAsset* gltf_asset = GetGltfAsset().Get();
  if (!gltf_asset) {
    return;
  }

  MissingTransformProvider missing_transform_provider(gltf_scene);

  animation::GltfAnimation::BoneTargetLookup<Trsf> out_transforms =
      blend_out_anim.anim->EvaluateTransform(
          static_cast<float>(absl::ToDoubleSeconds(blend_out_anim.t)),
          &blend_out_anim.cursor.trs, &missing_transform_provider);
  animation::GltfAnimation::BoneTargetLookup<Trsf> in_transforms =
      channel.anim_playback.anim->EvaluateTransform(
          static_cast<float>(absl::ToDoubleSeconds(channel.anim_playback.t)),
          &channel.anim_playback.cursor.trs, &missing_transform_provider);

  auto anim_progress =
      absl::ToDoubleSeconds(channel.blend_anim->elapsed_seconds) /
      absl::ToDoubleSeconds(channel.blend_anim->blend_time_seconds);
  float weight = std::clamp(anim_progress, 0.0, 1.0);

  const model::SkeletonData& skeleton = gltf_asset->GetModelData().Skeleton();
  PairedSpan<const PreciseTransform, model::BoneData> local_transforms =
      skeleton.bones.Span<model::BoneData::Fields::kLocalTransform>();

  for (auto& pair : channel.blend_anim->blending_bone_targets) {
    BoneId bone = pair.first;
    const BlendingAnim::BlendingBoneTarget& blending_bone_target = pair.second;

    Trsf out_transform =
        blending_bone_target.out_id.has_value()
            ? out_transforms[blending_bone_target.out_id.value()]
            : Trsf(local_transforms[bone]);
    Trsf in_transform = blending_bone_target.in_id.has_value()
                            ? in_transforms[blending_bone_target.in_id.value()]
                            : Trsf(local_transforms[bone]);

    Trsf trs = lerpTransform(out_transform, in_transform, weight);
    gltf_scene->SetLocalTransformFromBone(bone, trs);
    if (NodeHandle node = gltf_scene->GetNodeFromBone(bone)) {
      PlaybackUpdatedEvent ev;
      ev.animation_index = channel.anim_playback.anim_index;
      GetView()
          .GetComponentManager()
          .GetComponentSystem<GltfAnimator>()
          .SendOrQueuePlaybackUpdatedEvents(ev, {node});
    }
  }

  blend_out_anim.anim->SanitizeT(blend_out_anim.looping, &blend_out_anim.t);

  if (channel.blend_anim->elapsed_seconds >=
      channel.blend_anim->blend_time_seconds) {
    channel.blend_anim.reset();
  }
}

absl::Status GltfAnimator::CanPlay(
    const PlayCommand& play_command, int32_t& out_anim_index,
    const animation::GltfAnimation*& out_anim) const {
  const GltfAsset* target_gltf_asset = GetGltfAsset().Get();
  if (!target_gltf_asset) {
    return absl::FailedPreconditionError(
        "Cannot play animation with no model.");
  }

  GltfAsset::AnimId anim_id;
  if (const std::string* name = play_command.name()) {
    anim_id = target_gltf_asset->GetAnimId(*name);
    if (!anim_id) {
      std::string msg = absl::StrFormat(
          "Cannot play animation named '%s' that doesn't exist on model.",
          name->c_str());
      return absl::InvalidArgumentError(msg);
    }
  } else {
    int index = play_command.index() ? *play_command.index() : 0;
    anim_id = GltfAsset::AnimId::At(index);
  }

  out_anim_index = static_cast<int32_t>(anim_id);
  out_anim = target_gltf_asset->GetGltfAnimData(anim_id);
  if (!out_anim) {
    std::string msg = absl::StrFormat(
        "Cannot play animation index [%d] on model that has %d "
        "animations.",
        out_anim_index, target_gltf_asset->GetAnimNames().size());
    return absl::OutOfRangeError(msg);
  }

  return absl::OkStatus();
}

void GltfAnimator::SetTargetGltfAsset(const AssetPtr<GltfAsset>& gltf_asset) {
  target_gltf_asset_ = gltf_asset;
  GetNode()->AddComponent<GltfScene>(gltf_asset);
}

AssetPtr<GltfAsset> GltfAnimator::GetGltfAsset() const {
  if (target_gltf_asset_) {
    return target_gltf_asset_;
  }
  if (const auto model = GetTargetModel()) {
    return model->GetGltfAsset();
  }
  return {};
}

const ComponentHandle<GltfRenderer> GltfAnimator::GetTargetModel() const {
  return GetNode()->GetComponent<GltfRenderer>();
}

void GltfAnimator::ConstrainAnimationTime(ElapsedTimeProvider provider,
                                          PlaybackChannelId channel_id) {
  // TODO: per-channel constrained time is blocked on this bug. See
  // the buganizer link for details.
#if BUG_308888791_IS_FIXED
  if (IsAllChannelsId(channel_id)) {
    for (auto channel_id : playback_channel_lookup_) {
      PlaybackChannel& channel = playback_channels_[channel_id.second];
      channel.elapsed_time_provider = provider;
    }
  } else if (auto channel = GetPlaybackChannel(channel_id)) {
    channel->elapsed_time_provider = provider;
  }
#else
  elapsed_time_provider_ = provider;
#endif
}

void GltfAnimator::SetSpeedMultiplier(float speed_multiplier,
                                      PlaybackChannelId channel_id) {
  if (IsAllChannelsId(channel_id)) {
    for (auto channel_id : playback_channel_lookup_) {
      PlaybackChannel& channel = playback_channels_[channel_id.second];
      channel.anim_playback.speed_multiplier = speed_multiplier;
    }
  } else if (auto channel = GetPlaybackChannel(channel_id)) {
    channel->anim_playback.speed_multiplier = speed_multiplier;
  }
}

float GltfAnimator::GetSpeedMultiplier(PlaybackChannelId channel_id) const {
  if (auto channel = GetPlaybackChannel(channel_id)) {
    return channel->anim_playback.speed_multiplier;
  }
  return 1.0f;
}

void GltfAnimator::SetLooping(bool looping, PlaybackChannelId channel_id) {
  if (IsAllChannelsId(channel_id)) {
    for (auto channel_id : playback_channel_lookup_) {
      PlaybackChannel& channel = playback_channels_[channel_id.second];
      channel.anim_playback.looping = looping;
    }
  } else if (PlaybackChannel* channel = GetPlaybackChannel(channel_id)) {
    channel->anim_playback.looping = looping;
  }
}

bool GltfAnimator::IsLooping(PlaybackChannelId channel_id) const {
  if (IsAllChannelsId(channel_id)) {
    for (auto channel_id : playback_channel_lookup_) {
      if (!playback_channels_[channel_id.second].anim_playback.looping) {
        return false;
      }
    }
    return true;
  } else if (const PlaybackChannel* channel = GetPlaybackChannel(channel_id)) {
    return channel->anim_playback.looping;
  }

  return false;
}

absl::Duration GltfAnimator::GetPlaybackTime(PlaybackChannelId channel_id) {
  if (auto channel = GetPlaybackChannel(channel_id)) {
    GltfAnimPlayback& playback = channel->anim_playback;
    return playback.t - playback.anim->FirstT();
  }
  return absl::ZeroDuration();
}

absl::Duration GltfAnimator::GetAnimationDuration(
    PlaybackChannelId channel_id) {
  if (auto channel = GetPlaybackChannel(channel_id)) {
    GltfAnimPlayback& playback = channel->anim_playback;
    return playback.anim->Duration();
  }
  return absl::ZeroDuration();
}

std::optional<int32_t> GltfAnimator::GetAnimationIndex(
    PlaybackChannelId channel_id) const {
  if (const PlaybackChannel* channel = GetPlaybackChannel(channel_id)) {
    return channel->anim_playback.anim_index;
  }
  return std::nullopt;
}

GltfAnimator::PlaybackChannel* GltfAnimator::GetPlaybackChannel(
    PlaybackChannelId channel_id) {
  if (!IsAllChannelsId(channel_id)) {
    auto iter = playback_channel_lookup_.find(channel_id);
    if (iter != playback_channel_lookup_.end()) {
      return &playback_channels_[iter->second];
    }
  }
  return nullptr;
}

const GltfAnimator::PlaybackChannel* GltfAnimator::GetPlaybackChannel(
    PlaybackChannelId channel_id) const {
  if (!IsAllChannelsId(channel_id)) {
    auto iter = playback_channel_lookup_.find(channel_id);
    if (iter != playback_channel_lookup_.end()) {
      return &playback_channels_[iter->second];
    }
  }
  return nullptr;
}

#if IMP_RUNTIME(DEV)
void GltfAnimator::DrawEditorUi() {
  AssetPtr<GltfAsset> asset = GetGltfAsset();
  if (!asset) {
    return;
  }

  if (asset->AnimationCount() == 0) {
    ImGui::Text("Asset has no associated animation");
    return;
  }

  ImGui::InputInt("Playback channel",
                  reinterpret_cast<int*>(&current_channel_id_));
  CurrentAnimation& current_animation =
      current_animations_[current_channel_id_];

  // Animation selection
  absl::Span<const std::string> anim_names = asset->GetAnimNames();
  if (ImGui::BeginCombo("Animation list",
                        anim_names[current_animation.index].c_str())) {
    for (size_t i = 0; i < asset->AnimationCount(); ++i) {
      bool selected = false;
      ImGui::Selectable(anim_names[i].c_str(), &selected);
      if (selected) {
        GltfAnimatorState::AnimOptions options;
        options.looping = current_animation.loop;
        options.speed_multiplier = current_animation.speed_multiplier;
        options.playback_channel = current_channel_id_;
        current_animation.index = i;
        current_animation.time = 0.0f;
        Play(anim_names[i], options);
      }
    }
    ImGui::EndCombo();
  }

  if (ImGui::Button("Set as starting animation")) {
    PlayCommand play_command;
    *play_command.mutable_name() = anim_names[current_animation.index];
    if (state_.starting_animation) {
      play_command.options = state_.starting_animation->options;
    } else {
      play_command.options = GltfAnimatorState::AnimOptions();
    }
    state_.starting_animation = play_command;
  }

  // Play pause stop and loop controls
  if (ImGui::Button("Play")) {
    Resume(anim_names[current_animation.index], current_animation);
  }

  ImGui::SameLine();
  if (ImGui::Button("Pause")) {
    // Don't overwrite current_animation.time if it's already paused.
    if (IsPlaying()) {
      current_animation.time =
          absl::ToDoubleSeconds(GetPlaybackTime(current_channel_id_));
      Stop(current_channel_id_);
    }
  }

  ImGui::SameLine();
  if (ImGui::Button("Stop")) {
    current_animation.time = 0.0f;
    Restart(current_channel_id_);
    Stop(current_channel_id_);
  }

  ImGui::SameLine();
  if (ImGui::Checkbox("Looping", &current_animation.loop)) {
    Resume(anim_names[current_animation.index], current_animation);
  }

  // Finer frame controls
  ImGui::Text(">>> = 1s, >> = 0.1s, > = 0.01s");
  std::vector<std::pair<std::string, float>> advance_times = {
      std::make_pair("<<<", -1.0f), std::make_pair("<<", -0.1f),
      std::make_pair("<", -0.01f),  std::make_pair(">", 0.01f),
      std::make_pair(">>", 0.1f),   std::make_pair(">>>", 1.0f)};
  bool first = true;
  float curr_time =
      IsPlaying() ? absl::ToDoubleSeconds(GetPlaybackTime(current_channel_id_))
                  : current_animation.time;
  const animation::GltfAnimation* anim = GetGltfAsset()->GetGltfAnimData(
      GltfAsset::AnimId::At(current_animation.index));
  float total_time = absl::ToDoubleSeconds(anim->Duration());
  for (auto [advance_str, advance_time] : advance_times) {
    if (!first) {
      ImGui::SameLine();
    }
    first = false;
    if (ImGui::Button(advance_str.c_str())) {
      current_animation.time =
          std::clamp(curr_time + advance_time, 0.0f, total_time);
      Resume(anim_names[current_animation.index], current_animation);
      Stop(current_channel_id_);
    }
  }

  // Seekbar and current time
  if (ImGui::SliderFloat("Playback Seeker", &curr_time, 0.0f, total_time,
                         "%.2f")) {
    current_animation.time = curr_time;
    Resume(anim_names[current_animation.index], current_animation);
    Stop(current_channel_id_);
  }
  ImGui::Text("Playback: %.02f / %.02f s", curr_time, total_time);

  // Set playback speed of animation
  if (ImGui::BeginCombo(
          "Playback Speed",
          absl::StrFormat("%.2fx", current_animation.speed_multiplier)
              .c_str())) {
    for (int i = 1; i <= 8; ++i) {
      float speed = 0.25f * i;
      if (ImGui::Selectable(absl::StrFormat("%.2fx", speed).c_str())) {
        current_animation.speed_multiplier = speed;
      }
    }
    ImGui::EndCombo();
    Resume(anim_names[current_animation.index], current_animation);
  }
}

void GltfAnimator::Resume(absl::string_view anim_name,
                          const CurrentAnimation& current_animation) {
  GltfAnimatorState::AnimOptions options;
  options.looping = current_animation.loop;
  options.speed_multiplier = current_animation.speed_multiplier;
  options.start_time_seconds = current_animation.time;
  options.playback_channel = current_channel_id_;
  Play(anim_name, options);
}
#endif  // IMP_RUNTIME(DEV)

}  // namespace imp
