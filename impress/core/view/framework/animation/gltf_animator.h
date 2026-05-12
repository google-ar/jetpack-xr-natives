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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ANIMATION_GLTF_ANIMATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ANIMATION_GLTF_ANIMATOR_H_

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "core/animation/gltf_animation.h"
#include "core/animation/texture_transform_animation.h"
#include "core/assets/asset_ptr.h"
#include "core/common/robin_map.h"
#include "core/config.h"
#include "core/material_library/material_param_value.h"
#include "core/math/transform.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/component_traits.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node.h"
#include "core/ncsb/system.h"
#include "core/ncsb/update_phase.h"
#include "core/view/base_view.h"
#include "core/view/framework/animation/animator_events.proto.imp.h"
#include "core/view/framework/animation/gltf_animator_state.proto.imp.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/utils/frame_time.h"

namespace imp {
// Plays animations from GltfAssets. By default, it uses the GltfAsset from
// and updates the transforms for the GltfRenderer on this component's node.
// However, it can also play GltfAssets directly, from which it will use the
// bones, but without any renderables. It will send a PlaybackUpdatedEvent on
// every frame to all the targeted nodes in the animation, which need to be
// created by GetOrCreateNode(). It can also SetPaths() for individual nodes
// without GltfAssets.

// Each playback channel plays one animation at a time per component, or two for
// brief periods when transitioning from one animation to another (see
// PlayCommand::AnimOptions::blend_time_seconds). Multiple animations can be
// played at the same time by playing them on different channels *provided that
// the sets of bones affected by all channels are disjoint*. If multiple
// simultaneously-playing channels attempt to drive the same bone, the behavior
// is undefined. (In practice what happens is one of the channels will "win"
// over the others and drive the contested bone, but there are no guarantees on
// which channel that will be.)
//
// TODO: Support PlayAnim overload that takes dynamically created
// animation from GltfAnimation::Builder.
class GltfAnimator : public Component {
 public:
  using ElapsedTimeProvider = std::function<absl::StatusOr<absl::Duration>()>;
  using Options = GltfAnimatorState::AnimOptions;
  using PlayCommand = GltfAnimatorState::PlayCommand;
  using PlaybackChannelId = GltfAnimatorState::PlaybackChannelId;

  static constexpr PlaybackChannelId kDefaultChannelId{.id = 0};

  // Any arbitrary integer is a valid playback channel ID with the one
  // exception of this special ID `kAllChannels`. This ID is a valid argument to
  // Stop(), Restart(), SetSpeedMultiplier(), IsLooping(), SetLooping() and
  // ConstrainAnimationTime() only.
  // For all other functions that accept a PlaybackChannelId, including any that
  // do so indirectly via GltfAnimatorState::AnimOptions such as Play(), this ID
  // may not be used.
  static constexpr PlaybackChannelId kAllChannels{.id = -1};

  class System : public ComponentSystem<GltfAnimator> {
   public:
    explicit System(BaseView* view);

    void PreComponentsUpdated(const FrameTime& frame_time) override;

    void SendOrQueuePlaybackUpdatedEvents(PlaybackUpdatedEvent ev,
                                          std::vector<NodeHandle> targets);
    void SendOrQueuePlaybackLoopedEvent(PlaybackLoopedEvent ev,
                                        NodeHandle target);
    void SendOrQueuePlaybackEndedEvent(PlaybackEndedEvent ev,
                                       NodeHandle target);
    void SendOrQueuePlaybackPausedEvent(PlaybackPausedEvent ev,
                                        NodeHandle target);
    void SendOrQueuePlaybackResumedEvent(PlaybackResumedEvent ev,
                                         NodeHandle target);
    void SendOrQueuePlaybackActiveStatusChangedEvent(
        PlaybackActiveStatusChangedEvent ev, NodeHandle target);

   private:
    void DispatchQueuedEvents();

    // We pause sending events during UpdatePositions and then send them after
    // we finish updating positions. This is because we don't update world
    // transforms until after we've finished updating positions for performance
    // reasons, so we must wait to send events until the world transforms are
    // calculated.
    bool are_events_paused_ = false;
    using QueuedPlaybackUpdatedEvent =
        std::pair<PlaybackUpdatedEvent, std::vector<NodeHandle>>;
    std::vector<QueuedPlaybackUpdatedEvent> playback_updated_event_queue_;
    using QueuedPlaybackLoopedEvent =
        std::pair<PlaybackLoopedEvent, NodeHandle>;
    std::vector<QueuedPlaybackLoopedEvent> playback_looped_event_queue_;
    using QueuedPlaybackEndedEvent = std::pair<PlaybackEndedEvent, NodeHandle>;
    std::vector<QueuedPlaybackEndedEvent> playback_ended_event_queue_;
    using QueuedPlaybackPausedEvent =
        std::pair<PlaybackPausedEvent, NodeHandle>;
    std::vector<QueuedPlaybackPausedEvent> playback_paused_event_queue_;
    using QueuedPlaybackResumedEvent =
        std::pair<PlaybackResumedEvent, NodeHandle>;
    std::vector<QueuedPlaybackResumedEvent> playback_resumed_event_queue_;
    using QueuedPlaybackActiveStatusChangedEvent =
        std::pair<PlaybackActiveStatusChangedEvent, NodeHandle>;
    std::vector<QueuedPlaybackActiveStatusChangedEvent>
        playback_active_status_changed_event_queue_;
  };

  GltfAnimator();

  // Default, does not start playing automatically, unless there is a state
  // with a PlayCommand set.
  void Setup();

  // Create a component with |gltf_asset|, does not start playing
  // automatically. |gltf_asset| must outlast the lifetime of this component.
  void Setup(const AssetPtr<GltfAsset>& gltf_asset);

  // Start playing the specified animation automatically with the options passed
  // in. It plays from |gltf_asset| if provided, or the GltfRenderer of this
  // node otherwise. If there is no index or name, plays the first animation in
  // the model's list of animations. |gltf_asset| must outlast the lifetime of
  // this component.
  void Setup(const PlayCommand& play_command,
             const AssetPtr<GltfAsset>& gltf_asset = AssetPtr<GltfAsset>());

  void OnActiveStatusChanged(bool is_active);

  // Checks if a PlayCommand can be executed by checking if the animation name
  // or index are correct.
  absl::Status CanPlay(const PlayCommand& play_command) const;

  // Plays the specified animation, from |gltf_asset| if not null, or last
  // specified GltfAsset otherwise. If there is no index or name, plays the
  // first animation in the model's list of animations. |gltf_asset| must
  // outlast the lifetime of this component.
  //
  // If this component is inactive, it will not actually play until it is
  // active.
  //
  // Note: We only support blending between animations affecting the same list
  // of bones.
  void Play(const PlayCommand& play_command,
            const AssetPtr<GltfAsset>& gltf_asset = AssetPtr<GltfAsset>());

  void Play(absl::string_view anim_name,
            GltfAnimatorState::AnimOptions options =
                GltfAnimatorState::AnimOptions());

  // Same as Play, but instead of fataling for errors such as no asset, wrong
  // animation name, or index out of bounds, it will return a status with the
  // error.
  absl::Status PlaySafely(
      const PlayCommand& play_command,
      const AssetPtr<GltfAsset>& gltf_asset = AssetPtr<GltfAsset>());

  // Returns the node with this name from the GltfAsset, creating it if it
  // didn't exist yet. This allows listening to PlaybackUpdatedEvents that are
  // sent to all targeted nodes during the animation.
  NodeHandle GetOrCreateNode(absl::string_view name);

  // Returns the root node under which all bones are. Children can be added to
  // this node and they will be available for SetPaths, if their name is not
  // used by a previous node.
  NodeHandle GetRoot() const;

  // Stops the currently playing animation on one playback channel, or all
  // animations on all playback channels if passed the special channel ID value
  // `kAllChannels`.
  void Stop(PlaybackChannelId channel_id = kAllChannels);

  // Restarts the currently playing animation on one playback channel, or all
  // animations on all playback channels if passed the special channel ID value
  // `kAllChannels`.
  // Restarting an animation amounts to resetting its playback time to
  // its configured start time, without modifying any of the rest of the
  // playback options. An animation must already be playing in order to be
  // restarted; a channel on which Stop() has been previously called is
  // effectively cleared and cannot be "Restart()"ed. To restart in that case
  // you must call Play() again.
  void Restart(PlaybackChannelId channel_id = kAllChannels);

  bool IsPlaying(
      std::optional<PlaybackChannelId> channel_id = std::nullopt) const;

  // Attempt to constrain the animation time to the provided time by varying the
  // animation speed. This allows syncing of the animation time to the playback
  // times of other systems. Note that this does a best effort attempt to align
  // the animation time to the provided one, and will take some time before they
  // fully sync up.
  // Note: Currently this only supports speed multiplier of 1.
  // TODO: Rename to SetElapsedTimeProvider and define / implement
  // how it interacts with animation playback events, in particular looping.
  void ConstrainAnimationTime(ElapsedTimeProvider provider,
                              PlaybackChannelId channel_id = kDefaultChannelId);

  // Sets the speed multiplier for the given channel if an animation is playing
  // on it.
  //
  // Note: This is expensive and does not work well with values more than a
  // couple magnitudes from 1.0.
  void SetSpeedMultiplier(float speed_multiplier,
                          PlaybackChannelId channel_id = kDefaultChannelId);

  // Returns the current animation speed multiplier. Default is 1.0.
  float GetSpeedMultiplier(
      PlaybackChannelId channel_id = kDefaultChannelId) const;

  // Sets if looping is enabled for the given channel if an animation is
  // playing on it.
  void SetLooping(bool looping,
                  PlaybackChannelId channel_id = kDefaultChannelId);

  // Returns true if there is an animation playing on the given channel and it
  // is looping.
  bool IsLooping(PlaybackChannelId channel_id = kDefaultChannelId) const;

  // Pauses or resumes the animation on the given channel (if one is playing),
  // or all animations on all channels if kAllChannels is passed.
  void SetPaused(bool paused, PlaybackChannelId channel_id = kDefaultChannelId);

  // Returns true if there is an animation playing on the given channel and it
  // is paused.
  bool IsPaused(PlaybackChannelId channel_id = kDefaultChannelId) const;

  // Set the current playback time for the given channel if an animation is
  // playing on it.
  void SetPlaybackTime(absl::Duration playback_time,
                       PlaybackChannelId channel_id = kDefaultChannelId);

  // Returns the current animation playback time, within the original duration.
  // Options::speed_multiplier affects how this updates compared to real time.
  absl::Duration GetPlaybackTime(
      PlaybackChannelId channel_id = kDefaultChannelId);

  // Returns the current elapsed animation time, within the bounds defined
  // Options::start_time and Options::end_time parameters used in the
  // PlayCommand used to began the animation playback.
  // Options::speed_multiplier affects how this updates compared to real time.
  absl::Duration GetElapsedTime(
      PlaybackChannelId channel_id = kDefaultChannelId);

  // Returns the starting time of an animation that is currently running (in
  // terms of the animation's key frame time).
  absl::Duration GetStartTime(
      PlaybackChannelId channel_id = kDefaultChannelId) const;

  // Returns the ending time of an animation that is currently running (in terms
  // of the animation's key frame time).
  absl::Duration GetEndTime(
      PlaybackChannelId channel_id = kDefaultChannelId) const;

  // Get the total duration for the current animation playback. Accounts for the
  // configured start time and end time if set.
  absl::Duration GetPlaybackDuration(
      PlaybackChannelId channel_id = kDefaultChannelId);

  // Get the total original duration for the current animation.
  absl::Duration GetAnimationDuration(
      PlaybackChannelId channel_id = kDefaultChannelId);

  // Get the index of the animation in the glTF asset on the given channel.
  // Returns nullopt if there is no animation on the given channel.
  std::optional<int32_t> GetAnimationIndex(
      PlaybackChannelId channel_id = kDefaultChannelId) const;

  // Returns the glTF Asset that contains the glTF animations this animator can
  // play. This is either the GltfAsset owned by the GltfRenderer on the same
  // node, or the GltfAsset passed into GltfAnimator::Setup if playing an
  // animation without rendering the glTF.
  AssetPtr<GltfAsset> GetGltfAsset() const;

 private:
  using BoneId = GltfScene::BoneId;
  // glTF animation data accessors can only be single-precision floats, so no
  // need for us to handle PreciseTransforms here.
  // https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#accessor-data-types
  using Trsf = Transform<float>;
  using MaterialTarget = animation::GltfAnimation::MaterialTarget;
  using TextureTransformParameters = std::vector<
      animation::TextureTransformAnimation::TextureTransformParameter>;

  struct GltfAnimPlayback {
    // Index of the animation within the glTF asset.
    // int32 is used to match index type from the PlayCommand struct.
    int32_t anim_index = -1;
    const animation::GltfAnimation* anim;
    animation::GltfAnimation::Cursor cursor;
    absl::Duration t;
    absl::Duration start_time;
    std::optional<absl::Duration> end_time;
    float speed_multiplier;
    uint32_t looping : 1;
    uint32_t paused : 1;
    uint32_t loop_count : 30;
  };

  // Supports blending between previous animation and a new one.
  struct BlendingAnim {
    GltfAnimPlayback blend_out_anim;
    struct BlendingBoneTarget {
      // If set, use the trs from the out animation, otherwise use bind pose.
      absl::optional<animation::GltfAnimation::BoneTargetId> out_id;
      // If set, use the trs from the in animation, otherwise use bind pose.
      absl::optional<animation::GltfAnimation::BoneTargetId> in_id;
    };
    RobinMap<BoneId, BlendingBoneTarget> blending_bone_targets;
    absl::Duration blend_time_seconds;
    absl::Duration elapsed_seconds;
  };

  struct PlaybackChannel {
    GltfAnimPlayback anim_playback;
    absl::optional<BlendingAnim> blend_anim;
    bool active;
    bool persist;
  };

  const ComponentHandle<GltfRenderer> GetTargetModel() const;

  void SetTargetGltfAsset(const AssetPtr<GltfAsset>& gltf_asset);

  void PlayAnim(int32_t anim_index, const animation::GltfAnimation* anim,
                const Options& options);
  void AdvanceAnimationPlayback(absl::Duration delta_time);
  bool AdvanceAnimationPlayback(PlaybackChannel& channel,
                                absl::Duration delta_time,
                                ComponentHandle<GltfScene> gltf_scene);
  absl::Duration GetNextPlaybackTime(absl::Duration frame_time_delta,
                                     GltfAnimPlayback& animation_playback);
  void AdvanceBlendAnimation(PlaybackChannel& channel,
                             absl::Duration delta_time);
  absl::Status ApplyMaterialAnimation(PlaybackChannel& channel);
  absl::Status ApplyTextureTransformAnimation(
      TextureTransformParameters& texture_transform_parameters,
      GenericMaterial* material) const;
  void ApplyLightPunctualAnimation(PlaybackChannel& channel);
  absl::Status CanPlay(const PlayCommand& play_command, int32_t& out_anim_index,
                       const animation::GltfAnimation*& out_anim) const;
  PlaybackChannel* GetPlaybackChannel(PlaybackChannelId channel_id);
  const PlaybackChannel* GetPlaybackChannel(PlaybackChannelId channel_id) const;

  bool IsAllChannelsId(PlaybackChannelId channel_id) const {
    return channel_id.id == kAllChannels.id;
  }

  struct PlaybackChannelIdComparator {
    bool operator()(const PlaybackChannelId& a,
                    const PlaybackChannelId& b) const {
      return a.id < b.id;
    }
  };
  using PlaybackChannelLookup =
      std::map<PlaybackChannelId, int, PlaybackChannelIdComparator>;

  AssetPtr<GltfAsset> target_gltf_asset_;
  PlaybackChannelLookup playback_channel_lookup_;
  std::vector<PlaybackChannel> playback_channels_;
  ElapsedTimeProvider elapsed_time_provider_;

  GltfAnimatorState state_;

 public:
  using IsfInfo = IsfInfo<&GltfAnimator::state_, IsfDependencies<GltfRenderer>>;
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kStart;

#if IMP_RUNTIME(DEV)
  void DrawEditorUi();

 private:
  // Used to store the last played animation and time (if paused)
  PlaybackChannelId current_channel_id_ = kDefaultChannelId;
  struct CurrentAnimation {
    size_t index = 0;
    float start_time = 0.0f;
    float time = 0.0f;
    float end_time = 0.0f;
    bool loop = true;
    float speed_multiplier = 1.0f;
  };
  std::map<PlaybackChannelId, CurrentAnimation, PlaybackChannelIdComparator>
      current_animations_;

  void PlayAnimationFromEditorUi(absl::string_view anim_name,
                                 const CurrentAnimation& current_animation);

  void SetAnimationPlaybackTimeInEditor(absl::string_view anim_name,
                                        CurrentAnimation& current_animation);

  float GetAnimationFirstT(const CurrentAnimation& current_animation);

  float GetAnimationLastT(const CurrentAnimation& current_animation);
#endif  // IMP_RUNTIME(DEV)
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ANIMATION_GLTF_ANIMATOR_H_
