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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_PROPERTY_ANIMATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_PROPERTY_ANIMATOR_H_

#include <optional>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "absl/types/variant.h"
#include "core/animation/curve.h"
#include "core/animation/property_animator_state.proto.imp.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/robin_set.h"
#include "core/math/math.proto.imp.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/isf_info.h"
#include "core/view/framework/animation/animation.proto.imp.h"
#include "core/view/framework/animation/animation_frame.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

class PropertyAnimator;

// A PropertyAnimation provides playback control for a dynamic animation based
// on an AnimationSampler that specifies times, values, an interpolation
// mode, and an associated lambda that defines an operation to perform on every
// frame update.
// PropertyAnimations are defined and returned by the PropertyAnimator
// component. PropertyAnimator::AddAnimation returns a
// std::unique_ptr<PropertyAnimation>. When the unique pointer goes out of
// scope, animation playback ends.
class PropertyAnimation {
 public:
  // Indicates that animation playback has reached the end, or the animation has
  // been stopped.
  // Sent to the node that the originating PropertyAnimator is attached to.
  // This event will NOT bubble.
  struct PlaybackEndedEvent : public Event {};

  // Options for controlling animation playback.
  struct PlaybackOptions {
    bool looping = false;
  };

  ~PropertyAnimation();
  // Starts playback of the animation lambda, if it is not already playing. If
  // the PropertyAnimator is disabled, playback does not start until
  // PropertyAnimator is reenabled.
  void Play();
  // Starts playback of the animation lambda, and returns a Future that resolves
  // when the animation ends.
  Future<absl::Status> PlayAsync();
  // Sets the animation progress to and explicitly evaluates the animation
  // lambda at the given time before starting playback.
  void PlayFrom(absl::Duration t);
  // Sets the animation progress to and explicitly evaluates the animation
  // lambda at the given time before starting playback. Initialize a future
  // object to represent the playback status.
  Future<absl::Status> PlayFromAsync(absl::Duration t);
  // Pauses playback of the animation lambda.
  void Pause();
  // Stops playback of the animation and resets the animation to initial state.
  void Stop();
  // Resets animation progress and starts playback from the start.
  // If the PropertyAnimator is disabled, playback does not start until
  // PropertyAnimator is reenabled.
  void Restart();
  // Sets PlaybackOptions for the animation for all subsequent playbacks.
  void SetPlaybackOptions(const PlaybackOptions& options);
  // Gets the current PlaybackOptions.
  const PlaybackOptions& GetPlaybackOptions() const;
  // Returns true if the PropertyAnimation is currently animating.
  bool IsPlaying() const;
  // Returns true if the PropertyAnimation is updatable, otherwise returns false
  // and additional calls to Play/Pause/Restart will not animate.
  // PropertyAnimation is playable if it has a valid PropertyAnimator and
  // RemoveAnimation has not been called.
  bool IsPlayable() const;
  // Returns the currently elapsed animation time, within the original duration.
  absl::Duration GetPlaybackTime() const;
  // Returns the total original duration for the animation. This corresponds to
  // the last time value in the AnimationSampler used to create the
  // PropertyAnimation.
  absl::Duration GetDuration() const;

 private:
  // Curve variants for the valid animation interpolation modes for property
  // animations.
  template <typename T>
  using CurveVariant = absl::variant<
      animation::StepCurve<T>, animation::LinearCurve<T>,
      animation::EaseInSineCurve<T>, animation::EaseOutSineCurve<T>,
      animation::EaseInOutSineCurve<T>, animation::EaseInQuadCurve<T>,
      animation::EaseOutQuadCurve<T>, animation::EaseInOutQuadCurve<T>,
      animation::EaseInCubicCurve<T>, animation::EaseOutCubicCurve<T>,
      animation::EaseInOutCubicCurve<T>, animation::EaseInBackCurve<T>,
      animation::EaseOutBackCurve<T>, animation::EaseInOutBackCurve<T>>;

  using PropertyCurveVariant =
      std::variant<CurveVariant<float>, CurveVariant<float2>,
                   CurveVariant<float3>, CurveVariant<float4>,
                   CurveVariant<quatf>>;
  using FramesVariant = std::variant<std::vector<animation::Frame<float>>,
                                     std::vector<animation::Frame<float2>>,
                                     std::vector<animation::Frame<float3>>,
                                     std::vector<animation::Frame<float4>>,
                                     std::vector<animation::Frame<quatf>>>;
  using ValueVariant = std::variant<float, float2, float3, float4, quatf>;

  using PropertyAnimationCursor = animation::BaseCurve::Cursor;

  struct AnimationData {
    explicit AnimationData(std::vector<float> times, FramesVariant frames,
                           PropertyCurveVariant curve)
        : times(std::move(times)),
          frames(std::move(frames)),
          curve(std::move(curve)) {}
    std::vector<float> times;
    FramesVariant frames;
    PropertyCurveVariant curve;
  };

  ComponentHandle<PropertyAnimator> animator_;
  const std::vector<float> times_;
  const FramesVariant frames_;
  const PropertyCurveVariant property_curve_;
  const Invocable<void(ValueVariant value)> animate_fn_;
  PlaybackOptions playback_options_{};
  PropertyAnimationCursor cursor_{};
  absl::Duration playback_time_ = absl::ZeroDuration();
  bool try_play_on_update_ = false;
  bool last_playback_ended_ = false;
  std::optional<WeakFuture<absl::Status>> playback_future_{std::nullopt};

  void Update(absl::Duration delta_time);
  ValueVariant Evaluate(float t);
  void Reset();
  void SendPlaybackEndedEvent();

  template <typename Fn>
  static absl::StatusOr<std::unique_ptr<PropertyAnimation>> Create(
      NodeHandle animator_node, const AnimationSampler* sampler, Fn animate_fn);

  PropertyAnimation(ComponentHandle<PropertyAnimator> animator,
                    AnimationData data,
                    Invocable<void(ValueVariant value)> animate_fn);

  PropertyAnimation(const PropertyAnimation&) = delete;
  PropertyAnimation& operator=(const PropertyAnimation&) = delete;

  template <typename Curve>
  static absl::StatusOr<Curve> CreateCurve(const std::vector<float>* times,
                                           const FramesVariant* frames);

  static absl::StatusOr<AnimationData> CreateAnimationData(
      const AnimationSampler* sampler);
  static absl::StatusOr<PropertyAnimation::PropertyCurveVariant>
  CreatePropertyCurve(InterpolationMode interpolation_mode,
                      const std::vector<float>* times,
                      const FramesVariant* frames);
  static absl::StatusOr<FramesVariant> GetAnimationFrames(
      const AnimationValues* sampler);

  // Helper to extract the arg type from animate_fn.
  template <typename Fn, typename Arg, typename Ret>
  static Arg GetAnimateFnArg(Ret (Fn::*)(Arg) const);
  template <typename Fn, typename Arg, typename Ret>
  static Arg GetAnimateFnArg(Ret (Fn::*)(Arg));

  friend class PropertyAnimator;
};

// Allows adding dynamic animations to a scene that can be applied to any
// property through an animate_fn lambda that is called with interpolated
// AnimationSampler values for the current playback time. Playback control is
// on the returned PropertyAnimation object. If the PropertyAnimator component
// that a given PropertyAnimation is added to is disabled, then playback of the
// PropertyAnimation will not resume until the PropertyAnimator is reenabled.
class PropertyAnimator : public Component {
 public:
  // Automatically updates all PropertyAnimations associated with the current
  // node.
  void Update(const FrameTime& frame_time);

  // Allows creating dynamic animations by connecting interpolated animation
  // times and values from the specified AnimationSampler to a given animate_fn.
  // This returns a std::unique_ptr<PropertyAnimation> that can be used to
  // control animation playback while it is in scope. animate_fn is the function
  // that is called in order to animate, and will be called on every frame
  // while the animation is playing. The parameter type of Fn should match the
  // AnimationValueType specified in the AnimationSampler proto. Animation
  // playback does not start immediately; call Play() on the returned
  // PropertyAnimation in order to start.
  template <typename Fn>
  absl::StatusOr<std::unique_ptr<PropertyAnimation>> AddAnimation(
      const AnimationSampler* sampler, Fn animate_fn);

  // Same as above, but uses name of an AnimationSampler, if it was added in the
  // .isf file via PropertyAnimatorState, to connect an animate_fn lambda to the
  // matching sampler's animation values. Multiple animate_fns can use the same
  // AnimationSampler.
  template <typename Fn>
  absl::StatusOr<std::unique_ptr<PropertyAnimation>> AddAnimation(
      absl::string_view sampler_name, Fn animate_fn);

  // Returns true if PropertyAnimation is in the update list.
  bool IsPlayable(const PropertyAnimation* animation) const;
  // Removes PropertyAnimation from the update list.
  void RemoveAnimation(PropertyAnimation* animation);

 private:
  PropertyAnimatorState state_;
  RobinSet<PropertyAnimation*> animations_;

 public:
  using IsfInfo = IsfInfo<&PropertyAnimator::state_>;
};

template <typename Curve>
absl::StatusOr<Curve> PropertyAnimation::CreateCurve(
    const std::vector<float>* times, const FramesVariant* frames) {
  // Asserts that AnimationSampler's float time values can be safely converted
  // to Curve::FrameTime's float struct.
  static_assert(sizeof(typename Curve::FrameTime) == sizeof(float));
  auto time_values = typename Curve::TimeSpan(absl::MakeSpan(
      reinterpret_cast<const typename Curve::FrameTime*>(times->data()),
      times->size()));

  typename Curve::ValueSpan curve_values;
  MP_RETURN_IF_ERROR(absl::visit(
      [&curve_values](auto&& frame_vector) -> absl::Status {
        using ValueType =
            typename std::decay_t<decltype(frame_vector[0])>::ValueType;
        if (!std::is_same_v<ValueType, typename Curve::ValueType>) {
          return absl::InternalError(
              "Animation frame type does not match requested curve type");
        }
        auto value_span =
            absl::MakeSpan(reinterpret_cast<const typename Curve::FrameValue*>(
                               frame_vector.data()),
                           frame_vector.size());
        curve_values = typename Curve::ValueSpan(value_span);
        return absl::OkStatus();
      },
      *frames));

  return Curve::Create(time_values, curve_values);
}

template <typename Fn>
absl::StatusOr<std::unique_ptr<PropertyAnimation>> PropertyAnimation::Create(
    NodeHandle animator_node, const AnimationSampler* sampler, Fn animate_fn) {
  using FnType = typename std::remove_reference<Fn>::type;
  using AnimateFnArgType = decltype(GetAnimateFnArg(&FnType::operator()));

  static_assert(
      kIsAnyOf<AnimateFnArgType, float, float2, float3, float4, quatf>,
      "Lambda function has an invalid parameter type.");
  MP_RETURN_IF_ERROR(absl::visit(
      [](auto&& values_variant) -> absl::Status {
        using ArrayType = std::decay_t<decltype(values_variant)>;
        if constexpr (std::is_same_v<ArrayType, std::monostate>) {
          return absl::FailedPreconditionError(
              "Missing AnimationSampler values_array");
        } else {
          using ValueType = std::decay_t<decltype(values_variant.values[0])>;
          if (!std::is_same_v<ValueType, AnimateFnArgType>) {
            return absl::InvalidArgumentError(
                "AnimationSampler values and animate_fn parameter must have "
                "same type");
          }
        }
        return absl::OkStatus();
      },
      sampler->values_array.type));

  MP_ASSIGN_OR_RETURN(AnimationData data, CreateAnimationData(sampler));

  Invocable<void(ValueVariant)> invocable =
      [fn = std::forward<Fn>(animate_fn)](ValueVariant val) mutable {
        AnimateFnArgType arg = std::get<AnimateFnArgType>(val);
        fn(arg);
      };
  return absl::WrapUnique<PropertyAnimation>(
      new PropertyAnimation(animator_node->GetComponent<PropertyAnimator>(),
                            std::move(data), std::move(invocable)));
}

template <typename Fn>
absl::StatusOr<std::unique_ptr<PropertyAnimation>>
PropertyAnimator::AddAnimation(absl::string_view sampler_name, Fn animate_fn) {
  auto name_to_sampler = state_.samplers.find(std::string(sampler_name));
  if (name_to_sampler == state_.samplers.end()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "AnimationSampler with name '%s' not found", sampler_name));
  }
  const AnimationSampler* sampler = &name_to_sampler->second;
  return AddAnimation(sampler, std::forward<Fn>(animate_fn));
}

template <typename Fn>
absl::StatusOr<std::unique_ptr<PropertyAnimation>>
PropertyAnimator::AddAnimation(const AnimationSampler* sampler, Fn animate_fn) {
  absl::StatusOr<std::unique_ptr<PropertyAnimation>> animation =
      PropertyAnimation::Create(GetNode(), sampler,
                                std::forward<Fn>(animate_fn));
  if (animation.ok()) {
    animations_.emplace(animation->get());
  }
  return animation;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_PROPERTY_ANIMATOR_H_
