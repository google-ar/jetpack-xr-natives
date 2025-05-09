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

#include "core/animation/property_animator.h"

#include <algorithm>
#include <optional>
#include <type_traits>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "core/animation/curve.h"
#include "core/animation/curve_values.h"
#include "core/async/future.h"
#include "core/view/framework/animation/animation.proto.imp.h"
#include "core/view/framework/animation/animation_frame.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace {
constexpr absl::string_view kPlayAsyncLoopingIsNotSupportedMessage =
    "Looping animation is not supported for PlayAsync";
}

PropertyAnimation::PropertyAnimation(
    ComponentHandle<PropertyAnimator> animator, AnimationData data,
    Invocable<void(ValueVariant value)> animate_fn)
    : animator_(animator),
      times_(std::move(data.times)),
      frames_(std::move(data.frames)),
      property_curve_(std::move(data.curve)),
      animate_fn_(std::move(animate_fn)) {}

PropertyAnimation::~PropertyAnimation() {
  if (animator_) {
    animator_->RemoveAnimation(this);
  }

  // Return an aborted status to differentiate between animation stopped and
  // animation destroyed, such that we can customize the behavior for situations
  // like app crashes.
  if (playback_future_.has_value()) {
    absl::optional<Future<absl::Status>> playback_future =
        playback_future_->Lock();
    if (playback_future) {
      playback_future->Return(absl::AbortedError(
          "PropertyAnimation was deleted before the animation completed"));
    }
  }
}

absl::StatusOr<PropertyAnimation::AnimationData>
PropertyAnimation::CreateAnimationData(const AnimationSampler* sampler) {
  MP_ASSIGN_OR_RETURN(FramesVariant frames,
                   GetAnimationFrames(&sampler->values_array));
  // Make a copy of the times vector to store with each PropertyAnimation
  // because curves rely on an absl::Span. This removes the need to keep an
  // AnimationSampler in scope after adding an animation.
  std::vector<float> times = sampler->times_seconds;
  MP_ASSIGN_OR_RETURN(
      PropertyAnimation::PropertyCurveVariant curve,
      CreatePropertyCurve(sampler->interpolation, &times, &frames));
  return AnimationData(std::move(times), std::move(frames), std::move(curve));
}

absl::StatusOr<PropertyAnimation::PropertyCurveVariant>
PropertyAnimation::CreatePropertyCurve(InterpolationMode interpolation_mode,
                                       const std::vector<float>* times,
                                       const FramesVariant* frames) {
  absl::StatusOr<PropertyAnimation::PropertyCurveVariant> curve_or =
      absl::visit(
          [interpolation_mode, &times, &frames](auto&& frame)
              -> absl::StatusOr<PropertyAnimation::PropertyCurveVariant> {
            using ValueType =
                typename std::decay_t<decltype(frame[0])>::ValueType;
            static_assert(
                kIsAnyOf<ValueType, float, float2, float3, float4, quatf>,
                "Provided AnimationSampler values are an invalid type.");
            switch (interpolation_mode) {
              case INTERPOLATION_DEFAULT:
              case INTERPOLATION_LINEAR: {
                return CreateCurve<animation::LinearCurve<ValueType>>(times,
                                                                      frames);
              }
              case INTERPOLATION_STEP: {
                return CreateCurve<animation::StepCurve<ValueType>>(times,
                                                                    frames);
              }
              case INTERPOLATION_EASE_IN_SINE: {
                return CreateCurve<animation::EaseInSineCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_OUT_SINE: {
                return CreateCurve<animation::EaseOutSineCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_IN_OUT_SINE: {
                return CreateCurve<animation::EaseInOutSineCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_IN_QUAD: {
                return CreateCurve<animation::EaseInQuadCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_OUT_QUAD: {
                return CreateCurve<animation::EaseOutQuadCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_IN_OUT_QUAD: {
                return CreateCurve<animation::EaseInOutQuadCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_IN_CUBIC: {
                return CreateCurve<animation::EaseInCubicCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_OUT_CUBIC: {
                return CreateCurve<animation::EaseOutCubicCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_IN_OUT_CUBIC: {
                return CreateCurve<animation::EaseInOutCubicCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_IN_BACK: {
                return CreateCurve<animation::EaseInBackCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_OUT_BACK: {
                return CreateCurve<animation::EaseOutBackCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_EASE_IN_OUT_BACK: {
                return CreateCurve<animation::EaseInOutBackCurve<ValueType>>(
                    times, frames);
              }
              case INTERPOLATION_CUBIC: {
                return absl::UnimplementedError("Not yet implemented");
              }
            }
          },
          *frames);
  return curve_or;
}

absl::StatusOr<PropertyAnimation::FramesVariant>
PropertyAnimation::GetAnimationFrames(const AnimationValues* sampler_values) {
  return absl::visit(
      [](auto&& values_variant)
          -> absl::StatusOr<PropertyAnimation::FramesVariant> {
        using T = std::decay_t<decltype(values_variant)>;
        if constexpr (std::is_same_v<T, absl::monostate>) {
          return absl::FailedPreconditionError(
              "Missing AnimationSampler values_array");
        } else {
          return animation::ToFrame(values_variant.values);
        }
      },
      sampler_values->type);
}

void PropertyAnimation::Update(absl::Duration delta_time) {
  if (last_playback_ended_) {
    try_play_on_update_ = false;
  }
  if (!try_play_on_update_) return;

  float curr_time_seconds =
      std::min(absl::ToDoubleSeconds(playback_time_ + delta_time),
               absl::ToDoubleSeconds(GetDuration()));
  playback_time_ = absl::Seconds(curr_time_seconds);
  animate_fn_(Evaluate(curr_time_seconds));

  // If finished playing, stop playback and reset.
  if (playback_time_ == GetDuration()) {
    SendPlaybackEndedEvent();

    if (playback_options_.looping) {
      Restart();
    } else {
      last_playback_ended_ = true;
    }

    if (playback_future_.has_value()) {
      absl::optional<Future<absl::Status>> playback_future =
          playback_future_->Lock();
      if (playback_future) {
        playback_future->Return(absl::OkStatus());
      }
    }
  }
}

void PropertyAnimation::PlayFrom(absl::Duration t) {
  playback_time_ = t;
  animate_fn_(Evaluate(absl::ToDoubleSeconds(t)));
  Play();
}

Future<absl::Status> PropertyAnimation::PlayFromAsync(absl::Duration t) {
  if (playback_options_.looping) {
    playback_future_ = std::nullopt;
    return Future<absl::Status>(
        absl::FailedPreconditionError(kPlayAsyncLoopingIsNotSupportedMessage));
  }

  Future<absl::Status> playback_future;
  playback_future_ = playback_future;
  PlayFrom(t);

  return playback_future;
}

PropertyAnimation::ValueVariant PropertyAnimation::Evaluate(float t) {
  return absl::visit(
      [this, t](const auto& curve_variant) -> PropertyAnimation::ValueVariant {
        return absl::visit(
            [this, t](const auto& typed_curve_variant)
                -> PropertyAnimation::ValueVariant {
              return typed_curve_variant.Eval(t, &cursor_);
            },
            curve_variant);
      },
      property_curve_);
}

void PropertyAnimation::SetPlaybackOptions(const PlaybackOptions& options) {
  playback_options_ = options;
}

const PropertyAnimation::PlaybackOptions&
PropertyAnimation::GetPlaybackOptions() const {
  return playback_options_;
}

void PropertyAnimation::Play() {
  if (last_playback_ended_) {
    Reset();
  }
  try_play_on_update_ = true;
}

Future<absl::Status> PropertyAnimation::PlayAsync() {
  if (playback_options_.looping) {
    playback_future_ = std::nullopt;
    return Future<absl::Status>(
        absl::FailedPreconditionError(kPlayAsyncLoopingIsNotSupportedMessage));
  }

  Future<absl::Status> playback_future;
  playback_future_ = playback_future;
  Play();

  return playback_future;
}

void PropertyAnimation::Pause() { try_play_on_update_ = false; }

void PropertyAnimation::Stop() {
  SendPlaybackEndedEvent();
  Pause();
  Reset();

  if (playback_future_.has_value()) {
    absl::optional<Future<absl::Status>> playback_future =
        playback_future_->Lock();
    if (playback_future) {
      playback_future->Return(
          absl::CancelledError("Animation stopped before finished"));
    }
  }
}

void PropertyAnimation::Restart() {
  Reset();
  Play();
}

void PropertyAnimation::Reset() {
  cursor_ = {};
  playback_time_ = absl::ZeroDuration();
  last_playback_ended_ = false;
}

void PropertyAnimation::SendPlaybackEndedEvent() {
  PlaybackEndedEvent end{};
  end.SetPropagationMode(Event::PropagationMode::kNone);
  animator_->GetNode()->Send(end);
}

bool PropertyAnimation::IsPlayable() const {
  return animator_ && animator_->IsPlayable(this);
}

bool PropertyAnimation::IsPlaying() const {
  return IsPlayable() && animator_->IsActive() && try_play_on_update_;
}

absl::Duration PropertyAnimation::GetPlaybackTime() const {
  return playback_time_;
}

absl::Duration PropertyAnimation::GetDuration() const {
  return absl::visit(
      [](const auto& curve_variant) {
        return absl::visit(
            [](const auto& typed_curve_variant) {
              return absl::Seconds(typed_curve_variant.GetMaxT());
            },
            curve_variant);
      },
      property_curve_);
}

void PropertyAnimator::Update(const FrameTime& frame_time) {
  for (auto animation : animations_) {
    animation->Update(frame_time.GetDeltaTime());
  }
}

bool PropertyAnimator::IsPlayable(const PropertyAnimation* animation) const {
  return animations_.find(const_cast<PropertyAnimation*>(animation)) !=
         animations_.end();
}

void PropertyAnimator::RemoveAnimation(PropertyAnimation* animation) {
  animations_.erase(animation);
}

}  // namespace imp
