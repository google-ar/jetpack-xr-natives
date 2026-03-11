/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_RAMP_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_RAMP_H_

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <type_traits>

#include "core/common/log.h"
#include "absl/time/time.h"
#include "core/common/enum_flags.h"
#include "core/common/platform_helpers.h"

namespace svxr {

namespace svxr_details {

// Filament specialization
template <typename T>
auto Magnitude(T in) ->
    typename std::enable_if_t<std::is_floating_point_v<decltype(length(in))>,
                              float> {
  return length(in);
}

// Floating point specialization
template <typename T>
auto Magnitude(T in) ->
    typename std::enable_if_t<std::is_floating_point_v<decltype(std::abs(T{}))>,
                              float> {
  return std::abs(in);
}

// Positions snap to target values when their distance is below this threshold.
constexpr float TravelEpsilon() { return 1.0e-5f; }
// The smallest expressible duration (in seconds), to prevent divides by zero.
constexpr float DurationEpsilon() { return 1.0e-4f; }

}  // namespace svxr_details

// Ramp<T> manages a T that can be linearly interpolated to a target value.  The
// managed value is updated via Step() or Snap().  A Ramp must be setup with its
// initial value and, optionally, a method to determine the travel vector
// between two values.  The primary use case for the get_travel_fn method is to
// allow for e.g. heading values whose travel is computed as the  'short way'
// around a circle.
template <typename T>
class Ramp {
 public:
  Ramp() = default;
  using get_travel_fn = T (*)(T*, T*);
  explicit Ramp(
      T initial_value,
      get_travel_fn get_travel_fn = [](T* position, T* target) {
        return *target - *position;
      });
  void Setup(
      T initial_value,
      get_travel_fn get_travel_fn = [](T* position, T* target) {
        return *target - *position;
      });

  // Set an interpolation target and a duration for the entire interpolation.
  void SetTarget(T target, absl::Duration duration);
  void SetTargetWithUnitDuration(T target, absl::Duration unit_duration);

  // Returns true if the current value matches the target.
  bool Step(absl::Duration duration);

  // Immediately assume the target value.
  void Snap();

  T Get() const;
  T GetTarget() const;
  bool IsAtTarget() const;

 protected:
  enum class Flag : uint8_t {
    Initialized = (1 << 0),
    AtTarget = (1 << 1),
  };

  T position_ = {};
  T velocity_;
  T target_;
  get_travel_fn get_travel_fn_;
  imp::Flags<Flag> flags_;
};

template <typename T>
Ramp<T>::Ramp(T initial_value, get_travel_fn get_travel_fn) {
  Setup(initial_value, get_travel_fn);
}

template <typename T>
void Ramp<T>::Setup(T initial_value, get_travel_fn get_travel_fn) {
  get_travel_fn_ = get_travel_fn;
  SetTarget(initial_value, absl::ZeroDuration());
  Snap();
  flags_.Set(Flag::Initialized);
}

template <typename T>
void Ramp<T>::SetTarget(T target, absl::Duration duration) {
  if (flags_.Test(Flag::AtTarget) && target_ == target) {
    // No-op if we've already Stepped to this specific target value.
    return;
  }
  target_ = target;
  T travel = get_travel_fn_(&position_, &target_);
  auto dT = static_cast<float>(absl::ToDoubleSeconds(duration));
  velocity_ = travel / std::max(dT, svxr_details::DurationEpsilon());
  flags_.Set(Flag::AtTarget, false);
}

template <typename T>
void Ramp<T>::SetTargetWithUnitDuration(T target,
                                        absl::Duration unit_duration) {
  if (flags_.Test(Flag::AtTarget) && target_ == target) {
    // No-op if we've already Stepped to this specific target value.
    return;
  }
  target_ = target;
  T travel = get_travel_fn_(&position_, &target_);
  auto dT = static_cast<float>(absl::ToDoubleSeconds(unit_duration)) *
            svxr_details::Magnitude(travel);
  velocity_ = travel / std::max(dT, svxr_details::DurationEpsilon());
  flags_.Set(Flag::AtTarget, false);
}

// Returns true if we are at our target.
template <typename T>
bool Ramp<T>::Step(absl::Duration duration) {
  if (flags_.Test(Flag::AtTarget)) {
    return true;
  }
  if (!flags_.Test(Flag::Initialized)) {
    IMP_LOG(imp::FATAL) << "Tried to Step a ramp without calling Setup";
  }

  T remaining_travel = get_travel_fn_(&position_, &target_);
  auto dT = static_cast<float>(absl::ToDoubleSeconds(duration));
  T possible_travel = velocity_ * dT;
  if (float remaining_distance = svxr_details::Magnitude(remaining_travel);
      remaining_distance > svxr_details::TravelEpsilon() &&
      remaining_distance > svxr_details::Magnitude(possible_travel)) {
    // Not at target yet.
    position_ += possible_travel;
    return false;
  }

  Snap();
  return true;
}

template <typename T>
void Ramp<T>::Snap() {
  position_ = target_;
  velocity_ = T{0};
  flags_.Set(Flag::AtTarget);
}

template <typename T>
T Ramp<T>::Get() const {
  return position_;
}

template <typename T>
T Ramp<T>::GetTarget() const {
  return target_;
}

template <typename T>
bool Ramp<T>::IsAtTarget() const {
  return flags_.Test(Flag::AtTarget);
}

}  // namespace svxr
#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_RAMP_H_
