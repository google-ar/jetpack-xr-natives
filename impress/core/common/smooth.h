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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_SMOOTH_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_SMOOTH_H_

#include <algorithm>
#include <functional>

#include "core/common/enum_flags.h"
#include "filament/libs/math/include/math/vec3.h"
#include "filament/libs/math/include/math/vec4.h"

namespace imp {

struct SmoothParameters {
  constexpr SmoothParameters(float in_velocity_limit,
                             float in_acceleration_limit,
                             float in_deceleration_limit = 0);
  constexpr SmoothParameters() : SmoothParameters(100, 100) {}

  // Magnitude limit (units per second) of controlled velocity.
  float velocity_limit;
  // Magnitude limit (units per second) of acceleration.
  float acceleration_limit;
  // Magnitude limit (units per second) of deceleration during approach.
  float deceleration_limit;
};

// Model a physical particle, which given a constant
// acceleration can define it's position at time t thusly:
// P(t) = P(0) + V(0) * t + 0.5 * A * t^2
// V(t) = V(0) + A * t
namespace smooth {

// Filament specialization
template <typename T>
auto Magnitude(T in) -> typename std::enable_if_t<
    std::is_floating_point<decltype(length(in))>::value, float> {
  return length(in);
}

// Mathfu specialization
template <typename T>
auto Magnitude(T in) -> typename std::enable_if_t<
    std::is_floating_point<decltype(in.Length())>::value, float> {
  return in.Length();
}

// Floating point specialization
template <typename T>
auto Magnitude(T in) -> typename std::enable_if_t<
    std::is_floating_point<decltype(std::abs(T{}))>::value, float> {
  return std::abs(in);
}

constexpr float TravelEpsilon() { return 1.0e-5f; }
// Simulation stepping is useless if the parameters are too small.
constexpr float ParameterEpsilon() { return 1.0e-3f; }
template <class T>
constexpr T ZeroValue() {
  return T{0.0f};
}

// Given our remaining travel to target and our deceleration limit, compute
// the speed (velocity magnitude) that would ballistically reach a velocity of
// 0 at the moment we complete travel.
inline float IdealSpeed(const SmoothParameters& params,
                        float travel_magnitude) {
  // V(t) = 0 = V(0) + A * t
  // t = -V(0) / A
  //
  // Substituting t gives us:
  // P(t) = P(0) + V(0) * (-V(0) / A) + 0.5 * A * (-V(0) / A)^2
  //
  // Which simplifies to:
  // P(t) = P(0) - (0.5 * V(0)^2) / A
  //
  // Solving for V(0) yields:
  // (A * (P(t) - P(0)) / -0.5 = V(0)^2
  // V(0) = sqrt(-2 * A * (P(t) - P(0)))
  //
  // (note that acceleration points the opposite way of travel by definition,
  // hence a non-imaginary root).  We specify deceleration limit with a
  // positive number, so we remove a minus sign to compensate.
  float result = sqrt(2.0f * params.deceleration_limit * travel_magnitude);
  // Undersell our ideal speed slighty.  When travel_magnitude is large, the
  // fraction won't matter, but close to the target, it affords a bit of
  // wiggle room that prevents overshoot.  Value has been experimentally
  // derived via tests.
  const float kIdealSpeedSafety = 0.95f;
  return result * kIdealSpeedSafety;
}

template <class T>
T TargetVelocity(const SmoothParameters& params, const T velocity,
                 const T travel, const float travel_magnitude, const float dT) {
  if (travel_magnitude < TravelEpsilon()) {
    // Remaining travel is too small to act on; come to a stop.
    return ZeroValue<T>();
  } else if (dT == 0.0f) {
    // We can't compute the velocity but it doesn't matter because there's
    // a zero dT.  Just return the input velocity.
    return velocity;
  } else {
    // 'Exact' velocity is what places us exactly on our target
    const T exact_velocity = travel / dT;
    const float exact_speed = Magnitude(exact_velocity);
    const float ideal_speed = IdealSpeed(params, travel_magnitude);
    const float limit_speed = std::min(params.velocity_limit, ideal_speed);

    const float speed = std::min(exact_speed, limit_speed);
    // Take care to combine speed and exact speed first, since they're each
    // very large/small but resolve to a reasonable fraction.
    return exact_velocity * (speed / exact_speed);
  }
}

template <class T>
T ComputeImpulse(const SmoothParameters& params, const T velocity,
                 const T travel, const float travel_magnitude, const float dT,
                 bool* out_is_accelerating) {
  const T target_velocity =
      TargetVelocity(params, velocity, travel, travel_magnitude, dT);
  T ideal_impulse = (target_velocity - velocity);
  float ideal_impulse_magnitude = Magnitude(ideal_impulse);

  // To prevent hysteresis when we're cruising, bias towards deceleration when
  // deciding whether we are or not.  To count as acceleration, our target
  // velocity must be 1% bigger than our target velocity.
  constexpr float kDecelerationBias = 1.0f + 1.0e-2f;
  const bool accelerating =
      Magnitude(target_velocity) >= Magnitude(velocity) * kDecelerationBias;

  if (out_is_accelerating) *out_is_accelerating = accelerating;
  const float impulse_limit = dT * (accelerating ? params.acceleration_limit
                                                 : params.deceleration_limit);
  const T impulse =
      (ideal_impulse_magnitude > impulse_limit)
          ? ideal_impulse * (impulse_limit / ideal_impulse_magnitude)
          : ideal_impulse;

  const T next_velocity = impulse + velocity;
  const float next_speed = Magnitude(next_velocity);
  const T next_travel = travel - next_velocity * dT;
  const float next_ideal_speed = IdealSpeed(params, Magnitude(next_travel));
  // Undersell our stopping force, since the frame when we expect to stop may
  // be slightly shorter than this one.  This value has been experimentally
  // derived via test.
  const float kFramerateVarianceSafety = 0.75f;
  const float estimated_stopping_force =
      dT * params.deceleration_limit * kFramerateVarianceSafety;
  const float next_limit_speed = std::min(
      next_ideal_speed + estimated_stopping_force, params.velocity_limit);
  if (next_speed <= next_limit_speed || travel_magnitude <= TravelEpsilon()) {
    return impulse;
  }

  // Our computed impulse with our given simulation step will have us
  // moving faster than is safe for the given travel remainder.
  const auto safety_scale = (next_limit_speed / next_speed);
  const auto safe_next_velocity = next_velocity * safety_scale;
  const auto velocity_correction = safe_next_velocity - next_velocity;
  const auto corrected_impulse = impulse + velocity_correction;
  const float corrected_impulse_magnitude = Magnitude(corrected_impulse);

  if (corrected_impulse_magnitude <= impulse_limit) {
    return corrected_impulse;
  }

  // Our corrected impulse was itself too big.  normalize to limit and pray.
  return corrected_impulse * (impulse_limit / corrected_impulse_magnitude);
}

}  // namespace smooth

// Smoothly update `position` and `velocity` along `travel`, with a
// simulation step size `dT` (seconds).  Returns true if travel is exhausted
// and motion is stopped.
template <class T>
bool SmoothStep(const SmoothParameters& params, T* position, T* velocity,
                T travel, float dT, bool* out_is_accelerating = nullptr) {
  assert(position && velocity);
  // Compute and apply an impulse (instantaneous acceleration) to velocity.
  // The acceleration would be impulse / dT; Using an impulse keeps things
  // stable by avoiding dividing a small impulse by a small dT.
  const float travel_magnitude = smooth::Magnitude(travel);
  const T impulse = smooth::ComputeImpulse(
      params, *velocity, travel, travel_magnitude, dT, out_is_accelerating);
  *velocity += impulse;
  *position += *velocity * dT;
  if (*velocity == smooth::ZeroValue<T>() &&
      travel_magnitude < smooth::TravelEpsilon()) {
    *position += travel;
    return true;
  }
  return false;
}

inline constexpr SmoothParameters::SmoothParameters(float in_velocity_limit,
                                                    float in_acceleration_limit,
                                                    float in_deceleration_limit)
    : velocity_limit(std::max(smooth::ParameterEpsilon(), in_velocity_limit)),
      acceleration_limit(
          std::max(smooth::ParameterEpsilon(), in_acceleration_limit)),
      deceleration_limit(
          std::max(smooth::ParameterEpsilon(),
                   (in_deceleration_limit <= 0 ? in_acceleration_limit
                                               : in_deceleration_limit))) {}

// Helper type for using SmoothController.
template <typename T>
class Smooth {
 public:
  Smooth() = default;
  // Constructs with control parameters and a method
  using GetTravelFunc = std::function<T(T*, T*)>;
  Smooth(const SmoothParameters& params, T initial_value,
         GetTravelFunc get_travel_func = DefaultGetTravelFunc) {
    Setup(params, initial_value, get_travel_func);
  }

  void Setup(const SmoothParameters& params, T initial_value,
             GetTravelFunc get_travel_func = DefaultGetTravelFunc) {
    params_ = params;
    get_travel_func_ = get_travel_func;
    SetTarget(initial_value);
    Snap();
    flags_ |= SmoothFlags::Initialized;
  }

  void SetTarget(T target) {
    target_ = target;
    flags_ &= ~ToFlags(SmoothFlags::AtTarget);
  }

  // Returns true if we are at our target.
  bool Step(float dT) {
    if (flags_.Test(SmoothFlags::AtTarget)) {
      return true;
    }
    bool is_accelerating = flags_.Test(SmoothFlags::Accelerating);
    bool at_target = SmoothStep(params_, &position_, &velocity_,
                                get_travel_func_(&position_, &target_), dT,
                                &is_accelerating);
    flags_.Set(SmoothFlags::Accelerating, is_accelerating);
    flags_.Set(SmoothFlags::AtTarget, at_target);
    return at_target;
  }

  void Snap() {
    position_ = target_;
    velocity_ = T{0};
    flags_ |= SmoothFlags::AtTarget;
    flags_ &= ~ToFlags(SmoothFlags::Accelerating);
  }

  T Get() const { return position_; }
  T GetTarget() const { return target_; }
  bool IsAtTarget() const { return !!(flags_ & SmoothFlags::AtTarget); }

 protected:
  enum class SmoothFlags : uint8_t {
    Initialized = (1 << 0),
    Accelerating = (1 << 1),
    AtTarget = (1 << 2),
  };

  SmoothParameters params_;
  GetTravelFunc get_travel_func_;
  T position_;
  T velocity_;
  T target_;
  Flags<SmoothFlags> flags_;
  static T DefaultGetTravelFunc(T* position, T* target) {
    return *target - *position;
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_SMOOTH_H_
