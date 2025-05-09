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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_CURVE_VALUES_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_CURVE_VALUES_H_

#include <cstdint>
#include <type_traits>

#include "core/animation/easing_functions.h"
#include "core/math/math.h"

namespace imp::animation {

// Curve types based on glTF interpolation modes.
enum class CurveType : uint8_t {
  // Value: {position}
  // Behavior: Eval(v0, v1, t) always returns v0.position (the earlier value).
  kStep,
  // Value: {position}
  // Behavior: Eval(v0, v1, t) linearly interpolates between v0.position and
  // v1.position.
  kLinear,
  // Value: {in_tangent, position, out_tangent}
  // Behavior: Eval(v0, v1, t, d) is evaluated as segment in a Catmull-Rom
  // spline, i.e. a cubic Hermite spline whose tangents (out_tangent for v0 and
  // in_tangent for v1) are scaled by segment duration.
  kCubicSpline,
};

// Curve types based on easing curve formulas.
enum class EasingCurveType : uint8_t {
  kEaseInSine,
  kEaseOutSine,
  kEaseInOutSine,
  kEaseInQuad,
  kEaseOutQuad,
  kEaseInOutQuad,
  kEaseInCubic,
  kEaseOutCubic,
  kEaseInOutCubic,
  kEaseInBack,
  kEaseOutBack,
  kEaseInOutBack,
};

// A templated value type for use in Curve.  Given a curve type and output value
// type, defines the storage type for the value array in the curve, as well as
// an Interpolate() method for use in evaluation.
template <CurveType kType, typename V>
struct CurveFrameValue {};

template <typename V>
struct CurveFrameValue<CurveType::kStep, V> {
  using ValueType = V;

  static inline V Interpolate(
      const CurveFrameValue<CurveType::kStep, V>& v0,
      const CurveFrameValue<CurveType::kStep, V>& /*v1*/, float /*t*/,
      float /*duration*/);

  V position;
};

// TODO Update to use more appropriate sized arrays for morph
// animation size rather than max size.
template <typename V>
struct CurveFrameValue<CurveType::kStep, std::array<V, 256>> {
  using ValueType = std::array<V, 256>;

  static inline std::array<V, 256> Interpolate(
      const CurveFrameValue<CurveType::kStep, std::array<V, 256>>& v0,
      const CurveFrameValue<CurveType::kStep, std::array<V, 256>>& /*v1*/,
      float /*t*/, float /*duration*/);

  std::array<V, 256> position;
};

template <typename V>
struct CurveFrameValue<CurveType::kLinear, V> {
  using ValueType = V;

  static inline V Interpolate(const CurveFrameValue<CurveType::kLinear, V>& v0,
                              const CurveFrameValue<CurveType::kLinear, V>& v1,
                              float t, float /*duration*/);

  V position;
};

template <typename V>
struct CurveFrameValue<CurveType::kLinear, std::array<V, 256>> {
  using ValueType = std::array<V, 256>;

  static inline std::array<V, 256> Interpolate(
      const CurveFrameValue<CurveType::kLinear, std::array<V, 256>>& v0,
      const CurveFrameValue<CurveType::kLinear, std::array<V, 256>>& v1,
      float t, float /*duration*/);

  std::array<V, 256> position;
};

template <typename V>
struct CurveFrameValue<CurveType::kCubicSpline, V> {
  using ValueType = V;

  static inline V Interpolate(
      const CurveFrameValue<CurveType::kCubicSpline, V>& v0,
      const CurveFrameValue<CurveType::kCubicSpline, V>& v1, float t,
      float duration);
  V in_tangent;
  V position;
  V out_tangent;
};

template <typename V>
struct CurveFrameValue<CurveType::kCubicSpline, std::array<V, 256>> {
  using ValueType = std::array<V, 256>;

  static inline std::array<V, 256> Interpolate(
      const CurveFrameValue<CurveType::kCubicSpline, std::array<V, 256>>& v0,
      const CurveFrameValue<CurveType::kCubicSpline, std::array<V, 256>>& v1,
      float t, float duration);
  std::array<V, 256> in_tangent;
  std::array<V, 256> position;
  std::array<V, 256> out_tangent;
};

template <typename V>
V CurveFrameValue<CurveType::kStep, V>::Interpolate(
    const CurveFrameValue<CurveType::kStep, V>& v0,
    const CurveFrameValue<CurveType::kStep, V>& /*v1*/, float /*t*/,
    float /*duration*/) {
  return v0.position;
}

template <typename V>
std::array<V, 256>
CurveFrameValue<CurveType::kStep, std::array<V, 256>>::Interpolate(
    const CurveFrameValue<CurveType::kStep, std::array<V, 256>>& v0,
    const CurveFrameValue<CurveType::kStep, std::array<V, 256>>& /*v1*/,
    float /*t*/, float /*duration*/) {
  return v0.position;
}

template <typename V>
V CurveFrameValue<CurveType::kLinear, V>::Interpolate(
    const CurveFrameValue<CurveType::kLinear, V>& v0,
    const CurveFrameValue<CurveType::kLinear, V>& v1, float t,
    float /*duration*/) {
  if constexpr (std::is_same_v<V, quatf>) {
    return slerp(v0.position, v1.position, t);
  } else {
    return lerp(v0.position, v1.position, t);
  }
}

template <typename V>
std::array<V, 256>
CurveFrameValue<CurveType::kLinear, std::array<V, 256>>::Interpolate(
    const CurveFrameValue<CurveType::kLinear, std::array<V, 256>>& v0,
    const CurveFrameValue<CurveType::kLinear, std::array<V, 256>>& v1, float t,
    float /*duration*/) {
  assert(v0.position.size() == v1.position.size());
  std::array<V, 256> result;
  for (int i = 0; i < v0.position.size(); i++) {
    if constexpr (std::is_same_v<V, quatf>) {
      result[i] = slerp(v0.position[i], v1.position[i], t);
    } else {
      result[i] = lerp(v0.position[i], v1.position[i], t);
    }
  }
  return result;
}

template <typename V>
V CurveFrameValue<CurveType::kCubicSpline, V>::Interpolate(
    const CurveFrameValue<CurveType::kCubicSpline, V>& v0,
    const CurveFrameValue<CurveType::kCubicSpline, V>& v1, float t,
    float duration) {
  // https://en.wikipedia.org/wiki/Cubic_Hermite_spline
  float tt = t * t, ttt = tt * t;
  float s0 = 2 * ttt - 3 * tt + 1;
  float s1 = ttt - 2 * tt + t;
  float s2 = 3 * tt - 2 * ttt;
  float s3 = ttt - tt;
  V p0 = v0.position;
  V m0 = v0.out_tangent * duration;
  V p1 = v1.position;
  V m1 = v1.in_tangent * duration;
  return s0 * p0 + s1 * m0 + s2 * p1 + s3 * m1;
}

template <typename V>
std::array<V, 256>
CurveFrameValue<CurveType::kCubicSpline, std::array<V, 256>>::Interpolate(
    const CurveFrameValue<CurveType::kCubicSpline, std::array<V, 256>>& v0,
    const CurveFrameValue<CurveType::kCubicSpline, std::array<V, 256>>& v1,
    float t, float duration) {
  assert(v0.position.size() == v1.position.size());
  std::array<V, 256> result;
  for (int i = 0; i < v0.position.size(); i++) {
    // https://en.wikipedia.org/wiki/Cubic_Hermite_spline
    float tt = t * t, ttt = tt * t;
    float s0 = 2 * ttt - 3 * tt + 1;
    float s1 = ttt - 2 * tt + t;
    float s2 = 3 * tt - 2 * ttt;
    float s3 = ttt - tt;
    V p0 = v0.position[i];
    V m0 = v0.out_tangent[i] * duration;
    V p1 = v1.position[i];
    V m1 = v1.in_tangent[i] * duration;
    result[i] = s0 * p0 + s1 * m0 + s2 * p1 + s3 * m1;
  }
  return result;
}

template <EasingCurveType kType, typename V>
struct EasingFrameValue {
  using ValueType = V;

  static inline V Interpolate(const EasingFrameValue<kType, V>& v0,
                              const EasingFrameValue<kType, V>& v1, float t,
                              float /*duration*/);
  V position;
};

template <EasingCurveType kType, typename V>
V EasingFrameValue<kType, V>::Interpolate(const EasingFrameValue<kType, V>& v0,
                                          const EasingFrameValue<kType, V>& v1,
                                          float t, float /*duration*/) {
  t = clamp(t, 0.f, 1.f);
  switch (kType) {
    case EasingCurveType::kEaseInSine: {
      t = EaseInSine(t);
      break;
    }
    case EasingCurveType::kEaseOutSine: {
      t = EaseOutSine(t);
      break;
    }
    case EasingCurveType::kEaseInOutSine: {
      t = EaseInOutSine(t);
      break;
    }
    case EasingCurveType::kEaseInQuad: {
      t = EaseInQuad(t);
      break;
    }
    case EasingCurveType::kEaseOutQuad: {
      t = EaseOutQuad(t);
      break;
    }
    case EasingCurveType::kEaseInOutQuad: {
      t = EaseInOutQuad(t);
      break;
    }
    case EasingCurveType::kEaseInCubic: {
      t = EaseInCubic(t);
      break;
    }
    case EasingCurveType::kEaseOutCubic: {
      t = EaseOutCubic(t);
      break;
    }
    case EasingCurveType::kEaseInOutCubic: {
      t = EaseInOutCubic(t);
      break;
    }
    case EasingCurveType::kEaseInBack: {
      t = EaseInBack(t);
      break;
    }
    case EasingCurveType::kEaseOutBack: {
      t = EaseOutBack(t);
      break;
    }
    case EasingCurveType::kEaseInOutBack: {
      t = EaseInOutBack(t);
      break;
    }
  }
  if constexpr (std::is_same_v<V, quatf>) {
    return slerp(v0.position, v1.position, t);
  } else {
    return lerp(v0.position, v1.position, t);
  }
}

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_CURVE_VALUES_H_
