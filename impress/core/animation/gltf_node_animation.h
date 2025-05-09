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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_SQT_ANIMATION_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_SQT_ANIMATION_H_

#include <utility>
#include <variant>

#include "absl/status/statusor.h"
#include "core/animation/curve.h"
#include "core/math/math.h"

namespace imp::animation {

// Forward declare the flatbuffer type used to construct GltfNodeAnimation.
namespace schemas {
struct GltfNodeAnimation;
}  // namespace schemas

// Animation type which supports the glTF animation permutation space.
class GltfNodeAnimation {
 public:
  template <typename T>
  using CurveVariant = absl::variant<absl::monostate, T, StepCurve<T>,
                                     LinearCurve<T>, CubicCurve<T>>;

  using TranslationVariant = CurveVariant<float3>;
  using RotationVariant = CurveVariant<quatf>;
  using ScaleVariant = CurveVariant<float3>;

  struct Cursor {
    BaseCurve::Cursor translation_cursor;
    BaseCurve::Cursor rotation_cursor;
    BaseCurve::Cursor scale_cursor;
  };

  struct MissingChannelProvider {
    virtual ~MissingChannelProvider() {}

    virtual float3 GetMissingTranslation() = 0;
    virtual quatf GetMissingRotation() = 0;
    virtual float3 GetMissingScale() = 0;
  };

  Cursor CreateCursor() { return {}; }

  static absl::StatusOr<GltfNodeAnimation> Create(
      const schemas::GltfNodeAnimation *animation);

  Transform<float> Eval(
      float t, Cursor *cursor,
      MissingChannelProvider *missing_channel_provider = nullptr) const;

 protected:
  GltfNodeAnimation(TranslationVariant translation, RotationVariant rotation,
                    ScaleVariant scale)
      : translation_(std::move(translation)),
        rotation_(std::move(rotation)),
        scale_(std::move(scale)) {}

  float3 EvalTranslation(
      float t, BaseCurve::Cursor *cursor,
      MissingChannelProvider *missing_channel_provider) const;
  quatf EvalRotation(float t, BaseCurve::Cursor *cursor,
                     MissingChannelProvider *missing_channel_provider) const;
  float3 EvalScale(float t, BaseCurve::Cursor *cursor,
                   MissingChannelProvider *missing_channel_provider) const;

 private:
  TranslationVariant translation_;
  RotationVariant rotation_;
  ScaleVariant scale_;
};

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_SQT_ANIMATION_H_
