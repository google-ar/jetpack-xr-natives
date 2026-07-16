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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_MORPH_TARGET_ANIMATION_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_MORPH_TARGET_ANIMATION_H_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/animation/curve.h"
#include "core/animation/schemas/gltf_animation_generated.h"

namespace imp::animation {

// Animation type supporting morph target animations.
class MorphTargetAnimation {
 public:
  // Curve variants for the valid animation interpolation modes for morph target
  // animations. Guaranteed to always be
  // absl::variant<T, StepCurve<T>, LinearCurve<T>, CubicCurve<T>>.
  template <typename T>
  using CurveVariant =
      std::variant<T, StepCurve<T>, LinearCurve<T>, CubicCurve<T>>;

  // The WeightsVariant represents animation curves in a fixed size array
  // with a max size of Filament's max supported morph targets.
  using WeightVariant = CurveVariant<std::array<float, 256>>;
  // Alias for the BaseCurve::Cursor that controls animation of the weights
  // channel for morph target animations. Guaranteed to always be
  // BaseCurve::Cursor.
  using WeightsCursor = BaseCurve::Cursor;

  // Variant type for morph target animation extras.
  using ExtraValue = std::variant<int32_t, float, std::string>;

  MorphTargetAnimation(MorphTargetAnimation&& other) = default;
  MorphTargetAnimation& operator=(MorphTargetAnimation&& other) = default;

  MorphTargetAnimation(const MorphTargetAnimation& other) = delete;
  MorphTargetAnimation& operator=(const MorphTargetAnimation& other) = delete;

  // Statically creates a new MorphTargetAnimation for the corresponding
  // flatbuffer data.
  static absl::StatusOr<MorphTargetAnimation> Create(
      const schemas::MorphTargetAnimation& mt_anim);

  // Returns a new WeightsCursor for storing a MorphTargetAnimation's animation
  // progress for usage in Evaluate calls (see below).
  WeightsCursor CreateCursor() { return {}; }

  // Returns the morph target weights values for the animation, given
  // a frame time in seconds and WeightsCursor that tracks the animation
  // progress.
  std::array<float, 256> Evaluate(float frame_time,
                                  WeightsCursor* cursor) const;

  // Returns the extra value for the given key if it exists and is of type T.
  template <typename T>
  std::optional<T> GetExtra(absl::string_view key) const {
    auto it = extras_.find(key);
    if (it != extras_.end() && std::holds_alternative<T>(it->second)) {
      return std::get<T>(it->second);
    }
    return std::nullopt;
  }

 private:
  explicit MorphTargetAnimation(
      WeightVariant weights,
      absl::flat_hash_map<std::string, ExtraValue> extras)
      : weights_(std::move(weights)), extras_(std::move(extras)) {}
  WeightVariant weights_;
  absl::flat_hash_map<std::string, ExtraValue> extras_;
};

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_MORPH_TARGET_ANIMATION_H_
