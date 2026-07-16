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

#include "core/animation/morph_target_animation.h"

#include <assert.h>

#include <array>
#include <string>
#include <type_traits>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "absl/types/variant.h"
#include "flatbuffers/vector.h"
#include "core/animation/curve.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/math/flatbuffer_support.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::animation {

namespace {
template <class T>
struct AlwaysFalse : std::false_type {};

using ::flatbuffers::Vector;

constexpr absl::string_view kAdditiveWeightIndexExtraName =
    "additiveWeightIndex";

template <typename Curve, typename SchemaValue>
absl::StatusOr<Curve> CreateCurve(
    const Vector<const schemas::FrameTime*>& times,
    const Vector<const SchemaValue*>& values) {
  static_assert(sizeof(SchemaValue) == sizeof(typename Curve::FrameValue),
                "Invalid Casts");
  return Curve::Create(
      typename Curve::TimeSpan(absl::MakeSpan(
          reinterpret_cast<const typename Curve::FrameTime*>(times.Get(0)),
          times.size())),
      typename Curve::ValueSpan(absl::MakeSpan(
          reinterpret_cast<const typename Curve::FrameValue*>(values.Get(0)),
          values.size())));
}

template <typename T>
struct CurveVariantTraits {};

// TODO Update to use more appropriate sized arrays for morph
// animation size.
template <>
struct CurveVariantTraits<std::array<float, 256>> {
  using Type = schemas::ChannelFloatVector;
  using ConstantSchema = schemas::ConstantFloatVector;
  using StepSchema = schemas::StepCurveFloatVector;
  using LinearSchema = schemas::LinearCurveFloatVector;
  using CubicSchema = schemas::CubicCurveFloatVector;
};

template <typename T>
absl::StatusOr<MorphTargetAnimation::CurveVariant<T>> CreateCurveVariant(
    typename CurveVariantTraits<T>::Type curve_type, const void* curve_union) {
  using Traits = CurveVariantTraits<T>;
  static_assert(sizeof(schemas::FrameTime) == sizeof(BaseCurve::FrameTime),
                "Invalid Casts");
  switch (curve_type) {
    case Traits::Type::kConstant: {
      auto* constant =
          static_cast<const typename Traits::ConstantSchema*>(curve_union);
      return flatbuffers::UnPack(constant->value());
    }
    case Traits::Type::kStep: {
      auto* step = static_cast<const typename Traits::StepSchema*>(curve_union);
      return CreateCurve<StepCurve<T>>(*step->times(), *step->values());
    }
    case Traits::Type::kLinear: {
      auto* linear =
          static_cast<const typename Traits::LinearSchema*>(curve_union);
      return CreateCurve<LinearCurve<T>>(*linear->times(), *linear->values());
    }
    case Traits::Type::kCubic: {
      auto* cubic =
          static_cast<const typename Traits::CubicSchema*>(curve_union);
      return CreateCurve<CubicCurve<T>>(*cubic->times(), *cubic->values());
    }
    default: {
      return absl::InvalidArgumentError("Invalid curve type");
    }
  }
}

}  // namespace

absl::StatusOr<MorphTargetAnimation> MorphTargetAnimation::Create(
    const schemas::MorphTargetAnimation& mt_anim) {
  using CurveType = std::array<float, 256>;

  MP_ASSIGN_OR_RETURN(
      auto weights,
      CreateCurveVariant<CurveType>(mt_anim.weights_type(), mt_anim.weights()));

  absl::flat_hash_map<std::string, ExtraValue> extras;
  // Add any extras from the flatbuffer.
  if (mt_anim.additive_weight_index().has_value()) {
    extras.emplace(kAdditiveWeightIndexExtraName,
                   mt_anim.additive_weight_index().value());
  }

  return MorphTargetAnimation(std::move(weights), std::move(extras));
}

std::array<float, 256> MorphTargetAnimation::Evaluate(
    float frame_time, WeightsCursor* cursor) const {
  return absl::visit(
      [frame_time, cursor](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::array<float, 256>>) {
          return arg;
        } else if constexpr (std::is_same_v<
                                 T, StepCurve<std::array<float, 256>>>) {
          return arg.Eval(frame_time, cursor);
        } else if constexpr (std::is_same_v<
                                 T, LinearCurve<std::array<float, 256>>>) {
          return (arg.Eval(frame_time, cursor));
        } else if constexpr (std::is_same_v<
                                 T, CubicCurve<std::array<float, 256>>>) {
          return (arg.Eval(frame_time, cursor));
        } else {
          static_assert(AlwaysFalse<T>::value, "non-exhaustive visitor!");
        }
        return std::array<float, 256>{};
      },
      weights_);
}
}  // namespace imp::animation
