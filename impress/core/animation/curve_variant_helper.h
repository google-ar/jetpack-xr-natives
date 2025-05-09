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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_CURVE_VARIANT_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_CURVE_VARIANT_HELPER_H_

#include <cassert>
#include <type_traits>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "absl/types/variant.h"
#include "core/animation/curve.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/vec.h"

namespace imp::animation {

using flatbuffers::Vector;

template <typename T>
using CurveVariant = absl::variant<absl::monostate, T, StepCurve<T>,
                                   LinearCurve<T>, CubicCurve<T>>;

template <typename Curve, typename SchemaValue>
absl::StatusOr<Curve> CreateCurve(
    const Vector<const schemas::FrameTime*>* times,
    const Vector<const SchemaValue*>* values) {
  static_assert(sizeof(SchemaValue) == sizeof(typename Curve::FrameValue),
                "Invalid Casts");
  assert(times && values);
  return Curve::Create(
      typename Curve::TimeSpan(absl::MakeSpan(
          reinterpret_cast<const typename Curve::FrameTime*>(times->Get(0)),
          times->size())),
      typename Curve::ValueSpan(absl::MakeSpan(
          reinterpret_cast<const typename Curve::FrameValue*>(values->Get(0)),
          values->size())));
}

template <typename T>
struct CurveVariantTraits {};

template <>
struct CurveVariantTraits<float4> {
  using Type = schemas::ChannelFloat4;
  using ConstantSchema = schemas::ConstantFloat4;
  using StepSchema = schemas::StepCurveFloat4;
  using LinearSchema = schemas::LinearCurveFloat4;
  using CubicSchema = schemas::CubicCurveFloat4;
};

template <>
struct CurveVariantTraits<float> {
  using Type = schemas::ChannelFloat;
  using ConstantSchema = schemas::ConstantFloat;
  using StepSchema = schemas::StepCurveFloat;
  using LinearSchema = schemas::LinearCurveFloat;
  using CubicSchema = schemas::CubicCurveFloat;
};

template <>
struct CurveVariantTraits<float2> {
  using Type = schemas::ChannelFloat2;
  using ConstantSchema = schemas::ConstantFloat2;
  using StepSchema = schemas::StepCurveFloat2;
  using LinearSchema = schemas::LinearCurveFloat2;
  using CubicSchema = schemas::CubicCurveFloat2;
};

template <>
struct CurveVariantTraits<float3> {
  using Type = schemas::ChannelFloat3;
  using ConstantSchema = schemas::ConstantFloat3;
  using StepSchema = schemas::StepCurveFloat3;
  using LinearSchema = schemas::LinearCurveFloat3;
  using CubicSchema = schemas::CubicCurveFloat3;
};

template <typename T>
absl::StatusOr<CurveVariant<T>> CreateCurveVariant(
    typename CurveVariantTraits<T>::Type curve_type, const void* curve_union) {
  using Traits = CurveVariantTraits<T>;
  static_assert(sizeof(schemas::FrameTime) == sizeof(BaseCurve::FrameTime),
                "Invalid Casts");

  switch (curve_type) {
    default:
      return absl::InternalError("Invalid");

    case Traits::Type::kConstant: {
      auto* constant =
          static_cast<const typename Traits::ConstantSchema*>(curve_union);
      return flatbuffers::UnPack(constant->value());
    }
    case Traits::Type::kStep: {
      auto* step = static_cast<const typename Traits::StepSchema*>(curve_union);
      return CreateCurve<StepCurve<T>>(step->times(), step->values());
    }
    case Traits::Type::kLinear: {
      auto* linear =
          static_cast<const typename Traits::LinearSchema*>(curve_union);
      return CreateCurve<LinearCurve<T>>(linear->times(), linear->values());
    }
    case Traits::Type::kCubic: {
      auto* cubic =
          static_cast<const typename Traits::CubicSchema*>(curve_union);
      return CreateCurve<CubicCurve<T>>(cubic->times(), cubic->values());
    }
    case Traits::Type::NONE: {
      return CurveVariant<T>{};
    }
  }
}

template <typename V>
absl::optional<V> EvalValue(float t, BaseCurve::Cursor* cursor,
                            const CurveVariant<V>& curve) {
  bool has_value = true;
  V result = absl::visit(
      [t, cursor, &has_value](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, V>) {
          return arg;
        } else if constexpr (std::is_same_v<T, StepCurve<V>>) {
          return arg.Eval(t, cursor);
        } else if constexpr (std::is_same_v<T, LinearCurve<V>>) {
          return arg.Eval(t, cursor);
        } else if constexpr (std::is_same_v<T, CubicCurve<V>>) {
          return arg.Eval(t, cursor);
        } else {
          has_value = false;
          return V{};
        }
      },
      curve);
  if (!has_value) return absl::nullopt;
  return result;
}

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_CURVE_VARIANT_HELPER_H_
