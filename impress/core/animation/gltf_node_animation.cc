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

#include "core/animation/gltf_node_animation.h"

#include "absl/status/statusor.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/filament_helpers.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::animation {
namespace {
template <class T>
struct AlwaysFalse : std::false_type {};

using flatbuffers::Vector;

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
struct CurveVariantTraits<float3> {
  using Type = schemas::ChannelFloat3;
  using ConstantSchema = schemas::ConstantFloat3;
  using StepSchema = schemas::StepCurveFloat3;
  using LinearSchema = schemas::LinearCurveFloat3;
  using CubicSchema = schemas::CubicCurveFloat3;
};

template <>
struct CurveVariantTraits<quatf> {
  using Type = schemas::ChannelQuatf;
  using ConstantSchema = schemas::ConstantQuatf;
  using StepSchema = schemas::StepCurveQuatf;
  using LinearSchema = schemas::LinearCurveQuatf;
  using CubicSchema = schemas::CubicCurveQuatf;
};

template <typename T>
absl::StatusOr<GltfNodeAnimation::CurveVariant<T>> CreateCurveVariant(
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
      return GltfNodeAnimation::CurveVariant<T>{};
    }
  }
}

}  // namespace

absl::StatusOr<GltfNodeAnimation> GltfNodeAnimation::Create(
    const schemas::GltfNodeAnimation* animation) {
  MP_ASSIGN_OR_RETURN(CurveVariant<float3> translation,
                   CreateCurveVariant<float3>(animation->translation_type(),
                                              animation->translation()));
  MP_ASSIGN_OR_RETURN(CurveVariant<quatf> rotation,
                   CreateCurveVariant<quatf>(animation->rotation_type(),
                                             animation->rotation()));
  MP_ASSIGN_OR_RETURN(CurveVariant<float3> scale,
                   CreateCurveVariant<float3>(animation->scale_type(),  // ^
                                              animation->scale()));

  return GltfNodeAnimation(std::move(translation), std::move(rotation),
                           std::move(scale));
}

Transform<float> GltfNodeAnimation::Eval(
    float t, Cursor* cursor,
    MissingChannelProvider* missing_channel_provider) const {
  return Transform<float>(
      EvalTranslation(t, &cursor->translation_cursor, missing_channel_provider),
      EvalRotation(t, &cursor->rotation_cursor, missing_channel_provider),
      EvalScale(t, &cursor->scale_cursor, missing_channel_provider));
}

float3 GltfNodeAnimation::EvalTranslation(
    float t, BaseCurve::Cursor* cursor,
    MissingChannelProvider* missing_channel_provider) const {
  return absl::visit(
      [t, cursor, missing_channel_provider](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, float3>)
          return arg;
        else if constexpr (std::is_same_v<T, StepCurve<float3>>)
          return arg.Eval(t, cursor);
        else if constexpr (std::is_same_v<T, LinearCurve<float3>>)
          return arg.Eval(t, cursor);
        else if constexpr (std::is_same_v<T, CubicCurve<float3>>)
          return arg.Eval(t, cursor);
        else if constexpr (std::is_same_v<T, std::monostate>)
          return missing_channel_provider
                     ? missing_channel_provider->GetMissingTranslation()
                     : kZero3;
        else
          static_assert(AlwaysFalse<T>::value, "non-exhaustive visitor!");
        return float3{};
      },
      translation_);
}

quatf GltfNodeAnimation::EvalRotation(
    float t, BaseCurve::Cursor* cursor,
    MissingChannelProvider* missing_channel_provider) const {
  return absl::visit(
      [t, cursor, missing_channel_provider](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, quatf>)
          return arg;
        else if constexpr (std::is_same_v<T, StepCurve<quatf>>)
          return arg.Eval(t, cursor);
        else if constexpr (std::is_same_v<T, LinearCurve<quatf>>)
          return normalize(arg.Eval(t, cursor));
        else if constexpr (std::is_same_v<T, CubicCurve<quatf>>)
          return normalize(arg.Eval(t, cursor));
        else if constexpr (std::is_same_v<T, std::monostate>)
          return missing_channel_provider
                     ? missing_channel_provider->GetMissingRotation()
                     : kIdentityQuatf;
        else
          static_assert(AlwaysFalse<T>::value, "non-exhaustive visitor!");
        return quatf{};
      },
      rotation_);
}

float3 GltfNodeAnimation::EvalScale(
    float t, BaseCurve::Cursor* cursor,
    MissingChannelProvider* missing_channel_provider) const {
  return absl::visit(
      [t, cursor, missing_channel_provider](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, float3>)
          return arg;
        else if constexpr (std::is_same_v<T, StepCurve<float3>>)
          return arg.Eval(t, cursor);
        else if constexpr (std::is_same_v<T, LinearCurve<float3>>)
          return arg.Eval(t, cursor);
        else if constexpr (std::is_same_v<T, CubicCurve<float3>>)
          return arg.Eval(t, cursor);
        else if constexpr (std::is_same_v<T, std::monostate>)
          return missing_channel_provider
                     ? missing_channel_provider->GetMissingScale()
                     : kOne3;
        else
          static_assert(AlwaysFalse<T>::value, "non-exhaustive visitor!");
        return float3{};
      },
      scale_);
}

}  // namespace imp::animation
