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

#include "core/animation/gltf_animation.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "flatbuffers/buffer.h"
#include "core/animation/gltf_node_animation.h"
#include "core/animation/light_punctual_animation.h"
#include "core/animation/material_animation.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/math/math.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::animation {

using flatbuffers::Offset;
using flatbuffers::Vector;

namespace {

template <CurveType kType, typename V>
struct CurveTypeTraits {};

template <>
struct CurveTypeTraits<CurveType::kStep, float3> {
  using Frame = schemas::StepFrameFloat3;
  using Builder = schemas::StepCurveFloat3Builder;
  using ChannelType = schemas::ChannelFloat3;
  static constexpr ChannelType kChannel = ChannelType::kStep;
};
template <>
struct CurveTypeTraits<CurveType::kLinear, float3> {
  using Frame = schemas::LinearFrameFloat3;
  using Builder = schemas::LinearCurveFloat3Builder;
  using ChannelType = schemas::ChannelFloat3;
  static constexpr ChannelType kChannel = ChannelType::kLinear;
};
template <>
struct CurveTypeTraits<CurveType::kCubicSpline, float3> {
  using Frame = schemas::CubicFrameFloat3;
  using Builder = schemas::CubicCurveFloat3Builder;
  using ChannelType = schemas::ChannelFloat3;
  static constexpr ChannelType kChannel = ChannelType::kCubic;
};

template <>
struct CurveTypeTraits<CurveType::kStep, quatf> {
  using Frame = schemas::StepFrameQuatf;
  using Builder = schemas::StepCurveQuatfBuilder;
  using ChannelType = schemas::ChannelQuatf;
  static constexpr ChannelType kChannel = ChannelType::kStep;
};
template <>
struct CurveTypeTraits<CurveType::kLinear, quatf> {
  using Frame = schemas::LinearFrameQuatf;
  using Builder = schemas::LinearCurveQuatfBuilder;
  using ChannelType = schemas::ChannelQuatf;
  static constexpr ChannelType kChannel = ChannelType::kLinear;
};
template <>
struct CurveTypeTraits<CurveType::kCubicSpline, quatf> {
  using Frame = schemas::CubicFrameQuatf;
  using Builder = schemas::CubicCurveQuatfBuilder;
  using ChannelType = schemas::ChannelQuatf;
  static constexpr ChannelType kChannel = ChannelType::kCubic;
};

template <typename V>
typename CurveTypeTraits<CurveType::kStep, V>::Frame Pack(
    CurveFrameValue<CurveType::kStep, V> rhs) {
  return typename CurveTypeTraits<CurveType::kStep, V>::Frame(
      flatbuffers::Pack(rhs.position));
}

template <typename V>
typename CurveTypeTraits<CurveType::kLinear, V>::Frame Pack(
    CurveFrameValue<CurveType::kLinear, V> rhs) {
  return typename CurveTypeTraits<CurveType::kLinear, V>::Frame(
      flatbuffers::Pack(rhs.position));
}

template <typename V>
typename CurveTypeTraits<CurveType::kCubicSpline, V>::Frame Pack(
    CurveFrameValue<CurveType::kCubicSpline, V> rhs) {
  return typename CurveTypeTraits<CurveType::kCubicSpline, V>::Frame(
      flatbuffers::Pack(rhs.in_tangent), flatbuffers::Pack(rhs.position),
      flatbuffers::Pack(rhs.out_tangent));
}

template <CurveType kType, typename V,
          typename InputFrame = CurveFrameValue<kType, V>>
auto CreateValues(flatbuffers::FlatBufferBuilder& fbb, V* values,
                  size_t count) {
  using Traits = CurveTypeTraits<kType, V>;

  size_t num_frames = count;
  if constexpr (kType == CurveType::kCubicSpline) {
    num_frames /= 3;
  }
  std::function<void(size_t i, typename Traits::Frame*)> get_values =
      [values](size_t i, typename Traits::Frame* dest) {
        if constexpr (kType == CurveType::kCubicSpline) {
          size_t frame_index = i * 3;
          *dest = Pack(InputFrame{values[frame_index], values[frame_index + 1],
                                  values[frame_index + 2]});
        } else {
          *dest = Pack(InputFrame{values[i]});
        }
      };
  return fbb.CreateVectorOfStructs(num_frames, get_values);
}

}  // namespace

absl::Status GltfAnimation::CreateTrsAnimations(
    const Vector<const schemas::GltfNodeAnimationTarget*>* in_targets,
    const Vector<Offset<schemas::GltfNodeAnimation>>* in_animations,
    BoneTargetSpan& out_targets,
    BoneTargetLookup<GltfNodeAnimation>& out_animations) {
  auto has_trs_animation = in_animations && in_animations->size() > 0;
  if (in_animations->size() != in_targets->size()) {
    return absl::InternalError("Invalid animation");
  }

  if (has_trs_animation) {
    out_animations.reserve(out_animations.size());
    for (const schemas::GltfNodeAnimation* in_animation : *in_animations) {
      MP_ASSIGN_OR_RETURN(GltfNodeAnimation animation,
                       GltfNodeAnimation::Create(in_animation));
      out_animations.emplace_back(std::move(animation));
    }

    static_assert(
        sizeof(schemas::GltfNodeAnimationTarget) == sizeof(BoneTarget),
        "Bad size");
    out_targets = BoneTargetSpan(absl::Span<const BoneTarget>(
        reinterpret_cast<const BoneTarget*>(in_targets->Get(0)),
        in_targets->size()));
  }

  return absl::OkStatus();
}

absl::Status GltfAnimation::CreateMorphTargetAnimations(
    const Vector<const schemas::GltfNodeAnimationTarget*>* in_targets,
    const Vector<Offset<schemas::MorphTargetAnimation>>* in_animations,
    BoneTargetSpan& out_targets,
    BoneTargetLookup<MorphTargetAnimation>& out_animations) {
  auto has_morph_target_animation = in_animations && in_animations->size() > 0;
  if (in_animations->size() != in_targets->size()) {
    return absl::InternalError("Invalid animation");
  }

  if (has_morph_target_animation) {
    out_animations.reserve(in_animations->size());
    for (const schemas::MorphTargetAnimation* in_animation : *in_animations) {
      MP_ASSIGN_OR_RETURN(MorphTargetAnimation animation,
                       MorphTargetAnimation::Create(*in_animation));
      out_animations.emplace_back(std::move(animation));
    }

    static_assert(
        sizeof(schemas::GltfNodeAnimationTarget) == sizeof(BoneTarget),
        "Bad size");
    out_targets = BoneTargetSpan(absl::Span<const BoneTarget>(
        reinterpret_cast<const BoneTarget*>(in_targets->Get(0)),
        in_targets->size()));
  }

  return absl::OkStatus();
}

absl::Status GltfAnimation::CreateMaterialAnimations(
    const Vector<const schemas::MaterialAnimationTarget*>* in_targets,
    const Vector<Offset<schemas::MaterialAnimation>>* in_animations,
    MaterialTargetSpan& out_targets,
    MaterialTargetLookup<MaterialAnimation>& out_animations) {
  if (in_animations->size() != in_targets->size()) {
    return absl::InternalError("Invalid material animation");
  }

  bool has_material_animation = in_animations && in_animations->size() > 0;
  if (has_material_animation) {
    out_animations.reserve(out_animations.size());
    for (const schemas::MaterialAnimation* in_animation : *in_animations) {
      MP_ASSIGN_OR_RETURN(MaterialAnimation animation,
                       MaterialAnimation::Create(in_animation));
      out_animations.emplace_back(std::move(animation));
    }

    static_assert(
        sizeof(schemas::MaterialAnimationTarget) == sizeof(MaterialTarget),
        "Bad size");
    out_targets = MaterialTargetSpan(absl::Span<const MaterialTarget>(
        reinterpret_cast<const MaterialTarget*>(in_targets->Get(0)),
        in_targets->size()));
  }

  return absl::OkStatus();
}

absl::Status GltfAnimation::CreateLightPunctualAnimations(
    const Vector<const schemas::LightAnimationTarget*>* in_targets,
    const Vector<Offset<schemas::LightPunctualAnimation>>* in_animations,
    LightTargetSpan& out_targets,
    LightTargetLookup<LightPunctualAnimation>& out_animations) {
  if (in_animations->size() != in_targets->size()) {
    return absl::InternalError("Invalid light animation");
  }

  bool has_light_animation = in_animations && in_animations->size() > 0;
  if (has_light_animation) {
    out_animations.reserve(out_animations.size());
    for (const schemas::LightPunctualAnimation* in_animation : *in_animations) {
      MP_ASSIGN_OR_RETURN(LightPunctualAnimation animation,
                       LightPunctualAnimation::Create(in_animation));
      out_animations.emplace_back(std::move(animation));
    }

    static_assert(sizeof(schemas::LightAnimationTarget) == sizeof(LightTarget),
                  "Bad size");
    out_targets = LightTargetSpan(absl::Span<const LightTarget>(
        reinterpret_cast<const LightTarget*>(in_targets->Get(0)),
        in_targets->size()));
  }

  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<GltfAnimation>> GltfAnimation::Create(
    FlatBufferAccess<schemas::GltfAnimation> access) {
  const Vector<const schemas::GltfNodeAnimationTarget*>* in_targets =
      access->node_targets();
  const Vector<Offset<schemas::GltfNodeAnimation>>* in_animations =
      access->node_animations();
  BoneTargetLookup<GltfNodeAnimation> trs_animations;
  BoneTargetSpan trs_targets;
  auto trs_animations_status = CreateTrsAnimations(in_targets, in_animations,
                                                   trs_targets, trs_animations);
  if (!trs_animations_status.ok()) {
    return trs_animations_status;
  }

  const Vector<const schemas::GltfNodeAnimationTarget*>*
      in_morph_target_node_targets = access->morph_target_node_targets();
  const Vector<Offset<schemas::MorphTargetAnimation>>*
      in_morph_target_animations = access->morph_target_animations();
  BoneTargetLookup<MorphTargetAnimation> morph_target_animations;
  BoneTargetSpan morph_target_node_targets;
  auto morph_target_animations_status = CreateMorphTargetAnimations(
      in_morph_target_node_targets, in_morph_target_animations,
      morph_target_node_targets, morph_target_animations);
  if (!morph_target_animations_status.ok()) {
    return morph_target_animations_status;
  }

  const Vector<const schemas::MaterialAnimationTarget*>* in_material_targets =
      access->material_targets();
  const Vector<Offset<schemas::MaterialAnimation>>* in_material_animations =
      access->material_animations();
  MaterialTargetLookup<MaterialAnimation> material_animations;
  MaterialTargetSpan material_targets;
  auto material_animations_status =
      CreateMaterialAnimations(in_material_targets, in_material_animations,
                               material_targets, material_animations);
  if (!material_animations_status.ok()) {
    return material_animations_status;
  }

  const Vector<const schemas::LightAnimationTarget*>* in_light_targets =
      access->light_targets();
  const Vector<Offset<schemas::LightPunctualAnimation>>* in_light_animations =
      access->light_animations();
  LightTargetLookup<LightPunctualAnimation> light_animations;
  LightTargetSpan light_targets;
  absl::Status light_animations_status = CreateLightPunctualAnimations(
      in_light_targets, in_light_animations, light_targets, light_animations);
  if (!light_animations_status.ok()) {
    return light_animations_status;
  }

  return absl::WrapUnique(new GltfAnimation(
      std::move(trs_targets), std::move(trs_animations),
      std::move(morph_target_node_targets), std::move(morph_target_animations),
      std::move(material_targets), std::move(material_animations),
      std::move(light_targets), std::move(light_animations),
      std::move(access)));
}

GltfAnimation::Cursor GltfAnimation::CreateCursor() const {
  TRSCursor trs(trs_targets_.size());
  WeightsCursor weights(morph_target_animation_targets_.size());
  MaterialCursor material_cursor(material_animation_targets_.size());
  LightPunctualCursor light_cursor(light_punctual_animation_targets_.size());
  return {trs, weights, material_cursor, light_cursor};
}

GltfAnimation::BoneTargetLookup<Transform<float>>
GltfAnimation::EvaluateTransform(
    float t, TRSCursor* cursor,
    MissingTransformProvider* missing_transform_provider) const {
  BoneTargetLookup<Transform<float>> result;
  result.reserve(trs_targets_.size());
  assert(cursor && cursor->size() == trs_animations_.size());

  if (missing_transform_provider) {
    MissingChannelProvider missing_channel_provider(missing_transform_provider);
    TRSCursor& cursor_ref = *cursor;
    absl::c_transform(trs_targets_.Ids<BoneTargetId>(),
                      std::back_inserter(result),
                      [this, t, &cursor_ref,
                       &missing_channel_provider](BoneTargetId id) mutable {
                        missing_channel_provider.SetBone(trs_targets_[id].bone);
                        return trs_animations_[id].Eval(
                            t, &cursor_ref[id], &missing_channel_provider);
                      });
  } else {
    std::transform(trs_animations_.begin(), trs_animations_.end(),
                   cursor->begin(), std::back_inserter(result),
                   [t](const GltfNodeAnimation& animation,
                       GltfNodeAnimation::Cursor& cursor) -> Transform<float> {
                     return animation.Eval(t, &cursor);
                   });
  }
  return result;
}

GltfAnimation::BoneTargetLookup<std::array<float, 256>>
GltfAnimation::EvaluateMorphTargetAnimations(float t,
                                             WeightsCursor* cursor) const {
  BoneTargetLookup<std::array<float, 256>> result;
  result.reserve(morph_target_animation_targets_.size());
  assert(cursor && cursor->size() == morph_target_animations_.size());
  std::transform(morph_target_animations_.begin(),
                 morph_target_animations_.end(), cursor->begin(),
                 std::back_inserter(result),
                 [t](const MorphTargetAnimation& animation,
                     MorphTargetAnimation::WeightsCursor& weights_cursor)
                     -> std::array<float, 256> {
                   return animation.Evaluate(t, &weights_cursor);
                 });
  return result;
}

// TODO: add unit test for this function to test that material
// animation data is loaded correctly.
GltfAnimation::MaterialTargetLookup<MaterialAnimation::MaterialParameter>
GltfAnimation::EvaluateMaterialParameters(float t,
                                          MaterialCursor* cursor) const {
  MaterialTargetLookup<MaterialParameter> result;
  result.reserve(material_animation_targets_.size());
  assert(cursor && cursor->size() == material_animations_.size());

  std::transform(material_animations_.begin(), material_animations_.end(),
                 cursor->begin(), std::back_inserter(result),
                 [t](const MaterialAnimation& animation,
                     MaterialAnimation::Cursor& cursor) -> MaterialParameter {
                   return animation.Eval(t, &cursor);
                 });
  return result;
}

GltfAnimation::LightTargetLookup<LightPunctualAnimation::LightParameter>
GltfAnimation::EvaluateLightParameters(float t,
                                       LightPunctualCursor* cursor) const {
  LightTargetLookup<LightParameter> result;
  result.reserve(light_punctual_animation_targets_.size());
  assert(cursor && cursor->size() == light_punctual_animations_.size());

  std::transform(light_punctual_animations_.begin(),
                 light_punctual_animations_.end(), cursor->begin(),
                 std::back_inserter(result),
                 [t](const LightPunctualAnimation& animation,
                     LightPunctualAnimation::Cursor& cursor) -> LightParameter {
                   return animation.Eval(t, &cursor);
                 });
  return result;
}

bool GltfAnimation::SanitizeT(bool repeat, absl::Duration* t) const {
  return SanitizeT(repeat, t, FirstT(), LastT());
}

bool GltfAnimation::SanitizeT(bool repeat, absl::Duration* t,
                              absl::Duration start_time) const {
  return SanitizeT(repeat, t, start_time, LastT());
}

bool GltfAnimation::SanitizeT(bool repeat, absl::Duration* t,
                              absl::Duration start_time,
                              absl::Duration end_time) const {
  absl::Duration animation_start = FirstT();
  absl::Duration animation_end = LastT();
  absl::Duration first_t = clamp(start_time, animation_start, animation_end);
  absl::Duration last_t = clamp(end_time, animation_start, animation_end);
  const bool last_before_first = last_t < first_t;
  if ((last_before_first && *t > first_t) ||
      (!last_before_first && *t < first_t)) {
    // Emulate pre-existing behavior: ticking an animator with a time before the
    // start time (e.g. a negative time) cause the animation to immediately end.
    return true;
  }

  if ((last_before_first && *t > last_t) ||
      (!last_before_first && *t < last_t)) {
    return false;
  }

  if (auto duration = last_before_first ? first_t - last_t : last_t - first_t;
      repeat && duration > absl::ZeroDuration()) {
    absl::Duration start_t = last_before_first ? last_t : first_t;
    *t = start_t + ((*t - start_t) % duration);
  } else {
    *t = last_t;
  }
  return true;
}

absl::Duration GltfAnimation::FirstT() const {
  return absl::Seconds(access_->first_t());
}
absl::Duration GltfAnimation::LastT() const {
  return absl::Seconds(access_->last_t());
}
absl::Duration GltfAnimation::Duration() const { return LastT() - FirstT(); }

GltfAnimation::GltfAnimation(
    BoneTargetSpan trs_targets,
    BoneTargetLookup<GltfNodeAnimation> trs_animations,
    BoneTargetSpan mt_targets,
    BoneTargetLookup<MorphTargetAnimation> mt_animations,
    MaterialTargetSpan material_animation_targets,
    MaterialTargetLookup<MaterialAnimation> material_animations,
    LightTargetSpan light_punctual_animation_targets,
    LightTargetLookup<LightPunctualAnimation> light_punctual_animations,
    FlatBufferAccess<schemas::GltfAnimation> access)
    : trs_targets_(std::move(trs_targets)),
      trs_animations_(std::move(trs_animations)),
      morph_target_animation_targets_(std::move(mt_targets)),
      morph_target_animations_(std::move(mt_animations)),
      material_animation_targets_(std::move(material_animation_targets)),
      material_animations_(std::move(material_animations)),
      light_punctual_animation_targets_(
          std::move(light_punctual_animation_targets)),
      light_punctual_animations_(std::move(light_punctual_animations)),
      access_(std::move(access)) {}

GltfAnimation::Builder::Builder() noexcept
    : first_t_(std::numeric_limits<float>::max()),
      last_t_(std::numeric_limits<float>::lowest()) {}

GltfAnimation::Builder::Builder(Builder&& rhs) noexcept = default;
GltfAnimation::Builder::~Builder() noexcept = default;
GltfAnimation::Builder& GltfAnimation::Builder::operator=(
    Builder&& rhs) noexcept = default;

GltfAnimation::Builder& GltfAnimation::Builder::Name(
    absl::string_view name) noexcept {
  name_ = std::string(name);
  return *this;
}

flatbuffers::Offset<flatbuffers::Vector<const schemas::FrameTime*>>
GltfAnimation::Builder::CreateTimes(absl::Span<float> times) {
  if (!times.empty()) {
    first_t_ = std::min(first_t_, times.front());
    last_t_ = std::max(last_t_, times.back());
  }
  std::function<void(size_t i, schemas::FrameTime*)> get_times =
      [&times](size_t i, schemas::FrameTime* dest) {
        *dest = schemas::FrameTime(times[i]);
      };
  return fbb_.CreateVectorOfStructs(times.size(), get_times);
}

template <CurveType kType, typename V>
auto GltfAnimation::Builder::CreateUnion(absl::Span<float> times,
                                         absl::Span<V> values) {
  using Traits = CurveTypeTraits<kType, V>;

  if constexpr (kType == CurveType::kCubicSpline) {
    assert(times.size() == values.size() / 3 &&
           "The number of CubicSpline values must be 3x the number of times. "
           "Every three elements represents the in tangent, position, and out "
           "tangent of a frame respectively.");
  } else {
    assert(times.size() == values.size() &&
           "The number of values must much the number of times.");
  }

  // Build the 'times' and 'values' offsets before the containing type.
  auto times_offset = CreateTimes(times);
  auto values_offset =
      CreateValues<kType, V>(fbb_, values.data(), values.size());
  typename Traits::Builder builder(fbb_);
  builder.add_values(values_offset);
  builder.add_times(times_offset);
  return TypedUnion<typename Traits::ChannelType>{Traits::kChannel,
                                                  builder.Finish().Union()};
}

GltfAnimation::Builder& GltfAnimation::Builder::Translation(
    model::BoneId bone_id, float3 constant_value) {
  bone_targets_.emplace(bone_id);

  translations_[bone_id] = TranslationUnion{
      schemas::ChannelFloat3::kConstant,
      fbb_.CreateStruct(
              schemas::ConstantFloat3(flatbuffers::Pack(constant_value)))
          .Union()};

  return *this;
}

GltfAnimation::Builder& GltfAnimation::Builder::Translation(
    model::BoneId bone_id, absl::Span<float> times,
    absl::Span<float3> curve_values, CurveType curve_type) {
  bone_targets_.emplace(bone_id);

  switch (curve_type) {
    case CurveType::kStep:
      translations_[bone_id] =
          CreateUnion<CurveType::kStep>(times, curve_values);
      break;
    case CurveType::kLinear:
      translations_[bone_id] =
          CreateUnion<CurveType::kLinear>(times, curve_values);
      break;
    case CurveType::kCubicSpline:
      translations_[bone_id] =
          CreateUnion<CurveType::kCubicSpline>(times, curve_values);
      break;
  }

  return *this;
}

GltfAnimation::Builder& GltfAnimation::Builder::Rotation(model::BoneId bone_id,
                                                         quatf constant_value) {
  bone_targets_.emplace(bone_id);

  rotations_[bone_id] = RotationUnion{
      schemas::ChannelQuatf::kConstant,
      fbb_.CreateStruct(
              schemas::ConstantQuatf(flatbuffers::Pack(constant_value)))
          .Union()};

  return *this;
}
GltfAnimation::Builder& GltfAnimation::Builder::Rotation(
    model::BoneId bone_id, absl::Span<float> times,
    absl::Span<quatf> curve_values, CurveType curve_type) {
  bone_targets_.emplace(bone_id);

  switch (curve_type) {
    case CurveType::kStep:
      rotations_[bone_id] = CreateUnion<CurveType::kStep>(times, curve_values);
      break;
    case CurveType::kLinear:
      rotations_[bone_id] =
          CreateUnion<CurveType::kLinear>(times, curve_values);
      break;
    case CurveType::kCubicSpline:
      rotations_[bone_id] =
          CreateUnion<CurveType::kCubicSpline>(times, curve_values);
      break;
  }

  return *this;
}

GltfAnimation::Builder& GltfAnimation::Builder::Scale(model::BoneId bone_id,
                                                      float3 constant_value) {
  bone_targets_.emplace(bone_id);

  scales_[bone_id] = ScaleUnion{
      schemas::ChannelFloat3::kConstant,
      fbb_.CreateStruct(
              schemas::ConstantFloat3(flatbuffers::Pack(constant_value)))
          .Union()};

  return *this;
}
GltfAnimation::Builder& GltfAnimation::Builder::Scale(
    model::BoneId bone_id, absl::Span<float> times,
    absl::Span<float3> curve_values, CurveType curve_type) {
  bone_targets_.emplace(bone_id);

  switch (curve_type) {
    case CurveType::kStep:
      scales_[bone_id] = CreateUnion<CurveType::kStep>(times, curve_values);
      break;
    case CurveType::kLinear:
      scales_[bone_id] = CreateUnion<CurveType::kLinear>(times, curve_values);
      break;
    case CurveType::kCubicSpline:
      scales_[bone_id] =
          CreateUnion<CurveType::kCubicSpline>(times, curve_values);
      break;
  }

  return *this;
}

absl::StatusOr<std::unique_ptr<GltfAnimation>> GltfAnimation::Builder::Build() {
  std::vector<flatbuffers::Offset<schemas::GltfNodeAnimation>> node_animations;
  node_animations.reserve(bone_targets_.size());
  std::vector<schemas::GltfNodeAnimationTarget> node_animation_targets;
  node_animation_targets.reserve(bone_targets_.size());

  for (model::BoneId bone_id : bone_targets_) {
    node_animation_targets.push_back(uint16_t{bone_id});

    const TranslationUnion& t = translations_[bone_id];
    const RotationUnion& r = rotations_[bone_id];
    const ScaleUnion& s = scales_[bone_id];

    node_animations.push_back(schemas::CreateGltfNodeAnimation(
        fbb_, t.type, t.offset, r.type, r.offset, s.type, s.offset));
  }

  std::vector<flatbuffers::Offset<schemas::MorphTargetAnimation>>
      morph_target_animations;
  std::vector<schemas::GltfNodeAnimationTarget> morph_target_animation_targets;

  std::vector<flatbuffers::Offset<schemas::MaterialAnimation>>
      material_animations;
  std::vector<schemas::MaterialAnimationTarget> material_animation_targets;

  std::vector<flatbuffers::Offset<schemas::LightPunctualAnimation>>
      light_animations;
  std::vector<schemas::LightAnimationTarget> light_animation_targets;

  auto root = CreateGltfAnimation(
      fbb_, fbb_.CreateString(name_), first_t_, last_t_,
      fbb_.CreateVector(node_animations),
      fbb_.CreateVectorOfStructs(node_animation_targets),
      fbb_.CreateVector(morph_target_animations),
      fbb_.CreateVectorOfStructs(morph_target_animation_targets),
      fbb_.CreateVector(material_animations),
      fbb_.CreateVectorOfStructs(material_animation_targets),
      fbb_.CreateVector(light_animations),
      fbb_.CreateVectorOfStructs(light_animation_targets));

  fbb_.Finish(root, "ImpA");
  FlatBufferAccess<schemas::GltfAnimation> fb_anim;
  MP_RETURN_IF_ERROR(CreateFlatBufferAccess(&fbb_, &fb_anim));
  return GltfAnimation::Create(std::move(fb_anim));
}

GltfAnimation::MissingChannelProvider::MissingChannelProvider(
    GltfAnimation::MissingTransformProvider* missing_channel_provider)
    : missing_transform_provider_(missing_channel_provider),
      bone_(BoneId::At(0)) {}

float3 GltfAnimation::MissingChannelProvider::GetMissingTranslation() {
  return GetTransform().translation;
}
quatf GltfAnimation::MissingChannelProvider::GetMissingRotation() {
  return GetTransform().rotation;
}
float3 GltfAnimation::MissingChannelProvider::GetMissingScale() {
  return GetTransform().scale;
}

void GltfAnimation::MissingChannelProvider::SetBone(BoneId bone) {
  cached_transform_.reset();
  bone_ = bone;
}

Transform<float>& GltfAnimation::MissingChannelProvider::GetTransform() {
  if (cached_transform_.has_value()) {
    return cached_transform_.value();
  }

  cached_transform_ = missing_transform_provider_->GetMissingTransform(bone_);
  return cached_transform_.value();
}

}  // namespace imp::animation
