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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_ANIMATION_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_ANIMATION_H_

#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/vector.h"
#include "core/animation/gltf_node_animation.h"
#include "core/animation/light_punctual_animation.h"
#include "core/animation/material_animation.h"
#include "core/animation/morph_target_animation.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/paired_vector.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"
#include "core/common/typed_id.h"
#include "core/common/typed_span.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"

namespace imp::animation {

// Provides a typed dressing atop schemas::GltfAnimation
class GltfAnimation {
 public:
  using BoneId = model::BoneId;
  struct BoneTarget {
    const BoneId bone;
  };
  using MaterialTarget = model::ModelData::MaterialId;
  using LightTarget = model::ModelData::LightPunctualId;

  struct MissingTransformProvider {
    virtual ~MissingTransformProvider() {}

    virtual Transform<float> GetMissingTransform(BoneId bone) = 0;
  };

  class Builder {
   public:
    explicit Builder() noexcept;

    Builder(Builder const& rhs) = delete;
    Builder(Builder&& rhs) noexcept;
    ~Builder() noexcept;
    Builder& operator=(Builder& rhs) = delete;
    Builder& operator=(Builder&& rhs) noexcept;

    Builder& Name(absl::string_view name) noexcept;

    Builder& Translation(model::BoneId, float3 constant_value);
    Builder& Translation(model::BoneId, absl::Span<float> times,
                         absl::Span<float3> curve_values,
                         CurveType curve_type = CurveType::kLinear);

    Builder& Rotation(model::BoneId, quatf constant_value);
    Builder& Rotation(model::BoneId, absl::Span<float> times,
                      absl::Span<quatf> curve_values,
                      CurveType curve_type = CurveType::kLinear);

    Builder& Scale(model::BoneId, float3 constant_value);
    Builder& Scale(model::BoneId, absl::Span<float> times,
                   absl::Span<float3> curve_values,
                   CurveType curve_type = CurveType::kLinear);

    // TODO: Add Weights Builder

    absl::StatusOr<std::unique_ptr<GltfAnimation>> Build();

   private:
    template <typename Enum>
    struct TypedUnion {
      Enum type = Enum::NONE;
      flatbuffers::Offset<void> offset = 0;
    };
    using TranslationUnion = TypedUnion<schemas::ChannelFloat3>;
    using RotationUnion = TypedUnion<schemas::ChannelQuatf>;
    using ScaleUnion = TypedUnion<schemas::ChannelFloat3>;

    template <CurveType kType, typename V>
    auto CreateUnion(absl::Span<float> times, absl::Span<V> values);

    flatbuffers::Offset<flatbuffers::Vector<const schemas::FrameTime*>>
    CreateTimes(absl::Span<float> times);

    flatbuffers::FlatBufferBuilder fbb_;
    RobinMap<model::BoneId, TranslationUnion> translations_;
    RobinMap<model::BoneId, RotationUnion> rotations_;
    RobinMap<model::BoneId, ScaleUnion> scales_;
    RobinSet<model::BoneId> bone_targets_;
    float first_t_;
    float last_t_;
    std::string name_;
  };

  static absl::StatusOr<std::unique_ptr<GltfAnimation>> Create(
      FlatBufferAccess<schemas::GltfAnimation> access);

  using BoneTargetId = TypedId<const BoneTarget, int16_t>;
  using BoneTargetSpan = TypedSpan<const BoneTarget>;
  template <typename T>
  using BoneTargetLookup = PairedVector<T, const BoneTarget>;

  using MaterialTargetId = TypedId<const MaterialTarget, int16_t>;
  template <typename T>
  using MaterialTargetLookup = PairedVector<T, const MaterialTarget>;
  using MaterialTargetSpan = TypedSpan<const MaterialTarget>;

  using LightTargetId = TypedId<const LightTarget, int16_t>;
  template <typename T>
  using LightTargetLookup = PairedVector<T, const LightTarget>;
  using LightTargetSpan = TypedSpan<const LightTarget>;

  using TRSCursor = BoneTargetLookup<GltfNodeAnimation::Cursor>;
  using WeightsCursor = BoneTargetLookup<MorphTargetAnimation::WeightsCursor>;
  using MaterialCursor = MaterialTargetLookup<MaterialAnimation::Cursor>;
  using LightPunctualCursor = LightTargetLookup<LightPunctualAnimation::Cursor>;
  struct Cursor {
    // Cursor for animation of the translation/rotation/scale channel.
    // Call with EvaluateTransform(t, cursor.trs).
    TRSCursor trs;
    // Cursor for the animation of the weights channel.
    // Call with EvaluateTransform(t, cursor.weights).
    WeightsCursor weights;
    // Cursor for the animation of the material parameter channels.
    // Call with EvaluateTransform(t, cursor.material_parameters).
    MaterialCursor material_parameters;
    // Cursor for the animation of the light punctual channels.
    // Call with EvaluateTransform(t, cursor.light_punctuals).
    LightPunctualCursor light_punctuals;
  };

  Cursor CreateCursor() const;
  // Evaluates t/r/s transform values for bone animations.
  // glTF animation data accessors can only be single-precision floats, so no
  // need for us to handle PreciseTransforms here.
  // https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#accessor-data-types
  BoneTargetLookup<Transform<float>> EvaluateTransform(
      float t, TRSCursor* cursor,
      MissingTransformProvider* missing_transform_provider = nullptr) const;
  // Returns the bone targets for the t/r/s animations.
  const GltfAnimation::BoneTargetSpan& TransformTargets() const {
    return trs_targets_;
  }
  // Evaluates the weights values for morph target animations.
  BoneTargetLookup<std::array<float, 256>> EvaluateMorphTargetAnimations(
      float t, WeightsCursor* cursor) const;
  // Returns the bone targets for morph target animations.
  const BoneTargetSpan& MorphTargetAnimationTargets() const {
    return morph_target_animation_targets_;
  }

  // Evaluates the material parameter for material animations.
  GltfAnimation::MaterialTargetLookup<MaterialAnimation::MaterialParameter>
  EvaluateMaterialParameters(float t, MaterialCursor* cursor) const;
  const MaterialTargetSpan& MaterialAnimationTargets() const {
    return material_animation_targets_;
  }

  // Evaluates the light punctual for light punctual animations.
  GltfAnimation::LightTargetLookup<LightPunctualAnimation::LightParameter>
  EvaluateLightParameters(float t, LightPunctualCursor* cursor) const;
  const LightTargetSpan& LightPunctualAnimationTargets() const {
    return light_punctual_animation_targets_;
  }

  absl::Duration FirstT() const;
  absl::Duration LastT() const;
  absl::Duration Duration() const;

  bool SanitizeT(bool repeat, absl::Duration* t) const;

 private:
  using MaterialParameter = MaterialAnimation::MaterialParameter;
  using LightParameter = LightPunctualAnimation::LightParameter;

  class MissingChannelProvider
      : public GltfNodeAnimation::MissingChannelProvider {
   public:
    explicit MissingChannelProvider(
        MissingTransformProvider* missing_channel_provider);

    float3 GetMissingTranslation() override;
    quatf GetMissingRotation() override;
    float3 GetMissingScale() override;

    void SetBone(BoneId bone);

   private:
    Transform<float>& GetTransform();

    MissingTransformProvider* missing_transform_provider_;

    absl::optional<Transform<float>> cached_transform_;
    BoneId bone_;
  };

  static absl::Status CreateTrsAnimations(
      const flatbuffers::Vector<const schemas::GltfNodeAnimationTarget*>*
          in_targets,
      const flatbuffers::Vector<
          flatbuffers::Offset<schemas::GltfNodeAnimation>>* in_animations,
      BoneTargetSpan& out_targets,
      BoneTargetLookup<GltfNodeAnimation>& out_animations);

  static absl::Status CreateMorphTargetAnimations(
      const flatbuffers::Vector<const schemas::GltfNodeAnimationTarget*>*
          in_targets,
      const flatbuffers::Vector<
          flatbuffers::Offset<schemas::MorphTargetAnimation>>* in_animations,
      BoneTargetSpan& out_targets,
      BoneTargetLookup<MorphTargetAnimation>& out_animations);

  static absl::Status CreateMaterialAnimations(
      const flatbuffers::Vector<const schemas::MaterialAnimationTarget*>*
          in_targets,
      const flatbuffers::Vector<
          flatbuffers::Offset<schemas::MaterialAnimation>>* in_animations,
      MaterialTargetSpan& out_targets,
      MaterialTargetLookup<MaterialAnimation>& out_animations);

  static absl::Status CreateLightPunctualAnimations(
      const flatbuffers::Vector<const schemas::LightAnimationTarget*>*
          in_targets,
      const flatbuffers::Vector<
          flatbuffers::Offset<schemas::LightPunctualAnimation>>* in_animations,
      LightTargetSpan& out_targets,
      LightTargetLookup<LightPunctualAnimation>& out_animations);

  GltfAnimation(
      BoneTargetSpan trs_targets,
      BoneTargetLookup<GltfNodeAnimation> trs_animations,
      BoneTargetSpan mt_targets,
      BoneTargetLookup<MorphTargetAnimation> mt_animations,
      MaterialTargetSpan material_animation_targets,
      MaterialTargetLookup<MaterialAnimation> material_animations,
      LightTargetSpan light_punctual_animation_targets,
      LightTargetLookup<LightPunctualAnimation> light_punctual_animations,
      FlatBufferAccess<schemas::GltfAnimation> access);

  BoneTargetSpan trs_targets_;
  BoneTargetLookup<GltfNodeAnimation> trs_animations_;
  BoneTargetSpan morph_target_animation_targets_;
  BoneTargetLookup<MorphTargetAnimation> morph_target_animations_;
  // TODO: Release GltfAnimation's dependency to MaterialAnimation.
  MaterialTargetSpan material_animation_targets_;
  MaterialTargetLookup<MaterialAnimation> material_animations_;
  LightTargetSpan light_punctual_animation_targets_;
  LightTargetLookup<LightPunctualAnimation> light_punctual_animations_;
  FlatBufferAccess<schemas::GltfAnimation> access_;
};

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_ANIMATION_H_
