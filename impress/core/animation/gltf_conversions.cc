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

#include "core/animation/gltf_conversions.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "flatbuffers/array.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "core/animation/gltf_conversions_animation_pointer.h"
#include "core/animation/gltf_conversions_helper.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/enum_flags.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/typed_span.h"
#include "core/loader/provider/gltf/accessor_reader.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/loader/provider/gltf/gltf_lookup.h"
#include "core/math/math.h"
#include "core/model/skeleton_data.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::animation {
namespace {

using imp::gltf::imp_proto::AnimationSampler;
using imp::loader::details::AccessorReader;
using imp::loader::details::DenseDataAccess;
using imp::loader::details::provider_gltf::AnimationId;
using imp::loader::details::provider_gltf::ChannelId;
using imp::loader::details::provider_gltf::GltfLookup;
using imp::loader::details::provider_gltf::MaterialId;
using imp::loader::details::provider_gltf::NodeId;

template <typename ValueType>
flatbuffers::Offset<flatbuffers::Vector<const ValueType*>> CreateWeightsVector(
    absl::Span<const float> data, flatbuffers::FlatBufferBuilder* fbb,
    int values_per_keyframe) {
  auto channels = data.size() / values_per_keyframe;
  const int num_comp = std::min(static_cast<int>(filament::MAX_MORPH_TARGETS),
                                values_per_keyframe);

  std::function<void(size_t i, ValueType* dest)> get_values =
      [data, num_comp, values_per_keyframe](size_t i, ValueType* dest) {
        // ValueType (e.g. LinearFrameFloatVector) is a flatbuffer struct
        // with at least one field `position`: fixed array of floats.
        //
        // Default constructor of such type will default-initialize `position`.
        //
        // However, memory at `dest` is not initialized, because this lambda
        // got called by flatbuffers::CreateVectorOfStructs and that function
        // just allocates chunk of memory to fit `channels` number of structs
        // without performing any initialization.
        //
        // Cycle below may not write to all elements in `position` and in turn
        // memory sanitizer will complain when all elements of `position` are
        // accessed (e.g. serialization).
        //
        // We can just call memset:
        //
        //  memset(const_cast<float*>(dest->position()->data()), 0,
        //        dest->position()->size() * sizeof(float));
        //
        // But this may add extra work in the long run (e.g. if new field is
        // added).
        //
        // So, let's just call `placement new` and rely on the logic of default
        // constructor.
        //
        // NOTE: `placement new` does not allocate new memory, it just
        // constructs ValueType at `dest`.
        dest = new (dest) ValueType();

        float* comp_pos = const_cast<float*>(dest->position()->data());

        for (int comp = 0; comp < num_comp; comp++) {
          comp_pos[comp] = data[comp + i * values_per_keyframe];
        }
      };
  return fbb->CreateVectorOfStructs(channels, get_values);
}

absl::StatusOr<flatbuffers::Offset<void>> SerializeWeights(
    imp::animation::schemas::ChannelFloatVector type,
    absl::Span<const float> time_data, absl::Span<const float> value_data,
    flatbuffers::FlatBufferBuilder* fbb) {
  if (value_data.size() % time_data.size() != 0) {
    return absl::InternalError("Invalid value_data/time_data size: channel");
  }
  const int values_per_keyframe = value_data.size() / time_data.size();

  auto times = CreateVector<imp::animation::schemas::FrameTime>(time_data, fbb);
  switch (type) {
    case imp::animation::schemas::ChannelFloatVector::kStep: {
      auto values =
          CreateWeightsVector<imp::animation::schemas::StepFrameFloatVector>(
              value_data, fbb, values_per_keyframe);
      return imp::animation::schemas::CreateStepCurveFloatVector(*fbb, times,
                                                                 values)
          .Union();
      break;
    }
    case imp::animation::schemas::ChannelFloatVector::kLinear: {
      auto values =
          CreateWeightsVector<imp::animation::schemas::LinearFrameFloatVector>(
              value_data, fbb, values_per_keyframe);
      return imp::animation::schemas::CreateLinearCurveFloatVector(*fbb, times,
                                                                   values)
          .Union();
      break;
    }
    case imp::animation::schemas::ChannelFloatVector::kCubic: {
      if (values_per_keyframe % 3 != 0) {
        return absl::InternalError("Invalid channel");
      }
      const int num_morph_targets = values_per_keyframe / 3;
      const float* const in_tangents = &value_data[0];
      const float* const spline_verts = &value_data[num_morph_targets];
      const float* const out_tangents = &value_data[num_morph_targets * 2];

      auto channels = value_data.size() / values_per_keyframe;
      const int num_comp = std::min(
          static_cast<int>(filament::MAX_MORPH_TARGETS), num_morph_targets);
      std::function<void(size_t i,
                         imp::animation::schemas::CubicFrameFloatVector* dest)>
          get_values = [num_comp, values_per_keyframe, in_tangents,
                        spline_verts, out_tangents](
                           size_t i,
                           imp::animation::schemas::CubicFrameFloatVector*
                               dest) mutable {
            // See detailed comment in `CreateWeightsVector`.
            //
            // Call default constructor via `placement new` to
            // default-initialize `in_tangent`, `position`, `out_tangent` and
            // prevent potential issues with memory sanitizer.
            dest = new (dest) imp::animation::schemas::CubicFrameFloatVector();

            float* comp_in = const_cast<float*>(dest->in_tangent()->data());
            float* comp_pos = const_cast<float*>(dest->position()->data());
            float* comp_out = const_cast<float*>(dest->out_tangent()->data());

            for (int comp = 1; comp < num_comp; comp++) {
              comp_in[comp] = in_tangents[comp + i * values_per_keyframe];
              comp_pos[comp] = spline_verts[comp + i * values_per_keyframe];
              comp_out[comp] =
                  i == 0 ? out_tangents[comp + i * values_per_keyframe]
                         : out_tangents[comp + (i - 1) * values_per_keyframe];
            }
          };
      auto values = fbb->CreateVectorOfStructs(channels, get_values);
      return imp::animation::schemas::CreateCubicCurveFloatVector(*fbb, times,
                                                                  values)
          .Union();
      break;
    }
    default: {
      break;
    }
  }
  return absl::InternalError("Invalid morph target animation type");
}

template <typename T>
OptionalError AddWeightsChannel(const imp::gltf::imp_proto::Gltf& gltf,
                                const AnimationSampler sampler,
                                flatbuffers::FlatBufferBuilder* fbb,
                                T* out_type,
                                flatbuffers::Offset<void>* out_union,
                                absl::optional<Domain>* out_domain) {
  if (!sampler.output) return absl::InternalError("Invalid sampler");
  MP_ASSIGN_OR_RETURN(AccessorReader reader,
                   AccessorReader::Create(gltf, *sampler.output));
  DenseDataAccess weights;

  // According to the spec, vector animations must be 3d floating point values.
  if (reader.GetComponentType() != imp::gltf::imp_proto::ComponentType::FLOAT ||
      reader.GetType() != ExpectedLayoutType<T>()) {
    // Special case: support normalized UNSIGNED_BYTE weights.
    if (reader.GetComponentType() ==
            imp::gltf::imp_proto::ComponentType::UNSIGNED_BYTE &&
        reader.GetType() == "SCALAR") {
      MP_ASSIGN_OR_RETURN(weights, reader.GetPackedFloatData());
    } else {
      return absl::InternalError("Vector Channel output format incorrect");
    }
  } else {
    weights = reader.GetPackedData();
  }

  MP_ASSIGN_OR_RETURN(absl::Span<const float> time_data,
                   GetTimeData(gltf, sampler));
  MP_ASSIGN_OR_RETURN(T curve_type, GetCurveType<T>(sampler.interpolation));

  if (weights.GetCount() % time_data.size() != 0) {
    return absl::InternalError("Invalid value_data/time_data size: channel");
  }
  auto value_data =
      absl::Span<const float>(weights.ReadRawData<float>(), reader.GetCount());
  MP_RETURN_IF_ERROR(ValidateKeyframeData(time_data, value_data));
  MP_ASSIGN_OR_RETURN(flatbuffers::Offset<void> curve_union,
                   SerializeWeights(curve_type, time_data, value_data, fbb));

  *out_type = curve_type;
  *out_union = curve_union;
  out_domain->emplace(Domain{time_data.front(), time_data.back()});
  return NoError();
}

OptionalError SerializeSqtAnimation(
    const imp::gltf::imp_proto::Gltf& gltf, const GltfLookup& lookup,
    NodeId node, AnimationId animation, flatbuffers::FlatBufferBuilder* fbb,
    absl::optional<flatbuffers::Offset<animation::schemas::GltfNodeAnimation>>*
        out_offset,
    absl::optional<
        flatbuffers::Offset<animation::schemas::MorphTargetAnimation>>*
        out_mt_offset,
    absl::optional<Domain>* out_domain) {
  const GltfLookup::ChannelSet& channel_set = lookup.channel_sets[animation];
  ChannelId translation_channel =
      GltfLookup::GetValueOrDefault(channel_set.translation_channels, node);
  ChannelId rotation_channel =
      GltfLookup::GetValueOrDefault(channel_set.rotation_channels, node);
  ChannelId scale_channel =
      GltfLookup::GetValueOrDefault(channel_set.scale_channels, node);
  ChannelId weights_channel =
      GltfLookup::GetValueOrDefault(channel_set.weights_channels, node);
  TypedSpan<const imp::gltf::imp_proto::AnimationChannel> animation_channels(
      lookup.animations[animation].channels);
  const std::vector<AnimationSampler>& animation_samplers =
      lookup.animations[animation].samplers;

  absl::optional<Domain> trs_domain;
  absl::optional<Domain> weights_domain;

  if (translation_channel || rotation_channel || scale_channel) {
    schemas::ChannelFloat3 translation_type = schemas::ChannelFloat3::NONE;
    flatbuffers::Offset<void> translation;
    absl::optional<Domain> translation_domain;

    if (translation_channel) {
      MP_RETURN_IF_ERROR(AddChannel(
          gltf,
          animation_samplers[*animation_channels[translation_channel].sampler],
          fbb, &translation_type, &translation, &translation_domain));
    }
    schemas::ChannelQuatf rotation_type = schemas::ChannelQuatf::NONE;
    flatbuffers::Offset<void> rotation;
    absl::optional<Domain> rotation_domain;
    if (rotation_channel) {
      MP_RETURN_IF_ERROR(AddChannel(
          gltf,
          animation_samplers[*animation_channels[rotation_channel].sampler],
          fbb, &rotation_type, &rotation, &rotation_domain));
    }

    schemas::ChannelFloat3 scale_type = schemas::ChannelFloat3::NONE;
    flatbuffers::Offset<void> scale;
    absl::optional<Domain> scale_domain;
    if (scale_channel) {
      MP_RETURN_IF_ERROR(AddChannel(
          gltf, animation_samplers[*animation_channels[scale_channel].sampler],
          fbb, &scale_type, &scale, &scale_domain));
    }

    out_offset->emplace(schemas::CreateGltfNodeAnimation(
        *fbb, translation_type, translation, rotation_type, rotation,
        scale_type, scale));
    trs_domain = MergeDomains(MergeDomains(translation_domain, rotation_domain),
                              scale_domain);
    *out_domain = trs_domain;
  }

  if (weights_channel) {
    schemas::ChannelFloatVector weights_type =
        schemas::ChannelFloatVector::NONE;
    flatbuffers::Offset<void> weights;

    const imp::gltf::imp_proto::AnimationChannel& channel =
        animation_channels[weights_channel];
    std::optional<int32_t> additive_weight_index =
        channel.extras.has_value() ? channel.extras->additive_weight_index
                                   : std::nullopt;

    MP_RETURN_IF_ERROR(
        AddWeightsChannel(gltf, animation_samplers[*channel.sampler], fbb,
                          &weights_type, &weights, &weights_domain));
    out_mt_offset->emplace(animation::schemas::CreateMorphTargetAnimation(
        *fbb, weights_type, weights, additive_weight_index));
    *out_domain = weights_domain;
  }

  if (trs_domain.has_value() && weights_domain.has_value()) {
    *out_domain = MergeDomains(trs_domain, weights_domain);
  }

  return NoError();
}

}  // namespace

absl::StatusOr<FlatBufferAccess<schemas::GltfAnimation>> GetAnimation(
    const imp::gltf::imp_proto::Gltf& gltf,
    const imp::loader::details::provider_gltf::GltfLookup& lookup,
    imp::loader::details::provider_gltf::AnimationId animation) {
  flatbuffers::FlatBufferBuilder fbb;

  // Prepare the components of the animation.
  std::vector<flatbuffers::Offset<animation::schemas::GltfNodeAnimation>>
      node_animations;
  std::vector<animation::schemas::GltfNodeAnimationTarget> node_targets;
  std::vector<flatbuffers::Offset<animation::schemas::MorphTargetAnimation>>
      mt_animations;
  std::vector<animation::schemas::GltfNodeAnimationTarget> mt_targets;
  std::vector<flatbuffers::Offset<animation::schemas::MaterialAnimation>>
      material_animations;
  std::vector<animation::schemas::MaterialAnimationTarget> material_targets;

  enum class ScratchFlags : uint8_t {
    kInThisAnim = (1 << 0),
  };
  GltfLookup::NodeLookup<Flags<ScratchFlags>> node_scratch_flags;
  GltfLookup::MaterialLookup<Flags<ScratchFlags>> material_scratch_flags;
  node_scratch_flags.Pair(gltf.nodes);
  material_scratch_flags.Pair(gltf.materials);

  constexpr absl::string_view kPointer = "pointer";
  constexpr absl::string_view kNode = "nodes";
  constexpr absl::string_view kMaterial = "materials";

  for (const imp::gltf::imp_proto::AnimationChannel& c :
       lookup.animations[animation].channels) {
    if (c.target.path != kPointer) {
      if (!c.target.node) {
        return Error("Animation has no target node.");
      }
      NodeId target = NodeId::At(*c.target.node);
      if (node_scratch_flags[target] & ScratchFlags::kInThisAnim) continue;
      node_scratch_flags[target] |= ScratchFlags::kInThisAnim;
    } else if (c.target.extensions.animation_pointer.has_value()) {
      // Using KHR_animation_pointer.

      // Analyzes the target, such as "/nodes/0/weights". In the example,
      // tokens[2] indicates the index of a node, while the animation happens
      // on the morph target weights of that node.
      absl::string_view pointer =
          c.target.extensions.animation_pointer->pointer;
      std::vector<std::string> tokens = absl::StrSplit(pointer, '/');
      if (tokens.size() < 4) {
        return Error("Pointer is too short %.*s, skipped.", pointer.size(),
                     pointer.data());
      }

      if (tokens[1] == kNode) {
        size_t node_id;
        if (!absl::SimpleAtoi(tokens[2], &node_id)) {
          return Error("Invalid node index.");
        }
        NodeId target = NodeId::At(node_id);
        if (node_scratch_flags[target] & ScratchFlags::kInThisAnim) continue;
        node_scratch_flags[target] |= ScratchFlags::kInThisAnim;
      } else if (tokens[1] == kMaterial) {
        size_t material_id;
        if (!absl::SimpleAtoi(tokens[2], &material_id)) {
          return Error("Invalid material index.");
        }
        MaterialId target = MaterialId::At(material_id);
        if (material_scratch_flags[target] & ScratchFlags::kInThisAnim) {
          continue;
        }
        material_scratch_flags[target] |= ScratchFlags::kInThisAnim;
      }
    }
  }

  float first_time = std::numeric_limits<float>::max();
  float last_time = std::numeric_limits<float>::lowest();

  // Go through all the GLTF nodes in order. If a T/R/S or weights animation
  // is found, add the animation and bone target to the respective vector.
  for (const GltfLookup::BoneEntry& bone : lookup.bone_entries) {
    NodeId node_id = bone.node;
    if (!(node_scratch_flags[node_id] & ScratchFlags::kInThisAnim)) continue;

    absl::optional<flatbuffers::Offset<animation::schemas::GltfNodeAnimation>>
        animation_offset;
    absl::optional<
        flatbuffers::Offset<animation::schemas::MorphTargetAnimation>>
        mt_anim_offset;
    absl::optional<Domain> animation_domain;
    MP_RETURN_IF_ERROR(SerializeSqtAnimation(gltf, lookup, node_id, animation,
                                          &fbb, &animation_offset,
                                          &mt_anim_offset, &animation_domain))
        << "Adding animation data";

    if (animation_domain) {
      first_time = std::min(first_time, animation_domain->min);
      last_time = std::max(last_time, animation_domain->max);
    }

    auto bone_target =
        uint16_t{lookup.bone_entries.IdOf(bone).CastTo<model::BoneId>()};
    if (animation_offset.has_value()) {
      node_animations.push_back(animation_offset.value());
      node_targets.push_back(bone_target);
    }
    if (mt_anim_offset.has_value()) {
      mt_animations.push_back(mt_anim_offset.value());
      mt_targets.push_back(bone_target);
    }
  }

  // Go through all the GLTF materials in order. If a material animation is
  // found, add the animation and material target to the respective vector.
  for (size_t material_index = 0; material_index < lookup.materials.size();
       material_index++) {
    MaterialId material_id = MaterialId::At(material_index);
    if (!(material_scratch_flags[material_id] & ScratchFlags::kInThisAnim))
      continue;

    absl::optional<flatbuffers::Offset<animation::schemas::MaterialAnimation>>
        animation_offset;
    absl::optional<Domain> animation_domain;
    MP_RETURN_IF_ERROR(
        SerializeMaterialAnimation(gltf, lookup, material_id, animation, &fbb,
                                   &animation_offset, &animation_domain))
        << "Adding material animation data";

    if (animation_domain) {
      first_time = std::min(first_time, animation_domain->min);
      last_time = std::max(last_time, animation_domain->max);
    }

    if (animation_offset.has_value()) {
      material_animations.push_back(animation_offset.value());
      material_targets.push_back(material_index);
    }
  }

  std::vector<flatbuffers::Offset<animation::schemas::LightPunctualAnimation>>
      out_light_animations;
  std::vector<animation::schemas::LightAnimationTarget> out_light_targets;
  MP_RETURN_IF_ERROR(GetLightAnimation(gltf, lookup, animation, &fbb,
                                    out_light_animations, out_light_targets,
                                    first_time, last_time));

  auto name = lookup.animations[animation].name;
  auto loaded_anim_fb = animation::schemas::CreateGltfAnimation(
      fbb, fbb.CreateString(name.data(), name.size()), first_time, last_time,
      fbb.CreateVector(node_animations),
      fbb.CreateVectorOfStructs(node_targets), fbb.CreateVector(mt_animations),
      fbb.CreateVectorOfStructs(mt_targets),
      fbb.CreateVector(material_animations),
      fbb.CreateVectorOfStructs(material_targets),
      fbb.CreateVector(out_light_animations),
      fbb.CreateVectorOfStructs(out_light_targets));
  fbb.Finish(loaded_anim_fb, "ImpA");

  FlatBufferAccess<schemas::GltfAnimation> result;
  MP_RETURN_IF_ERROR(CreateFlatBufferAccess(&fbb, &result));
  return result;
}

}  // namespace imp::animation
