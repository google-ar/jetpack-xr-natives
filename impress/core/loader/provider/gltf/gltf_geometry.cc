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

#include "core/loader/provider/gltf/gltf_geometry.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/optional.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/geometry/include/geometry/SurfaceOrientation.h"
#include "filament/libs/math/include/math/norm.h"
#include "filament/libs/math/include/math/vec3.h"
#include "core/common/buffer_access.h"
#include "core/common/data_helpers.h"
#include "core/common/filament_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/robin_set.h"
#include "core/common/schemas/render_generated.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/loader/provider/extensions/gltf_extension_mesh_features.h"
#include "core/loader/provider/gltf/accessor_reader.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_attribute.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/entity_data.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_gltf {
namespace {

using ::imp::gltf::imp_proto::Accessor;
using ::imp::gltf::imp_proto::Gltf;
using ::imp::gltf::imp_proto::Primitive;

// Stores a mapping of Gltf2Attribute to AccessorId's in an array.
class AttributeLookup {
  static constexpr size_t kGltf2AttributeCount =
      static_cast<size_t>(VertexAttribute::MAX) + 1;

 public:
  AccessorId& operator[](Gltf2Attribute attribute) {
    return storage_[static_cast<size_t>(attribute)];
  }
  const AccessorId& operator[](Gltf2Attribute attribute) const {
    return storage_[static_cast<size_t>(attribute)];
  }
  void AddExtraAttribute(const std::string& attribute_name, AccessorId id) {
    extra_attributes_.insert({attribute_name, id});
  }
  AccessorId GetExtraAttribute(const std::string& attribute_name) const {
    auto it = extra_attributes_.find(attribute_name);
    return (it != extra_attributes_.end()) ? it->second : AccessorId{};
  }
  OptionalError Visit(
      std::function<OptionalError(Gltf2Attribute, AccessorId)> visitor) const {
    for (size_t i = 0; i < kGltf2AttributeCount; i++) {
      MP_RETURN_IF_ERROR(visitor(static_cast<Gltf2Attribute>(i), storage_[i]));
    }
    return NoError();
  }

 private:
  using Storage = std::array<AccessorId, kGltf2AttributeCount>;
  Storage storage_;
  std::map<std::string, AccessorId> extra_attributes_;
};

struct PrimitiveTarget {
  std::optional<AccessorId> position = std::nullopt;
  std::optional<AccessorId> tangent = std::nullopt;
  std::optional<AccessorId> normal = std::nullopt;
};

struct PrimitiveAccessors {
  AttributeLookup vertex_attributes;
  AccessorId indices;
  std::vector<PrimitiveTarget> targets;
};

struct DenseDataAndType {
  DenseDataAccess data;
  schemas::AttributeType type;
};

OptionalError GetMorphTargetsFromPrimitive(const Gltf& gltf,
                                           const Primitive& primitive,
                                           PrimitiveAccessors* out_accessors) {
  size_t size = primitive.targets.size();
  out_accessors->targets.reserve(size);
  if (size > filament::MAX_MORPH_TARGETS) {
    IMP_LOG(imp::WARNING) << size << " morph targets were specified, but only "
                 << filament::MAX_MORPH_TARGETS
                 << " morph targets can be supported.";
  }
  for (uint8_t morph_target_index = 0;
       morph_target_index < std::min(size, filament::MAX_MORPH_TARGETS);
       ++morph_target_index) {
    auto& target = primitive.targets[morph_target_index];
    PrimitiveTarget current_target;
    if (target.position.has_value()) {
      int accessor_id = static_cast<int>(target.position.value());
      if (accessor_id < 0) {
        return Error(
            "Invalid value passed in for morph target position at index %i",
            morph_target_index);
      }
      current_target.position = AccessorId::At(accessor_id);
    }

    if (target.normal.has_value()) {
      int accessor_id = static_cast<int>(target.normal.value());
      if (accessor_id < 0) {
        return Error(
            "Invalid value passed in for morph target normal at index %i",
            morph_target_index);
      }
      current_target.normal = AccessorId::At(accessor_id);
    }

    if (target.tangent.has_value()) {
      int accessor_id = static_cast<int>(target.tangent.value());
      if (accessor_id < 0) {
        return Error(
            "Invalid value passed in for morph target tangent at index %i",
            morph_target_index);
      }
      current_target.tangent = AccessorId::At(accessor_id);
    }

    out_accessors->targets.push_back(current_target);
  }

  return NoError();
}

OptionalError GetAccessorsFromPrimitive(const Gltf& gltf,
                                        const Primitive& primitive,
                                        PrimitiveAccessors* out_accessors) {
  for (auto& attribute_pair : primitive.attributes) {
    std::optional<Gltf2Attribute> attribute =
        GetGlTF2VertexAttribute(attribute_pair.first);
    if (attribute) {
      out_accessors->vertex_attributes[*attribute] =
          AccessorId::At(attribute_pair.second);
    } else {
      out_accessors->vertex_attributes.AddExtraAttribute(
          attribute_pair.first, AccessorId::At(attribute_pair.second));
    }
  }
  if (primitive.indices) {
    out_accessors->indices = AccessorId::At(*primitive.indices);
  }
  if (!primitive.targets.empty()) {
    MP_RETURN_IF_ERROR(
        GetMorphTargetsFromPrimitive(gltf, primitive, out_accessors));
  }
  return NoError();
}

OptionalError GetAccessorsFromPrimitives(
    const Gltf& gltf, const std::vector<Primitive>& primitives,
    std::vector<PrimitiveAccessors>* out_accessors) {
  out_accessors->resize(primitives.size());
  for (const Primitive& primitive : primitives) {
    MP_RETURN_IF_ERROR(GetAccessorsFromPrimitive(
        gltf, primitive, &out_accessors->at(&primitive - &primitives.front())));
  }
  return NoError();
}

OptionalError GetAccessorBounds(const gltf::imp_proto::Gltf& gltf,
                                AccessorId accessor_id,
                                filament::Box* out_bounds) {
  auto accessor_index = static_cast<int>(accessor_id);
  if (accessor_index < 0 || accessor_index >= gltf.accessors.size()) {
    return Error("Invalid accessor");
  }
  const Accessor& accessor = gltf.accessors[accessor_index];

  if (accessor.type != "VEC3") {
    return Error("Can only get bounds of vec3's");
  }

  const auto* min_values =
      accessor.min.size() == 3 ? accessor.min.data() : nullptr;
  const auto* max_values =
      accessor.max.size() == 3 ? accessor.max.data() : nullptr;
  if (min_values && max_values) {
    *out_bounds = filament::Box{}.set(
        float3{min_values[0], min_values[1], min_values[2]},
        float3{max_values[0], max_values[1], max_values[2]});
    // Scales the bounds if they happen to be normalized values.
    if (accessor.normalized) {
      float inverse_bounds_scale;
      switch (accessor.component_type) {
        case gltf::imp_proto::ComponentType::BYTE:
          inverse_bounds_scale = INT8_MAX;
          break;
        case gltf::imp_proto::ComponentType::UNSIGNED_BYTE:
          inverse_bounds_scale = UINT8_MAX;
          break;
        case gltf::imp_proto::ComponentType::SHORT:
          inverse_bounds_scale = INT16_MAX;
          break;
        case gltf::imp_proto::ComponentType::UNSIGNED_SHORT:
          inverse_bounds_scale = UINT16_MAX;
          break;
        default:
          return Error("Unsupported component type to get bounds from: %d",
                       accessor.component_type);
      }
      out_bounds->center /= inverse_bounds_scale;
      out_bounds->halfExtent /= inverse_bounds_scale;
    }
  } else {
    MP_ASSIGN_OR_RETURN(AccessorReader position_reader,
                     AccessorReader::Create(gltf, accessor_id));
    DenseDataAccess positions = position_reader.GetData();

    using float_limits = std::numeric_limits<float>;
    float3 pos_min{float_limits::max()};
    float3 pos_max{float_limits::lowest()};

    for (size_t i = 0; i < positions.GetCount(); i++) {
      pos_min = min(pos_min, *positions.At<float3>(i));
      pos_max = max(pos_max, *positions.At<float3>(i));
    }

    *out_bounds = filament::Box{}.set(pos_min, pos_max);
  }
  return NoError();
}

OptionalError GetAttributeInfo(
    const gltf::imp_proto::Gltf& gltf, const Gltf2Attribute attribute,
    AccessorId attribute_accessor,
    std::optional<schemas::VertexAttributeInfo>* out_attribute_info) {
  int accessor_index = static_cast<int>(attribute_accessor);
  if (accessor_index < 0 || accessor_index >= gltf.accessors.size())
    return Error("Invalid accessor");
  const Accessor& accessor = gltf.accessors[accessor_index];
  schemas::AttributeType attribute_type;
  MP_RETURN_IF_ERROR(GetAttributeType(accessor.type, accessor.component_type,
                                   &attribute_type));
  out_attribute_info->emplace(schemas::VertexAttributeInfo(
      GetVertexAttribute(attribute), attribute_type, 0, accessor.normalized));
  return NoError();
}

// Builds orientations from normals and conditionally with tangents.
// This function calculates surface orientations, which is used as tangents
// downstream.
// Please be aware that this function moves tangents (if exists) to reuse the
// space, so the input tangents will be invalid after this function returns.
// `SurfaceOrientation` accepts strided streams of data when using normals and
// tangents, so this will convert quantized data if necessary but will not
// compact existing float data.
// Use AccessorReader::GetPackedFloatData() to prepare data before passing in.
absl::StatusOr<BufferAccess> GetOrientations(const DenseDataAccess& normals,
                                             DenseDataAccess* tangents) {
  if (normals.GetStride() != sizeof(float3)) {
    return absl::InvalidArgumentError("Normals is not packed.");
  }

  size_t vertex_count = normals.GetCount();

  if (tangents) {
    if (tangents->GetStride() != sizeof(float4)) {
      return absl::InvalidArgumentError("Tangents is not packed.");
    }
    if (vertex_count != tangents->GetCount()) {
      return Error("Mismatch between number of normals %i and tangents %i",
                   vertex_count, tangents->GetCount());
    }
  }

  filament::geometry::SurfaceOrientation* orientation_builder =
      filament::geometry::SurfaceOrientation::Builder()
          .vertexCount(vertex_count)
          .normals(normals.ReadRawData<float3>(), sizeof(float3))
          .tangents(tangents ? tangents->ReadRawData<float4>() : nullptr,
                    tangents ? sizeof(float4) : 0)
          .build();

  if (!orientation_builder) {
    return Error("Failed to create orientations");
  }

  // Writes out orientation data.
  BufferAccess orientations_buffer_access;
  uint8_t* orientations_ptr = BufferAccess::Create(
      sizeof(float4) * vertex_count, &orientations_buffer_access);
  orientation_builder->getQuats(reinterpret_cast<quatf*>(orientations_ptr),
                                vertex_count);
  delete orientation_builder;

  return orientations_buffer_access;
}

template <typename T>
void NormalizeIntegerWeights(DenseDataAccess& weights_data) {
  // TODO: The spec requires that the sum of the four weights be
  // equal to T::max (255 or 65535). This function does not guarantee that.
  for (auto weights_index = 0u; weights_index < weights_data.GetCount();
       ++weights_index) {
    T* weight = weights_data.At<T>(weights_index);
    int32_t sum = weight->x + weight->y + weight->z + weight->w;
    if (sum > 0 && sum != kMaxValue<typename T::value_type>) {
      int4 int_weight = int4{weight->x, weight->y, weight->z, weight->w} *
                        kMaxValue<typename T::value_type> / sum;
      *weight = T{int_weight.x, int_weight.y, int_weight.z, int_weight.w};
    }
  }
}

OptionalError NormalizeWeights(DenseDataAndType& weights_data_and_type) {
  // Packed formats may support UBYTE4 vertex weights
  if (weights_data_and_type.type == schemas::AttributeType::UBYTE4) {
    NormalizeIntegerWeights<ubyte4>(weights_data_and_type.data);
    return NoError();
  }
  if (weights_data_and_type.type == schemas::AttributeType::USHORT4) {
    NormalizeIntegerWeights<ushort4>(weights_data_and_type.data);
    return NoError();
  }
  if (weights_data_and_type.type != schemas::AttributeType::FLOAT4) {
    return Error("wrong format");
  }
  // Most incoming glTF weights data is already normalized. In those cases we
  // should not touch the data.
  // TODO: Adding this tolerance check will require updating some
  // scuba goldens, so leaving it out for now to add in its own CL along with
  // the updated test data.
#if 1
  constexpr float kTolerance = 0.0f;
#else
  constexpr float kTolerance = 2e-7f;  // This number comes from the glTF
  spec.
#endif
  for (auto weights_index = 0u;
       weights_index < weights_data_and_type.data.GetCount(); ++weights_index) {
    float4* weight = weights_data_and_type.data.At<float4>(weights_index);
    float sum = weight->x + weight->y + weight->z + weight->w;
    if (sum > 0 && std::abs(sum - 1.0f) > kTolerance) {
      *weight /= sum;
    }
  }
  return NoError();
}

template <typename T>
void LimitJointIndices(DenseDataAccess& joints_data,
                       uint16_t sampled_bone_count) {
  uint16_t max_index = sampled_bone_count ? sampled_bone_count - 1 : 0;
  T limit = T(max_index, max_index, max_index, max_index);
  for (auto joints_index = 0u; joints_index < joints_data.GetCount();
       ++joints_index) {
    T* joint = joints_data.At<T>(joints_index);
    if (auto clamped = min(*joint, limit); *joint != clamped) {
      *joint = clamped;
    }
  }
}

absl::StatusOr<BufferAccess> GenerateOrientationsFromNormalsAndTexcoords(
    const gltf::imp_proto::Gltf& gltf, const DenseDataAccess& positions,
    const DenseDataAccess& normals, const DenseDataAccess& texcoords,
    const absl::optional<DenseDataAccess>& indices_data) {
  // The SurfaceOrientation builder will not accept strided data when building
  // orientations with UVs. Use AccessorReader::GetPackedFloatData() to prepare
  // data before passing in.
  if (positions.GetStride() != sizeof(float3) ||
      normals.GetStride() != sizeof(float3) ||
      texcoords.GetStride() != sizeof(float2)) {
    return absl::InvalidArgumentError(
        "The SurfaceOrientation builder will not accept unpacked data.");
  }

  filament::geometry::SurfaceOrientation::Builder builder;
  size_t vertex_count = positions.GetCount();

  // Supplies required attribute streams to the builder. Stride is always zero
  // due to a filament requirement.
  builder.vertexCount(vertex_count)
      .normals(normals.ReadRawData<filament::math::float3>())
      .uvs(texcoords.ReadRawData<filament::math::float2>())
      .positions(positions.ReadRawData<filament::math::float3>());

  // Supplies indices if available, or specifies triangle count from position
  // count.
  if (!indices_data) {
    builder.triangleCount(vertex_count / 3);
  } else {
    builder.triangleCount(indices_data->GetCount() / 3);
    if (indices_data->GetStride() == 2) {
      builder.triangles(indices_data->ReadRawData<filament::math::ushort3>());
    } else {
      builder.triangles(indices_data->ReadRawData<filament::math::uint3>());
    }
  }

  filament::geometry::SurfaceOrientation* surface_orientations =
      builder.build();

  if (!surface_orientations) {
    return Error("Failed to create orientations");
  }

  BufferAccess orientations_buffer_access;
  uint8_t* orientations_ptr = BufferAccess::Create(
      sizeof(float4) * vertex_count, &orientations_buffer_access);
  surface_orientations->getQuats(reinterpret_cast<quatf*>(orientations_ptr),
                                 vertex_count);
  delete surface_orientations;

  return orientations_buffer_access;
}

// Converts buffer from float4 to short4 format. This is currently used for
// tangents data.
BufferAccess ConvertFloat4ToShort4Buffer(const uint8_t* input_ptr,
                                         int elements_count) {
  auto src_ptr = reinterpret_cast<const float4*>(input_ptr);
  BufferAccess short4_buffer;
  auto dest_ptr = reinterpret_cast<short4*>(
      BufferAccess::Create(elements_count * sizeof(short4), &short4_buffer));
  std::transform(
      src_ptr, src_ptr + elements_count, dest_ptr,
      [](const float4& v) { return filament::math::packSnorm16(v); });
  return short4_buffer;
}

// Creates a PendingMorphTargetBuffer::Attribute that contains positions and
// tangents for each morph target. Unlike other attributes, orientations may
// not be copied from the glTF. Matching sets of normals and tangents must be
// combined together or generated before being added to out_attributes. If these
// are not present in the glTF, base tangents are copied over.
// TODO: Add tests for AppendMorphTargetVertexBufferBlock to ensure
// normal and tangent values are processed as expected.
OptionalError AppendMorphTargetAttributes(
    const gltf::imp_proto::Gltf& gltf, const AttributeLookup& attribute_lookup,
    const std::vector<PrimitiveTarget> targets,
    const absl::optional<DenseDataAccess>& indices_data,
    AccessorId normal_map_texcoord_id,
    std::vector<LoadedModelBuilder::MorphTargetBlock>* out_attributes) {
  for (const PrimitiveTarget& target : targets) {
    // Step 1: Gets morph target positions.
    if (!target.position.has_value()) {
      return absl::InvalidArgumentError("Morph target must have a position");
    }
    MP_ASSIGN_OR_RETURN(AccessorReader morph_target_positions_reader,
                     AccessorReader::Create(gltf, *target.position));
    MP_ASSIGN_OR_RETURN(DenseDataAccess morph_target_position_data,
                     morph_target_positions_reader.GetPackedFloatData());
    BufferAccess dest_positions_access =
        morph_target_position_data.ReleaseBufferAccess();

    // Step 2: Gets morph target normals.
    // 2.a: Finds Accessor for base normal data.
    AccessorId base_normals_accessor_id =
        attribute_lookup[Gltf2Attribute::NORMAL];
    if (!base_normals_accessor_id) {
      return Error(
          "Cannot process tangents for morph targets without base normals");
    }
    MP_ASSIGN_OR_RETURN(AccessorReader base_normals_reader,
                     AccessorReader::Create(gltf, base_normals_accessor_id));
    MP_ASSIGN_OR_RETURN(DenseDataAccess absolute_morphed_normals,
                     base_normals_reader.GetPackedFloatData());

    // 2.b Adds morph normal data (if any) to base normal data.
    bool has_morph_target_normals_accessor_id = target.normal.has_value();
    if (has_morph_target_normals_accessor_id) {
      MP_ASSIGN_OR_RETURN(AccessorReader morph_target_normals_reader,
                       AccessorReader::Create(gltf, *target.normal));
      MP_RETURN_IF_ERROR(morph_target_normals_reader.FillValues<float3>(
          /*destination*/ absolute_morphed_normals,
          AccessorReader::RetrievalMode::kAdd));
    }

    // Step 3: Gets morph target tangents.
    // 3.a: Finds Accessor for the base/underlying tangent data which we are
    // adding the morph tangent data to.
    AccessorId base_tangents_accessor_id =
        attribute_lookup[Gltf2Attribute::TANGENT];
    bool has_morph_target_tangents_accessor_id = target.tangent.has_value();

    // 3.b(1) Base tangents exists: Generates orientation with normals and
    // tangents.
    // Adjusts tangents by morph differences if provided, otherwise use base
    // tangent values.
    if (base_tangents_accessor_id) {
      MP_ASSIGN_OR_RETURN(AccessorReader base_tangents_reader,
                       AccessorReader::Create(gltf, base_tangents_accessor_id));
      MP_ASSIGN_OR_RETURN(DenseDataAccess dest_tangents_data,
                       base_tangents_reader.GetPackedFloatData());
      if (dest_tangents_data.GetStride() != sizeof(float4)) {
        return absl::InvalidArgumentError("dest_tangents_data is not packed");
      }
      if (has_morph_target_tangents_accessor_id) {
        MP_ASSIGN_OR_RETURN(AccessorReader morph_target_tangents_reader,
                         AccessorReader::Create(gltf, *target.tangent));
        // Adds float3 data to first 3 elements of a float4 data.
        MP_RETURN_IF_ERROR(morph_target_tangents_reader.FillValues<float3>(
            /*destination*/ dest_tangents_data,
            AccessorReader::RetrievalMode::kAdd));
      }

      // Generates orientation with normals and tangents
      MP_ASSIGN_OR_RETURN(
          BufferAccess orientations,
          GetOrientations(absolute_morphed_normals, &dest_tangents_data));

      // Move data from created BufferAccess to a new
      // PendingMorphTargetBuffer::Attribute.
      // Tangents are packed into short4 format for filament's
      // MorphTargetBuffer.
      out_attributes->push_back(LoadedModelBuilder::MorphTargetBlock(
          std::move(dest_positions_access),
          ConvertFloat4ToShort4Buffer(orientations.Data(),
                                      absolute_morphed_normals.GetCount())));
      continue;
    } else {
      if (has_morph_target_tangents_accessor_id) {
        return Error(
            "Cannot process tangents for morph targets without base tangents");
      }
    }

    // 3.b(2) No base tangents: Tries to generate orientations by passing
    // normals and texcoords to filament's SurfaceOrientation helper.
    if (!base_tangents_accessor_id && normal_map_texcoord_id) {
      // To generate tangents, we need to calculate the final vertex positions
      // modified by the morph target positions.
      AccessorId base_positions_accessor_id =
          attribute_lookup[Gltf2Attribute::POSITION];
      MP_ASSIGN_OR_RETURN(
          AccessorReader base_positions_reader,
          AccessorReader::Create(gltf, base_positions_accessor_id));
      MP_ASSIGN_OR_RETURN(DenseDataAccess absolute_positions,
                       base_positions_reader.GetPackedFloatData());
      MP_RETURN_IF_ERROR(morph_target_positions_reader.FillValues<float3>(
          absolute_positions, AccessorReader::RetrievalMode::kAdd));

      MP_ASSIGN_OR_RETURN(AccessorReader texcoord_reader,
                       AccessorReader::Create(gltf, normal_map_texcoord_id));
      MP_ASSIGN_OR_RETURN(DenseDataAccess texcoord_data,
                       texcoord_reader.GetPackedFloatData());

      auto tangents = GenerateOrientationsFromNormalsAndTexcoords(
          gltf, absolute_positions, absolute_morphed_normals, texcoord_data,
          indices_data);

      if (tangents.ok()) {
        // Move data from positions and tangents buffer to a new
        // PendingMorphTargetBuffer::Attribute and skip to next target.
        out_attributes->push_back(LoadedModelBuilder::MorphTargetBlock(
            std::move(dest_positions_access),
            ConvertFloat4ToShort4Buffer(tangents.value().Data(),
                                        base_normals_reader.GetCount())));
        continue;
      } else {
        IMP_LOG(imp::WARNING) << tangents.status();
      }
    }

    // 3.b(3) No base tangents, no textcoords or generate tangents from
    // textcoords failed: Generates orientations with normals only.
    MP_ASSIGN_OR_RETURN(BufferAccess orientations,
                     GetOrientations(absolute_morphed_normals, nullptr));

    // Move data from created BufferAccess to a new
    // PendingMorphTargetBuffer::Attribute.
    // Tangents are packed into short4 format for filament's
    // MorphTargetBuffer.
    out_attributes->push_back(LoadedModelBuilder::MorphTargetBlock(
        std::move(dest_positions_access),
        ConvertFloat4ToShort4Buffer(orientations.Data(),
                                    absolute_morphed_normals.GetCount())));
  }
  return absl::OkStatus();
}

int GetJointsAndWeightsAttributesCount(
    const AttributeLookup& attribute_lookup) {
  // Both of JOINTS_n and WEIGHTS_n must exist for each n for that attribute
  // to count. They must also be contiguous. I.e. if there is a _0, _1, and _3
  // (but not _2), then only the _0 and _1 will be used.

  // JOINTS_0 and WEIGHTS_0 are regular attributes in the AttributeLookup, and
  // JOINTS_1+ and WEIGHTS_1+ are "extra" attributes.
  if (!attribute_lookup[Gltf2Attribute::JOINTS_0] ||
      !attribute_lookup[Gltf2Attribute::WEIGHTS_0]) {
    return 0;
  }
  int i = 1;
  while (attribute_lookup.GetExtraAttribute(absl::StrCat("JOINTS_", i)) &&
         attribute_lookup.GetExtraAttribute(absl::StrCat("WEIGHTS_", i))) {
    ++i;
  }
  return i;
}

float4 GetJointsForVertexAsFloat4(DenseDataAndType& joints,
                                  size_t vertex_index) {
  switch (joints.type) {
    case schemas::AttributeType::UBYTE4:
      return float4(*joints.data.At<ubyte4>(vertex_index));
    case schemas::AttributeType::USHORT4:
      return float4(*joints.data.At<ushort4>(vertex_index));
    default:
      return float4{0};
  }
}

float4 GetWeightsForVertexAsFloat4(DenseDataAndType& weights,
                                   size_t vertex_index) {
  constexpr float ubyte_scale = 1.0f / kMaxValue<uint8_t>;
  constexpr float ushort_scale = 1.0f / kMaxValue<uint16_t>;
  switch (weights.type) {
    case schemas::AttributeType::UBYTE4:
      return float4(*weights.data.At<ubyte4>(vertex_index)) * ubyte_scale;
    case schemas::AttributeType::USHORT4:
      return float4(*weights.data.At<ushort4>(vertex_index)) * ushort_scale;
    case schemas::AttributeType::FLOAT4:
      return *weights.data.At<float4>(vertex_index);
    default:
      return float4{0};
  }
}

OptionalError PackJointsAndSkinningWeights(
    const gltf::imp_proto::Gltf& gltf, const AttributeLookup& attribute_lookup,
    std::vector<float2>* out_bone_indices_and_weights) {
  int num_weight_attributes =
      GetJointsAndWeightsAttributesCount(attribute_lookup);

  // We only need to pack if there is more than one set. Otherwise the bone
  // indices and skinning weights get loaded into the vertex buffer directly,
  // in the BONE_INDICES and BONE_WEIGHTS attributes.
  if (num_weight_attributes <= 1) return NoError();

  std::vector<std::unique_ptr<AccessorReader>> joint_reader(
      num_weight_attributes);
  std::vector<std::unique_ptr<AccessorReader>> weight_reader(
      num_weight_attributes);
  MP_ASSIGN_OR_RETURN(
      AccessorReader joint_reader_0,
      AccessorReader::Create(gltf, attribute_lookup[Gltf2Attribute::JOINTS_0]));
  joint_reader[0] = std::make_unique<AccessorReader>(joint_reader_0);
  MP_ASSIGN_OR_RETURN(AccessorReader weight_reader_0,
                   AccessorReader::Create(
                       gltf, attribute_lookup[Gltf2Attribute::WEIGHTS_0]));
  weight_reader[0] = std::make_unique<AccessorReader>(weight_reader_0);
  for (int i = 1; i < num_weight_attributes; ++i) {
    MP_ASSIGN_OR_RETURN(
        AccessorReader joint_reader_i,
        AccessorReader::Create(gltf, attribute_lookup.GetExtraAttribute(
                                         absl::StrCat("JOINTS_", i))));
    joint_reader[i] = std::make_unique<AccessorReader>(joint_reader_i);
    MP_ASSIGN_OR_RETURN(
        AccessorReader weight_reader_i,
        AccessorReader::Create(gltf, attribute_lookup.GetExtraAttribute(
                                         absl::StrCat("WEIGHTS_", i))));
    weight_reader[i] = std::make_unique<AccessorReader>(weight_reader_i);
  }

  auto vertex_count = joint_reader[0]->GetCount();
  auto compare_vertex_count =
      [vertex_count](const std::unique_ptr<AccessorReader>& reader) {
        return reader->GetCount() != vertex_count;
      };
  if (absl::c_any_of(joint_reader, compare_vertex_count) ||
      absl::c_any_of(weight_reader, compare_vertex_count)) {
    return Error(
        "All JOINTS_n and WEIGHTS_n attributes must have the same count.");
  }

  out_bone_indices_and_weights->resize(4 * vertex_count *
                                       num_weight_attributes);
  float2* dst = out_bone_indices_and_weights->data();

  std::vector<DenseDataAndType> joints_data(num_weight_attributes);
  for (auto attr = 0u; attr < num_weight_attributes; ++attr) {
    MP_ASSIGN_OR_RETURN(schemas::AttributeType attribute_type,
                     joint_reader[attr]->GetAttributeType());
    joints_data[attr] = DenseDataAndType{.data = joint_reader[attr]->GetData(),
                                         .type = attribute_type};
  }
  std::vector<DenseDataAndType> weights_data(num_weight_attributes);
  for (auto attr = 0u; attr < num_weight_attributes; ++attr) {
    MP_ASSIGN_OR_RETURN(schemas::AttributeType attribute_type,
                     weight_reader[attr]->GetAttributeType());
    weights_data[attr] = DenseDataAndType{
        .data = weight_reader[attr]->GetData(), .type = attribute_type};
  }

  for (auto vertex_index = 0u; vertex_index < vertex_count; ++vertex_index) {
    float2* first_entry_for_current_vertex = dst;
    float weight_sum = 0.0f;
    for (auto attr = 0u; attr < num_weight_attributes; ++attr) {
      float4 joints =
          GetJointsForVertexAsFloat4(joints_data[attr], vertex_index);
      float4 weights =
          GetWeightsForVertexAsFloat4(weights_data[attr], vertex_index);
      for (int i = 0; i < 4; ++i, ++dst) {
        dst->x = joints[i];
        dst->y = weights[i];
        weight_sum += weights[i];
      }
    }
    // Normalize
    constexpr float kTolerance = 2e-7f;  // This number comes from the glTF spec
    if (weight_sum > 0.0f && std::abs(weight_sum - 1.f) > kTolerance) {
      float scale = 1.0f / weight_sum;
      for (float2* f = first_entry_for_current_vertex; f != dst; ++f) {
        f->y *= scale;
      }
    }
  }

  return NoError();
}

absl::StatusOr<DenseDataAndType> CreateBufferForAttribute(
    const gltf::imp_proto::Gltf& gltf, Gltf2Attribute attribute,
    AccessorId attribute_accessor) {
  MP_ASSIGN_OR_RETURN(AccessorReader reader,
                   AccessorReader::Create(gltf, attribute_accessor));
  MP_ASSIGN_OR_RETURN(schemas::AttributeType attribute_type,
                   reader.GetAttributeType());
  absl::StatusOr<DenseDataAccess> data;

  switch (attribute) {
    case Gltf2Attribute::TANGENT: {
      // We don't use tangent data as is, but rather recompute tangent
      // orientations from normals (and tangents if available).  Since the
      // original gltf data may be read only, we allocate a new buffer for this
      // case.  Tangents require a float4 type, per spec.
      if (attribute_type != schemas::AttributeType::FLOAT4) {
        // TODO: The logic below assumes that incoming tangents are
        // FLOAT4. However we are supposed to support BYTE4 and SHORT4 tangents
        // too because of KHR_mesh_quantization. The logic below is not correct,
        // but we need to comment out this Error return for now because loading
        // a mesh with bad tangent data is preferable to not loading the mesh
        // altogether.
        // The straightforward solution would be to merge this case statement
        // with the WEIGHTS_0 case below, as the necessary logic would be almost
        // identical. However there's other logic elsewhere in this file that
        // also assumes FLOAT4, so that wouldn't work. In particular, there's an
        // assert somewhere that the size of both the NORMAL and TANGENT blocks
        // are vertex_count * sizeof(quatf);

        // return Error("TANGENT attribute must have type FLOAT4.");
      }
      data = reader.GetPackedFloatData(AccessorReader::CopyOption::kForceCopy);
      break;
    }
    case Gltf2Attribute::WEIGHTS_0: {
      // Weights are also copied into a new buffer before sending to filament
      // because they are explicitly normalized.
      if (attribute_type != schemas::AttributeType::FLOAT4 &&
          attribute_type != schemas::AttributeType::UBYTE4 &&
          attribute_type != schemas::AttributeType::USHORT4) {
        return absl::InvalidArgumentError(absl::StrCat(
            "Invalid data type %s for WEIGHTS_0 attribute. Valid types are "
            "FLOAT4, USHORT4, or UBYTE4.",
            EnumNameAttributeType(attribute_type)));
      }
      data = reader.GetPackedData(AccessorReader::CopyOption::kForceCopy);
      break;
    }
    default: {
      // Must create a copy as the data should outlive the loader.
      data = reader.GetPackedData(AccessorReader::CopyOption::kForceCopy);
    }
  }
  if (!data.ok()) return data.status();
  return DenseDataAndType{std::move(*data), attribute_type};
}

OptionalError GetVertexBufferInfoFromAttributeLookup(
    const gltf::imp_proto::Gltf& gltf, const AttributeLookup& attribute_lookup,
    const absl::optional<DenseDataAccess>& indices_data,
    AccessorId normal_map_texcoord_id, uint16_t sampled_joint_count,
    bool advanced_skinning,
    std::vector<LoadedModelBuilder::VertexBlock>* out_blocks) {
  return attribute_lookup.Visit([&gltf, &indices_data, out_blocks,
                                 &attribute_lookup, normal_map_texcoord_id,
                                 sampled_joint_count, advanced_skinning](
                                    Gltf2Attribute attribute,
                                    AccessorId attribute_accessor)
                                    -> OptionalError {
    // Early-out on empty attributes.
    if (!attribute_accessor) return NoError();

    if ((attribute == Gltf2Attribute::JOINTS_0 ||
         attribute == Gltf2Attribute::WEIGHTS_0) &&
        advanced_skinning) {
      return NoError();
    }

    if (attribute != Gltf2Attribute::NORMAL) {
      std::optional<schemas::VertexAttributeInfo> pending_attribute;
      MP_RETURN_IF_ERROR(GetAttributeInfo(gltf, attribute, attribute_accessor,
                                       &pending_attribute));
      MP_ASSIGN_OR_RETURN(
          DenseDataAndType data_with_type,
          CreateBufferForAttribute(gltf, attribute, attribute_accessor));

      if (attribute == Gltf2Attribute::TANGENT) {
        // Process our tangents into orientations.
        if (!attribute_lookup[Gltf2Attribute::NORMAL])
          return Error("can't generate orientations without normals");

        MP_ASSIGN_OR_RETURN(AccessorReader normals_reader,
                         AccessorReader::Create(
                             gltf, attribute_lookup[Gltf2Attribute::NORMAL]));
        MP_ASSIGN_OR_RETURN(AccessorReader tangents_reader,
                         AccessorReader::Create(
                             gltf, attribute_lookup[Gltf2Attribute::TANGENT]));
        if (normals_reader.GetCount() != tangents_reader.GetCount()) {
          return absl::FailedPreconditionError(
              "Tangents/Normals buffer count mismatch");
        }

        MP_ASSIGN_OR_RETURN(DenseDataAccess normals_data,
                         normals_reader.GetPackedFloatData());
        MP_ASSIGN_OR_RETURN(BufferAccess orientations,
                         GetOrientations(normals_data, &data_with_type.data));
        out_blocks->push_back(LoadedModelBuilder::VertexBlock{
            *pending_attribute, std::move(orientations),
            static_cast<uint32_t>(data_with_type.data.GetStride())});

      } else if (attribute == Gltf2Attribute::WEIGHTS_0 &&
                 attribute_lookup[Gltf2Attribute::JOINTS_0]) {
        // Ensure weights for vertex skinning sum to 1.0
        MP_RETURN_IF_ERROR(NormalizeWeights(data_with_type));

        out_blocks->push_back(LoadedModelBuilder::VertexBlock{
            *pending_attribute, data_with_type.data.ReleaseBufferAccess(),
            static_cast<uint32_t>(data_with_type.data.GetStride())});
      } else if (attribute == Gltf2Attribute::JOINTS_0) {
        if (data_with_type.type == schemas::AttributeType::UBYTE4) {
          LimitJointIndices<ubyte4>(data_with_type.data, sampled_joint_count);
        } else if (data_with_type.type == schemas::AttributeType::USHORT4) {
          LimitJointIndices<ushort4>(data_with_type.data, sampled_joint_count);
        } else {
          return Error("Wrong joints format");
        }
        out_blocks->push_back(LoadedModelBuilder::VertexBlock{
            *pending_attribute, data_with_type.data.ReleaseBufferAccess(),
            static_cast<uint32_t>(data_with_type.data.GetStride())});
      } else {
        out_blocks->push_back(LoadedModelBuilder::VertexBlock{
            *pending_attribute, data_with_type.data.ReleaseBufferAccess(),
            static_cast<uint32_t>(data_with_type.data.GetStride())});
      }
      return NoError();
    }

    // Gets VertexBuffer information from attribute of Gltf2Attribute::NORMAL.
    // Notes that normals are used to generate orientations and pass to
    // downstream as schemas::VertexAttribute::TANGENTS.
    if (attribute == Gltf2Attribute::NORMAL) {
      // If tangents exist, normals are already processed earlier.
      if (attribute_lookup[Gltf2Attribute::TANGENT]) {
        return NoError();
      }

      // Conjure a new buffer to hold our tangents, and point to it with a
      // new VertexBlockInfo.
      std::vector<schemas::VertexAttributeInfo> conjured_attributes;
      conjured_attributes.push_back(schemas::VertexAttributeInfo{
          schemas::VertexAttribute::TANGENTS, schemas::AttributeType::FLOAT4, 0,
          false});

      MP_ASSIGN_OR_RETURN(AccessorReader normals_reader,
                       AccessorReader::Create(
                           gltf, attribute_lookup[Gltf2Attribute::NORMAL]));
      MP_ASSIGN_OR_RETURN(DenseDataAccess normals_data,
                       normals_reader.GetPackedFloatData());

      if (normal_map_texcoord_id) {
        MP_ASSIGN_OR_RETURN(AccessorReader positions_reader,
                         AccessorReader::Create(
                             gltf, attribute_lookup[Gltf2Attribute::POSITION]));
        MP_ASSIGN_OR_RETURN(DenseDataAccess positions_data,
                         positions_reader.GetPackedFloatData());
        MP_ASSIGN_OR_RETURN(AccessorReader texcoord_reader,
                         AccessorReader::Create(gltf, normal_map_texcoord_id));
        MP_ASSIGN_OR_RETURN(DenseDataAccess texcoord_data,
                         texcoord_reader.GetPackedFloatData());

        if (auto orientations = GenerateOrientationsFromNormalsAndTexcoords(
                gltf, positions_data, normals_data, texcoord_data,
                indices_data);
            orientations.ok()) {
          out_blocks->push_back(LoadedModelBuilder::VertexBlock(
              std::move(conjured_attributes), std::move(*orientations),
              static_cast<uint32_t>(sizeof(float4))));
          return NoError();
        } else {
          IMP_LOG(imp::WARNING) << orientations.status();
        }
      }

      // No tangents or textcoords provided, generates orientation with normals
      // only.
      MP_ASSIGN_OR_RETURN(BufferAccess orientations,
                       GetOrientations(normals_data, nullptr));
      out_blocks->push_back(LoadedModelBuilder::VertexBlock(
          std::move(conjured_attributes), std::move(orientations),
          static_cast<uint32_t>(sizeof(float4))));
    }

    return NoError();
  });
}

}  // namespace

OptionalError ProcessPrimitives(
    const gltf::imp_proto::Gltf& gltf,
    const std::vector<gltf::imp_proto::Primitive>& primitives,
    const filament::math::mat4& transform, uint16_t sampled_joint_count,
    LoadedModelBuilder* builder,
    GltfPrimitiveVector<ProcessedPrimitive>* out_processed_primitives,
    filament::Box* out_bounds,
    model::MorphTargetBufferId* out_morph_target_buffer_id) {
  if (primitives.empty()) {
    return Error("no geometry");
  }

  std::vector<PrimitiveAccessors> primitives_accessors;
  MP_RETURN_IF_ERROR(
      GetAccessorsFromPrimitives(gltf, primitives, &primitives_accessors));

  out_processed_primitives->reserve(primitives.size());
  filament::Box mesh_bounds = NilBounds();

  std::vector<std::vector<LoadedModelBuilder::MorphTargetBlock>>
      morph_target_attributes_per_primitive;
  size_t total_vertex_count = 0;

  for (auto i = 0u; i < primitives_accessors.size(); ++i) {
    PrimitiveAccessors& primitive_accessors = primitives_accessors[i];

    // Finds the texcoord id for the normal map if it exists.
    AccessorId normal_map_texcoord_id;
    const Primitive& primitive = primitives[i];
    if (primitive.material) {
      const gltf::imp_proto::Material& material =
          gltf.materials[*primitive.material];
      // If the normal texture is specified, try finding its texcoord
      // specification.
      if (material.normal_texture.index) {
        Gltf2Attribute normal_map_texcoord_attribute;
        if (material.normal_texture.tex_coord == 0) {
          normal_map_texcoord_attribute = Gltf2Attribute::TEXCOORD_0;
        } else if (material.normal_texture.tex_coord == 1) {
          normal_map_texcoord_attribute = Gltf2Attribute::TEXCOORD_1;
        } else {
          return Error("Invalid texcoord for normal map: %d",
                       material.normal_texture.tex_coord);
        }
        normal_map_texcoord_id =
            primitive_accessors
                .vertex_attributes[normal_map_texcoord_attribute];
      }
    }

    int position_accessor = static_cast<int>(
        primitive_accessors.vertex_attributes[Gltf2Attribute::POSITION]);
    if (position_accessor >= gltf.accessors.size()) {
      return Error("Invalid position accessor");
    }
    size_t vertex_count = gltf.accessors[position_accessor].count;

    absl::optional<DenseDataAccess> indices_data;
    if (primitive_accessors.indices) {
      MP_ASSIGN_OR_RETURN(
          AccessorReader indices_reader,
          AccessorReader::Create(gltf, primitive_accessors.indices));
      if (indices_reader.GetType() == "SCALAR") {
        indices_data = indices_reader.GetPackedData();
      } else if (indices_reader.GetType() == "VEC3") {
        indices_data = DenseDataAccess(
            indices_reader.GetPackedData().ReleaseBufferAccess(),
            indices_reader.GetCount() * 3, indices_reader.GetStride() / 3);
      } else {
        return absl::FailedPreconditionError(
            absl::StrCat("Invalid indices type: %s", indices_reader.GetType()));
      }

      // Convert 8-bit indices to 16-bit, since filament doesn't support 8-bit
      // indices.
      if (indices_data->GetStride() == 1) {
        size_t count = indices_data->GetCount();
        BufferAccess indices_buffer_16;
        // uint16_t is 2 bytes.
        BufferAccess::Create(count * 2, &indices_buffer_16);
        DenseDataAccess indices_data_16(std::move(indices_buffer_16), count,
                                        /*stride=*/2);

        for (size_t i = 0; i < indices_data->GetCount(); ++i) {
          *indices_data_16.At<uint16_t>(i) = *indices_data->At<uint8_t>(i);
        }

        indices_data = std::move(indices_data_16);
      }
    } else {
      // No indices data; it will notify GetIndexBufferId to conjure.
    }

    std::vector<LoadedModelBuilder::VertexBlock> vertex_blocks;
    LoadedModelBuilder::IndexBufferId index_buffer =
        builder->AddIndexBuffer(indices_data, vertex_count);

    std::vector<float2> bone_indices_and_weights;
    MP_RETURN_IF_ERROR(PackJointsAndSkinningWeights(
        gltf, primitive_accessors.vertex_attributes,
        &bone_indices_and_weights));
    LoadedModelBuilder::SkinningBufferId skinning_buffer_id;
    bool advanced_skinning = false;
    if (!bone_indices_and_weights.empty()) {
      skinning_buffer_id = builder->AddSkinningBuffer(bone_indices_and_weights);
      advanced_skinning = true;
    }

    MP_RETURN_IF_ERROR(GetVertexBufferInfoFromAttributeLookup(
        gltf, primitive_accessors.vertex_attributes, indices_data,
        normal_map_texcoord_id, sampled_joint_count, advanced_skinning,
        &vertex_blocks));

    std::vector<LoadedModelBuilder::MorphTargetBlock>
        primitive_morph_target_attributes;
    uint32_t morph_target_index = 0;
    uint32_t morph_target_count = 0;
    MP_RETURN_IF_ERROR(AppendMorphTargetAttributes(
        gltf, primitive_accessors.vertex_attributes,
        primitive_accessors.targets, indices_data, normal_map_texcoord_id,
        &primitive_morph_target_attributes));
    if (!primitive_morph_target_attributes.empty()) {
      morph_target_index = total_vertex_count;
      morph_target_count = vertex_count;
      total_vertex_count += vertex_count;

      if (!morph_target_attributes_per_primitive.empty() &&
          morph_target_attributes_per_primitive.back().size() !=
              primitive_morph_target_attributes.size()) {
        return absl::FailedPreconditionError(
            "All primivites on a mesh must have the same number of morph "
            "targets.");
      }

      morph_target_attributes_per_primitive.push_back(
          std::move(primitive_morph_target_attributes));
    }

    filament::Box primitive_bounds;
    MP_RETURN_IF_ERROR(GetAccessorBounds(
        gltf, primitive_accessors.vertex_attributes[Gltf2Attribute::POSITION],
        &primitive_bounds));
    mesh_bounds.unionSelf(primitive_bounds);

    RobinSet<int> required_materials;
    if (primitive.material) {
      required_materials.insert(primitive.material.value());
    } else {
      // -1 Represents default material.
      required_materials.insert(-1);
    }
    if (primitive.extensions.materials_variants) {
      for (const auto& materials_variant :
           primitive.extensions.materials_variants->mappings) {
        required_materials.insert(materials_variant.material);
      }
    }

    std::vector<gltf::imp_proto::Primitive::FeatureIdTexture>
        feature_id_textures = extensions::ResolveMeshFeatures(primitive);

    out_processed_primitives->push_back(ProcessedPrimitive{
        std::move(vertex_blocks), vertex_count, index_buffer,
        TransformBounds(primitive_bounds, transform), skinning_buffer_id,
        morph_target_index, morph_target_count, std::move(feature_id_textures),
        std::move(required_materials)});
  }

  std::vector<LoadedModelBuilder::MorphTargetBlock> morph_target_attributes;
  if (morph_target_attributes_per_primitive.size() == 1) {
    morph_target_attributes =
        std::move(morph_target_attributes_per_primitive.front());
  } else if (morph_target_attributes_per_primitive.size() > 1) {
    // Combine the morph target attributes for all primitives
    // First find the total buffer sizes for each morph target.
    size_t num_morph_targets =
        morph_target_attributes_per_primitive.back().size();
    std::vector<size_t> position_sizes(num_morph_targets);
    std::vector<size_t> tangent_sizes(num_morph_targets);

    for (std::vector<LoadedModelBuilder::MorphTargetBlock>&
             primitive_morph_target_attributes :
         morph_target_attributes_per_primitive) {
      for (size_t i = 0; i < primitive_morph_target_attributes.size(); ++i) {
        LoadedModelBuilder::MorphTargetBlock& primitive_morph_target_block =
            primitive_morph_target_attributes[i];
        position_sizes[i] += primitive_morph_target_block.positions.Size();
        tangent_sizes[i] += primitive_morph_target_block.tangents.Size();
      }
    }

    struct MorphTargetAttributesData {
      std::unique_ptr<uint8_t[]> positions_data;
      size_t current_position_offset = 0;
      std::unique_ptr<uint8_t[]> tangents_data;
      size_t current_tangent_offset = 0;
    };

    std::vector<MorphTargetAttributesData> morph_target_attributes_data(
        num_morph_targets);

    // Create & fill the the combined data.
    for (std::vector<LoadedModelBuilder::MorphTargetBlock>&
             primitive_morph_target_attributes :
         morph_target_attributes_per_primitive) {
      for (size_t i = 0; i < primitive_morph_target_attributes.size(); ++i) {
        LoadedModelBuilder::MorphTargetBlock& primitive_morph_target_block =
            primitive_morph_target_attributes[i];
        MorphTargetAttributesData& morph_target_data =
            morph_target_attributes_data[i];

        // Create the data storage if it hasn't been created already.
        if (!morph_target_data.positions_data) {
          morph_target_data.positions_data =
              std::make_unique<uint8_t[]>(position_sizes[i]);
        }
        if (!morph_target_data.tangents_data) {
          morph_target_data.tangents_data =
              std::make_unique<uint8_t[]>(tangent_sizes[i]);
        }

        std::copy_n(primitive_morph_target_block.positions.Data(),
                    primitive_morph_target_block.positions.Size(),
                    morph_target_data.positions_data.get() +
                        morph_target_data.current_position_offset);
        morph_target_data.current_position_offset +=
            primitive_morph_target_block.positions.Size();

        std::copy_n(primitive_morph_target_block.tangents.Data(),
                    primitive_morph_target_block.tangents.Size(),
                    morph_target_data.tangents_data.get() +
                        morph_target_data.current_tangent_offset);
        morph_target_data.current_tangent_offset +=
            primitive_morph_target_block.tangents.Size();
      }
    }

    morph_target_attributes.reserve(num_morph_targets);
    for (size_t i = 0; i < num_morph_targets; ++i) {
      MorphTargetAttributesData& morph_target_data =
          morph_target_attributes_data[i];
      morph_target_attributes.push_back(LoadedModelBuilder::MorphTargetBlock(
          BufferAccess(std::move(morph_target_data.positions_data),
                       position_sizes[i]),
          BufferAccess(std::move(morph_target_data.tangents_data),
                       tangent_sizes[i])));
    }
  }

  if (!morph_target_attributes.empty()) {
    *out_morph_target_buffer_id = builder->AddMorphTargetBuffer(
        std::move(morph_target_attributes), total_vertex_count);
  }

  *out_bounds = mesh_bounds;

  return NoError();
}

}  // namespace imp::loader::details::provider_gltf
