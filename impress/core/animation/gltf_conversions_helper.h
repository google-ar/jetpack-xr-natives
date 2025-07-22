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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_CONVERSIONS_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_CONVERSIONS_HELPER_H_

#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/data_helpers.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/gltf/accessor_reader.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/loader/provider/gltf/gltf_lookup.h"
#include "core/math/vec.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::animation {

using imp::gltf::imp_proto::AnimationSampler;
using imp::loader::details::provider_gltf::AnimationId;
using imp::loader::details::provider_gltf::ChannelId;
using imp::loader::details::provider_gltf::GltfLookup;
using imp::loader::details::provider_gltf::MaterialId;
using imp::loader::details::provider_gltf::NodeId;

struct Domain {
  float min;
  float max;
};

template <typename T>
absl::string_view ExpectedLayoutType() = delete;

template <>
inline absl::string_view ExpectedLayoutType<schemas::ChannelFloat3>() {
  return "VEC3";
}

template <>
inline absl::string_view ExpectedLayoutType<schemas::ChannelFloat4>() {
  return "VEC4";
}

template <>
inline absl::string_view ExpectedLayoutType<schemas::ChannelQuatf>() {
  return "VEC4";
}

template <>
inline absl::string_view ExpectedLayoutType<schemas::ChannelFloatVector>() {
  return "SCALAR";
}

template <>
inline absl::string_view ExpectedLayoutType<schemas::ChannelFloat>() {
  return "SCALAR";
}

template <>
inline absl::string_view ExpectedLayoutType<schemas::ChannelFloat2>() {
  return "VEC2";
}

template <typename T>
int ValueChannelCount() = delete;

template <>
inline int ValueChannelCount<schemas::ChannelFloat3>() {
  return 3;
}

template <>
inline int ValueChannelCount<schemas::ChannelFloat4>() {
  return 4;
}

template <>
inline int ValueChannelCount<schemas::ChannelQuatf>() {
  return 4;
}

template <>
inline int ValueChannelCount<schemas::ChannelFloat>() {
  return 1;
}

template <>
inline int ValueChannelCount<schemas::ChannelFloat2>() {
  return 2;
}

/// Parses and returns a span of floating point time data for |sampler.input|,
/// or NullOpt if the GLTF referred to an invalid accessor.
absl::StatusOr<absl::Span<const float>> GetTimeData(
    const imp::gltf::imp_proto::Gltf& gltf, AnimationSampler sampler);

template <typename ValueType>
flatbuffers::Offset<flatbuffers::Vector<const ValueType*>> CreateVector(
    absl::Span<const float> data, flatbuffers::FlatBufferBuilder* fbb) {
  static_assert((sizeof(ValueType) % sizeof(float)) == 0, "invalid ValueType");
  constexpr auto channels = sizeof(ValueType) / sizeof(float);
  return fbb->CreateVectorOfStructs(
      reinterpret_cast<const ValueType*>(data.data()), data.size() / channels);
}

template <typename T>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    T type, absl::Span<const float> time_data,
    absl::Span<const float> value_data,
    flatbuffers::FlatBufferBuilder* fbb) = delete;

template <>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    schemas::ChannelFloat3 type, absl::Span<const float> time_data,
    absl::Span<const float> value_data, flatbuffers::FlatBufferBuilder* fbb);

template <>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    schemas::ChannelFloat4 type, absl::Span<const float> time_data,
    absl::Span<const float> value_data, flatbuffers::FlatBufferBuilder* fbb);

template <>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    schemas::ChannelQuatf type, absl::Span<const float> time_data,
    absl::Span<const float> value_data, flatbuffers::FlatBufferBuilder* fbb);

template <>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    schemas::ChannelFloat type, absl::Span<const float> time_data,
    absl::Span<const float> value_data, flatbuffers::FlatBufferBuilder* fbb);

template <>
absl::StatusOr<flatbuffers::Offset<void>> Serialize(
    schemas::ChannelFloat2 type, absl::Span<const float> time_data,
    absl::Span<const float> value_data, flatbuffers::FlatBufferBuilder* fbb);

template <typename T>
absl::StatusOr<T> GetCurveType(absl::string_view type) {
  if (type.empty() || type == "LINEAR") {
    return T::kLinear;
  } else if (type == "STEP") {
    return T::kStep;
  } else if (type == "CUBICSPLINE") {
    return T::kCubic;
  }
  return Error("Vector channel had invalid interpolation type '%.*s'",
               type.size(), type.data());
}

absl::Status ValidateKeyframeData(absl::Span<const float> time_data,
                                  absl::Span<const float> value_data);

template <typename T>
OptionalError AddChannel(const imp::gltf::imp_proto::Gltf& gltf,
                         const AnimationSampler sampler,
                         flatbuffers::FlatBufferBuilder* fbb, T* out_type,
                         flatbuffers::Offset<void>* out_union,
                         absl::optional<Domain>* out_domain) {
  if (!sampler.output) return absl::InternalError("Invalid sampler");
  MP_ASSIGN_OR_RETURN(
      loader::details::AccessorReader reader,
      loader::details::AccessorReader::Create(gltf, *sampler.output));
  loader::details::DenseDataAccess dense_data_storage;

  // According to the spec, vector animations must be 3d floating point values.
  if (reader.GetComponentType() != imp::gltf::imp_proto::ComponentType::FLOAT ||
      reader.GetType() != ExpectedLayoutType<T>()) {
    // Special case: support normalized SHORT4 orientations.
    if (reader.GetComponentType() ==
            imp::gltf::imp_proto::ComponentType::SHORT &&
        reader.GetType() == "VEC4") {
      MP_ASSIGN_OR_RETURN(dense_data_storage, reader.GetPackedFloatData());
    } else {
      return absl::InternalError("Vector Channel output format incorrect");
    }
  } else {
    dense_data_storage = reader.GetPackedData();
  }

  MP_ASSIGN_OR_RETURN(absl::Span<const float> time_data,
                   GetTimeData(gltf, sampler));
  MP_ASSIGN_OR_RETURN(T curve_type, GetCurveType<T>(sampler.interpolation));

  // The input and output samplers need an equal number of values, unless using
  // cubic spline interpolation, in which case there should be three times as
  // many outputs as inputs.
  const size_t expected_count =
      (curve_type == T::kCubic) ? time_data.size() * 3 : time_data.size();
  if (reader.GetCount() != expected_count) {
    return absl::InternalError("Invalid channel");
  }
  auto channel_count = ValueChannelCount<T>();
  auto value_data =
      absl::Span<const float>(dense_data_storage.ReadRawData<float>(),
                              channel_count * reader.GetCount());
  MP_RETURN_IF_ERROR(ValidateKeyframeData(time_data, value_data));
  MP_ASSIGN_OR_RETURN(flatbuffers::Offset<void> curve_union,
                   Serialize(curve_type, time_data, value_data, fbb));

  *out_type = curve_type;
  *out_union = curve_union;
  out_domain->emplace(Domain{time_data.front(), time_data.back()});
  return NoError();
}

absl::optional<Domain> MergeDomains(const absl::optional<Domain>& a,
                                    const absl::optional<Domain>& b);

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_CONVERSIONS_HELPER_H_
