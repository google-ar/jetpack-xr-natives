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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SCHEMA_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SCHEMA_UTILS_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace imp::split_engine {

// Converts an Android XR flatbuffer error code to an absl::Status.
absl::Status ErrorCodeToStatus(android_xr::schemas::ErrorCode error_code,
                               absl::string_view error_message);

// Converts an absl::Status to an Android XR flatbuffer error code.
android_xr::schemas::ErrorCode StatusToErrorCode(absl::Status status);

// Serializes the given flatbuffer table to a vector of bytes.
template <typename TableT>
std::vector<uint8_t> SerializeTable(flatbuffers::FlatBufferBuilder& fbb,
                                    flatbuffers::Offset<TableT> table_offset) {
  fbb.Finish(table_offset);
  std::vector<uint8_t> response_data(fbb.GetSize());
  response_data.assign(fbb.GetBufferSpan().begin(), fbb.GetBufferSpan().end());
  return response_data;
}

// Serializes any of android_xr::schemas::ResponseTypes to a vector of bytes.
template <typename ResponseT>
std::vector<uint8_t> SerializeResponse(
    flatbuffers::FlatBufferBuilder& fbb,
    flatbuffers::Offset<ResponseT> response_offset) {
  flatbuffers::Offset<android_xr::schemas::Response> response =
      android_xr::schemas::CreateResponse(
          fbb, android_xr::schemas::ResponseTypesTraits<ResponseT>::enum_value,
          response_offset.Union());
  return SerializeTable(fbb, response);
}

// Math struct helpers.
android_xr::schemas::Bool Pack(const bool& obj);
bool UnPack(const android_xr::schemas::Bool& obj);
android_xr::schemas::Bool2 Pack(const bool2& obj);
bool2 UnPack(const android_xr::schemas::Bool2& obj);
android_xr::schemas::Bool3 Pack(const bool3& obj);
bool3 UnPack(const android_xr::schemas::Bool3& obj);
android_xr::schemas::Bool4 Pack(const bool4& obj);
bool4 UnPack(const android_xr::schemas::Bool4& obj);
android_xr::schemas::Int Pack(const int32_t& obj);
int32_t UnPack(const android_xr::schemas::Int& obj);
android_xr::schemas::Int2 Pack(const int2& obj);
int2 UnPack(const android_xr::schemas::Int2& obj);
android_xr::schemas::Int3 Pack(const int3& obj);
int3 UnPack(const android_xr::schemas::Int3& obj);
android_xr::schemas::Int4 Pack(const int4& obj);
int4 UnPack(const android_xr::schemas::Int4& obj);
android_xr::schemas::Uint Pack(const uint32_t& obj);
uint32_t UnPack(const android_xr::schemas::Uint& obj);
android_xr::schemas::Uint2 Pack(const uint2& obj);
uint2 UnPack(const android_xr::schemas::Uint2& obj);
android_xr::schemas::Uint3 Pack(const uint3& obj);
uint3 UnPack(const android_xr::schemas::Uint3& obj);
android_xr::schemas::Uint4 Pack(const uint4& obj);
uint4 UnPack(const android_xr::schemas::Uint4& obj);
android_xr::schemas::Float Pack(const float& obj);
float UnPack(const android_xr::schemas::Float& obj);
android_xr::schemas::Float2 Pack(const float2& obj);
float2 UnPack(const android_xr::schemas::Float2& obj);
android_xr::schemas::Float3 Pack(const float3& obj);
float3 UnPack(const android_xr::schemas::Float3& obj);
android_xr::schemas::Float4 Pack(const float4& obj);
float4 UnPack(const android_xr::schemas::Float4& obj);
android_xr::schemas::Quatf Pack(const quatf& obj);
quatf UnPack(const android_xr::schemas::Quatf& obj);
android_xr::schemas::Mat3f Pack(const mat3f& obj);
mat3f UnPack(const android_xr::schemas::Mat3f& obj);
android_xr::schemas::Mat4f Pack(const mat4f& obj);
mat4f UnPack(const android_xr::schemas::Mat4f& obj);
android_xr::schemas::Mat4 Pack(const mat4& obj);
mat4 UnPack(const android_xr::schemas::Mat4& obj);

// Helper to convert an std::optional to ptr for adding to
template <typename T>
const T* PointerFromOptional(const std::optional<T>& value) {
  return value.has_value() ? &value.value() : nullptr;
}

// Wrapper for converting an std::optional of a native type to a pointer of a
// FlatBuffer struct. It holds a copy of the FlatBuffer struct and implicitly
// converts to a pointer to it. This allows inline usage securely because the
// temporary wrapper survives until the end of the full expression.
template <typename OutType, typename InType>
struct FbbStructWrapper {
  std::optional<OutType> storage;

  explicit FbbStructWrapper(const std::optional<InType>& in) {
    if (in.has_value()) {
      storage.emplace(OutType(*in));
    }
  }

  operator const OutType*() const {
    return storage.has_value() ? &storage.value() : nullptr;
  }
};

// Helper to convert an std::optional to a FlatBuffer struct of the same
// underlying primitive type, i.e. from std::optional<float> to
// flatbuffers::Offset<android_xr::schemas::Float>.
template <typename OutType, typename InType>
FbbStructWrapper<OutType, InType> StructFromOptional(
    const std::optional<InType>& in) {
  return FbbStructWrapper<OutType, InType>(in);
}

// Packs the proto MaterialPreCompileOptions to the flatbuffer format.
flatbuffers::Offset<android_xr::schemas::MaterialPrecompileOptions> Pack(
    flatbuffers::FlatBufferBuilder& fbb,
    const MaterialPreCompileOptions& options);

// Unpacks the proto MaterialPreCompileOptions from the flatbuffer format.
MaterialPreCompileOptions UnPack(
    const android_xr::schemas::MaterialPrecompileOptions& options);

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SCHEMA_UTILS_H_
