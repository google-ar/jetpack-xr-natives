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

#include "core/split_engine/flatbuffer_utils.h"

#include <cstdint>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace imp::split_engine {

absl::Status ErrorCodeToStatus(android_xr::schemas::ErrorCode error_code,
                               absl::string_view error_message) {
  switch (error_code) {
    case android_xr::schemas::ErrorCode::INVALID_ARGUMENT_ERROR:
      return absl::InvalidArgumentError(error_message);
    case android_xr::schemas::ErrorCode::FAILED_PRECONDITION_ERROR:
      return absl::FailedPreconditionError(error_message);
    case android_xr::schemas::ErrorCode::NOT_FOUND_ERROR:
      return absl::NotFoundError(error_message);
    case android_xr::schemas::ErrorCode::ALREADY_EXISTS_ERROR:
      return absl::AlreadyExistsError(error_message);
    case android_xr::schemas::ErrorCode::OUT_OF_RANGE_ERROR:
      return absl::OutOfRangeError(error_message);
    case android_xr::schemas::ErrorCode::PERMISSION_DENIED_ERROR:
      return absl::PermissionDeniedError(error_message);
    case android_xr::schemas::ErrorCode::INTERNAL_ERROR:
      return absl::InternalError(error_message);
    case android_xr::schemas::ErrorCode::UNIMPLEMENTED_ERROR:
      return absl::UnimplementedError(error_message);
    case android_xr::schemas::ErrorCode::ABORTED_ERROR:
      return absl::AbortedError(error_message);
    case android_xr::schemas::ErrorCode::UNAVAILABLE_ERROR:
      return absl::UnavailableError(error_message);
    case android_xr::schemas::ErrorCode::RESOURCE_EXHAUSTED_ERROR:
      return absl::ResourceExhaustedError(error_message);
    case android_xr::schemas::ErrorCode::DEADLINE_EXCEEDED_ERROR:
      return absl::DeadlineExceededError(error_message);
    case android_xr::schemas::ErrorCode::DATA_LOSS_ERROR:
      return absl::DataLossError(error_message);
    case android_xr::schemas::ErrorCode::UNAUTHENTICATED_ERROR:
      return absl::UnauthenticatedError(error_message);
    case android_xr::schemas::ErrorCode::CANCELLED_ERROR:
      return absl::CancelledError(error_message);
    case android_xr::schemas::ErrorCode::UNKNOWN_ERROR:
    default:
      return absl::UnknownError(error_message);
  }
}

android_xr::schemas::ErrorCode StatusToErrorCode(absl::Status status) {
  switch (status.code()) {
    case absl::StatusCode::kInvalidArgument:
      return android_xr::schemas::ErrorCode::INVALID_ARGUMENT_ERROR;
    case absl::StatusCode::kFailedPrecondition:
      return android_xr::schemas::ErrorCode::FAILED_PRECONDITION_ERROR;
    case absl::StatusCode::kNotFound:
      return android_xr::schemas::ErrorCode::NOT_FOUND_ERROR;
    case absl::StatusCode::kAlreadyExists:
      return android_xr::schemas::ErrorCode::ALREADY_EXISTS_ERROR;
    case absl::StatusCode::kOutOfRange:
      return android_xr::schemas::ErrorCode::OUT_OF_RANGE_ERROR;
    case absl::StatusCode::kPermissionDenied:
      return android_xr::schemas::ErrorCode::PERMISSION_DENIED_ERROR;
    case absl::StatusCode::kInternal:
      return android_xr::schemas::ErrorCode::INTERNAL_ERROR;
    case absl::StatusCode::kUnimplemented:
      return android_xr::schemas::ErrorCode::UNIMPLEMENTED_ERROR;
    case absl::StatusCode::kAborted:
      return android_xr::schemas::ErrorCode::ABORTED_ERROR;
    case absl::StatusCode::kUnavailable:
      return android_xr::schemas::ErrorCode::UNAVAILABLE_ERROR;
    case absl::StatusCode::kResourceExhausted:
      return android_xr::schemas::ErrorCode::RESOURCE_EXHAUSTED_ERROR;
    case absl::StatusCode::kDeadlineExceeded:
      return android_xr::schemas::ErrorCode::DEADLINE_EXCEEDED_ERROR;
    case absl::StatusCode::kDataLoss:
      return android_xr::schemas::ErrorCode::DATA_LOSS_ERROR;
    case absl::StatusCode::kUnauthenticated:
      return android_xr::schemas::ErrorCode::UNAUTHENTICATED_ERROR;
    case absl::StatusCode::kCancelled:
      return android_xr::schemas::ErrorCode::CANCELLED_ERROR;
    case absl::StatusCode::kUnknown:
    default:
      return android_xr::schemas::ErrorCode::UNKNOWN_ERROR;
  }
}

android_xr::schemas::Bool Pack(const bool& obj) {
  return android_xr::schemas::Bool(obj);
}

bool UnPack(const android_xr::schemas::Bool& obj) { return obj.value(); }

android_xr::schemas::Bool2 Pack(const bool2& obj) {
  return android_xr::schemas::Bool2(obj.x, obj.y);
}

bool2 UnPack(const android_xr::schemas::Bool2& obj) {
  return bool2(obj.x(), obj.y());
}

android_xr::schemas::Bool3 Pack(const bool3& obj) {
  return android_xr::schemas::Bool3(obj.x, obj.y, obj.z);
}

bool3 UnPack(const android_xr::schemas::Bool3& obj) {
  return bool3(obj.x(), obj.y(), obj.z());
}

android_xr::schemas::Bool4 Pack(const bool4& obj) {
  return android_xr::schemas::Bool4(obj.x, obj.y, obj.z, obj.w);
}

bool4 UnPack(const android_xr::schemas::Bool4& obj) {
  return bool4(obj.x(), obj.y(), obj.z(), obj.w());
}

android_xr::schemas::Int Pack(const int& obj) {
  return android_xr::schemas::Int(obj);
}

int UnPack(const android_xr::schemas::Int& obj) { return obj.value(); }

android_xr::schemas::Int2 Pack(const int2& obj) {
  return android_xr::schemas::Int2(obj.x, obj.y);
}

int2 UnPack(const android_xr::schemas::Int2& obj) {
  return int2(obj.x(), obj.y());
}

android_xr::schemas::Int3 Pack(const int3& obj) {
  return android_xr::schemas::Int3(obj.x, obj.y, obj.z);
}

int3 UnPack(const android_xr::schemas::Int3& obj) {
  return int3(obj.x(), obj.y(), obj.z());
}

android_xr::schemas::Int4 Pack(const int4& obj) {
  return android_xr::schemas::Int4(obj.x, obj.y, obj.z, obj.w);
}

int4 UnPack(const android_xr::schemas::Int4& obj) {
  return int4(obj.x(), obj.y(), obj.z(), obj.w());
}

android_xr::schemas::Uint Pack(const uint32_t& obj) {
  return android_xr::schemas::Uint(obj);
}

uint32_t UnPack(const android_xr::schemas::Uint& obj) { return obj.value(); }

android_xr::schemas::Uint2 Pack(const uint2& obj) {
  return android_xr::schemas::Uint2(obj.x, obj.y);
}

uint2 UnPack(const android_xr::schemas::Uint2& obj) {
  return uint2(obj.x(), obj.y());
}

android_xr::schemas::Uint3 Pack(const uint3& obj) {
  return android_xr::schemas::Uint3(obj.x, obj.y, obj.z);
}

uint3 UnPack(const android_xr::schemas::Uint3& obj) {
  return uint3(obj.x(), obj.y(), obj.z());
}

android_xr::schemas::Uint4 Pack(const uint4& obj) {
  return android_xr::schemas::Uint4(obj.x, obj.y, obj.z, obj.w);
}

uint4 UnPack(const android_xr::schemas::Uint4& obj) {
  return uint4(obj.x(), obj.y(), obj.z(), obj.w());
}

android_xr::schemas::Float Pack(const float& obj) {
  return android_xr::schemas::Float(obj);
}

float UnPack(const android_xr::schemas::Float& obj) { return obj.value(); }

android_xr::schemas::Float2 Pack(const float2& obj) {
  return android_xr::schemas::Float2(obj.x, obj.y);
}

float2 UnPack(const android_xr::schemas::Float2& obj) {
  return float2(obj.x(), obj.y());
}

android_xr::schemas::Float3 Pack(const float3& obj) {
  return android_xr::schemas::Float3(obj.x, obj.y, obj.z);
}

float3 UnPack(const android_xr::schemas::Float3& obj) {
  return float3(obj.x(), obj.y(), obj.z());
}

android_xr::schemas::Float4 Pack(const float4& obj) {
  return android_xr::schemas::Float4(obj.x, obj.y, obj.z, obj.w);
}

float4 UnPack(const android_xr::schemas::Float4& obj) {
  return float4(obj.x(), obj.y(), obj.z(), obj.w());
}

android_xr::schemas::Quatf Pack(const quatf& obj) {
  return android_xr::schemas::Quatf(obj.x, obj.y, obj.z, obj.w);
}

quatf UnPack(const android_xr::schemas::Quatf& obj) {
  return quatf(obj.w(), obj.x(), obj.y(), obj.z());
}

android_xr::schemas::Mat3f Pack(const mat3f& obj) {
  return android_xr::schemas::Mat3f(obj[0][0], obj[0][1], obj[0][2],  //
                                    obj[1][0], obj[1][1], obj[1][2],  //
                                    obj[2][0], obj[2][1], obj[2][2]);
}

mat3f UnPack(const android_xr::schemas::Mat3f& obj) {
  return mat3f(obj.m00(), obj.m01(), obj.m02(),  //
               obj.m10(), obj.m11(), obj.m12(),  //
               obj.m20(), obj.m21(), obj.m22());
}

android_xr::schemas::Mat4f Pack(const mat4f& obj) {
  return android_xr::schemas::Mat4f(
      obj[0][0], obj[0][1], obj[0][2], obj[0][3],  //
      obj[1][0], obj[1][1], obj[1][2], obj[1][3],  //
      obj[2][0], obj[2][1], obj[2][2], obj[2][3],  //
      obj[3][0], obj[3][1], obj[3][2], obj[3][3]);
}

mat4f UnPack(const android_xr::schemas::Mat4f& obj) {
  return mat4f(obj.m00(), obj.m01(), obj.m02(), obj.m03(),  //
               obj.m10(), obj.m11(), obj.m12(), obj.m13(),  //
               obj.m20(), obj.m21(), obj.m22(), obj.m23(),  //
               obj.m30(), obj.m31(), obj.m32(), obj.m33());
}

android_xr::schemas::Mat4 Pack(const mat4& obj) {
  return android_xr::schemas::Mat4(
      obj[0][0], obj[0][1], obj[0][2], obj[0][3],  //
      obj[1][0], obj[1][1], obj[1][2], obj[1][3],  //
      obj[2][0], obj[2][1], obj[2][2], obj[2][3],  //
      obj[3][0], obj[3][1], obj[3][2], obj[3][3]);
}

mat4 UnPack(const android_xr::schemas::Mat4& obj) {
  return mat4(obj.m00(), obj.m01(), obj.m02(), obj.m03(),  //
              obj.m10(), obj.m11(), obj.m12(), obj.m13(),  //
              obj.m20(), obj.m21(), obj.m22(), obj.m23(),  //
              obj.m30(), obj.m31(), obj.m32(), obj.m33());
}

flatbuffers::Offset<android_xr::schemas::MaterialPrecompileOptions> Pack(
    flatbuffers::FlatBufferBuilder& fbb,
    const MaterialPreCompileOptions& options) {
  if (options.spherical_harmonics_bands.has_value()) {
    IMP_LOG(imp::WARNING) << "spherical_harmonics_bands is not supported.";
  }
  if (options.shadow_sampling_quality !=
      MaterialPreCompileOptions::ShadowSamplingQuality::
          SHADOW_SAMPLING_QUALITY_UNSPECIFIED) {
    IMP_LOG(imp::WARNING) << "shadow_sampling_quality is not supported.";
  }
  if (options.compile_by_view.has_value()) {
    IMP_LOG(imp::WARNING) << "compile_by_view is not supported.";
  }

  std::vector<
      flatbuffers::Offset<android_xr::schemas::MaterialPrecompileConstant>>
      constant_offsets;
  constant_offsets.reserve(options.constants.size());
  for (const MaterialPreCompileConstant& constant : options.constants) {
    flatbuffers::Offset<android_xr::schemas::MaterialPrecompileConstant>
        constant_offset;
    switch (constant.value.index()) {
      case MaterialPreCompileConstant::kValue_Unknown:
        IMP_LOG(imp::WARNING) << "Received null for the precompile constant "
                     << constant.name << ", Ignoring.";
        break;
      case MaterialPreCompileConstant::kValue_IntValue: {
        if (!constant.int_value()) {
          IMP_LOG(imp::WARNING) << "Received null for the precompile constant "
                       << constant.name << ", Ignoring.";
          break;
        }
        android_xr::schemas::Int int_value{*constant.int_value()};
        constant_offset =
            android_xr::schemas::CreateMaterialPrecompileConstantDirect(
                fbb, constant.name.c_str(),
                android_xr::schemas::MaterialPrecompileConstantValue::Int,
                fbb.CreateStruct(int_value).Union());
        break;
      }
      case MaterialPreCompileConstant::kValue_FloatValue: {
        if (!constant.float_value()) {
          IMP_LOG(imp::WARNING) << "Received null for the precompile constant "
                       << constant.name << ", Ignoring.";
          break;
        }
        android_xr::schemas::Float float_value{*constant.float_value()};
        constant_offset =
            android_xr::schemas::CreateMaterialPrecompileConstantDirect(
                fbb, constant.name.c_str(),
                android_xr::schemas::MaterialPrecompileConstantValue::Float,
                fbb.CreateStruct(float_value).Union());
        break;
      }
      case MaterialPreCompileConstant::kValue_BoolValue: {
        if (!constant.bool_value()) {
          IMP_LOG(imp::WARNING) << "Received null for the precompile constant "
                       << constant.name << ", Ignoring.";
          break;
        }
        android_xr::schemas::Bool bool_value{*constant.bool_value()};
        constant_offset =
            android_xr::schemas::CreateMaterialPrecompileConstantDirect(
                fbb, constant.name.c_str(),
                android_xr::schemas::MaterialPrecompileConstantValue::Bool,
                fbb.CreateStruct(bool_value).Union());
        break;
      }
    }
    constant_offsets.push_back(constant_offset);
  }
  using ::android_xr::schemas::VariantFilter;
  VariantFilter variant_filter = VariantFilter::NONE;
  auto add = [&](auto mode, VariantFilter bit) {
    if (mode == MaterialPreCompileVariants::EXCLUDED) {
      variant_filter |= bit;
    }
  };
  add(options.variants.directional_lighting,
      VariantFilter::DIRECTIONAL_LIGHTING);
  add(options.variants.dynamic_lighting, VariantFilter::DYNAMIC_LIGHTING);
  add(options.variants.shadow_receiver, VariantFilter::SHADOW_RECEIVER);
  add(options.variants.skinning, VariantFilter::SKINNING);
  add(options.variants.fog, VariantFilter::FOG);
  add(options.variants.vsm, VariantFilter::VSM);
  add(options.variants.ssr, VariantFilter::SSR);
  add(options.variants.ste, VariantFilter::STE);

  return android_xr::schemas::CreateMaterialPrecompileOptions(
      fbb, fbb.CreateVector(constant_offsets), variant_filter);
}

MaterialPreCompileOptions UnPack(
    const android_xr::schemas::MaterialPrecompileOptions& options) {
  MaterialPreCompileOptions precompile_options;
  if (!options.precompile_constants()) {
    return precompile_options;
  }
  for (const android_xr::schemas::MaterialPrecompileConstant* constant :
       *options.precompile_constants()) {
    if (!constant->name()) {
      IMP_LOG(imp::WARNING) << "Received precompile constant with empty name. "
                   << "Ignoring.";
      continue;
    }

    if (constant->value() == nullptr) {
      IMP_LOG(imp::WARNING) << "Received null for the precompile constant "
                   << constant->name()->c_str() << ", Ignoring.";
      continue;
    }

    MaterialPreCompileConstant precompile_constant;
    precompile_constant.name = constant->name()->c_str();

    switch (constant->value_type()) {
      case android_xr::schemas::MaterialPrecompileConstantValue::NONE:
        IMP_LOG(imp::WARNING) << "Received precompile constant with no value. "
                     << "Ignoring.";
        continue;
      case android_xr::schemas::MaterialPrecompileConstantValue::Int:
        precompile_constant.value =
            constant->value_as<android_xr::schemas::Int>()->value();
        break;
      case android_xr::schemas::MaterialPrecompileConstantValue::Float:
        precompile_constant.value =
            constant->value_as<android_xr::schemas::Float>()->value();
        break;
      case android_xr::schemas::MaterialPrecompileConstantValue::Bool:
        precompile_constant.value =
            constant->value_as<android_xr::schemas::Bool>()->value();
        break;
    }
    precompile_options.constants.push_back(precompile_constant);
  }

  using VariantFilter = android_xr::schemas::VariantFilter;
  VariantFilter mask = options.variant_filter();
  auto get = [&](VariantFilter bit) {
    return !!(mask & bit) ? MaterialPreCompileVariants::EXCLUDED
                          : MaterialPreCompileVariants::DEFAULT;
  };
  precompile_options.variants.directional_lighting =
      get(VariantFilter::DIRECTIONAL_LIGHTING);
  precompile_options.variants.dynamic_lighting =
      get(VariantFilter::DYNAMIC_LIGHTING);
  precompile_options.variants.shadow_receiver =
      get(VariantFilter::SHADOW_RECEIVER);
  precompile_options.variants.skinning = get(VariantFilter::SKINNING);
  precompile_options.variants.fog = get(VariantFilter::FOG);
  precompile_options.variants.vsm = get(VariantFilter::VSM);
  precompile_options.variants.ssr = get(VariantFilter::SSR);
  precompile_options.variants.ste = get(VariantFilter::STE);

  return precompile_options;
}

absl::StatusOr<android_xr::schemas::TextureFormat> Pack(
    filament::backend::TextureFormat format) {
  switch (format) {
    case filament::backend::TextureFormat::R8:
      return android_xr::schemas::TextureFormat::R8;
    case filament::backend::TextureFormat::R8_SNORM:
      return android_xr::schemas::TextureFormat::R8_SNORM;
    case filament::backend::TextureFormat::R8UI:
      return android_xr::schemas::TextureFormat::R8UI;
    case filament::backend::TextureFormat::R8I:
      return android_xr::schemas::TextureFormat::R8I;
    case filament::backend::TextureFormat::STENCIL8:
      return android_xr::schemas::TextureFormat::STENCIL8;
    case filament::backend::TextureFormat::R16F:
      return android_xr::schemas::TextureFormat::R16F;
    case filament::backend::TextureFormat::R16UI:
      return android_xr::schemas::TextureFormat::R16UI;
    case filament::backend::TextureFormat::R16I:
      return android_xr::schemas::TextureFormat::R16I;
    case filament::backend::TextureFormat::RG8:
      return android_xr::schemas::TextureFormat::RG8;
    case filament::backend::TextureFormat::RG8_SNORM:
      return android_xr::schemas::TextureFormat::RG8_SNORM;
    case filament::backend::TextureFormat::RG8UI:
      return android_xr::schemas::TextureFormat::RG8UI;
    case filament::backend::TextureFormat::RG8I:
      return android_xr::schemas::TextureFormat::RG8I;
    case filament::backend::TextureFormat::RGB565:
      return android_xr::schemas::TextureFormat::RGB565;
    case filament::backend::TextureFormat::RGB9_E5:
      return android_xr::schemas::TextureFormat::RGB9_E5;
    case filament::backend::TextureFormat::RGB5_A1:
      return android_xr::schemas::TextureFormat::RGB5_A1;
    case filament::backend::TextureFormat::RGBA4:
      return android_xr::schemas::TextureFormat::RGBA4;
    case filament::backend::TextureFormat::DEPTH16:
      return android_xr::schemas::TextureFormat::DEPTH16;
    case filament::backend::TextureFormat::RGB8:
      return android_xr::schemas::TextureFormat::RGB8;
    case filament::backend::TextureFormat::SRGB8:
      return android_xr::schemas::TextureFormat::SRGB8;
    case filament::backend::TextureFormat::RGB8_SNORM:
      return android_xr::schemas::TextureFormat::RGB8_SNORM;
    case filament::backend::TextureFormat::RGB8UI:
      return android_xr::schemas::TextureFormat::RGB8UI;
    case filament::backend::TextureFormat::RGB8I:
      return android_xr::schemas::TextureFormat::RGB8I;
    case filament::backend::TextureFormat::DEPTH24:
      return android_xr::schemas::TextureFormat::DEPTH24;
    case filament::backend::TextureFormat::R32F:
      return android_xr::schemas::TextureFormat::R32F;
    case filament::backend::TextureFormat::R32UI:
      return android_xr::schemas::TextureFormat::R32UI;
    case filament::backend::TextureFormat::R32I:
      return android_xr::schemas::TextureFormat::R32I;
    case filament::backend::TextureFormat::RG16F:
      return android_xr::schemas::TextureFormat::RG16F;
    case filament::backend::TextureFormat::RG16UI:
      return android_xr::schemas::TextureFormat::RG16UI;
    case filament::backend::TextureFormat::RG16I:
      return android_xr::schemas::TextureFormat::RG16I;
    case filament::backend::TextureFormat::R11F_G11F_B10F:
      return android_xr::schemas::TextureFormat::R11F_G11F_B10F;
    case filament::backend::TextureFormat::RGBA8:
      return android_xr::schemas::TextureFormat::RGBA8;
    case filament::backend::TextureFormat::SRGB8_A8:
      return android_xr::schemas::TextureFormat::SRGB8_A8;
    case filament::backend::TextureFormat::RGBA8_SNORM:
      return android_xr::schemas::TextureFormat::RGBA8_SNORM;
    case filament::backend::TextureFormat::UNUSED:
      return absl::InvalidArgumentError(
          "Texture format UNUSED is not supported.");
    case filament::backend::TextureFormat::RGB10_A2:
      return android_xr::schemas::TextureFormat::RGB10_A2;
    case filament::backend::TextureFormat::RGBA8UI:
      return android_xr::schemas::TextureFormat::RGBA8UI;
    case filament::backend::TextureFormat::RGBA8I:
      return android_xr::schemas::TextureFormat::RGBA8I;
    case filament::backend::TextureFormat::DEPTH32F:
      return android_xr::schemas::TextureFormat::DEPTH32F;
    case filament::backend::TextureFormat::DEPTH24_STENCIL8:
      return android_xr::schemas::TextureFormat::DEPTH24_STENCIL8;
    case filament::backend::TextureFormat::DEPTH32F_STENCIL8:
      return android_xr::schemas::TextureFormat::DEPTH32F_STENCIL8;
    case filament::backend::TextureFormat::RGB16F:
      return android_xr::schemas::TextureFormat::RGB16F;
    case filament::backend::TextureFormat::RGB16UI:
      return android_xr::schemas::TextureFormat::RGB16UI;
    case filament::backend::TextureFormat::RGB16I:
      return android_xr::schemas::TextureFormat::RGB16I;
    case filament::backend::TextureFormat::RG32F:
      return android_xr::schemas::TextureFormat::RG32F;
    case filament::backend::TextureFormat::RG32UI:
      return android_xr::schemas::TextureFormat::RG32UI;
    case filament::backend::TextureFormat::RG32I:
      return android_xr::schemas::TextureFormat::RG32I;
    case filament::backend::TextureFormat::RGBA16F:
      return android_xr::schemas::TextureFormat::RGBA16F;
    case filament::backend::TextureFormat::RGBA16UI:
      return android_xr::schemas::TextureFormat::RGBA16UI;
    case filament::backend::TextureFormat::RGBA16I:
      return android_xr::schemas::TextureFormat::RGBA16I;
    case filament::backend::TextureFormat::RGB32F:
      return android_xr::schemas::TextureFormat::RGB32F;
    case filament::backend::TextureFormat::RGB32UI:
      return android_xr::schemas::TextureFormat::RGB32UI;
    case filament::backend::TextureFormat::RGB32I:
      return android_xr::schemas::TextureFormat::RGB32I;
    case filament::backend::TextureFormat::RGBA32F:
      return android_xr::schemas::TextureFormat::RGBA32F;
    case filament::backend::TextureFormat::RGBA32UI:
      return android_xr::schemas::TextureFormat::RGBA32UI;
    case filament::backend::TextureFormat::RGBA32I:
      return android_xr::schemas::TextureFormat::RGBA32I;
    case filament::backend::TextureFormat::EAC_R11:
      return android_xr::schemas::TextureFormat::EAC_R11;
    case filament::backend::TextureFormat::EAC_R11_SIGNED:
      return android_xr::schemas::TextureFormat::EAC_R11_SIGNED;
    case filament::backend::TextureFormat::EAC_RG11:
      return android_xr::schemas::TextureFormat::EAC_RG11;
    case filament::backend::TextureFormat::EAC_RG11_SIGNED:
      return android_xr::schemas::TextureFormat::EAC_RG11_SIGNED;
    case filament::backend::TextureFormat::ETC2_RGB8:
      return android_xr::schemas::TextureFormat::ETC2_RGB8;
    case filament::backend::TextureFormat::ETC2_SRGB8:
      return android_xr::schemas::TextureFormat::ETC2_SRGB8;
    case filament::backend::TextureFormat::ETC2_RGB8_A1:
      return android_xr::schemas::TextureFormat::ETC2_RGB8_A1;
    case filament::backend::TextureFormat::ETC2_SRGB8_A1:
      return android_xr::schemas::TextureFormat::ETC2_SRGB8_A1;
    case filament::backend::TextureFormat::ETC2_EAC_RGBA8:
      return android_xr::schemas::TextureFormat::ETC2_EAC_RGBA8;
    case filament::backend::TextureFormat::ETC2_EAC_SRGBA8:
      return android_xr::schemas::TextureFormat::ETC2_EAC_SRGBA8;
    case filament::backend::TextureFormat::DXT1_RGB:
      return android_xr::schemas::TextureFormat::DXT1_RGB;
    case filament::backend::TextureFormat::DXT1_RGBA:
      return android_xr::schemas::TextureFormat::DXT1_RGBA;
    case filament::backend::TextureFormat::DXT3_RGBA:
      return android_xr::schemas::TextureFormat::DXT3_RGBA;
    case filament::backend::TextureFormat::DXT5_RGBA:
      return android_xr::schemas::TextureFormat::DXT5_RGBA;
    case filament::backend::TextureFormat::DXT1_SRGB:
      return android_xr::schemas::TextureFormat::DXT1_SRGB;
    case filament::backend::TextureFormat::DXT1_SRGBA:
      return android_xr::schemas::TextureFormat::DXT1_SRGBA;
    case filament::backend::TextureFormat::DXT3_SRGBA:
      return android_xr::schemas::TextureFormat::DXT3_SRGBA;
    case filament::backend::TextureFormat::DXT5_SRGBA:
      return android_xr::schemas::TextureFormat::DXT5_SRGBA;
    case filament::backend::TextureFormat::RGBA_ASTC_4x4:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_4x4;
    case filament::backend::TextureFormat::RGBA_ASTC_5x4:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_5x4;
    case filament::backend::TextureFormat::RGBA_ASTC_5x5:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_5x5;
    case filament::backend::TextureFormat::RGBA_ASTC_6x5:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_6x5;
    case filament::backend::TextureFormat::RGBA_ASTC_6x6:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_6x6;
    case filament::backend::TextureFormat::RGBA_ASTC_8x5:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_8x5;
    case filament::backend::TextureFormat::RGBA_ASTC_8x6:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_8x6;
    case filament::backend::TextureFormat::RGBA_ASTC_8x8:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_8x8;
    case filament::backend::TextureFormat::RGBA_ASTC_10x5:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_10x5;
    case filament::backend::TextureFormat::RGBA_ASTC_10x6:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_10x6;
    case filament::backend::TextureFormat::RGBA_ASTC_10x8:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_10x8;
    case filament::backend::TextureFormat::RGBA_ASTC_10x10:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_10x10;
    case filament::backend::TextureFormat::RGBA_ASTC_12x10:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_12x10;
    case filament::backend::TextureFormat::RGBA_ASTC_12x12:
      return android_xr::schemas::TextureFormat::RGBA_ASTC_12x12;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_4x4:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_4x4;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_5x4:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_5x4;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_5x5:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_5x5;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_6x5:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_6x5;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_6x6:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_6x6;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_8x5:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_8x5;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_8x6:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_8x6;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_8x8:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_8x8;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_10x5:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_10x5;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_10x6:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_10x6;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_10x8:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_10x8;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_10x10:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_10x10;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_12x10:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_12x10;
    case filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_12x12:
      return android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_12x12;
    case filament::backend::TextureFormat::RED_RGTC1:
      return android_xr::schemas::TextureFormat::RED_RGTC1;
    case filament::backend::TextureFormat::SIGNED_RED_RGTC1:
      return android_xr::schemas::TextureFormat::SIGNED_RED_RGTC1;
    case filament::backend::TextureFormat::RED_GREEN_RGTC2:
      return android_xr::schemas::TextureFormat::RED_GREEN_RGTC2;
    case filament::backend::TextureFormat::SIGNED_RED_GREEN_RGTC2:
      return android_xr::schemas::TextureFormat::SIGNED_RED_GREEN_RGTC2;
    case filament::backend::TextureFormat::RGB_BPTC_SIGNED_FLOAT:
      return android_xr::schemas::TextureFormat::RGB_BPTC_SIGNED_FLOAT;
    case filament::backend::TextureFormat::RGB_BPTC_UNSIGNED_FLOAT:
      return android_xr::schemas::TextureFormat::RGB_BPTC_UNSIGNED_FLOAT;
    case filament::backend::TextureFormat::RGBA_BPTC_UNORM:
      return android_xr::schemas::TextureFormat::RGBA_BPTC_UNORM;
    case filament::backend::TextureFormat::SRGB_ALPHA_BPTC_UNORM:
      return android_xr::schemas::TextureFormat::SRGB_ALPHA_BPTC_UNORM;
  }
}

absl::StatusOr<filament::backend::TextureFormat> UnPack(
    android_xr::schemas::TextureFormat format) {
  switch (format) {
    case android_xr::schemas::TextureFormat::R8:
      return filament::backend::TextureFormat::R8;
    case android_xr::schemas::TextureFormat::R8_SNORM:
      return filament::backend::TextureFormat::R8_SNORM;
    case android_xr::schemas::TextureFormat::R8UI:
      return filament::backend::TextureFormat::R8UI;
    case android_xr::schemas::TextureFormat::R8I:
      return filament::backend::TextureFormat::R8I;
    case android_xr::schemas::TextureFormat::STENCIL8:
      return filament::backend::TextureFormat::STENCIL8;
    case android_xr::schemas::TextureFormat::R16F:
      return filament::backend::TextureFormat::R16F;
    case android_xr::schemas::TextureFormat::R16UI:
      return filament::backend::TextureFormat::R16UI;
    case android_xr::schemas::TextureFormat::R16I:
      return filament::backend::TextureFormat::R16I;
    case android_xr::schemas::TextureFormat::RG8:
      return filament::backend::TextureFormat::RG8;
    case android_xr::schemas::TextureFormat::RG8_SNORM:
      return filament::backend::TextureFormat::RG8_SNORM;
    case android_xr::schemas::TextureFormat::RG8UI:
      return filament::backend::TextureFormat::RG8UI;
    case android_xr::schemas::TextureFormat::RG8I:
      return filament::backend::TextureFormat::RG8I;
    case android_xr::schemas::TextureFormat::RGB565:
      return filament::backend::TextureFormat::RGB565;
    case android_xr::schemas::TextureFormat::RGB9_E5:
      return filament::backend::TextureFormat::RGB9_E5;
    case android_xr::schemas::TextureFormat::RGB5_A1:
      return filament::backend::TextureFormat::RGB5_A1;
    case android_xr::schemas::TextureFormat::RGBA4:
      return filament::backend::TextureFormat::RGBA4;
    case android_xr::schemas::TextureFormat::DEPTH16:
      return filament::backend::TextureFormat::DEPTH16;
    case android_xr::schemas::TextureFormat::RGB8:
      return filament::backend::TextureFormat::RGB8;
    case android_xr::schemas::TextureFormat::SRGB8:
      return filament::backend::TextureFormat::SRGB8;
    case android_xr::schemas::TextureFormat::RGB8_SNORM:
      return filament::backend::TextureFormat::RGB8_SNORM;
    case android_xr::schemas::TextureFormat::RGB8UI:
      return filament::backend::TextureFormat::RGB8UI;
    case android_xr::schemas::TextureFormat::RGB8I:
      return filament::backend::TextureFormat::RGB8I;
    case android_xr::schemas::TextureFormat::DEPTH24:
      return filament::backend::TextureFormat::DEPTH24;
    case android_xr::schemas::TextureFormat::R32F:
      return filament::backend::TextureFormat::R32F;
    case android_xr::schemas::TextureFormat::R32UI:
      return filament::backend::TextureFormat::R32UI;
    case android_xr::schemas::TextureFormat::R32I:
      return filament::backend::TextureFormat::R32I;
    case android_xr::schemas::TextureFormat::RG16F:
      return filament::backend::TextureFormat::RG16F;
    case android_xr::schemas::TextureFormat::RG16UI:
      return filament::backend::TextureFormat::RG16UI;
    case android_xr::schemas::TextureFormat::RG16I:
      return filament::backend::TextureFormat::RG16I;
    case android_xr::schemas::TextureFormat::R11F_G11F_B10F:
      return filament::backend::TextureFormat::R11F_G11F_B10F;
    case android_xr::schemas::TextureFormat::RGBA8:
      return filament::backend::TextureFormat::RGBA8;
    case android_xr::schemas::TextureFormat::SRGB8_A8:
      return filament::backend::TextureFormat::SRGB8_A8;
    case android_xr::schemas::TextureFormat::RGBA8_SNORM:
      return filament::backend::TextureFormat::RGBA8_SNORM;
    case android_xr::schemas::TextureFormat::RGB10_A2:
      return filament::backend::TextureFormat::RGB10_A2;
    case android_xr::schemas::TextureFormat::RGBA8UI:
      return filament::backend::TextureFormat::RGBA8UI;
    case android_xr::schemas::TextureFormat::RGBA8I:
      return filament::backend::TextureFormat::RGBA8I;
    case android_xr::schemas::TextureFormat::DEPTH32F:
      return filament::backend::TextureFormat::DEPTH32F;
    case android_xr::schemas::TextureFormat::DEPTH24_STENCIL8:
      return filament::backend::TextureFormat::DEPTH24_STENCIL8;
    case android_xr::schemas::TextureFormat::DEPTH32F_STENCIL8:
      return filament::backend::TextureFormat::DEPTH32F_STENCIL8;
    case android_xr::schemas::TextureFormat::RGB16F:
      return filament::backend::TextureFormat::RGB16F;
    case android_xr::schemas::TextureFormat::RGB16UI:
      return filament::backend::TextureFormat::RGB16UI;
    case android_xr::schemas::TextureFormat::RGB16I:
      return filament::backend::TextureFormat::RGB16I;
    case android_xr::schemas::TextureFormat::RG32F:
      return filament::backend::TextureFormat::RG32F;
    case android_xr::schemas::TextureFormat::RG32UI:
      return filament::backend::TextureFormat::RG32UI;
    case android_xr::schemas::TextureFormat::RG32I:
      return filament::backend::TextureFormat::RG32I;
    case android_xr::schemas::TextureFormat::RGBA16F:
      return filament::backend::TextureFormat::RGBA16F;
    case android_xr::schemas::TextureFormat::RGBA16UI:
      return filament::backend::TextureFormat::RGBA16UI;
    case android_xr::schemas::TextureFormat::RGBA16I:
      return filament::backend::TextureFormat::RGBA16I;
    case android_xr::schemas::TextureFormat::RGB32F:
      return filament::backend::TextureFormat::RGB32F;
    case android_xr::schemas::TextureFormat::RGB32UI:
      return filament::backend::TextureFormat::RGB32UI;
    case android_xr::schemas::TextureFormat::RGB32I:
      return filament::backend::TextureFormat::RGB32I;
    case android_xr::schemas::TextureFormat::RGBA32F:
      return filament::backend::TextureFormat::RGBA32F;
    case android_xr::schemas::TextureFormat::RGBA32UI:
      return filament::backend::TextureFormat::RGBA32UI;
    case android_xr::schemas::TextureFormat::RGBA32I:
      return filament::backend::TextureFormat::RGBA32I;
    case android_xr::schemas::TextureFormat::EAC_R11:
      return filament::backend::TextureFormat::EAC_R11;
    case android_xr::schemas::TextureFormat::EAC_R11_SIGNED:
      return filament::backend::TextureFormat::EAC_R11_SIGNED;
    case android_xr::schemas::TextureFormat::EAC_RG11:
      return filament::backend::TextureFormat::EAC_RG11;
    case android_xr::schemas::TextureFormat::EAC_RG11_SIGNED:
      return filament::backend::TextureFormat::EAC_RG11_SIGNED;
    case android_xr::schemas::TextureFormat::ETC2_RGB8:
      return filament::backend::TextureFormat::ETC2_RGB8;
    case android_xr::schemas::TextureFormat::ETC2_SRGB8:
      return filament::backend::TextureFormat::ETC2_SRGB8;
    case android_xr::schemas::TextureFormat::ETC2_RGB8_A1:
      return filament::backend::TextureFormat::ETC2_RGB8_A1;
    case android_xr::schemas::TextureFormat::ETC2_SRGB8_A1:
      return filament::backend::TextureFormat::ETC2_SRGB8_A1;
    case android_xr::schemas::TextureFormat::ETC2_EAC_RGBA8:
      return filament::backend::TextureFormat::ETC2_EAC_RGBA8;
    case android_xr::schemas::TextureFormat::ETC2_EAC_SRGBA8:
      return filament::backend::TextureFormat::ETC2_EAC_SRGBA8;
    case android_xr::schemas::TextureFormat::DXT1_RGB:
      return filament::backend::TextureFormat::DXT1_RGB;
    case android_xr::schemas::TextureFormat::DXT1_RGBA:
      return filament::backend::TextureFormat::DXT1_RGBA;
    case android_xr::schemas::TextureFormat::DXT3_RGBA:
      return filament::backend::TextureFormat::DXT3_RGBA;
    case android_xr::schemas::TextureFormat::DXT5_RGBA:
      return filament::backend::TextureFormat::DXT5_RGBA;
    case android_xr::schemas::TextureFormat::DXT1_SRGB:
      return filament::backend::TextureFormat::DXT1_SRGB;
    case android_xr::schemas::TextureFormat::DXT1_SRGBA:
      return filament::backend::TextureFormat::DXT1_SRGBA;
    case android_xr::schemas::TextureFormat::DXT3_SRGBA:
      return filament::backend::TextureFormat::DXT3_SRGBA;
    case android_xr::schemas::TextureFormat::DXT5_SRGBA:
      return filament::backend::TextureFormat::DXT5_SRGBA;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_4x4:
      return filament::backend::TextureFormat::RGBA_ASTC_4x4;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_5x4:
      return filament::backend::TextureFormat::RGBA_ASTC_5x4;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_5x5:
      return filament::backend::TextureFormat::RGBA_ASTC_5x5;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_6x5:
      return filament::backend::TextureFormat::RGBA_ASTC_6x5;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_6x6:
      return filament::backend::TextureFormat::RGBA_ASTC_6x6;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_8x5:
      return filament::backend::TextureFormat::RGBA_ASTC_8x5;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_8x6:
      return filament::backend::TextureFormat::RGBA_ASTC_8x6;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_8x8:
      return filament::backend::TextureFormat::RGBA_ASTC_8x8;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_10x5:
      return filament::backend::TextureFormat::RGBA_ASTC_10x5;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_10x6:
      return filament::backend::TextureFormat::RGBA_ASTC_10x6;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_10x8:
      return filament::backend::TextureFormat::RGBA_ASTC_10x8;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_10x10:
      return filament::backend::TextureFormat::RGBA_ASTC_10x10;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_12x10:
      return filament::backend::TextureFormat::RGBA_ASTC_12x10;
    case android_xr::schemas::TextureFormat::RGBA_ASTC_12x12:
      return filament::backend::TextureFormat::RGBA_ASTC_12x12;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_4x4:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_4x4;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_5x4:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_5x4;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_5x5:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_5x5;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_6x5:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_6x5;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_6x6:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_6x6;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_8x5:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_8x5;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_8x6:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_8x6;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_8x8:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_8x8;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_10x5:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_10x5;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_10x6:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_10x6;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_10x8:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_10x8;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_10x10:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_10x10;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_12x10:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_12x10;
    case android_xr::schemas::TextureFormat::SRGB8_ALPHA8_ASTC_12x12:
      return filament::backend::TextureFormat::SRGB8_ALPHA8_ASTC_12x12;
    case android_xr::schemas::TextureFormat::RED_RGTC1:
      return filament::backend::TextureFormat::RED_RGTC1;
    case android_xr::schemas::TextureFormat::SIGNED_RED_RGTC1:
      return filament::backend::TextureFormat::SIGNED_RED_RGTC1;
    case android_xr::schemas::TextureFormat::RED_GREEN_RGTC2:
      return filament::backend::TextureFormat::RED_GREEN_RGTC2;
    case android_xr::schemas::TextureFormat::SIGNED_RED_GREEN_RGTC2:
      return filament::backend::TextureFormat::SIGNED_RED_GREEN_RGTC2;
    case android_xr::schemas::TextureFormat::RGB_BPTC_SIGNED_FLOAT:
      return filament::backend::TextureFormat::RGB_BPTC_SIGNED_FLOAT;
    case android_xr::schemas::TextureFormat::RGB_BPTC_UNSIGNED_FLOAT:
      return filament::backend::TextureFormat::RGB_BPTC_UNSIGNED_FLOAT;
    case android_xr::schemas::TextureFormat::RGBA_BPTC_UNORM:
      return filament::backend::TextureFormat::RGBA_BPTC_UNORM;
    case android_xr::schemas::TextureFormat::SRGB_ALPHA_BPTC_UNORM:
      return filament::backend::TextureFormat::SRGB_ALPHA_BPTC_UNORM;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported texture format: ", format));
  }
}

android_xr::schemas::SamplerType Pack(filament::backend::SamplerType sampler) {
  switch (sampler) {
    case filament::backend::SamplerType::SAMPLER_2D:
      return android_xr::schemas::SamplerType::SAMPLER_2D;
    case filament::backend::SamplerType::SAMPLER_2D_ARRAY:
      return android_xr::schemas::SamplerType::SAMPLER_2D_ARRAY;
    case filament::backend::SamplerType::SAMPLER_CUBEMAP:
      return android_xr::schemas::SamplerType::SAMPLER_CUBEMAP;
    case filament::backend::SamplerType::SAMPLER_EXTERNAL:
      return android_xr::schemas::SamplerType::SAMPLER_EXTERNAL;
    case filament::backend::SamplerType::SAMPLER_3D:
      return android_xr::schemas::SamplerType::SAMPLER_3D;
    case filament::backend::SamplerType::SAMPLER_CUBEMAP_ARRAY:
      return android_xr::schemas::SamplerType::SAMPLER_CUBEMAP_ARRAY;
  }
}

absl::StatusOr<filament::backend::SamplerType> UnPack(
    android_xr::schemas::SamplerType sampler) {
  switch (sampler) {
    case android_xr::schemas::SamplerType::SAMPLER_2D:
      return filament::backend::SamplerType::SAMPLER_2D;
    case android_xr::schemas::SamplerType::SAMPLER_2D_ARRAY:
      return filament::backend::SamplerType::SAMPLER_2D_ARRAY;
    case android_xr::schemas::SamplerType::SAMPLER_CUBEMAP:
      return filament::backend::SamplerType::SAMPLER_CUBEMAP;
    case android_xr::schemas::SamplerType::SAMPLER_EXTERNAL:
      return filament::backend::SamplerType::SAMPLER_EXTERNAL;
    case android_xr::schemas::SamplerType::SAMPLER_3D:
      return filament::backend::SamplerType::SAMPLER_3D;
    case android_xr::schemas::SamplerType::SAMPLER_CUBEMAP_ARRAY:
      return filament::backend::SamplerType::SAMPLER_CUBEMAP_ARRAY;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported sampler type: ", sampler));
  }
}

absl::StatusOr<android_xr::schemas::PixelDataFormat> Pack(
    filament::backend::PixelDataFormat format) {
  switch (format) {
    case filament::backend::PixelDataFormat::R:
      return android_xr::schemas::PixelDataFormat::R;
    case filament::backend::PixelDataFormat::R_INTEGER:
      return android_xr::schemas::PixelDataFormat::R_INTEGER;
    case filament::backend::PixelDataFormat::RG:
      return android_xr::schemas::PixelDataFormat::RG;
    case filament::backend::PixelDataFormat::RG_INTEGER:
      return android_xr::schemas::PixelDataFormat::RG_INTEGER;
    case filament::backend::PixelDataFormat::RGB:
      return android_xr::schemas::PixelDataFormat::RGB;
    case filament::backend::PixelDataFormat::RGB_INTEGER:
      return android_xr::schemas::PixelDataFormat::RGB_INTEGER;
    case filament::backend::PixelDataFormat::RGBA:
      return android_xr::schemas::PixelDataFormat::RGBA;
    case filament::backend::PixelDataFormat::RGBA_INTEGER:
      return android_xr::schemas::PixelDataFormat::RGBA_INTEGER;
    case filament::backend::PixelDataFormat::UNUSED:
      return absl::InvalidArgumentError("Unused pixel data format");
    case filament::backend::PixelDataFormat::DEPTH_COMPONENT:
      return android_xr::schemas::PixelDataFormat::DEPTH_COMPONENT;
    case filament::backend::PixelDataFormat::DEPTH_STENCIL:
      return android_xr::schemas::PixelDataFormat::DEPTH_STENCIL;
    case filament::backend::PixelDataFormat::ALPHA:
      return android_xr::schemas::PixelDataFormat::ALPHA;
  }
}

absl::StatusOr<filament::backend::PixelDataFormat> UnPack(
    android_xr::schemas::PixelDataFormat format) {
  switch (format) {
    case android_xr::schemas::PixelDataFormat::R:
      return filament::backend::PixelDataFormat::R;
    case android_xr::schemas::PixelDataFormat::R_INTEGER:
      return filament::backend::PixelDataFormat::R_INTEGER;
    case android_xr::schemas::PixelDataFormat::RG:
      return filament::backend::PixelDataFormat::RG;
    case android_xr::schemas::PixelDataFormat::RG_INTEGER:
      return filament::backend::PixelDataFormat::RG_INTEGER;
    case android_xr::schemas::PixelDataFormat::RGB:
      return filament::backend::PixelDataFormat::RGB;
    case android_xr::schemas::PixelDataFormat::RGB_INTEGER:
      return filament::backend::PixelDataFormat::RGB_INTEGER;
    case android_xr::schemas::PixelDataFormat::RGBA:
      return filament::backend::PixelDataFormat::RGBA;
    case android_xr::schemas::PixelDataFormat::RGBA_INTEGER:
      return filament::backend::PixelDataFormat::RGBA_INTEGER;
    case android_xr::schemas::PixelDataFormat::DEPTH_COMPONENT:
      return filament::backend::PixelDataFormat::DEPTH_COMPONENT;
    case android_xr::schemas::PixelDataFormat::DEPTH_STENCIL:
      return filament::backend::PixelDataFormat::DEPTH_STENCIL;
    case android_xr::schemas::PixelDataFormat::ALPHA:
      return filament::backend::PixelDataFormat::ALPHA;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported pixel data format: ", format));
  }
}

android_xr::schemas::PixelDataType Pack(filament::backend::PixelDataType type) {
  switch (type) {
    case filament::backend::PixelDataType::UBYTE:
      return android_xr::schemas::PixelDataType::UBYTE;
    case filament::backend::PixelDataType::BYTE:
      return android_xr::schemas::PixelDataType::BYTE;
    case filament::backend::PixelDataType::USHORT:
      return android_xr::schemas::PixelDataType::USHORT;
    case filament::backend::PixelDataType::SHORT:
      return android_xr::schemas::PixelDataType::SHORT;
    case filament::backend::PixelDataType::UINT:
      return android_xr::schemas::PixelDataType::UINT;
    case filament::backend::PixelDataType::INT:
      return android_xr::schemas::PixelDataType::INT;
    case filament::backend::PixelDataType::HALF:
      return android_xr::schemas::PixelDataType::HALF;
    case filament::backend::PixelDataType::FLOAT:
      return android_xr::schemas::PixelDataType::FLOAT;
    case filament::backend::PixelDataType::COMPRESSED:
      return android_xr::schemas::PixelDataType::COMPRESSED;
    case filament::backend::PixelDataType::UINT_10F_11F_11F_REV:
      return android_xr::schemas::PixelDataType::UINT_10F_11F_11F_REV;
    case filament::backend::PixelDataType::USHORT_565:
      return android_xr::schemas::PixelDataType::USHORT_565;
    case filament::backend::PixelDataType::UINT_2_10_10_10_REV:
      return android_xr::schemas::PixelDataType::UINT_2_10_10_10_REV;
  }
}

absl::StatusOr<filament::backend::PixelDataType> UnPack(
    android_xr::schemas::PixelDataType type) {
  switch (type) {
    case android_xr::schemas::PixelDataType::UBYTE:
      return filament::backend::PixelDataType::UBYTE;
    case android_xr::schemas::PixelDataType::BYTE:
      return filament::backend::PixelDataType::BYTE;
    case android_xr::schemas::PixelDataType::USHORT:
      return filament::backend::PixelDataType::USHORT;
    case android_xr::schemas::PixelDataType::SHORT:
      return filament::backend::PixelDataType::SHORT;
    case android_xr::schemas::PixelDataType::UINT:
      return filament::backend::PixelDataType::UINT;
    case android_xr::schemas::PixelDataType::INT:
      return filament::backend::PixelDataType::INT;
    case android_xr::schemas::PixelDataType::HALF:
      return filament::backend::PixelDataType::HALF;
    case android_xr::schemas::PixelDataType::FLOAT:
      return filament::backend::PixelDataType::FLOAT;
    case android_xr::schemas::PixelDataType::COMPRESSED:
      return filament::backend::PixelDataType::COMPRESSED;
    case android_xr::schemas::PixelDataType::UINT_10F_11F_11F_REV:
      return filament::backend::PixelDataType::UINT_10F_11F_11F_REV;
    case android_xr::schemas::PixelDataType::USHORT_565:
      return filament::backend::PixelDataType::USHORT_565;
    case android_xr::schemas::PixelDataType::UINT_2_10_10_10_REV:
      return filament::backend::PixelDataType::UINT_2_10_10_10_REV;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported pixel data type: ", type));
  }
}

}  // namespace imp::split_engine
