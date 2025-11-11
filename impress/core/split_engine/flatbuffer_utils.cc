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

#include <vector>

#include "core/common/log.h"
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
  if (options.variants.directional_lighting !=
          MaterialPreCompileVariants::DEFAULT ||
      options.variants.dynamic_lighting !=
          MaterialPreCompileVariants::DEFAULT ||
      options.variants.shadow_receiver != MaterialPreCompileVariants::DEFAULT ||
      options.variants.fog != MaterialPreCompileVariants::DEFAULT ||
      options.variants.skinning != MaterialPreCompileVariants::DEFAULT ||
      options.variants.ssr != MaterialPreCompileVariants::DEFAULT ||
      options.variants.ste != MaterialPreCompileVariants::DEFAULT ||
      options.variants.vsm != MaterialPreCompileVariants::DEFAULT) {
    IMP_LOG(imp::WARNING) << "variants is not supported.";
  }
  if (options.spherical_harmonics_bands.has_value()) {
    IMP_LOG(imp::WARNING) << "spherical_harmonics_bands is not supported.";
  }
  if (options.shadow_sampling_quality !=
      MaterialPreCompileOptions::ShadowSamplingQuality::
          SHADOW_SAMPLING_QUALITY_UNSPECIFIED) {
    IMP_LOG(imp::WARNING) << "shadow_sampling_quality is not supported.";
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
  return android_xr::schemas::CreateMaterialPrecompileOptions(
      fbb, fbb.CreateVector(constant_offsets));
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
  return precompile_options;
}

}  // namespace imp::split_engine
