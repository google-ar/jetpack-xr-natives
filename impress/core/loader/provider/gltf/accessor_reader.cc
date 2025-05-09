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

#include "core/loader/provider/gltf/accessor_reader.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "filament/libs/math/include/math/norm.h"
#include "filament/libs/math/include/math/vec3.h"
#include "core/common/buffer_access.h"
#include "core/common/platform_helpers.h"
#include "core/common/schemas/render_generated.h"
#include "core/loader/details/bundle_resource_helpers.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_attribute.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details {
namespace {
using ::filament::math::float3;
using ::imp::gltf::Accessor;
using ::imp::gltf::Buffer;
using ::imp::gltf::BufferView;
using ::imp::gltf::Gltf;

absl::StatusOr<const uint8_t*> GetDataFromBufferView(
    const Gltf& gltf, const Accessor& accessor, const BufferView& buffer_view) {
  if (!buffer_view.buffer.has_value() ||
      (*buffer_view.buffer >= gltf.buffers.size())) {
    return absl::InvalidArgumentError("BufferView had invalid Buffer");
  }
  const Buffer& buffer = gltf.buffers[*buffer_view.buffer];
  size_t base_offset = static_cast<size_t>(buffer_view.byte_offset) +
                       static_cast<size_t>(accessor.byte_offset);
  size_t buffer_size =
      std::min(buffer.access.size(), static_cast<size_t>(buffer.byte_length));

  if (base_offset >= buffer_size)
    return absl::OutOfRangeError("AttributeLayout base was out of range");

  return reinterpret_cast<const uint8_t*>(&buffer.access.at(base_offset));
}

}  // namespace

// The size of the indexable type in an accessor.
size_t AccessorReader::GetAccessorTypeSize(
    absl::string_view type, imp::gltf::ComponentType component_type) {
  // Logic taken from cgltf_calc_size.
  size_t size = 0;
  switch (component_type) {
    case imp::gltf::ComponentType::BYTE:
    case imp::gltf::ComponentType::UNSIGNED_BYTE:
      size = 1;
      break;
    case imp::gltf::ComponentType::SHORT:
    case imp::gltf::ComponentType::UNSIGNED_SHORT:
      size = 2;
      break;
    case imp::gltf::ComponentType::UNSIGNED_INT:
    case imp::gltf::ComponentType::FLOAT:
      size = 4;
      break;
    default:
      break;
  }
  if (type == "MAT2" && size == 1) {
    return 8 * size;
  } else if (type == "MAT3" && (size == 1 || size == 2)) {
    return 12 * size;
  } else if (type == "VEC2") {
    return 2 * size;
  } else if (type == "VEC3") {
    return 3 * size;
  } else if (type == "VEC4" || type == "MAT2") {
    return 4 * size;
  } else if (type == "MAT3") {
    return 9 * size;
  } else if (type == "MAT4") {
    return 16 * size;
  } else {
    return size;
  }
}

size_t AccessorReader::GetAccessorTypeSize(const Accessor& accessor) {
  return AccessorReader::GetAccessorTypeSize(accessor.type,
                                             accessor.component_type);
}

absl::StatusOr<AccessorReader> AccessorReader::Create(const Gltf& gltf,
                                                      int accessor_index) {
  AccessorReader reader;
  if (accessor_index < 0 || accessor_index >= gltf.accessors.size()) {
    return absl::InvalidArgumentError("Attribute had invalid Accessor");
  }
  const Accessor& accessor = gltf.accessors[accessor_index];

  if (accessor.buffer_view.has_value()) {
    if ((*accessor.buffer_view >= gltf.buffer_views.size())) {
      return absl::UnavailableError("Accessor had invalid BufferView");
    }

    const BufferView& buffer_view = gltf.buffer_views[*accessor.buffer_view];
    if (!buffer_view.buffer.has_value() ||
        (*buffer_view.buffer >= gltf.buffers.size())) {
      return absl::UnavailableError("BufferView had invalid Buffer");
    }
    const Buffer& buffer = gltf.buffers[*buffer_view.buffer];
    size_t base_offset = static_cast<size_t>(buffer_view.byte_offset) +
                         static_cast<size_t>(accessor.byte_offset);
    size_t buffer_size =
        std::min(buffer.access.size(), static_cast<size_t>(buffer.byte_length));

    if (base_offset >= buffer_size) {
      return absl::OutOfRangeError("AttributeLayout base was out of range");
    }

    const uint8_t* base =
        reinterpret_cast<const uint8_t*>(&buffer.access.at(base_offset));
    // The stride in buffer views may not be specified, in which case it is
    // implied to be the size of the type in the accessor.  It must be at least
    // the size of the accessor.
    size_t minimum_stride = GetAccessorTypeSize(accessor);
    size_t stride = (buffer_view.byte_stride > 0) ? buffer_view.byte_stride
                                                  : minimum_stride;
    size_t count = accessor.count;
    if (base == nullptr || stride == 0 || count == 0) {
      return absl::UnavailableError("Accessor specified no data");
    }
    if (stride < minimum_stride) {
      return absl::FailedPreconditionError(
          "BufferView had an insufficient stride");
    }
    if (base_offset + (stride * (count - 1)) + minimum_stride > buffer_size) {
      return absl::OutOfRangeError("AttributeLayout went past end of buffer");
    }

    reader.base_ = base;
    reader.stride_ = stride;
  } else {
    // This accessor is using default value 0s.
    size_t stride = GetAccessorTypeSize(accessor);
    if (accessor.count == 0 || stride == 0) {
      return absl::UnavailableError("Accessor specified no data");
    }

    reader.base_ = nullptr;
    reader.stride_ = stride;
  }
  reader.component_type_ = accessor.component_type;
  reader.type_ = accessor.type;
  reader.count_ = accessor.count;

  if (accessor.sparse.has_value()) {
    reader.sparse_accessor_ = SparseAccessorReader();
    reader.sparse_accessor_->count = accessor.sparse->count;
    reader.sparse_accessor_->indices_type =
        accessor.sparse->indices.component_type;

    if (accessor.sparse->indices.buffer_view >= gltf.buffer_views.size()) {
      return absl::UnavailableError("Invalid sparse indices");
    }
    MP_ASSIGN_OR_RETURN(
        reader.sparse_accessor_->indices_data,
        GetDataFromBufferView(
            gltf, accessor,
            gltf.buffer_views[accessor.sparse->indices.buffer_view]));

    if (accessor.sparse->values.buffer_view >= gltf.buffer_views.size()) {
      return absl::UnavailableError("Invalid sparse values");
    }
    MP_ASSIGN_OR_RETURN(
        reader.sparse_accessor_->sparse_values_data,
        GetDataFromBufferView(
            gltf, accessor,
            gltf.buffer_views[accessor.sparse->values.buffer_view]));
  }

  return reader;
}

absl::StatusOr<AccessorReader> AccessorReader::Create(
    const imp::gltf::Gltf& gltf, AccessorId accessor_id) {
  return Create(gltf, static_cast<int>(accessor_id));
}

BufferAccess AccessorReader::CopyAndPackData() const {
  // MAT4 is not an enum in filament::VertexBuffer::AttributeType, so that it
  // cannot be included in the mirrored type schema::AttributeType. This edge
  // case is handled separately.
  if (type_ == "MAT4" && component_type_ == gltf::ComponentType::FLOAT) {
    return CopyAndPackData<mat4f>();
  }
  // Used for indices data in some case.
  if (type_ == "VEC3" && component_type_ == gltf::ComponentType::UNSIGNED_INT) {
    return CopyAndPackData<uint3>();
  }
  schemas::AttributeType attribute_type;
  absl::Status status =
      provider_gltf::GetAttributeType(type_, component_type_, &attribute_type);
  if (!status.ok()) {
    IMP_LOG(imp::WARNING) << "type: " << type_
                 << ", component_type: " << static_cast<int>(component_type_);
    
  }
  switch (attribute_type) {
    case schemas::AttributeType::BYTE:
      return CopyAndPackData<int8_t>();
    case schemas::AttributeType::BYTE2:
      return CopyAndPackData<byte2>();
    case schemas::AttributeType::BYTE3:
      return CopyAndPackData<byte3>();
    case schemas::AttributeType::BYTE4:
      return CopyAndPackData<byte4>();
    case schemas::AttributeType::UBYTE:
      return CopyAndPackData<uint8_t>();
    case schemas::AttributeType::UBYTE2:
      return CopyAndPackData<ubyte2>();
    case schemas::AttributeType::UBYTE3:
      return CopyAndPackData<ubyte3>();
    case schemas::AttributeType::UBYTE4:
      return CopyAndPackData<ubyte4>();
    case schemas::AttributeType::SHORT:
      return CopyAndPackData<int16_t>();
    case schemas::AttributeType::SHORT2:
      return CopyAndPackData<short2>();
    case schemas::AttributeType::SHORT3:
      return CopyAndPackData<short3>();
    case schemas::AttributeType::SHORT4:
      return CopyAndPackData<short4>();
    case schemas::AttributeType::USHORT:
      return CopyAndPackData<uint16_t>();
    case schemas::AttributeType::USHORT2:
      return CopyAndPackData<ushort2>();
    case schemas::AttributeType::USHORT3:
      return CopyAndPackData<ushort3>();
    case schemas::AttributeType::USHORT4:
      return CopyAndPackData<ushort4>();
    case schemas::AttributeType::INT:
      return CopyAndPackData<int32_t>();
    case schemas::AttributeType::UINT:
      return CopyAndPackData<uint32_t>();
    case schemas::AttributeType::FLOAT:
      return CopyAndPackData<float>();
    case schemas::AttributeType::FLOAT2:
      return CopyAndPackData<float2>();
    case schemas::AttributeType::FLOAT3:
      return CopyAndPackData<float3>();
    case schemas::AttributeType::FLOAT4:
      return CopyAndPackData<float4>();
    default:
      absl::Status error = absl::FailedPreconditionError(
          absl::StrCat("Unsupported attribute type: %s",
                       EnumNameAttributeType(attribute_type)));
      
      return {};
  }
}

DenseDataAccess AccessorReader::GetData() const {
  if (base_ && !sparse_accessor_) {
    return DenseDataAccess(BufferAccess::Wrap(base_, stride_ * count_), count_,
                           stride_);
  }
  // If a copy need to be made anyway, a packed copy is made to save space.
  size_t packed_stride = GetAccessorTypeSize(type_, component_type_);
  return DenseDataAccess(CopyAndPackData(), count_, packed_stride);
}

DenseDataAccess AccessorReader::GetPackedData(
    AccessorReader::CopyOption copy_option) const {
  size_t packed_stride = GetAccessorTypeSize(type_, component_type_);

  // Example of data being not packed (a.k.a stride_ == packed_stride):
  // position and normal of a vertex are stored back-to-back,
  // "normal0|position0|normal1|position1|...."
  if (base_ && !sparse_accessor_ && stride_ == packed_stride &&
      copy_option != AccessorReader::CopyOption::kForceCopy) {
    return DenseDataAccess(BufferAccess::Wrap(base_, stride_ * count_), count_,
                           stride_);
  }
  return DenseDataAccess(CopyAndPackData(), count_, packed_stride);
}

absl::StatusOr<DenseDataAccess> AccessorReader::GetPackedFloatData(
    AccessorReader::CopyOption copy_option) const {
  if (component_type_ == gltf::FLOAT) {
    return GetPackedData(copy_option);
  } else {
    BufferAccess result;
    // Picks an unpack method to transcode with.
    switch (component_type_) {
      case gltf::BYTE:
        result = GetFloatBufferFromType(filament::math::unpackSnorm8);
        break;
      case gltf::UNSIGNED_BYTE:
        result = GetFloatBufferFromType(filament::math::unpackUnorm8);
        break;
      case gltf::SHORT:
        result = GetFloatBufferFromType(filament::math::unpackSnorm16);
        break;
      case gltf::UNSIGNED_SHORT:
        result = GetFloatBufferFromType(filament::math::unpackUnorm16);
        break;
      default:
        return absl::FailedPreconditionError(
            absl::StrCat("Unsupported component type: %d", component_type_));
    }
    auto attribute_type = GetAttributeType();
    return DenseDataAccess(
        std::move(result), count_,
        GetAccessorTypeSize(type_, imp::gltf::ComponentType::FLOAT));
  }
}

absl::StatusOr<schemas::AttributeType> AccessorReader::GetAttributeType()
    const {
  schemas::AttributeType attribute_type;
  MP_RETURN_IF_ERROR(
      provider_gltf::GetAttributeType(type_, component_type_, &attribute_type));
  if (GetAttributeTypeSize(attribute_type) > stride_) {
    return absl::FailedPreconditionError("Invalid Stride");
  }
  return attribute_type;
}

}  // namespace imp::loader::details
