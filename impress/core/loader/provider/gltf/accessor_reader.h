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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_ACCESSOR_READER_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_ACCESSOR_READER_H_

#include <stdbool.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/common/buffer_access.h"
#include "core/common/schemas/render_generated.h"
#include "core/common/typed_id.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details {

using AccessorId = TypedId<const imp::gltf::imp_proto::Accessor, int>;

// Reads data from a gltf accessor. A buffer will be created if the
// corresponding accessor uses undefined bufferview (a.k.a default value 0s), or
// has sparse values.
class AccessorReader {
 public:
  // The way to use retrieved accessor data.
  enum class RetrievalMode {
    // Adds accessor data to given array.
    kAdd,
    // Replaces given array with accessor data.
    kReplace
  };

  // Decide whether a copy should be made for the return data.
  enum class CopyOption {
    // If the original data in gltf fits the needs (e.g. non-sparse, packed),
    // refer to the original data and don't make a copy.
    kDefault,
    // Make a copy anyway.
    kForceCopy
  };

  // Creates a new AccessorReader, according to gltf information.
  static absl::StatusOr<AccessorReader> Create(
      const imp::gltf::imp_proto::Gltf& gltf, int accessor_index);

  static absl::StatusOr<AccessorReader> Create(
      const imp::gltf::imp_proto::Gltf& gltf, AccessorId accessor_id);

  absl::string_view GetType() const { return type_; }
  gltf::imp_proto::ComponentType GetComponentType() const {
    return component_type_;
  }
  size_t GetStride() const { return stride_; }
  size_t GetCount() const { return count_; }

  // Returns data defined by the accessor. Avoids copying when possible.
  DenseDataAccess GetData() const;

  // Returns packed data (data points are stored next to each other) defined by
  // the accessor. Avoids copying when possible.
  DenseDataAccess GetPackedData(
      CopyOption copy_option = CopyOption::kDefault) const;

  // In some case, weights/tangents/textcoords data is stored as integer data
  // (byte or short). The function converts those data into normalized float
  // values between 0 and 1.
  // https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#meshes-overview
  // shows that textcoord/color/weight can be defined with float/ubyte/ushort.
  // https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_mesh_quantization/README.md#extending-mesh-attributes
  // shows that positions/normals/tangents can be defined with byte/short in
  // case of KHR_mesh_quantization extension.
  //
  // Otherwise, if the data is of float format, simply
  // pack it and return.
  // TODO: Add unit test for GetPackedFloatData.
  absl::StatusOr<DenseDataAccess> GetPackedFloatData(
      CopyOption copy_option = CopyOption::kDefault) const;

  // Returns the attribute type of the accessor, if the accessor holds data for
  // an attribute.
  absl::StatusOr<schemas::AttributeType> GetAttributeType() const;

  // Fills data to existing array.
  template <typename T>
  absl::Status FillValues(DenseDataAccess& out_data,
                          RetrievalMode data_mode) const;

 private:
  // Holds information of sparse values.
  struct SparseAccessorReader {
    // The number of sparse values.
    size_t count = 0;
    gltf::imp_proto::ComponentType indices_type =
        gltf::imp_proto::ComponentType::INVALID;
    // The indices of sparse values.
    const uint8_t* indices_data = nullptr;
    // The sparse values.
    const uint8_t* sparse_values_data = nullptr;
  };

  AccessorReader() = default;

  static size_t GetAccessorTypeSize(
      absl::string_view type,
      imp::gltf::imp_proto::ComponentType component_type);
  static size_t GetAccessorTypeSize(const gltf::imp_proto::Accessor& accessor);

  // Adds only the sparse data to existing buffer (or overwrites the existing
  // buffer with only the sparse data). Ignores the base data.
  template <typename T>
  absl::Status ApplySparseValues(DenseDataAccess& out_data,
                                 RetrievalMode data_mode) const;

  // Denses and packs data of all types.
  BufferAccess CopyAndPackData() const;

  // Returns a copy of denseted and packed data.
  template <typename T>
  BufferAccess CopyAndPackData() const;

  // For data of integer types, normalize and pack it.
  template <typename ComponentType>
  BufferAccess GetFloatBufferFromType(float (*unpack)(ComponentType)) const;

  gltf::imp_proto::ComponentType component_type_;
  // The type is coming from gltf proto, which outlive this class, so that it is
  // safe to use string_view here.
  absl::string_view type_;
  // nullptr means default base as all zeros.
  const uint8_t* base_;
  size_t stride_;
  size_t count_;

  std::optional<SparseAccessorReader> sparse_accessor_;
};

template <typename T>
absl::Status AccessorReader::FillValues(DenseDataAccess& out_data,
                                        RetrievalMode data_mode) const {
  if (out_data.GetCount() != count_) {
    return absl::OutOfRangeError(
        "Reader data count doesn't match the destination data.");
  }
  if (data_mode == RetrievalMode::kAdd) {
    if (!base_) {
      MP_RETURN_IF_ERROR(ApplySparseValues<T>(out_data, RetrievalMode::kAdd));
    } else {
      // This line finally calls kReplace branch and checks for sparse values
      // there.
      DenseDataAccess dense_data = GetData();
      for (size_t i = 0; i < count_; ++i) {
        *out_data.At<T>(i) += *dense_data.At<T>(i);
      }
    }
  } else if (data_mode == RetrievalMode::kReplace) {
    if (!base_) {
      for (size_t i = 0; i < count_; ++i) {
        *out_data.At<T>(i) = T(0);
      }
    } else {
      // Check for alignment before the loop.
      const bool is_aligned =
          (reinterpret_cast<uintptr_t>(base_) % alignof(T) == 0) &&
          (stride_ % alignof(T) == 0);
      for (size_t i = 0; i < count_; ++i) {
        if (is_aligned) {
          *out_data.At<T>(i) =
              *reinterpret_cast<const T*>(base_ + (i * stride_));
        } else {
          // Use std::memcpy instead of reinterpret_cast to avoid memory
          // alignment issues, when buffer is loaded from 32-bit device.
          const uint8_t* src_ptr = base_ + (i * stride_);
          T temp_value;
          std::memcpy(&temp_value, src_ptr, sizeof(T));
          *out_data.At<T>(i) = temp_value;
        }
      }
    }
    MP_RETURN_IF_ERROR(ApplySparseValues<T>(out_data, RetrievalMode::kReplace));
  }
  return absl::OkStatus();
}

template <typename T>
absl::Status AccessorReader::ApplySparseValues(DenseDataAccess& out_data,
                                               RetrievalMode data_mode) const {
  if (!sparse_accessor_.has_value()) {
    return absl::OkStatus();
  }

  size_t indices_stride =
      GetAccessorTypeSize("SCALAR", sparse_accessor_->indices_type);
  size_t stride = GetAccessorTypeSize(type_, component_type_);
  for (size_t i = 0; i < sparse_accessor_->count; ++i) {
    size_t index = -1;
    switch (sparse_accessor_->indices_type) {
      case gltf::imp_proto::ComponentType::BYTE:
        index = *reinterpret_cast<const int8_t*>(
            sparse_accessor_->indices_data + (i * indices_stride));
        break;
      case gltf::imp_proto::ComponentType::UNSIGNED_BYTE:
        index = *reinterpret_cast<const uint8_t*>(
            sparse_accessor_->indices_data + (i * indices_stride));
        break;
      case gltf::imp_proto::ComponentType::SHORT:
        index = *reinterpret_cast<const int16_t*>(
            sparse_accessor_->indices_data + (i * indices_stride));
        break;
      case gltf::imp_proto::ComponentType::UNSIGNED_SHORT:
        index = *reinterpret_cast<const uint16_t*>(
            sparse_accessor_->indices_data + (i * indices_stride));
        break;
      case gltf::imp_proto::ComponentType::UNSIGNED_INT:
        index = *reinterpret_cast<const uint32_t*>(
            sparse_accessor_->indices_data + (i * indices_stride));
        break;
      default:
        return absl::FailedPreconditionError(
            absl::StrCat("Unsupported component type of sparse indices: %d",
                         sparse_accessor_->indices_type));
    }
    if (index > count_) {
      IMP_LOG(imp::WARNING) << "sparse index " << index << " out of range of " << count_;
      continue;
    }
    if (data_mode == AccessorReader::RetrievalMode::kAdd) {
      *out_data.At<T>(index) += *reinterpret_cast<const T*>(
          sparse_accessor_->sparse_values_data + (i * stride));
    } else if (data_mode == AccessorReader::RetrievalMode::kReplace) {
      *out_data.At<T>(index) = *reinterpret_cast<const T*>(
          sparse_accessor_->sparse_values_data + (i * stride));
    }
  }
  return absl::OkStatus();
}

template <typename T>
BufferAccess AccessorReader::CopyAndPackData() const {
  size_t size = sizeof(T);
  DenseDataAccess dense_data = DenseDataAccess(
      BufferAccess(std::make_unique<uint8_t[]>(count_ * size), count_ * size),
      count_, size);
  auto result = FillValues<T>(dense_data, RetrievalMode::kReplace);
  return dense_data.ReleaseBufferAccess();
}

template <typename ComponentType>
BufferAccess AccessorReader::GetFloatBufferFromType(
    float (*unpack)(ComponentType)) const {
  size_t float_type_size =
      GetAccessorTypeSize(type_, imp::gltf::imp_proto::ComponentType::FLOAT);
  size_t component_count = float_type_size / sizeof(float);

  DenseDataAccess dense_data = GetPackedData();

  BufferAccess float_buffer_access;
  float* float_buffer_ptr = reinterpret_cast<float*>(
      BufferAccess::Create(count_ * float_type_size, &float_buffer_access));

  for (size_t i = 0; i < count_; i++) {
    ComponentType* source_element = dense_data.At<ComponentType>(i);
    for (size_t j = 0; j < component_count; j++) {
      float_buffer_ptr[i * component_count + j] = (*unpack)(source_element[j]);
    }
  }
  return float_buffer_access;
}

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_ACCESSOR_READER_H_
