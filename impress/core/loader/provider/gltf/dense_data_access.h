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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_DENSE_DATA_ACCESS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_DENSE_DATA_ACCESS_H_

#include <cstddef>
#include <utility>

#include "absl/log/check.h"
#include "core/common/buffer_access.h"

namespace imp::loader::details {

// This is a wrapper class for BufferAccess, which provides point access to
// buffer data. The class can be used to hold a dense copy for sparse data.
class DenseDataAccess {
 public:
  // Constructs a DenseDataAccess by wrapping a BufferAccess.
  DenseDataAccess(BufferAccess flat_data, size_t count, size_t stride)
      : flat_data_(std::move(flat_data)), count_(count), stride_(stride) {}

  DenseDataAccess() : count_(0), stride_(0) {}

  // Returns const pointer of the data at the given id. This function is used
  // for reading the data.
  template <typename T>
  const T* At(size_t id) const {
    
    return reinterpret_cast<const T*>(flat_data_.Data() + (id * stride_));
  }

  // Return a const raw pointer access to data. This function is used for
  // reading the data.
  template <typename T>
  const T* ReadRawData() const {
    return reinterpret_cast<const T*>(flat_data_.Data());
  }

  // Returns a pointer to the data at the given id. This function is used for
  // modifying the data.
  template <typename T>
  T* At(size_t id) {
    
    flat_data_.GainOwnershipByCopying();
    return const_cast<T*>(
        reinterpret_cast<const T*>(flat_data_.Data() + (id * stride_)));
  }

  // Converts this DenseDataAccess to BufferAccess. This DenseDataAccess is
  // empty after this function returns.
  BufferAccess ReleaseBufferAccess() {
    count_ = 0;
    stride_ = 0;
    return std::move(flat_data_);
  }

  size_t GetCount() const { return count_; }

  // Many places need to use raw data, use stride to verify the raw data is
  // packed, so that it is safe to use raw data directly.
  size_t GetStride() const { return stride_; }

 private:
  BufferAccess flat_data_;
  size_t count_;
  size_t stride_;
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_DENSE_DATA_ACCESS_H_
