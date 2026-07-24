// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_SPLIT_ENGINE_FLATBUFFER_SIZE_CALCULATOR_H_
#define THIRD_PARTY_SPLIT_ENGINE_FLATBUFFER_SIZE_CALCULATOR_H_

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/base.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"

namespace split_engine {

// A helper class that calculates the exact wire format size and alignment
// requirements of a serialized FlatBuffer without allocating or writing to any
// byte buffers. This is used to determine required memory allocations upfront.
class FlatbufferSizeCalculator {
 public:
  FlatbufferSizeCalculator() = default;
  ~FlatbufferSizeCalculator() = default;

  FlatbufferSizeCalculator(const FlatbufferSizeCalculator&) = delete;
  FlatbufferSizeCalculator& operator=(const FlatbufferSizeCalculator&) = delete;
  FlatbufferSizeCalculator(FlatbufferSizeCalculator&&) = delete;
  FlatbufferSizeCalculator& operator=(FlatbufferSizeCalculator&&) = delete;

  // Simulates deduplicating vtables when calculating the size.
  // Mirrors flatbuffers::FlatBufferBuilder::DedupVtables(bool)
  void DedupVtables(bool dedup) { dedup_vtables_ = dedup; }

  // Clears the calculator's internal state.
  void Clear();

  // Tracks the maximum alignment required by any element in the buffer.
  void TrackMinAlign(size_t elem_size);

  // Simulates padding the buffer to the specified alignment.
  void Pad(size_t alignment);

  // Aligns such that when `len` bytes are written, an object can be written
  // after it with `alignment` without padding.
  void PreAlign(size_t len, size_t alignment);

  // --- Strings ---
  // Simulates creating a string of the given length.
  flatbuffers::Offset<flatbuffers::String> CreateString(size_t len);

  flatbuffers::Offset<flatbuffers::String> CreateString(absl::string_view str);

  // --- Vectors ---
  // Simulates creating a vector of `len` elements, where each element has size
  // `elem_size` and alignment `alignment`.
  template <typename T>
  flatbuffers::Offset<flatbuffers::Vector<T>> CreateVector(size_t len,
                                                           size_t elem_size,
                                                           size_t alignment);

  // Simulates creating a vector of `len` elements, automatically inferring the
  // element size and alignment from type `T`.
  template <typename T>
  flatbuffers::Offset<flatbuffers::Vector<T>> CreateVector(size_t len);

  // Simulates creating a vector of FlatBuffer offsets (e.g., vector of tables
  // or strings).
  template <typename T>
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<T>>>
  CreateOffsetVector(size_t len);

  // Simulates creating a vector of structs of the given length.
  template <typename T>
  flatbuffers::Offset<flatbuffers::Vector<const T*>> CreateVectorOfStructs(
      size_t len);

  // --- Structs ---
  // Simulates creating a standalone struct in the buffer (e.g., for union
  // members).
  template <typename T>
  flatbuffers::Offset<T> CreateStruct(const T& structobj);

  // --- Tables ---
  // Prepares the calculator for simulating table construction.
  void StartTable();

  // Simulates adding a scalar element to a table.
  template <typename T>
  void AddElement(size_t voffset, T val, T default_val);

  // Simulates adding an offset (string, vector, sub-table) to a table.
  template <typename T>
  void AddOffset(size_t voffset, flatbuffers::Offset<T> off);

  // Simulates adding an inline struct to a table.
  template <typename T>
  void AddStruct(size_t voffset, const T* structptr);

  // Completes table simulation and simulates writing the vtable and table
  // offset.
  template <typename T>
  flatbuffers::Offset<T> EndTable();

  // Simulates finishing the FlatBuffer by writing the root offset and
  // pre-aligning.
  template <typename T>
  void Finish(flatbuffers::Offset<T> root);

  // Returns the exact wire format size of the simulated FlatBuffer.
  size_t ComputeSize() const;

  // Optional helper to account for FlatBufferBuilder internal scratch working
  // memory when sizing initial buffer allocations to prevent dynamic
  // reallocations.
  void AddScratchSpace(size_t scratch_size = 1024);

 private:
  bool dedup_vtables_ = true;
  size_t offset_ = 0;
  size_t min_align_ = 1;
  size_t current_table_start_ = 0;
  size_t current_table_max_voffset_ = 0;
  std::vector<flatbuffers::voffset_t> current_vtable_;
  absl::flat_hash_set<std::vector<flatbuffers::voffset_t>> completed_vtables_;
};

template <typename T>
inline flatbuffers::Offset<flatbuffers::Vector<T>>
FlatbufferSizeCalculator::CreateVector(size_t len, size_t elem_size,
                                       size_t alignment) {
  TrackMinAlign(alignment);
  TrackMinAlign(4);  // vector length field is uint32_t
  PreAlign(len * elem_size, 4);
  PreAlign(len * elem_size, alignment);
  offset_ += elem_size * len;
  Pad(4);
  offset_ += 4;  // length field
  return flatbuffers::Offset<flatbuffers::Vector<T>>(offset_);
}

template <typename T>
inline flatbuffers::Offset<flatbuffers::Vector<T>>
FlatbufferSizeCalculator::CreateVector(size_t len) {
  return CreateVector<T>(len, sizeof(T), alignof(T));
}

template <typename T>
inline flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<T>>>
FlatbufferSizeCalculator::CreateOffsetVector(size_t len) {
  return CreateVector<flatbuffers::Offset<T>>(len, 4, 4);
}

template <typename T>
inline flatbuffers::Offset<flatbuffers::Vector<const T*>>
FlatbufferSizeCalculator::CreateVectorOfStructs(size_t len) {
  return CreateVector<const T*>(len, sizeof(T), alignof(T));
}

template <typename T>
inline flatbuffers::Offset<T> FlatbufferSizeCalculator::CreateStruct(
    const T& structobj) {
  TrackMinAlign(alignof(T));
  Pad(alignof(T));
  offset_ += sizeof(T);
  return flatbuffers::Offset<T>(offset_);
}

template <typename T>
inline void FlatbufferSizeCalculator::AddElement(size_t voffset, T val,
                                                 T default_val) {
  if (val != default_val) {
    TrackMinAlign(alignof(T));
    Pad(alignof(T));
    size_t field_off = offset_ - current_table_start_;
    offset_ += sizeof(T);
    if (voffset > current_table_max_voffset_) {
      current_table_max_voffset_ = voffset;
    }
    size_t vtable_index = voffset / 2;
    if (vtable_index >= current_vtable_.size()) {
      current_vtable_.resize(vtable_index + 1, 0);
    }
    current_vtable_[vtable_index] =
        static_cast<uint16_t>(field_off | (sizeof(T) << 12));
  }
}

template <typename T>
inline void FlatbufferSizeCalculator::AddOffset(size_t voffset,
                                                flatbuffers::Offset<T> off) {
  if (!off.IsNull()) {
    TrackMinAlign(4);
    Pad(4);
    size_t field_off = offset_ - current_table_start_;
    offset_ += 4;
    if (voffset > current_table_max_voffset_) {
      current_table_max_voffset_ = voffset;
    }
    size_t vtable_index = voffset / 2;
    if (vtable_index >= current_vtable_.size()) {
      current_vtable_.resize(vtable_index + 1, 0);
    }
    current_vtable_[vtable_index] =
        static_cast<uint16_t>(field_off | (4 << 12));
  }
}

template <typename T>
inline void FlatbufferSizeCalculator::AddStruct(size_t voffset,
                                                const T* structptr) {
  if (structptr != nullptr) {
    TrackMinAlign(alignof(T));
    Pad(alignof(T));
    size_t field_off = offset_ - current_table_start_;
    offset_ += sizeof(T);
    if (voffset > current_table_max_voffset_) {
      current_table_max_voffset_ = voffset;
    }
    size_t vtable_index = voffset / 2;
    if (vtable_index >= current_vtable_.size()) {
      current_vtable_.resize(vtable_index + 1, 0);
    }
    current_vtable_[vtable_index] =
        static_cast<uint16_t>(field_off | (sizeof(T) << 12));
  }
}

template <typename T>
inline flatbuffers::Offset<T> FlatbufferSizeCalculator::EndTable() {
  Pad(4);
  offset_ += 4;  // table offset pointing to vtable
  size_t vtable_size =
      (current_table_max_voffset_ > 0) ? (current_table_max_voffset_ + 2) : 4;
  size_t object_size = offset_ - current_table_start_;

  current_vtable_[0] = static_cast<uint16_t>(vtable_size);
  current_vtable_[1] = static_cast<uint16_t>(object_size);

  bool deduplicated = false;
  if (dedup_vtables_) {
    if (completed_vtables_.contains(current_vtable_)) {
      deduplicated = true;
    }
  }

  if (!deduplicated) {
    offset_ += vtable_size;
    if (dedup_vtables_) {
      completed_vtables_.insert(std::move(current_vtable_));
    }
  }

  return flatbuffers::Offset<T>(offset_);
}

template <typename T>
inline void FlatbufferSizeCalculator::Finish(flatbuffers::Offset<T> root) {
  TrackMinAlign(4);
  Pad(4);
  offset_ += 4;     // root offset
  Pad(min_align_);  // final buffer pre-alignment
}

}  // namespace split_engine

#endif  // THIRD_PARTY_SPLIT_ENGINE_FLATBUFFER_SIZE_CALCULATOR_H_
