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

#include "split_engine/flatbuffer_size_calculator.h"

#include <cstddef>

#include "absl/strings/string_view.h"
#include "flatbuffers/base.h"
#include "flatbuffers/string.h"

namespace split_engine {

void FlatbufferSizeCalculator::Clear() {
  offset_ = 0;
  min_align_ = 1;
  current_table_start_ = 0;
  current_table_max_voffset_ = 0;
  current_vtable_.clear();
  completed_vtables_.clear();
}

void FlatbufferSizeCalculator::TrackMinAlign(size_t elem_size) {
  if (elem_size > min_align_) {
    min_align_ = elem_size;
  }
}

void FlatbufferSizeCalculator::Pad(size_t alignment) {
  if (alignment <= 1) return;
  offset_ = alignment * ((offset_ + alignment - 1) / alignment);
}

void FlatbufferSizeCalculator::PreAlign(size_t len, size_t alignment) {
  if (len == 0) return;
  TrackMinAlign(alignment);
  size_t padding = ((~(offset_ + len)) + 1) & (alignment - 1);
  offset_ += padding;
}

flatbuffers::Offset<flatbuffers::String> FlatbufferSizeCalculator::CreateString(
    size_t len) {
  TrackMinAlign(4);
  // String content + null terminator
  offset_ += len + 1;
  Pad(4);
  offset_ += 4;  // length field (uint32_t)
  return flatbuffers::Offset<flatbuffers::String>(offset_);
}

flatbuffers::Offset<flatbuffers::String> FlatbufferSizeCalculator::CreateString(
    absl::string_view str) {
  return CreateString(str.size());
}

void FlatbufferSizeCalculator::StartTable() {
  current_table_start_ = offset_;
  current_table_max_voffset_ = 0;
  current_vtable_.clear();
  current_vtable_.resize(2, 0);
}

size_t FlatbufferSizeCalculator::ComputeSize() const { return offset_; }

void FlatbufferSizeCalculator::AddScratchSpace(size_t scratch_size) {
  offset_ += scratch_size;
}

}  // namespace split_engine
