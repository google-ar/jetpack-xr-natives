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

#include "core/atlas/atlas_packer.h"

#include <utility>

#include "core/math/vec.h"

namespace imp {

uint2 AtlasPacker::ScopedAtlasEntry::GetTopLeft() const {
  return entry_.top_left;
}

uint2 AtlasPacker::ScopedAtlasEntry::GetBottomRight() const {
  return entry_.bottom_right;
}

AtlasPacker::ScopedAtlasEntry::Id AtlasPacker::ScopedAtlasEntry::GetId() const {
  return entry_.id;
}

AtlasPacker::ScopedAtlasEntry AtlasPacker::ScopedAtlasEntry::Empty() {
  return ScopedAtlasEntry({0, 0}, {0, 0}, 0, nullptr);
}

AtlasPacker::ScopedAtlasEntry::ScopedAtlasEntry(uint2 top_left,
                                                uint2 bottom_right, Id id,
                                                AtlasPacker* packer)
    : entry_(AtlasEntry{.top_left = top_left,
                        .bottom_right = bottom_right,
                        .id = id,
                        .packer = packer}) {}

AtlasPacker::ScopedAtlasEntry::ScopedAtlasEntry(
    AtlasPacker::ScopedAtlasEntry&& rhs) noexcept
    : entry_(std::move(rhs.entry_)) {
  rhs.entry_.packer = nullptr;
}

AtlasPacker::ScopedAtlasEntry& AtlasPacker::ScopedAtlasEntry::operator=(
    AtlasPacker::ScopedAtlasEntry&& rhs) noexcept {
  if (this != &rhs) {
    if (entry_.packer) {
      entry_.packer->RemoveEntry(entry_.id);
    }

    entry_ = std::move(rhs.entry_);
    rhs.entry_.packer = nullptr;
  }
  return *this;
}

AtlasPacker::ScopedAtlasEntry::~ScopedAtlasEntry() {
  if (entry_.packer) {
    entry_.packer->RemoveEntry(entry_.id);
  }
}

AtlasPacker::AtlasPacker(uint2 texture_size) : texture_size_(texture_size) {}

AtlasPacker::~AtlasPacker() {}

uint2 AtlasPacker::GetTextureSize() const { return texture_size_; }

AtlasPacker::ScopedAtlasEntry AtlasPacker::CreateEntry(uint2 top_left,
                                                       uint2 bottom_right) {
  ScopedAtlasEntry result(top_left, bottom_right, next_entry_id_, this);
  next_entry_id_++;
  return result;
}

}  // namespace imp
