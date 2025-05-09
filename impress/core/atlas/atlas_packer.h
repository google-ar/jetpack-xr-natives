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

#ifndef THIRD_PARTY_IMPRESS_CORE_ATLAS_ATLAS_PACKER_H_
#define THIRD_PARTY_IMPRESS_CORE_ATLAS_ATLAS_PACKER_H_

#include <cstdint>

#include "absl/types/optional.h"
#include "core/math/vec.h"
#include "core/view/utils/macros.h"

namespace imp {

// Base class for a utility that dynamically creates texture atlas layouts.
//
// This utility doesn't actually manage a texture or providing any functionality
// for writing to a texture. It is strictly used for layout and intended to be
// used in conjunction with other tools like CanvasSource to actually draw to a
// texture.
class AtlasPacker {
 public:
  // An entry in the atlas returned by AtlasPacker::AddEntry.
  //
  // Provides information about the location of the entry within the texture
  // atlas.
  //
  // When this object goes out of scope, the entry is automatically released
  // from the atlas allowing the region to be reused by another entry.
  class IMP_WARN_UNUSED_RESULT ScopedAtlasEntry {
   public:
    using Id = uint32_t;

    // Creates an atlas entry that is empty.
    static ScopedAtlasEntry Empty();

    ScopedAtlasEntry(ScopedAtlasEntry&& rhs) noexcept;
    ScopedAtlasEntry& operator=(ScopedAtlasEntry&& rhs) noexcept;
    ~ScopedAtlasEntry();

    // Returns the top left location of the entry in the atlas.
    uint2 GetTopLeft() const;

    // Returns the bottom right location of the entry in the atlas.
    uint2 GetBottomRight() const;

    // Returns the id for the entry.
    Id GetId() const;

   private:
    struct AtlasEntry {
      uint2 top_left;
      uint2 bottom_right;
      Id id;
      AtlasPacker* packer = nullptr;
    };

    ScopedAtlasEntry(uint2 top_left, uint2 bottom_right, Id id,
                     AtlasPacker* packer);

    // AtlasEntries are move only.
    ScopedAtlasEntry(const ScopedAtlasEntry&) = delete;
    ScopedAtlasEntry& operator=(const ScopedAtlasEntry&) = delete;

    AtlasEntry entry_;

    friend class AtlasPacker;
  };

  explicit AtlasPacker(uint2 texture_size);

  virtual ~AtlasPacker();

  virtual absl::optional<ScopedAtlasEntry> AddEntry(uint2 size) = 0;

  uint2 GetTextureSize() const;

  virtual float GetUtilization() const = 0;

 protected:
  // Helper method used by subclasses to create entries.
  ScopedAtlasEntry CreateEntry(uint2 top_left, uint2 bottom_right);

  virtual void RemoveEntry(ScopedAtlasEntry::Id id) = 0;

 private:
  uint2 texture_size_;
  ScopedAtlasEntry::Id next_entry_id_ = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ATLAS_ATLAS_PACKER_H_
