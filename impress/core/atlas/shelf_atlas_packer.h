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

#ifndef THIRD_PARTY_IMPRESS_CORE_ATLAS_SHELF_ATLAS_PACKER_H_
#define THIRD_PARTY_IMPRESS_CORE_ATLAS_SHELF_ATLAS_PACKER_H_

#include <cstdint>
#include <deque>
#include <list>
#include <vector>

#include "core/atlas/atlas_packer.h"
#include "core/common/robin_map.h"
#include "core/math/vec.h"
namespace imp {

// A utility that dynamically creates texture atlas layouts.
//
// This utility doesn't actually manage a texture or providing any functionality
// for writing to a texture. It is strictly used for layout and intended to be
// used in conjunction with other tools like CanvasSource to actually draw to a
// texture.
//
// This uses a modified version of the Shelf heuristic for picking regions of
// the texture. In general, the Shelf heuristic is a fast implementation that
// produces inefficient packings for non-uniform regions. This makes it well
// suited for things like font glyphs.
//
// This implementation is modified in the following ways:
//   - It does additional tracking of the shelves and slots so that when entries
//     are removed old regions can be re-used effectively.
//  -  It searches for the best empty slot based on the entry's height instead
//  of always just using the next slot on the last shelf. This helps with region
//  reuse and improves handling of non-uniform regions for a slight performance
//  cost.
class ShelfAtlasPacker : public AtlasPacker {
 public:
  explicit ShelfAtlasPacker(uint2 texture_size);

  absl::optional<ScopedAtlasEntry> AddEntry(uint2 size) override;

  void RemoveEntry(ScopedAtlasEntry::Id id) override;

  float GetUtilization() const;

 private:
  // Tracks a slot in a shelf.
  struct ShelfSlot {
    bool filled = false;
    uint2 size = {0, 0};
    int x = 0;
  };

  // Uses a linked list to track slots so that it is easy to split a slot into
  // two with the remaining space when a slot is filled, and merge adjacent
  // slots back together when one is released.
  using ShelfSlotList = std::list<ShelfSlot>;
  using ShelfSlotListItr = ShelfSlotList::iterator;

  // Tracks a shelf.
  struct Shelf {
    // Top of the shelf in the atlas.
    int y = 0;
    // Height of the shelf in the atlas.
    // When an empty shelf is first created, it contains the remaining height in
    // the atlas. Once an entry is added, the height is updated to match the
    // entry and the shelf is split into two with the remaining height.
    int height = 0;
    // List of slots in the shelf.
    // When an empty shelf is first created it contains a single empty slot with
    // the entire width of the atlas. When an entry is added to a slot, the slot
    // is split into two with the remaining width.
    ShelfSlotList slots;
    // Tracks the slots that are currently free in the shelf.
    std::vector<ShelfSlotListItr> free_slots;
  };

  // Uses a linked list to track shelves so that it is easy to split a shelf
  // into two with the remaining space when an entry is added, and merge
  // adjacent shelves back together when a shelf becomes empty.
  using ShelfList = std::list<Shelf>;
  using ShelfListItr = ShelfList::iterator;

  // Tracks a slot location.
  // Used to release slots when entries are removed.
  //
  // The itrs are guaranteed not to be invalidated because they are stored in a
  // linked list.
  struct SlotLocation {
    ShelfListItr shelf_itr;
    ShelfSlotListItr slot_itr;
  };

  void AddShelf(ShelfListItr pos, int y, int height);

  ShelfList shelves_;
  RobinMap<ScopedAtlasEntry::Id, SlotLocation> ids_to_slots_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ATLAS_SHELF_ATLAS_PACKER_H_
