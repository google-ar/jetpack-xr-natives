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

#include "core/atlas/shelf_atlas_packer.h"

#include <algorithm>
#include <iterator>
#include <optional>

#include "core/atlas/atlas_packer.h"
#include "core/math/vec.h"

namespace imp {

ShelfAtlasPacker::ShelfAtlasPacker(uint2 texture_size)
    : AtlasPacker(texture_size) {
  // Start with a single empty shelf with one slot that covers the entire atlas.
  // As entries are added, the shelf and slots get split into additional slots
  // and shelves.
  AddShelf(shelves_.end(), 0, texture_size.y);
}

absl::optional<AtlasPacker::ScopedAtlasEntry> ShelfAtlasPacker::AddEntry(
    uint2 size) {
  std::optional<ShelfSlotListItr> best_slot;
  int best_slot_free_slot_index = -1;
  ShelfListItr best_shelf_itr;

  // Search through the currently free slots to find the best free slot that the
  // entry will fit in.
  for (ShelfListItr shelf_itr = shelves_.begin(); shelf_itr != shelves_.end();
       ++shelf_itr) {
    Shelf& shelf = *shelf_itr;
    for (int i = 0; i < shelf.free_slots.size(); ++i) {
      ShelfSlotListItr free_slot = shelf.free_slots.at(i);

      // Check that the slot is big enough.
      if (free_slot->size.x < size.x || free_slot->size.y < size.y) {
        continue;
      }

      // Check if it's better than the previous best.
      if (best_slot && (*best_slot)->size.y <= free_slot->size.y) {
        continue;
      }

      best_slot = free_slot;
      best_shelf_itr = shelf_itr;
      best_slot_free_slot_index = i;
    }
  }

  // No slot was found, that means that this atlas doesn't have room for this
  // entry.
  if (!best_slot) {
    return {};
  }

  // If this is the first filled slot in the shelf, split the shelf.
  Shelf& best_shelf = *best_shelf_itr;
  if (best_shelf.slots.size() == 1) {
    //  Create the new shelf.
    AddShelf(std::next(best_shelf_itr), best_shelf.y + size.y,
             best_shelf.height - size.y);

    // Update the best shelf.
    best_shelf.height = size.y;
  }

  // Fill the slot and split the remainder into a new slot.
  ShelfSlot& slot = **best_slot;
  if (slot.size.x > size.x) {
    ShelfSlotListItr new_slot_itr =
        best_shelf.slots.emplace(std::next(*best_slot));
    ShelfSlot& new_slot = *new_slot_itr;
    new_slot.size = uint2(slot.size.x - size.x, best_shelf.height);
    new_slot.x = slot.x + size.x;
    best_shelf.free_slots.push_back(new_slot_itr);
  }

  slot.filled = true;
  slot.size = size;
  best_shelf.free_slots.erase(best_shelf.free_slots.begin() +
                              best_slot_free_slot_index);

  ScopedAtlasEntry entry =
      CreateEntry(uint2{slot.x, best_shelf.y},
                  uint2{slot.x + size.x, best_shelf.y + size.y});

  // Track the entry so that it can be removed later.
  ids_to_slots_.emplace(entry.GetId(), SlotLocation{.shelf_itr = best_shelf_itr,
                                                    .slot_itr = *best_slot});

  return entry;
}

void ShelfAtlasPacker::RemoveEntry(ScopedAtlasEntry::Id id) {
  auto itr = ids_to_slots_.find(id);
  if (itr == ids_to_slots_.end()) {
    // This entry doesn't exist.
    return;
  }

  SlotLocation& slot_location = itr.value();
  ShelfSlotListItr current_slot_itr = slot_location.slot_itr;
  ShelfSlotListItr next_slot_itr = std::next(slot_location.slot_itr);
  ShelfSlotListItr previous_slot_itr = std::prev(slot_location.slot_itr);

  // Check if there is an adjacent slot to the right, if it's empty then merge
  // this slot into the right.
  assert(current_slot_itr->filled);
  if (next_slot_itr != slot_location.shelf_itr->slots.end() &&
      !next_slot_itr->filled) {
    next_slot_itr->x = current_slot_itr->x;
    next_slot_itr->size.x += current_slot_itr->size.x;
    slot_location.shelf_itr->slots.erase(current_slot_itr);
    current_slot_itr = next_slot_itr;
  }

  // Check if there is an adjacent slot to the left, if it's empty then merge
  // this slot into the left.
  if (previous_slot_itr != slot_location.shelf_itr->slots.end() &&
      !previous_slot_itr->filled) {
    previous_slot_itr->size.x += current_slot_itr->size.x;

    // If the current_slot_itr is already empty, that means that the slot was
    // already merged into the right. In that case, current_slot_itr needs to be
    // removed from the free slots as well.
    if (!current_slot_itr->filled) {
      auto free_slots_itr = std::find(
          slot_location.shelf_itr->free_slots.begin(),
          slot_location.shelf_itr->free_slots.end(), current_slot_itr);
      slot_location.shelf_itr->free_slots.erase(free_slots_itr);
    }
    slot_location.shelf_itr->slots.erase(current_slot_itr);
    current_slot_itr = previous_slot_itr;
  }

  // If the current slot is still filled, no merging occurred. Therefore, unfill
  // it.
  if (current_slot_itr->filled) {
    current_slot_itr->filled = false;
    current_slot_itr->size.y = slot_location.shelf_itr->height;
    slot_location.shelf_itr->free_slots.push_back(current_slot_itr);
  }

  // Next, we need to merge with adjacent shelves if the shelf is now empty.
  ShelfListItr current_shelf_itr = slot_location.shelf_itr;
  if (current_shelf_itr->slots.size() == 1) {
    // If there is an empty shelf below, merge it into this shelf.
    ShelfListItr next_shelf_itr = std::next(current_shelf_itr);
    ShelfSlotList& next_shelf_slots = next_shelf_itr->slots;
    if (next_shelf_itr != shelves_.end() && next_shelf_slots.size() == 1 &&
        !next_shelf_slots.back().filled) {
      current_shelf_itr->height += next_shelf_itr->height;
      current_shelf_itr->slots.back().size.y = current_shelf_itr->height;
      shelves_.erase(next_shelf_itr);
    }

    // If there is an empty shelf above, merge this shelf into it.
    ShelfListItr previous_shelf_itr = std::prev(current_shelf_itr);
    ShelfSlotList& previous_shelf_slots = previous_shelf_itr->slots;
    if (previous_shelf_itr != shelves_.end() &&
        previous_shelf_slots.size() == 1 &&
        !previous_shelf_slots.back().filled) {
      previous_shelf_itr->height += current_shelf_itr->height;
      previous_shelf_slots.back().size.y = previous_shelf_itr->height;
      shelves_.erase(current_shelf_itr);
    }
  }

  // Lastly, remove the id from the map.
  ids_to_slots_.erase(itr);
}

float ShelfAtlasPacker::GetUtilization() const {
  int2 atlas_size = GetTextureSize();
  float utilized_size = 0;
  for (auto shelf_itr = shelves_.begin(); shelf_itr != shelves_.end();
       ++shelf_itr) {
    const Shelf& shelf = *shelf_itr;
    auto slot_itr = shelf.slots.begin();
    for (int i = 0; i < shelf.slots.size() - 1; ++i) {
      utilized_size += slot_itr->size.x * shelf.height;
      ++slot_itr;
    }
  }

  return utilized_size / atlas_size.x / atlas_size.y;
}

void ShelfAtlasPacker::AddShelf(ShelfListItr pos, int y, int height) {
  Shelf& shelf = *shelves_.emplace(pos);
  shelf.y = y;
  shelf.height = height;
  ShelfSlot& slot = shelf.slots.emplace_back();
  slot.size = uint2{GetTextureSize().x, height};
  shelf.free_slots.push_back(--shelf.slots.end());
}

}  // namespace imp
