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

#include "core/input/pointer_event.h"

#include <utility>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "core/common/platform_helpers.h"

namespace imp {

PointerEvent::PointerEvent()
    : pointers_(std::vector<Pointer>{}),
      elapsed_time_(absl::ZeroDuration()),
      type_(PointerEventType::kCancel),
      changed_pointer_count_(0) {}

PointerEvent::PointerEvent(const PointerEventType& type,
                           const std::vector<Pointer>& pointers,
                           int changed_pointer_count,
                           absl::Duration elapsed_time)
    : pointers_(pointers),
      elapsed_time_(elapsed_time),
      type_(type),
      changed_pointer_count_(changed_pointer_count) {}

PointerEvent::~PointerEvent() {}

const Pointer& PointerEvent::GetChangedPointer(int index) const {
  assert(index < changed_pointer_count_);
  return pointers_[index];
}

int PointerEvent::GetIndexForChangedPointerId(Pointer::Id id) const {
  for (int i = 0; i < changed_pointer_count_; ++i) {
    if (pointers_[i].id == id) {
      return i;
    }
  }
  return -1;
}

absl::Span<const Pointer> PointerEvent::GetChangedPointers() const {
  return absl::Span<const Pointer>(pointers_.data(), changed_pointer_count_);
}

const Pointer& PointerEvent::GetPointer(int index) const {
  assert(index < pointers_.size());
  return pointers_[index];
}

int PointerEvent::GetIndexForPointerId(Pointer::Id id) const {
  for (int i = 0; i < pointers_.size(); ++i) {
    if (pointers_[i].id == id) {
      return i;
    }
  }
  return -1;
}

absl::Span<const Pointer> PointerEvent::GetPointers() const {
  return pointers_;
}

absl::Status PointerEvent::Update(const std::vector<Pointer::Id>& changed_ids,
                                  const std::vector<float2>& changed_points,
                                  const std::vector<float2>& changed_deltas) {
  size_t handled_count = 0;
  for (Pointer& pointer : pointers_) {
    auto id_iter = absl::c_find(changed_ids, pointer.id);
    if (id_iter == changed_ids.end()) continue;
    size_t changed_index = std::distance(&changed_ids.front(), &(*id_iter));
    size_t pointer_index = std::distance(&pointers_.front(), &pointer);
    ++handled_count;
    pointer.point = changed_points[changed_index];
    pointer.delta += changed_deltas[changed_index];
    if (pointer_index == changed_pointer_count_) {
      ++changed_pointer_count_;
    } else if (pointer_index > changed_pointer_count_) {
      // This isn't a concern until you get to three or more touches, at which
      // point the work of swapping pointers would be required.
      return absl::InternalError(
          "Event update created a gap in updated pointers");
    }
  }
  if (handled_count != changed_ids.size()) {
    return absl::InternalError("Update had untracked pointers");
  }

  return absl::OkStatus();
}

}  // namespace imp
