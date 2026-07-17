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

#include "core/input/pointer_event_processor.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/time/time.h"
#include "core/config.h"
#include "core/input/pointer_event.h"
#include "core/math/vec.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

PointerEventProcessor::PointerEventProcessor() : last_pointers_() {}

PointerEventProcessor::~PointerEventProcessor() {}

PointerEvent PointerEventProcessor::CreatePointerEvent(
    uint8_t action, const std::vector<Pointer::Id>& changed_ids,
    const std::vector<float2>& changed_points, absl::Duration elapsed_time,
    PointerEvent::DeviceType device_type) {
  const PointerEventType type = static_cast<PointerEventType>(action);
  if (changed_ids.size() != changed_points.size()) {
    IMP_LOG(imp::FATAL) << "Size of changed_ids(" << changed_ids.size()
               << ") differs from changed_points(" << changed_points.size()
               << ")";
  }
  int changed_count = changed_ids.size();
  std::vector<Pointer> pointers;
  pointers.reserve(changed_count);
  for (int i = 0; i < changed_count; ++i) {
    auto currId = changed_ids.at(i);
    const auto& currPoint = changed_points.at(i);
    const auto& prev = last_pointers_.find(currId);

    float2 delta =
        prev == last_pointers_.end() ? float2{0, 0} : currPoint - prev->second;

    if (type == PointerEventType::kDown || type == PointerEventType::kMove) {
      last_pointers_[currId] = currPoint;
    } else {
      last_pointers_.erase(currId);
    }

    pointers.push_back(Pointer{currId, currPoint, delta});
  }

  int changed_pointer_count = pointers.size();

  // Add remaining pointers that didn't change.
  for (auto pair : last_pointers_) {
    // This is not very efficient, but this will be 4-5 elements max ever,
    // and only run once per-frame, so not a big deal.
    bool is_changed_id = false;
    for (int i = 0; i < changed_ids.size(); i++) {
      if (changed_ids[i] == pair.first) {
        is_changed_id = true;
        continue;
      }
    }

    if (!is_changed_id) {
      pointers.push_back(Pointer{pair.first, pair.second, {0.0f}});
    }
  }

#if IMP_PLATFORM(IOS)
  // For move events, expand the changed pointer count so that all pointers
  // count as changed. This is important to create consistent behavior between
  // iOS and Android, because android always includes all pointers in Move
  // events.
  if (type == PointerEventType::kMove) {
    changed_pointer_count = pointers.size();
  }
#endif

  return PointerEvent(type, pointers, changed_pointer_count, elapsed_time,
                      device_type);
}

absl::Status PointerEventProcessor::UpdatePointerEvent(
    PointerEvent& event, const std::vector<Pointer::Id>& changed_ids,
    const std::vector<float2>& changed_points) {
  assert(changed_ids.size() == changed_points.size());
  std::vector<float2> changed_deltas(changed_points.size());
  size_t changed_count = changed_ids.size();
  for (size_t i = 0; i < changed_count; ++i) {
    Pointer::Id id = changed_ids[i];
    float2 point = changed_points[i];
    auto iter = last_pointers_.find(id);
    if (iter == last_pointers_.end()) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Tried to update unknown pointer id ", id, " for PointerEventType ",
          event.Type(), ". Existing pointers and their locations are: ",
          absl::StrJoin(
              last_pointers_, ", ",
              absl::PairFormatter(absl::AlphaNumFormatter(), "=",
                                  [](std::string* out, const float2& vec) {
                                    absl::StrAppend(out, ToString(vec));
                                  }))));
    }
    changed_deltas[i] = point - iter->second;
    last_pointers_[id] = point;
  }
  MP_RETURN_IF_ERROR(event.Update(changed_ids, changed_points, changed_deltas));

  return absl::OkStatus();
}

}  // namespace imp
