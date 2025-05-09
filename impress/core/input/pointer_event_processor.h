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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_POINTER_EVENT_PROCESSOR_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_POINTER_EVENT_PROCESSOR_H_

#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "core/input/pointer_event.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

using float2 = filament::math::float2;

// Processes pointer events. Holds necessary information from previous pointer
// events and creates new PointerEvent objects.
class PointerEventProcessor {
 public:
  PointerEventProcessor();
  ~PointerEventProcessor();

  PointerEvent CreatePointerEvent(uint8_t action,
                                  const std::vector<Pointer::Id>& changed_ids,
                                  const std::vector<float2>& changed_points,
                                  absl::Duration elapsed_time);
  absl::Status UpdatePointerEvent(PointerEvent& event,
                                  const std::vector<Pointer::Id>& changed_ids,
                                  const std::vector<float2>& changed_points);

 private:
  // Stores unique pointers from previous pointer events in order to calculate
  // location delta.
  tsl::robin_map<Pointer::Id, float2> last_pointers_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_POINTER_EVENT_PROCESSOR_H_
