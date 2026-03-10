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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_POINTER_EVENT_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_POINTER_EVENT_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "core/math/vec.h"
#include "filament/libs/math/include/math/vec2.h"

// TODO: Use protos instead of these types. Remove 'Message' suffix

namespace imp {

// Represents an individual pointer. A pointer is any point of contact on the
// screen made by a mouse cursor or touch.
struct Pointer {
  using Id = uint32_t;
  Id id;         // unique identifier of a pointer in an event sequence
  float2 point;  // x/y coordinates of current location in pixels
  float2 delta;  // change in x/y location from same pointer in last event
};

// LINT.IfChange
// Event type of the PointerEvent.
enum class PointerEventType : uint8_t {
  // One or more pointers are interrupted.
  kCancel = 0,
  // One or more pointers go down i.e., mouse click down, touch down.
  kDown = 1,
  // One or more pointers go up i.e., mouse click up, touch up.
  kUp = 2,
  // One or more pointers change coordinates.  On desktop, kMove is only sent
  // while the left mouse button is pressed.
  kMove = 3,
  // A pointer changed coordinates while hovering (mouse-only).
  kHover = 4,
  // Mouse wheel moves. This is associated with the float2 delta in the Pointer
  // struct.
  kWheel = 5,
  // A pointer entered the view area.
  kEnterView = 6,
  // A pointer left the view area.
  kLeaveView = 7,
};
// LINT.ThenChange(
//   //depot/google3/third_party/impress/java/com/google/ar/imp/view/input/\
//       InputManager.java
//   //depot/google3/third_party/impress/java/com/google/ar/imp/core/scripting/viewtexture/\
//       RenderViewToSurfaceTexture.java
//   //depot/google3/third_party/impress/javascript/core/wasm/constants.js:pointer_event_type
// )

// Represents a pointer event user interaction. Each event will contain one or
// more pointers that are associated with the PointerEventType that occurred.
class PointerEvent {
 public:
  PointerEvent();
  PointerEvent(const PointerEventType& type,
               const std::vector<Pointer>& pointers, int changed_pointer_count,
               absl::Duration elapsed_time);
  ~PointerEvent();

  // The event type that triggered this event.
  PointerEventType Type() const { return type_; }
  // Duration of time elapsed between system startup and time of event.
  absl::Duration ElapsedTime() const { return elapsed_time_; }
  // Count of pointers that changed during this event.
  // This will always be 1, except for kMove events.
  int ChangedPointerCount() const { return changed_pointer_count_; }
  // Count of all pointers currently in contact with the screen.  Changed
  // pointers will appear first.
  size_t PointerCount() const { return pointers_.size(); }
  // Returns pointer that changed during this event indexed at i. Pointer
  // indices are not guaranteed to be the same each time on touch devices; use
  // Pointer::Id to check for unique values.
  const Pointer& GetChangedPointer(int index = 0) const;
  // Returns index for given id or -1 if not found in the list of pointers that
  // changed during this event.
  int GetIndexForChangedPointerId(Pointer::Id id) const;

  // Returns pointer indexed at i. Pointer
  // indices are not guaranteed to be the same each time on touch devices; use
  // Pointer::Id to check for unique values.
  // Includes all pointers currently in contact with the screen.
  const Pointer& GetPointer(int index = 0) const;
  // Returns index for given id or -1 if not found in the list of pointers.
  // Includes all pointers currently in contact with the screen.
  int GetIndexForPointerId(Pointer::Id id) const;

  // Returns a span referencing changed pointers in the event.
  absl::Span<const Pointer> GetChangedPointers() const;

  // Returns a span referencing all pointers in the event.
  absl::Span<const Pointer> GetPointers() const;

  absl::Status Update(const std::vector<Pointer::Id>& changed_ids,
                      const std::vector<float2>& changed_points,
                      const std::vector<float2>& changed_deltas);

 private:
  std::vector<Pointer> pointers_;
  absl::Duration elapsed_time_;
  PointerEventType type_;
  uint8_t changed_pointer_count_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_POINTER_EVENT_H_
