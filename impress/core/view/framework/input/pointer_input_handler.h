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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_INPUT_POINTER_INPUT_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_INPUT_POINTER_INPUT_HANDLER_H_

#include <cassert>
#include <cstddef>
#include <vector>

#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "core/input/input_manager.h"
#include "core/input/pointer_event.h"
#include "core/input/wheel_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

class BaseView;

struct PointerHitEvent : public Event {
  PointerHitEvent() = default;
  template <typename T>
  explicit PointerHitEvent(const PointerEvent& event,
                           std::vector<std::vector<GenericRayHit<T>>> hits)
      : event(event), hits(std::move(hits)) {
    assert(this->event.PointerCount() == this->GetPointerCount());
  }

  // Gets the number of pointers currently in contact with the screen.
  size_t GetPointerCount() const;

  // Gets the first NodeHandle from the intersection list at the provided
  // pointer index.
  NodeHandle GetHitNode(size_t index = 0) const;

  // Gets the entire intersection list at the provided pointer index.
  std::vector<NodeHandle> GetAllIntersectingNodes(size_t index = 0) const;

  // Gets the first RayHit from the intersection list at the provided pointer
  // index, if the intersection list is non-empty. If precise transforms are
  // enabled, the RayHit is truncated to fit in a float instead of a double.
  absl::optional<RayHit> GetTruncatedRayHit(size_t index = 0) const;

  // Gets the first RayHit or DoubleRayHit from the intersection list at the
  // provided pointer index, if the intersection list is non-empty.
  absl::variant<absl::monostate, RayHit, DoubleRayHit> GetRayHitOrDoubleRayHit(
      size_t index = 0) const;

  PointerEvent event =
      PointerEvent(PointerEventType::kCancel, std::vector<Pointer>{}, 0,
                   absl::ZeroDuration());

  // A vector of all GenericRayHits from the respective ray cast. Indexed first
  // by pointer index.
  //
  // Ex. hits[0] returns a vector of all GenericRayHits returned by the
  // ray cast by the pointer with index 0.
  absl::variant<std::vector<std::vector<RayHit>>,
                std::vector<std::vector<DoubleRayHit>>>
      hits = {};
};

struct WheelScrollEvent : public Event {
  WheelScrollEvent() = default;
  explicit WheelScrollEvent(const WheelEvent& wheel_event)
      : event(wheel_event) {}
  WheelEvent event =
      WheelEvent(/*delta=*/{}, /*point=*/{}, absl::ZeroDuration());
};

// PointerInputHandler calculates and sends PointerHitEvents, which combines
// PointerEvents that have occurred since the last Update() call with the
// current onscreen RayHit that the pointer intersects.
class PointerInputHandler : public InputHandlerBase {
 public:
  // Sends the PointerHitEvents to the given Dispatcher.
  explicit PointerInputHandler(BaseView* view, Dispatcher* dispatcher);
  // Derives the Dispatcher from the view.
  explicit PointerInputHandler(BaseView* view);
  ~PointerInputHandler() override;
  void Update(InputManager* input_manager) override;
  // Consumes a PointerEvent and dispatches PointerHitEvents.
  void DispatchHitEvents(const PointerEvent& event);

 protected:
  // Casts a ray from a Pointer's screen location out into the view and
  // returns all RayHits, sorted bu distance.
  virtual std::vector<RayHit> IntersectPointer(const Pointer& p);
  virtual std::vector<DoubleRayHit> IntersectPointerPrecise(const Pointer& p);

  BaseView* view_;

 private:
  void DispatchWheelEvent(const WheelEvent& wheel);
  Dispatcher& dispatcher_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_INPUT_POINTER_INPUT_HANDLER_H_
