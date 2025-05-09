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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_GESTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_GESTURE_H_

#include <functional>
#include <utility>
#include <vector>

#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/utils/device.h"

namespace imp {

class View;
class NodeHandle;

/**
 * Base class for a gesture.
 *
 * A gesture represents a sequence of touch events that are detected to
 * represent a particular type of motion (i.e. Dragging, Pinching). Every
 * gesture begins in a kUnstarted state in order to start tracking if a given
 * pointer can be interpreted as the desired gesture. A gesture does not start
 * until TryStart() returns true.
 *
 * Gestures are created and updated by GestureRecognizers. A Gesture is
 * required to implement a static GetCreateFunction that returns a function
 * to create an instance of the gesture. This function will return a
 * absl::optional<MyGesture> if a gesture can be created from a given
 * PointerHitEvent, or absl::nullopt otherwise.
 *
 * Example:
 *
 * std::function<absl::optional<MyGesture>(const PointerHitEvent& pointer_hit)>
 * MyGesture::GetCreateFunction(View* view,
 *                              GesturePointerUtils* pointer_utils) {
 *     return [=](const PointerHitEvent& pointer_hit) {
 *       return pointer_hit.event.type == PointerEventType::kDown
 *                 ? absl::optional<MyGesture>(
 *                     MyGesture(view, pointer_utils, pointer_hit))
 *                 : absl::nullopt;
 *     };
 * }
 */

class Gesture {
 public:
  // Identifies a particular gesture to differentiate between multiple
  // gestures of the same type occurring at the same time.
  using Id = size_t;

  static constexpr Id kEmptyId = -1;

  Gesture(Dispatcher* dispatcher, GesturePointerUtils* pointer_utils,
          const PointerHitEvent& pointer_hit);
  virtual ~Gesture();
  void InitializeId(Id id);
  void OnPointerHitEvent(const PointerHitEvent& pointer_hit);
  // Returns true if the gesture was started.
  bool Started() const;
  // Returns true if a started gesture was finished.
  bool Finished() const;
  // Returns true if the gesture was cancelled. A potential gesture will be
  // cancelled if it doesn't meet the conditions to start the gesture. A started
  // gesture will be cancelled if the pointer is cancelled.
  bool Cancelled() const;

  // Base class for events sent by gestures.  Provides a mechanism for
  // cancelling the originating gesture.
  using CancelFn = std::function<void()>;
  class Event : public imp::Event {
   public:
    Event() : Event(Gesture::kEmptyId) {}
    explicit Event(Id id, CancelFn cancel = nullptr)
        : id_(id), cancel_(std::move(cancel)) {}

    Id GetId() const { return id_; }
    void CancelGesture() const {
      if (cancel_) {
        cancel_();
      }
    }

   private:
    Id id_;
    CancelFn cancel_;
  };

  static CancelFn MakeCancelFn(Gesture* gesture, const PointerHitEvent& hit);

 protected:
  // Returns true if the potential gesture can start.
  virtual bool TryStart(const PointerHitEvent& pointer_hit) = 0;
  virtual void OnUpdate(const PointerHitEvent& pointer_hit) = 0;
  virtual void OnFinish(const PointerHitEvent& pointer_hit) = 0;
  virtual void OnCancel() = 0;

  void Start(const PointerHitEvent& pointer_hit);
  void Finish(const PointerHitEvent& pointer_hit);
  void Cancel(const PointerHitEvent& pointer_hit);

  Dispatcher& GetDispatcher() const;
  GesturePointerUtils* GetPointerUtils() const;
  NodeHandle GetTargetNode() const;
  const std::vector<NodeHandle>& GetAllIntersectingNodes() const;
  Id GetId() const;

 private:
  // Potential gesture state journeys:
  // kUnstarted -> kUnstartedCancelled
  // kUnstarted -> kStarted -> kFinished
  // kUnstarted -> kStarted -> kCancelFinished
  enum class State : uint8_t {
    // Potential gesture was created from a pointer, but has not been started.
    kUnstarted = 0,
    // Potential gesture was cancelled (never started).
    kUnstartedCancelled = 1,
    // Gesture has started.
    kStarted = 2,
    // A started gesture has been cancelled and finished.
    kCancelFinished = 4,
    // A started gesture has been finished.
    kFinished = 5
  };

  Dispatcher* dispatcher_;
  GesturePointerUtils* pointer_utils_;
  NodeHandle target_node_;
  std::vector<NodeHandle> all_intersecting_nodes_;
  State state_;
  Id id_;
};

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_GESTURE_H_
