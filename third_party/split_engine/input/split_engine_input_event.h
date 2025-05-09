/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_SPLIT_ENGINE_INPUT_SPLIT_ENGINE_INPUT_EVENT_H_
#define THIRD_PARTY_SPLIT_ENGINE_INPUT_SPLIT_ENGINE_INPUT_EVENT_H_

#include <cstdint>
#include <memory>
#include <string>

#include "core/math/vec.h"
#include "core/ncsb/dispatcher/event.h"
#include "split_engine/input/split_engine_input_event_hit_info.h"

namespace android_xr {

// Represents an input event that's handled by native Split Engine. Should be
// kept in sync with its Java counterpart, the Android XR input event.
struct SplitEngineInputEvent : public imp::Event {
 public:
  enum class DispatchFlag : uint8_t {
    // LINT.IfChange(dispatch_flag)
    // Normal dispatch.
    NONE = 0,
    // This event was dispatched to this receiver only because pointer capture
    // was enabled.
    CAPTURED_POINTER = 1,
    // This event was also dispatched as a 2D Android input event.
    TWO_D = 2,
    // LINT.ThenChange(//depot/google3/third_party/split_engine/input/split_engine_input_event_jni.cc:dispatch_flag)
  };
  enum class DeviceType : uint8_t {
    // LINT.IfChange(device_type)
    UNKNOWN = 0,
    HEAD = 1,
    CONTROLLER = 2,
    HANDS = 3,
    MOUSE = 4,
    GAZE_AND_GESTURE = 5,
    DIRECT_TOUCH = 6,
    // LINT.ThenChange(//depot/google3/third_party/split_engine/input/split_engine_input_event_jni.cc:device_type)
  };

  enum class PointerType : uint8_t {
    // LINT.IfChange(pointer_type)
    DEFAULT = 0,
    LEFT = 1,
    RIGHT = 2,
    HEAD = 3,
    EYE = 4,
    // LINT.ThenChange(//depot/google3/third_party/split_engine/input/split_engine_input_event_jni.cc:pointer_type)
  };

  enum class Action : uint8_t {
    // LINT.IfChange(action)
    ACTION_DOWN = 0,
    ACTION_UP = 1,
    ACTION_MOVE = 2,
    ACTION_CANCEL = 3,
    ACTION_HOVER_MOVE = 4,
    ACTION_HOVER_ENTER = 5,
    ACTION_HOVER_EXIT = 6,
    // LINT.ThenChange(//depot/google3/third_party/split_engine/input/split_engine_input_event_jni.cc:action)
  };

  explicit SplitEngineInputEvent() = default;
  SplitEngineInputEvent& operator=(const SplitEngineInputEvent& other);
  std::string ToString() const;

  DispatchFlag dispatch_flag = DispatchFlag::NONE;
  // The type of the source of this event.
  DeviceType device_type = DeviceType::UNKNOWN;
  // The type of the individual pointer.
  PointerType pointer_type = PointerType::DEFAULT;
  // The time this event occurred, in the android.os.SystemClock#uptimeMillis
  // time base.
  int64_t timestamp_ms = 0;
  // The origin of the ray, in the receiver's task coordinate space. Will be
  // zero if the source is not ray-based, e.g., direct touch.
  imp::float3 origin;
  // A point indicating the direction the ray is pointing in, in the receiver's
  // task coordinate space. The ray is a vector starting at the origin point
  // and passing through the direction point.
  imp::float3 direction;
  // Bitmask of pressed buttons (mouse, controller, "select" gesture).
  // Currently is set to 1 when the action is either ACTION_DOWN or ACTION_MOVE,
  // 0 otherwise.
  int button_state = 0;
  // The action of the event.
  Action action;
  // The first scene node (closest to the ray origin) that was hit by the
  // input ray, if any.
  std::unique_ptr<SplitEngineInputEventHitInfo> hit_node = nullptr;
  // The second scene node that was hit by the input ray, if any.
  std::unique_ptr<SplitEngineInputEventHitInfo> secondary_hit_node = nullptr;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_INPUT_SPLIT_ENGINE_INPUT_EVENT_H_
