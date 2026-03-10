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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_KEYBOARD_EVENT_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_KEYBOARD_EVENT_H_

#include <vector>

#include "absl/time/time.h"
#include "core/input/key_codes.h"
#include "core/ncsb/dispatcher/event.h"

namespace imp {

// Represents an individual key.
struct Key {
  Key() {}
  Key(VirtualKeyCode code, KeyModifier modifier)
      : Key(code, ToFlags(modifier)) {}
  Key(VirtualKeyCode code, Flags<KeyModifier> modifier)
      : code(code), modifiers(modifier) {}
  VirtualKeyCode code = VirtualKeyCode::VK_NONE;
  Flags<KeyModifier> modifiers;
};

// Tracked keyboard event types.
// LINT.IfChange
enum class KeyboardEventType : uint8_t {
  kNone,
  kOnDown,
  kOnUp,
  kMax,
};
// LINT.ThenChange(
//   //depot/google3/third_party/impress/javascript/core/wasm/constants.js:keyboard_event_type
// )

// Represents a keyboard event user interaction.
class KeyboardEvent : public Event {
 public:
  KeyboardEvent();
  KeyboardEvent(const KeyboardEventType& type, const Key& key,
                absl::Duration elapsed_time);

  // The event type that triggered this event.
  KeyboardEventType type = KeyboardEventType::kNone;
  // The key that initiated this event.
  Key key;
  // Duration of time elapsed between system startup and time of event.
  absl::Duration elapsed_time;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_KEYBOARD_EVENT_H_
