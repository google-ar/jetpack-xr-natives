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

#include "core/scripting/message_handlers/forward_input_handler.h"

#include <string>
#include <vector>

#include "absl/time/time.h"
#include "core/input/input_manager.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/input/keyboard_event.proto.imp.h"
#include "core/input/wheel_event.proto.imp.h"
#include "core/scripting/web/web_key_codes.h"

namespace imp::scripting {

Future<absl::Status> ForwardInputHandler::HandleMessage(
    const PointerEventMessage& message) {
  if (message.pointers.empty()) {
    return Future<absl::Status>(absl::InvalidArgumentError(kNoPointersError));
  }
  auto& input_manager = view_.GetInputManager();
  std::vector<Pointer::Id> ids(message.pointers.size());
  std::vector<float2> points(message.pointers.size());
  for (auto i = 0; i < message.pointers.size(); i++) {
    const auto& pointer = message.pointers[i];
    ids[i] = pointer.id;
    points[i].x = pointer.point.x;
    points[i].y = pointer.point.y;
  }
  if (auto status = input_manager.ProcessPointerInput(
          message.type, ids, points, absl::Milliseconds(message.elapsed_time));
      !status.ok()) {
    return Future<absl::Status>(absl::InvalidArgumentError(kProcessError));
  }
  return Future<absl::Status>(absl::OkStatus());
}

Future<absl::Status> ForwardInputHandler::HandleMessage(
    const KeyboardEventMessage& message) {
  auto& input_manager = view_.GetInputManager();

  KeyboardEventType action = KeyboardEventType::kNone;
  if (message.type == KeyboardEventMessage::KeyboardEventTypeMessage::DOWN) {
    action = KeyboardEventType::kOnDown;
    if (message.key.length() == 1) input_manager.ProcessTextInput(message.key);
  } else if (message.type ==
             KeyboardEventMessage::KeyboardEventTypeMessage::UP) {
    action = KeyboardEventType::kOnUp;
  }
  Flags<KeyModifier> modifier_flags;
  modifier_flags.Set(KeyModifier::ALT, message.alt_key);
  modifier_flags.Set(KeyModifier::SHIFT, message.shift_key);
  modifier_flags.Set(KeyModifier::CTRL, message.ctrl_key);
  Key key = Key(ToVirtualKeyCode(message.code), modifier_flags);
  absl::Duration elapsed_time = absl::Milliseconds(0.1);
  return Future<absl::Status>(input_manager.ProcessKeyboardInput(
      static_cast<uint8_t>(action), key, elapsed_time));
}

Future<absl::Status> ForwardInputHandler::HandleMessage(
    const WheelEventMessage& message) {
  auto& input_manager = view_.GetInputManager();
  if (auto status = input_manager.ProcessWheelInput(
          message.delta, absl::Milliseconds(message.elapsed_time));
      !status.ok()) {
    return Future<absl::Status>(absl::InvalidArgumentError(kProcessError));
  }
  return Future<absl::Status>(absl::OkStatus());
}

}  // namespace imp::scripting
