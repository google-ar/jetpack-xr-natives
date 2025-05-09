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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_FORWARD_INPUT_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_FORWARD_INPUT_HANDLER_H_

#include "core/input/keyboard_event.proto.imp.h"
#include "core/input/pointer_event.proto.imp.h"
#include "core/input/wheel_event.proto.imp.h"
#include "core/scripting/multi_message_handler.h"
#include "core/view/base_view.h"

namespace imp::scripting {

// A MessageHandler for input event forwarding.
class ForwardInputHandler : public MultiMessageHandler {
 public:
  explicit ForwardInputHandler(BaseView& view) : view_(view) {
    AddHandler<PointerEventMessage, absl::Status>(this);
    AddHandler<KeyboardEventMessage, absl::Status>(this);
    AddHandler<WheelEventMessage, absl::Status>(this);
  }

  Future<absl::Status> HandleMessage(const PointerEventMessage& message);
  Future<absl::Status> HandleMessage(const KeyboardEventMessage& message);
  Future<absl::Status> HandleMessage(const WheelEventMessage& message);

  constexpr static absl::string_view kNoPointersError =
      "No pointers in PointerEventMessage";
  constexpr static absl::string_view kProcessError =
      "Failed to process pointer inputs";

 private:
  BaseView& view_;
};

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_FORWARD_INPUT_HANDLER_H_
