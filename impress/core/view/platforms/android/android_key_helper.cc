// Copyright 2025 Google LLC
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

#include "core/view/platforms/android/android_key_helper.h"

#include <cstdint>
#include <string>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/time/time.h"
#include "core/common/enum_flags.h"
#include "core/input/input_manager.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/base_view.h"
#include "core/view/view_events.h"

namespace imp {
namespace {

using ::imp::Flags;
using ::imp::Key;
using ::imp::KeyboardEventType;
using ::imp::KeyModifier;
using ::imp::VirtualKeyCode;
}  // namespace

VirtualKeyCode getVkCode(int key_code) {
  switch (key_code) {
    case 66:
      return VirtualKeyCode::VK_RETURN;
      break;
    case 67:
      return VirtualKeyCode::VK_BACKSPACE;
      break;
    case 62:
      return VirtualKeyCode::VK_SPACE;
      break;
    case 75:
      return VirtualKeyCode::VK_QUOTE;
      break;
    case 55:
      return VirtualKeyCode::VK_COMMA;
      break;
    case 69:
      return VirtualKeyCode::VK_MINUS;
      break;
    case 56:
      return VirtualKeyCode::VK_PERIOD;
      break;
    case 76:
      return VirtualKeyCode::VK_SLASH;
      break;
    case 7:
      return VirtualKeyCode::VK_0;
      break;
    case 8:
      return VirtualKeyCode::VK_1;
      break;
    case 9:
      return VirtualKeyCode::VK_2;
      break;
    case 10:
      return VirtualKeyCode::VK_3;
      break;
    case 11:
      return VirtualKeyCode::VK_4;
      break;
    case 12:
      return VirtualKeyCode::VK_5;
      break;
    case 13:
      return VirtualKeyCode::VK_6;
      break;
    case 14:
      return VirtualKeyCode::VK_7;
      break;
    case 15:
      return VirtualKeyCode::VK_8;
      break;
    case 16:
      return VirtualKeyCode::VK_9;
      break;
    case 74:
      return VirtualKeyCode::VK_SEMICOLON;
      break;
    case 70:
      return VirtualKeyCode::VK_EQUALS;
      break;
    case 71:
      return VirtualKeyCode::VK_LEFTBRACKET;
      break;
    case 73:
      return VirtualKeyCode::VK_BACKSLASH;
      break;
    case 72:
      return VirtualKeyCode::VK_RIGHTBRACKET;
      break;
    case 68:
      return VirtualKeyCode::VK_BACKQUOTE;
      break;
    case 29:
      return VirtualKeyCode::VK_a;
      break;
    case 30:
      return VirtualKeyCode::VK_b;
      break;
    case 31:
      return VirtualKeyCode::VK_c;
      break;
    case 32:
      return VirtualKeyCode::VK_d;
      break;
    case 33:
      return VirtualKeyCode::VK_e;
      break;
    case 34:
      return VirtualKeyCode::VK_f;
      break;
    case 35:
      return VirtualKeyCode::VK_g;
      break;
    case 36:
      return VirtualKeyCode::VK_h;
      break;
    case 37:
      return VirtualKeyCode::VK_i;
      break;
    case 38:
      return VirtualKeyCode::VK_j;
      break;
    case 39:
      return VirtualKeyCode::VK_k;
      break;
    case 40:
      return VirtualKeyCode::VK_l;
      break;
    case 41:
      return VirtualKeyCode::VK_m;
      break;
    case 42:
      return VirtualKeyCode::VK_n;
      break;
    case 43:
      return VirtualKeyCode::VK_o;
      break;
    case 44:
      return VirtualKeyCode::VK_p;
      break;
    case 45:
      return VirtualKeyCode::VK_q;
      break;
    case 46:
      return VirtualKeyCode::VK_r;
      break;
    case 47:
      return VirtualKeyCode::VK_s;
      break;
    case 48:
      return VirtualKeyCode::VK_t;
      break;
    case 49:
      return VirtualKeyCode::VK_u;
      break;
    case 50:
      return VirtualKeyCode::VK_v;
      break;
    case 51:
      return VirtualKeyCode::VK_w;
      break;
    case 52:
      return VirtualKeyCode::VK_x;
      break;
    case 53:
      return VirtualKeyCode::VK_y;
      break;
    case 54:
      return VirtualKeyCode::VK_z;
      break;
  }
  return VirtualKeyCode::VK_NONE;
}

KeyboardEventType getAction(int action) {
  if (action == 0) {
    return KeyboardEventType::kOnDown;
  } else if (action == 1) {
    return KeyboardEventType::kOnUp;
  }
  return KeyboardEventType::kNone;
}

void ProcessKeyboardEvent(BaseView* view, int char_code, int key_code,
                          int action, int modifiers) {
  Flags<KeyModifier> modifier_flags;
  modifier_flags.Set(KeyModifier::SHIFT, modifiers);
  Key key = Key(getVkCode(key_code), modifier_flags);
  absl::Duration elapsed_time = absl::Milliseconds(0.1);

  if (getAction(action) == KeyboardEventType::kOnDown) {
    view->GetInputManager().ProcessTextInput(
        std::string(1, static_cast<char>(char_code)));

    if (absl::Status status = view->GetInputManager().ProcessKeyboardInput(
            static_cast<uint8_t>(getAction(action)), key, elapsed_time);
        !status.ok()) {
      IMP_LOG(imp::ERROR) << "Unable to process keyboard input: " << status.ToString();
      return;
    }

  } else if (getAction(action) == KeyboardEventType::kOnUp) {
    // Schedule a key up events at the end of the frame.
    view->GetDispatcher().Connect(
        [view, key, elapsed_time](const imp::ViewPostFrameEvent& ev) {
          uint8_t action = static_cast<uint8_t>(imp::KeyboardEventType::kOnUp);
          absl::Status res = view->GetInputManager().ProcessKeyboardInput(
              action, key, elapsed_time);
          if (!res.ok()) {
            IMP_LOG(imp::ERROR) << "Unable to process keyboard input: "
                       << res.ToString();
          }
          ev.Disconnect();
        },
        view);
  }
}
}  // namespace imp
