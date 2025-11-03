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

#include "core/scripting/web/web_key_codes.h"

#include "absl/strings/string_view.h"
#include "core/input/key_codes.h"
#include "core/view/utils/string_map.h"

namespace imp::scripting {
namespace {
const imp::StringMap<VirtualKeyCode>& GetVirtualKeyCodeMap() {
  static const imp::StringMap<VirtualKeyCode>* map =
      new imp::StringMap<VirtualKeyCode>{
          //  The following are keys defined in imp::VirtualKeyCode
          {"Tab", VirtualKeyCode::VK_TAB},
          {"ArrowLeft", VirtualKeyCode::VK_LEFT},
          {"ArrowRight", VirtualKeyCode::VK_RIGHT},
          {"ArrowUp", VirtualKeyCode::VK_UP},
          {"ArrowDown", VirtualKeyCode::VK_DOWN},
          {"PageUp", VirtualKeyCode::VK_PAGEUP},
          {"PageDown", VirtualKeyCode::VK_PAGEDOWN},
          {"Home", VirtualKeyCode::VK_HOME},
          {"End", VirtualKeyCode::VK_END},
          {"Insert", VirtualKeyCode::VK_INSERT},
          {"Delete", VirtualKeyCode::VK_DELETE},
          {"Backspace", VirtualKeyCode::VK_BACKSPACE},
          {"Space", VirtualKeyCode::VK_SPACE},
          {"Enter", VirtualKeyCode::VK_RETURN},
          {"Escape", VirtualKeyCode::VK_ESCAPE},
          {"ControlLeft", VirtualKeyCode::VK_LEFT_CTRL},
          {"ShiftLeft", VirtualKeyCode::VK_LEFT_SHIFT},
          {"AltLeft", VirtualKeyCode::VK_LEFT_ALT},
          {"MetaLeft", VirtualKeyCode::VK_LEFT_SUPER},
          {"ControlRight", VirtualKeyCode::VK_RIGHT_CTRL},
          {"ShiftRight", VirtualKeyCode::VK_RIGHT_SHIFT},
          {"AltRight", VirtualKeyCode::VK_RIGHT_ALT},
          {"MetaRight", VirtualKeyCode::VK_RIGHT_SUPER},
          {"Digit0", VirtualKeyCode::VK_0},
          {"Digit1", VirtualKeyCode::VK_1},
          {"Digit2", VirtualKeyCode::VK_2},
          {"Digit3", VirtualKeyCode::VK_3},
          {"Digit4", VirtualKeyCode::VK_4},
          {"Digit5", VirtualKeyCode::VK_5},
          {"Digit6", VirtualKeyCode::VK_6},
          {"Digit7", VirtualKeyCode::VK_7},
          {"Digit8", VirtualKeyCode::VK_8},
          {"Digit9", VirtualKeyCode::VK_9},
          {"KeyA", VirtualKeyCode::VK_a},
          {"KeyB", VirtualKeyCode::VK_b},
          {"KeyC", VirtualKeyCode::VK_c},
          {"KeyD", VirtualKeyCode::VK_d},
          {"KeyE", VirtualKeyCode::VK_e},
          {"KeyF", VirtualKeyCode::VK_f},
          {"KeyG", VirtualKeyCode::VK_g},
          {"KeyH", VirtualKeyCode::VK_h},
          {"KeyI", VirtualKeyCode::VK_i},
          {"KeyJ", VirtualKeyCode::VK_j},
          {"KeyK", VirtualKeyCode::VK_k},
          {"KeyL", VirtualKeyCode::VK_l},
          {"KeyM", VirtualKeyCode::VK_m},
          {"KeyN", VirtualKeyCode::VK_n},
          {"KeyO", VirtualKeyCode::VK_o},
          {"KeyP", VirtualKeyCode::VK_p},
          {"KeyQ", VirtualKeyCode::VK_q},
          {"KeyR", VirtualKeyCode::VK_r},
          {"KeyS", VirtualKeyCode::VK_s},
          {"KeyT", VirtualKeyCode::VK_t},
          {"KeyU", VirtualKeyCode::VK_u},
          {"KeyV", VirtualKeyCode::VK_v},
          {"KeyW", VirtualKeyCode::VK_w},
          {"KeyX", VirtualKeyCode::VK_x},
          {"KeyY", VirtualKeyCode::VK_y},
          {"KeyZ", VirtualKeyCode::VK_z},
          {"F1", VirtualKeyCode::VK_F1},
          {"F2", VirtualKeyCode::VK_F2},
          {"F3", VirtualKeyCode::VK_F3},
          {"F4", VirtualKeyCode::VK_F4},
          {"F5", VirtualKeyCode::VK_F5},
          {"F6", VirtualKeyCode::VK_F6},
          {"F7", VirtualKeyCode::VK_F7},
          {"F8", VirtualKeyCode::VK_F8},
          {"F9", VirtualKeyCode::VK_F9},
          {"F10", VirtualKeyCode::VK_F10},
          {"F11", VirtualKeyCode::VK_F11},
          {"F12", VirtualKeyCode::VK_F12},
          {"F13", VirtualKeyCode::VK_F13},
          {"Quote", VirtualKeyCode::VK_QUOTE},
          {"Comma", VirtualKeyCode::VK_COMMA},
          {"Minus", VirtualKeyCode::VK_MINUS},
          {"Period", VirtualKeyCode::VK_PERIOD},
          {"Slash", VirtualKeyCode::VK_SLASH},
          {"Semicolon", VirtualKeyCode::VK_SEMICOLON},
          {"Equal", VirtualKeyCode::VK_EQUALS},
          {"BracketLeft", VirtualKeyCode::VK_LEFTBRACKET},
          {"Backslash", VirtualKeyCode::VK_BACKSLASH},
          {"BracketRight", VirtualKeyCode::VK_RIGHTBRACKET},
          {"Backquote", VirtualKeyCode::VK_BACKQUOTE},
          {"Capslock", VirtualKeyCode::VK_CAPSLOCK},
          {"Scrolllock", VirtualKeyCode::VK_SCROLLLOCK},
          {"NumLock", VirtualKeyCode::VK_NUMLOCK},
          {"Printscreen", VirtualKeyCode::VK_PRINTSCREEN},
          {"Pause", VirtualKeyCode::VK_PAUSE},
          {"Numpad0", VirtualKeyCode::VK_KEYPAD0},
          {"Numpad1", VirtualKeyCode::VK_KEYPAD1},
          {"Numpad2", VirtualKeyCode::VK_KEYPAD2},
          {"Numpad3", VirtualKeyCode::VK_KEYPAD3},
          {"Numpad4", VirtualKeyCode::VK_KEYPAD4},
          {"Numpad5", VirtualKeyCode::VK_KEYPAD5},
          {"Numpad6", VirtualKeyCode::VK_KEYPAD6},
          {"Numpad7", VirtualKeyCode::VK_KEYPAD7},
          {"Numpad8", VirtualKeyCode::VK_KEYPAD8},
          {"Numpad9", VirtualKeyCode::VK_KEYPAD9},
          {"NumpadDecimal", VirtualKeyCode::VK_KEYPAD_DECIMAL},
          {"NumpadDivide", VirtualKeyCode::VK_KEYPAD_DIVIDE},
          {"NumpadMultiply", VirtualKeyCode::VK_KEYPAD_MULTIPLY},
          {"NumpadSubtract", VirtualKeyCode::VK_KEYPAD_SUBTRACT},
          {"NumpadAdd", VirtualKeyCode::VK_KEYPAD_ADD},
          {"Enter", VirtualKeyCode::VK_KEYPAD_ENTER},
          {"Equal", VirtualKeyCode::VK_KEYPAD_EQUAL},
      };
  return *map;
}

}  // namespace

VirtualKeyCode ToVirtualKeyCode(absl::string_view web_key_code) {
  if (!GetVirtualKeyCodeMap().count(web_key_code)) {
    return VirtualKeyCode::VK_UNKNOWN;
  }

  return GetVirtualKeyCodeMap().at(web_key_code);
}
}  // namespace imp::scripting
