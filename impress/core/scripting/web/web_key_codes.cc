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

#include "core/input/key_codes.h"
#include "core/view/utils/string_map.h"

namespace imp::scripting {
namespace {
const imp::StringMap<VirtualKeyCode>& GetVirtualKeyCodeMap() {
  static const imp::StringMap<VirtualKeyCode>* map =
      new imp::StringMap<VirtualKeyCode>{
          //  the following are common keys on laptop keyboard
          {"Enter", VirtualKeyCode::VK_RETURN},
          {"Escape", VirtualKeyCode::VK_ESCAPE},
          {"Backspace", VirtualKeyCode::VK_BACKSPACE},
          {"Tab", VirtualKeyCode::VK_TAB},
          {"Space", VirtualKeyCode::VK_SPACE},
          {"Quote", VirtualKeyCode::VK_QUOTE},
          {"Comma", VirtualKeyCode::VK_COMMA},
          {"Minus", VirtualKeyCode::VK_MINUS},
          {"Period", VirtualKeyCode::VK_PERIOD},
          {"Slash", VirtualKeyCode::VK_SLASH},
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
          {"Semicolon", VirtualKeyCode::VK_SEMICOLON},
          {"Equal", VirtualKeyCode::VK_EQUALS},
          {"BracketLeft", VirtualKeyCode::VK_LEFTBRACKET},
          {"Backslash", VirtualKeyCode::VK_BACKSLASH},
          {"BracketRight", VirtualKeyCode::VK_RIGHTBRACKET},
          {"Backquote", VirtualKeyCode::VK_BACKQUOTE},
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
          {"Capslock", VirtualKeyCode::VK_CAPSLOCK},
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
          {"ArrowRight", VirtualKeyCode::VK_RIGHT},
          {"ArrowLeft", VirtualKeyCode::VK_LEFT},
          {"ArrowDown", VirtualKeyCode::VK_DOWN},
          {"ArrowUp", VirtualKeyCode::VK_UP},

          // the following are additional keys on extened keyboards
          {"Printscreen", VirtualKeyCode::VK_PRINTSCREEN},
          {"Scrolllock", VirtualKeyCode::VK_SCROLLLOCK},
          {"Pause", VirtualKeyCode::VK_PAUSE},
          {"Insert", VirtualKeyCode::VK_INSERT},
          {"Home", VirtualKeyCode::VK_HOME},
          {"PageUp", VirtualKeyCode::VK_PAGEUP},
          {"Delete", VirtualKeyCode::VK_DELETE},
          {"End", VirtualKeyCode::VK_END},
          {"PageDown", VirtualKeyCode::VK_PAGEDOWN},

          // the following are physical keys that may exist on some keyboards
          // Should we delete this section?
          {"Exclaim", VirtualKeyCode::VK_EXCLAIM},
          {"QuoteDouble", VirtualKeyCode::VK_QUOTEDBL},
          {"Hash", VirtualKeyCode::VK_HASH},
          {"Percent", VirtualKeyCode::VK_PERCENT},
          {"Dollar", VirtualKeyCode::VK_DOLLAR},
          {"Ampersand", VirtualKeyCode::VK_AMPERSAND},
          {"ParenLeft", VirtualKeyCode::VK_LEFTPAREN},
          {"ParenRight", VirtualKeyCode::VK_RIGHTPAREN},
          {"Asterisk", VirtualKeyCode::VK_ASTERISK},
          {"Plus", VirtualKeyCode::VK_PLUS},
          {"Colon", VirtualKeyCode::VK_COLON},
          {"Less", VirtualKeyCode::VK_LESS},
          {"Greater", VirtualKeyCode::VK_GREATER},
          {"Question", VirtualKeyCode::VK_QUESTION},
          {"At", VirtualKeyCode::VK_AT},
          {"Caret", VirtualKeyCode::VK_CARET},
          {"Underscore", VirtualKeyCode::VK_UNDERSCORE}};
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
