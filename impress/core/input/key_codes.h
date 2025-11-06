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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_KEY_CODES_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_KEY_CODES_H_

#include <stdint.h>

#include "core/common/enum_flags.h"

namespace imp {
// Virtual key codes representing a keyboard key.
// These values are directly mapped to ImGuiKey
enum class VirtualKeyCode : uint32_t {
  VK_NONE = 0,
  VK_UNKNOWN = 0,
  VK_TAB = 512,
  VK_LEFT,
  VK_RIGHT,
  VK_UP,
  VK_DOWN,
  VK_PAGEUP,
  VK_PAGEDOWN,
  VK_HOME,
  VK_END,
  VK_INSERT,
  VK_DELETE,
  VK_BACKSPACE,
  VK_SPACE,
  VK_RETURN,  // Enter
  VK_ESCAPE,
  VK_LEFT_CTRL,
  VK_LEFT_SHIFT,
  VK_LEFT_ALT,
  VK_LEFT_SUPER,
  VK_RIGHT_CTRL,
  VK_RIGHT_SHIFT,
  VK_RIGHT_ALT,
  VK_RIGHT_SUPER,
  VK_MENU,
  VK_0,
  VK_1,
  VK_2,
  VK_3,
  VK_4,
  VK_5,
  VK_6,
  VK_7,
  VK_8,
  VK_9,
  VK_a,
  VK_b,
  VK_c,
  VK_d,
  VK_e,
  VK_f,
  VK_g,
  VK_h,
  VK_i,
  VK_j,
  VK_k,
  VK_l,
  VK_m,
  VK_n,
  VK_o,
  VK_p,
  VK_q,
  VK_r,
  VK_s,
  VK_t,
  VK_u,
  VK_v,
  VK_w,
  VK_x,
  VK_y,
  VK_z,
  VK_F1,
  VK_F2,
  VK_F3,
  VK_F4,
  VK_F5,
  VK_F6,
  VK_F7,
  VK_F8,
  VK_F9,
  VK_F10,
  VK_F11,
  VK_F12,
  VK_F13,
  // F14-F24 are skipped as they are not common
  VK_QUOTE = 596,  // Apostrophe
  VK_COMMA,
  VK_MINUS,
  VK_PERIOD,
  VK_SLASH,
  VK_SEMICOLON,
  VK_EQUALS,
  VK_LEFTBRACKET,
  VK_BACKSLASH,
  VK_RIGHTBRACKET,
  VK_BACKQUOTE,  // GraveAccent
  VK_CAPSLOCK,
  VK_SCROLLLOCK,
  VK_NUMLOCK,
  VK_PRINTSCREEN,
  VK_PAUSE,
  VK_KEYPAD0,
  VK_KEYPAD1,
  VK_KEYPAD2,
  VK_KEYPAD3,
  VK_KEYPAD4,
  VK_KEYPAD5,
  VK_KEYPAD6,
  VK_KEYPAD7,
  VK_KEYPAD8,
  VK_KEYPAD9,
  VK_KEYPAD_DECIMAL,
  VK_KEYPAD_DIVIDE,
  VK_KEYPAD_MULTIPLY,
  VK_KEYPAD_SUBTRACT,
  VK_KEYPAD_ADD,
  VK_KEYPAD_ENTER,
  VK_KEYPAD_EQUAL,
};

// Key modifier flags indicate if CTRL is pressed, or ALT is pressed at the time
// of keyboard input. Multiple modifiers can be pressed at once thus they are
// represented as a bit field of values.
enum class KeyModifier : uint16_t {
  NONE = 0,
  LSHIFT = (1 << 0),
  RSHIFT = (1 << 1),
  LCTRL = (1 << 2),
  RCTRL = (1 << 3),
  LALT = (1 << 4),
  RALT = (1 << 5),
  LGUI = (1 << 6),
  RGUI = (1 << 7),
  NUM = (1 << 8),
  CAPS = (1 << 9),
  MODE = (1 << 10),
  CTRL = LCTRL | RCTRL,
  ALT = LALT | RALT,
  SHIFT = LSHIFT | RSHIFT,
  GUI = LGUI | RGUI,
  CTRL_OR_GUI = CTRL | GUI,
};

// Checks a modifier code for the presence of a modifier flag.
bool HasKeyModifier(KeyModifier modifier, Flags<KeyModifier> code);
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_KEY_CODES_H_
