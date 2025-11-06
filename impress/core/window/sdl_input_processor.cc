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

#include "core/window/sdl_input_processor.h"

#include <cstdint>
#include <vector>

#include "SDL2/include/SDL_events.h"
#include "SDL2/include/SDL_keyboard.h"
#include "SDL2/include/SDL_keycode.h"
#include "SDL2/include/SDL_mouse.h"
#include "SDL2/include/SDL_video.h"
#include "core/config.h"
#include "core/input/key_codes.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace {

using window::FilamentHost;

Flags<KeyModifier> GetKeyModifier(uint32_t modifier) {
  Flags<KeyModifier> result(KeyModifier::NONE);
  if ((KMOD_LSHIFT & modifier) > 0) {
    result |= KeyModifier::LSHIFT;
  }
  if ((KMOD_RSHIFT & modifier) > 0) {
    result |= KeyModifier::RSHIFT;
  }
  if ((KMOD_LCTRL & modifier) > 0) {
    result |= KeyModifier::LCTRL;
  }
  if ((KMOD_RCTRL & modifier) > 0) {
    result |= KeyModifier::RCTRL;
  }
  if ((KMOD_LALT & modifier) > 0) {
    result |= KeyModifier::LALT;
  }
  if ((KMOD_RALT & modifier) > 0) {
    result |= KeyModifier::RALT;
  }
  if ((KMOD_LGUI & modifier) > 0) {
    result |= KeyModifier::LGUI;
  }
  if ((KMOD_RGUI & modifier) > 0) {
    result |= KeyModifier::RGUI;
  }
  if ((KMOD_NUM & modifier) > 0) {
    result |= KeyModifier::NUM;
  }
  if ((KMOD_CAPS & modifier) > 0) {
    result |= KeyModifier::CAPS;
  }
  if ((KMOD_MODE & modifier) > 0) {
    result |= KeyModifier::MODE;
  }

  return result;
}

}  // namespace

// A multiplier for the mouse wheel delta to make it similar to the values seen
// on the web platform (direction is inverted and magnitude is much smaller).
static constexpr float kMouseWheelWebConsistencyMultiplier = -30.0f;

// The ordering of these values match those of imp::VirtualKeyCode
VirtualKeyCode GetVirtualKeyCode(int key_code) {
  switch (key_code) {
    case SDLK_TAB:
      return VirtualKeyCode::VK_TAB;
    case SDLK_LEFT:
      return VirtualKeyCode::VK_LEFT;
    case SDLK_RIGHT:
      return VirtualKeyCode::VK_RIGHT;
    case SDLK_UP:
      return VirtualKeyCode::VK_UP;
    case SDLK_DOWN:
      return VirtualKeyCode::VK_DOWN;
    case SDLK_PAGEUP:
      return VirtualKeyCode::VK_PAGEUP;
    case SDLK_PAGEDOWN:
      return VirtualKeyCode::VK_PAGEDOWN;
    case SDLK_HOME:
      return VirtualKeyCode::VK_HOME;
    case SDLK_END:
      return VirtualKeyCode::VK_END;
    case SDLK_INSERT:
      return VirtualKeyCode::VK_INSERT;
    case SDLK_DELETE:
      return VirtualKeyCode::VK_DELETE;
    case SDLK_BACKSPACE:
      return VirtualKeyCode::VK_BACKSPACE;
    case SDLK_SPACE:
      return VirtualKeyCode::VK_SPACE;
    case SDLK_RETURN:
      return VirtualKeyCode::VK_RETURN;
    case SDLK_ESCAPE:
      return VirtualKeyCode::VK_ESCAPE;
    case SDLK_LCTRL:
      return VirtualKeyCode::VK_LEFT_CTRL;
    case SDLK_LSHIFT:
      return VirtualKeyCode::VK_LEFT_SHIFT;
    case SDLK_LALT:
      return VirtualKeyCode::VK_LEFT_ALT;
    case SDLK_LGUI:
      return VirtualKeyCode::VK_LEFT_SUPER;
    case SDLK_RCTRL:
      return VirtualKeyCode::VK_RIGHT_CTRL;
    case SDLK_RSHIFT:
      return VirtualKeyCode::VK_RIGHT_SHIFT;
    case SDLK_RALT:
      return VirtualKeyCode::VK_RIGHT_ALT;
    case SDLK_RGUI:
      return VirtualKeyCode::VK_RIGHT_SUPER;
    case SDLK_MENU:
      return VirtualKeyCode::VK_MENU;
    case SDLK_0:
      return VirtualKeyCode::VK_0;
    case SDLK_1:
      return VirtualKeyCode::VK_1;
    case SDLK_2:
      return VirtualKeyCode::VK_2;
    case SDLK_3:
      return VirtualKeyCode::VK_3;
    case SDLK_4:
      return VirtualKeyCode::VK_4;
    case SDLK_5:
      return VirtualKeyCode::VK_5;
    case SDLK_6:
      return VirtualKeyCode::VK_6;
    case SDLK_7:
      return VirtualKeyCode::VK_7;
    case SDLK_8:
      return VirtualKeyCode::VK_8;
    case SDLK_9:
      return VirtualKeyCode::VK_9;
    case SDLK_a:
      return VirtualKeyCode::VK_a;
    case SDLK_b:
      return VirtualKeyCode::VK_b;
    case SDLK_c:
      return VirtualKeyCode::VK_c;
    case SDLK_d:
      return VirtualKeyCode::VK_d;
    case SDLK_e:
      return VirtualKeyCode::VK_e;
    case SDLK_f:
      return VirtualKeyCode::VK_f;
    case SDLK_g:
      return VirtualKeyCode::VK_g;
    case SDLK_h:
      return VirtualKeyCode::VK_h;
    case SDLK_i:
      return VirtualKeyCode::VK_i;
    case SDLK_j:
      return VirtualKeyCode::VK_j;
    case SDLK_k:
      return VirtualKeyCode::VK_k;
    case SDLK_l:
      return VirtualKeyCode::VK_l;
    case SDLK_m:
      return VirtualKeyCode::VK_m;
    case SDLK_n:
      return VirtualKeyCode::VK_n;
    case SDLK_o:
      return VirtualKeyCode::VK_o;
    case SDLK_p:
      return VirtualKeyCode::VK_p;
    case SDLK_q:
      return VirtualKeyCode::VK_q;
    case SDLK_r:
      return VirtualKeyCode::VK_r;
    case SDLK_s:
      return VirtualKeyCode::VK_s;
    case SDLK_t:
      return VirtualKeyCode::VK_t;
    case SDLK_u:
      return VirtualKeyCode::VK_u;
    case SDLK_v:
      return VirtualKeyCode::VK_v;
    case SDLK_w:
      return VirtualKeyCode::VK_w;
    case SDLK_x:
      return VirtualKeyCode::VK_x;
    case SDLK_y:
      return VirtualKeyCode::VK_y;
    case SDLK_z:
      return VirtualKeyCode::VK_z;
    case SDLK_F1:
      return VirtualKeyCode::VK_F1;
    case SDLK_F2:
      return VirtualKeyCode::VK_F2;
    case SDLK_F3:
      return VirtualKeyCode::VK_F3;
    case SDLK_F4:
      return VirtualKeyCode::VK_F4;
    case SDLK_F5:
      return VirtualKeyCode::VK_F5;
    case SDLK_F6:
      return VirtualKeyCode::VK_F6;
    case SDLK_F7:
      return VirtualKeyCode::VK_F7;
    case SDLK_F8:
      return VirtualKeyCode::VK_F8;
    case SDLK_F9:
      return VirtualKeyCode::VK_F9;
    case SDLK_F10:
      return VirtualKeyCode::VK_F10;
    case SDLK_F11:
      return VirtualKeyCode::VK_F11;
    case SDLK_F12:
      return VirtualKeyCode::VK_F12;
    case SDLK_F13:
      return VirtualKeyCode::VK_F13;
    case SDLK_QUOTE:
      return VirtualKeyCode::VK_QUOTE;
    case SDLK_COMMA:
      return VirtualKeyCode::VK_COMMA;
    case SDLK_MINUS:
      return VirtualKeyCode::VK_MINUS;
    case SDLK_PERIOD:
      return VirtualKeyCode::VK_PERIOD;
    case SDLK_SLASH:
      return VirtualKeyCode::VK_SLASH;
    case SDLK_SEMICOLON:
      return VirtualKeyCode::VK_SEMICOLON;
    case SDLK_EQUALS:
      return VirtualKeyCode::VK_EQUALS;
    case SDLK_LEFTBRACKET:
      return VirtualKeyCode::VK_LEFTBRACKET;
    case SDLK_BACKSLASH:
      return VirtualKeyCode::VK_BACKSLASH;
    case SDLK_RIGHTBRACKET:
      return VirtualKeyCode::VK_RIGHTBRACKET;
    case SDLK_BACKQUOTE:
      return VirtualKeyCode::VK_BACKQUOTE;
    case SDLK_CAPSLOCK:
      return VirtualKeyCode::VK_CAPSLOCK;
    case SDLK_SCROLLLOCK:
      return VirtualKeyCode::VK_SCROLLLOCK;
    case SDLK_NUMLOCKCLEAR:
      return VirtualKeyCode::VK_NUMLOCK;
    case SDLK_PRINTSCREEN:
      return VirtualKeyCode::VK_PRINTSCREEN;
    case SDLK_PAUSE:
      return VirtualKeyCode::VK_PAUSE;
    case SDLK_KP_0:
      return VirtualKeyCode::VK_KEYPAD0;
    case SDLK_KP_1:
      return VirtualKeyCode::VK_KEYPAD1;
    case SDLK_KP_2:
      return VirtualKeyCode::VK_KEYPAD2;
    case SDLK_KP_3:
      return VirtualKeyCode::VK_KEYPAD3;
    case SDLK_KP_4:
      return VirtualKeyCode::VK_KEYPAD4;
    case SDLK_KP_5:
      return VirtualKeyCode::VK_KEYPAD5;
    case SDLK_KP_6:
      return VirtualKeyCode::VK_KEYPAD6;
    case SDLK_KP_7:
      return VirtualKeyCode::VK_KEYPAD7;
    case SDLK_KP_8:
      return VirtualKeyCode::VK_KEYPAD8;
    case SDLK_KP_9:
      return VirtualKeyCode::VK_KEYPAD9;
    case SDLK_KP_DECIMAL:
      return VirtualKeyCode::VK_KEYPAD_DECIMAL;
    case SDLK_KP_DIVIDE:
      return VirtualKeyCode::VK_KEYPAD_DIVIDE;
    case SDLK_KP_MULTIPLY:
      return VirtualKeyCode::VK_KEYPAD_MULTIPLY;
    case SDLK_KP_MINUS:
      return VirtualKeyCode::VK_KEYPAD_SUBTRACT;
    case SDLK_KP_PLUS:
      return VirtualKeyCode::VK_KEYPAD_ADD;
    case SDLK_KP_ENTER:
      return VirtualKeyCode::VK_KEYPAD_ENTER;
    case SDLK_KP_EQUALS:
      return VirtualKeyCode::VK_KEYPAD_EQUAL;
    default:
      return VirtualKeyCode::VK_UNKNOWN;
  }
}

absl::Status ProcessInputFromSdlEvent(FilamentHost* host,
                                      InputManager* input_manager,
                                      SDL_Event* event) {
  switch (event->type) {
    case SDL_MOUSEMOTION: {
      if (event->motion.which == SDL_TOUCH_MOUSEID) {
        break;
      }

      imp::PointerEventType event_type = imp::PointerEventType::kHover;
      std::vector<imp::Pointer::Id> pointers;
      if (event->motion.state & SDL_BUTTON_LMASK) {
        pointers.push_back(kMousePointerIdLeft);
        event_type = imp::PointerEventType::kMove;
      }
      if (event->motion.state & SDL_BUTTON_RMASK) {
        // Prefer left over right instead of allowing both for now.
        pointers.push_back(kMousePointerIdRight);
        event_type = imp::PointerEventType::kMove;
      }
      if (pointers.empty()) {
        // If neither button is down, this is still a default pointer hover.
        pointers.push_back(kDefaultMousePointerId);
      }
      MP_RETURN_IF_ERROR(input_manager->ProcessPointerInput(
          static_cast<uint8_t>(event_type), pointers,
          std::vector<float2>(pointers.size(),
                              float2(event->motion.x, event->motion.y)),
          absl::Milliseconds(event->motion.timestamp)));

#if IMP_RUNTIME(DEV)
      MP_RETURN_IF_ERROR(host->QueueMouseInput(
          FilamentHost::Drag(int2(event->motion.x, event->motion.y),
                             int2(event->motion.xrel, event->motion.yrel))));
#endif  // IMP_RUNTIME(DEV)
      break;
    }
    case SDL_MOUSEBUTTONDOWN:
    case SDL_MOUSEBUTTONUP: {
      if (event->button.which == SDL_TOUCH_MOUSEID) {
        break;
      }

      const uint8_t eventType =
          (event->type == SDL_MOUSEBUTTONDOWN)
              ? static_cast<uint8_t>(imp::PointerEventType::kDown)
              : static_cast<uint8_t>(imp::PointerEventType::kUp);
      std::vector<imp::Pointer::Id> pointers;
      if (event->button.button == SDL_BUTTON_LEFT) {
        pointers.push_back(kMousePointerIdLeft);
      }
      if (event->button.button == SDL_BUTTON_RIGHT) {
        pointers.push_back(kMousePointerIdRight);
      }
      if (pointers.empty()) {
        break;
      }
      MP_RETURN_IF_ERROR(input_manager->ProcessPointerInput(
          eventType, pointers,
          std::vector<float2>(pointers.size(),
                              float2(event->button.x, event->button.y)),
          absl::Milliseconds(event->button.timestamp)));

#if IMP_RUNTIME(DEV)
      MP_RETURN_IF_ERROR(host->QueueMouseInput(
          event->type == SDL_MOUSEBUTTONDOWN
              ? FilamentHost::MouseInput(
                    FilamentHost::DragBegin{static_cast<int>(pointers[0])})
              : FilamentHost::MouseInput(
                    FilamentHost::DragEnd{static_cast<int>(pointers[0])})));
#endif  // IMP_RUNTIME(DEV)
      break;
    }
    case SDL_MOUSEWHEEL: {
      MP_RETURN_IF_ERROR(input_manager->ProcessWheelInput(
          // Depending on the platform wheel delta will go into x or y
          // component.
          static_cast<float>(event->wheel.y + event->wheel.x) *
              kMouseWheelWebConsistencyMultiplier,
          absl::Milliseconds(event->wheel.timestamp)));

#if IMP_RUNTIME(DEV)
      MP_RETURN_IF_ERROR(host->QueueMouseInput(
          FilamentHost::Wheel{int2(event->wheel.x, event->wheel.y)}));
#endif  // IMP_RUNTIME(DEV)
      break;
    }
    case SDL_TEXTINPUT: {
      input_manager->ProcessTextInput(event->text.text);
      break;
    }
    case SDL_KEYDOWN: {
      VirtualKeyCode vkey = GetVirtualKeyCode(event->key.keysym.sym);
      MP_RETURN_IF_ERROR(input_manager->ProcessKeyboardInput(
          static_cast<uint8_t>(KeyboardEventType::kOnDown),
          Key(vkey, GetKeyModifier(SDL_GetModState()).Value()),
          absl::Milliseconds(event->button.timestamp)));

      break;
    }
    case SDL_KEYUP: {
      VirtualKeyCode vkey = GetVirtualKeyCode(event->key.keysym.sym);
      MP_RETURN_IF_ERROR(input_manager->ProcessKeyboardInput(
          static_cast<uint8_t>(KeyboardEventType::kOnUp),
          Key(vkey, GetKeyModifier(SDL_GetModState()).Value()),
          absl::Milliseconds(event->button.timestamp)));

      break;
    }
    case SDL_WINDOWEVENT: {
      imp::PointerEventType event_type;
      if (event->window.event == SDL_WINDOWEVENT_ENTER) {
        event_type = imp::PointerEventType::kEnterView;
      } else if (event->window.event == SDL_WINDOWEVENT_LEAVE) {
        event_type = imp::PointerEventType::kLeaveView;
      } else {
        break;
      }
      int x, y;
      SDL_GetMouseState(&x, &y);
      MP_RETURN_IF_ERROR(input_manager->ProcessPointerInput(
          static_cast<uint8_t>(event_type), {kDefaultMousePointerId},
          {float2(x, y)}, absl::Milliseconds(event->motion.timestamp)));
    }
  }

  return absl::OkStatus();
}

}  // namespace imp
