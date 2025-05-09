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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_SDL_INPUT_PROCESSOR_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_SDL_INPUT_PROCESSOR_H_

#include "SDL2/include/SDL.h"
#include "absl/status/status.h"
#include "core/input/input_manager.h"
#include "core/window/filament_host.h"

namespace imp {

absl::Status ProcessInputFromSdlEvent(window::FilamentHost* host,
                                      InputManager* input_manager,
                                      SDL_Event* event);

// Converts from SDL key code to VirtualKeyCode.
VirtualKeyCode GetVirtualKeyCode(int key_code);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_SDL_INPUT_PROCESSOR_H_
