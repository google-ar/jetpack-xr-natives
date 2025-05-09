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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_SDL_VENUE_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_SDL_VENUE_H_

#include "core/common/optional_error.h"
#include "core/input/input_manager.h"
#include "core/view/utils/device.h"
#include "core/window/filament_host.h"

namespace imp {
namespace window {

using imp::Device;

// Create an SDL window, run it's message pump.
OptionalError SdlVenue(FilamentHost* host, imp::InputManager* inputManager,
                       Device* device);
OptionalError SdlVenue(FilamentHost* host, imp::InputManager* inputManager,
                       Device* device, const imp::uint2& dimensions);

}  // namespace window
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_SDL_VENUE_H_
