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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_NATIVE_WINDOW_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_NATIVE_WINDOW_HELPER_H_

struct SDL_Window;

// In order to render to an SDL window, we need a per-platform function to
// get a window pointer to associate with a SwapChain.  Declare the prototype
// as a C function to simplify defining it in multiple languages.
extern "C" void* Impress_getNativeWindow(SDL_Window* sdlWindow);

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_NATIVE_WINDOW_HELPER_H_
