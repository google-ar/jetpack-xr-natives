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

#include "core/view/platforms/desktop/clipboard/sdl_clipboard_handler.h"

#include <memory>
#include <string>

#include "SDL2/include/SDL_clipboard.h"

void SdlClipboardHandler::SetClipboardText(absl::string_view text) {
  SDL_SetClipboardText(std::string(text).c_str());
}

absl::string_view SdlClipboardHandler::GetClipboardText() {
  sdl_clipboard_text_buffer_ = std::make_unique<SdlClipboardTextBuffer>();
  return sdl_clipboard_text_buffer_->buffer;
}
