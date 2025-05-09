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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_DESKTOP_CLIPBOARD_SDL_CLIPBOARD_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_DESKTOP_CLIPBOARD_SDL_CLIPBOARD_HANDLER_H_

#include <memory>

#include "SDL2/include/SDL_clipboard.h"
#include "SDL2/include/SDL_stdinc.h"
#include "absl/strings/string_view.h"
#include "core/window/clipboard/clipboard_handler.h"

class SdlClipboardHandler : public ClipboardHandler {
 public:
  void SetClipboardText(absl::string_view text) override;
  // Please note that the returned string_view will live until the next call of
  // GetClipboardText().
  absl::string_view GetClipboardText() override;

 private:
  // SdlClipboardTextBuffer holds the buffer from SDL_GetClipboardText.
  // SdlClipboardTextBuffer calls SDL_GetClipboardText upon creation and frees
  // the buffer upon destruction since users are responsible for freeing the
  // buffer returned by SDL_GetClipboardText according to SDL documentation,
  struct SdlClipboardTextBuffer {
    explicit SdlClipboardTextBuffer() : buffer(SDL_GetClipboardText()) {}
    SdlClipboardTextBuffer(const SdlClipboardTextBuffer& rhs) = delete;
    SdlClipboardTextBuffer& operator=(SdlClipboardTextBuffer&& rhs) = delete;
    ~SdlClipboardTextBuffer() { SDL_free(buffer); }
    char* buffer;
  };

  // sdl_clipboard_text_buffer_ gets recreated upon calling
  // SdlClipboardHandler::GetClipboardText().
  //
  // Please note that this does not guarantee to reflect the current text data
  // in the clipboard as the clipboard could contain other texts set by
  // SetClipboardText() or even other processes.
  std::unique_ptr<SdlClipboardTextBuffer> sdl_clipboard_text_buffer_;
};

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_DESKTOP_CLIPBOARD_SDL_CLIPBOARD_HANDLER_H_
