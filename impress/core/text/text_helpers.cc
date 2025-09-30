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

#include "core/text/text_helpers.h"

#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "unicode/utf8.h"
#include "unicode/utypes.h"
#include "core/config.h"
#if IMP_PLATFORM(DESKTOP)
#include "unicode/uchar.h"
#elif IMP_PLATFORM(WASM)
#include "emscripten/emscripten.h"
#endif

namespace imp {

bool ContainsNonSeparableScript(absl::string_view text) {
  const char* s = text.data();
  int32_t length = text.length();
  int32_t si = 0;
  UChar32 c, peek;
  U8_NEXT(s, si, length, c);

  while (si < length) {
    U8_NEXT(s, si, length, peek);
    // If the current character is not separable, then it could potentially be
    // an emoji, an emoji sequence modifier, or a non-separable script. To check
    // if it's an emoji, we also need to check if the character following it is
    // an emoji sequence modifier or not.
    if (!IsSeparable(c) && !IsEmojiSequenceModifier(peek) &&
        !IsEmojiSequenceModifier(c) && !IsEmoji(c)) {
      return true;
    }
    c = peek;
  }
  return false;
}

bool IsSeparable(UChar32 c) {
  if (IsEmojiSequenceModifier(c)) {
    return false;
  }

  return (c >= 0x0000 && c < 0x0600) ||  // Latin and Hebrew
         (c >= 0x2000 && c < 0x206F) ||  // Punctuation
         (c >= 0x3000 && c < 0xD800);    // CJK
}

bool IsEmojiSequenceModifier(UChar32 c) {
  // Check if the character is a zero-width joiner, which is used for emoji
  // sequences: (broken link)
  // or if the character is a emoji variation selector, which is used to
  // colorize emojis that otherwise would render as normal text.
  // (broken link)
  return c == 0x200D || c == 0xFE0F;
}

bool IsEmoji(UChar32 c) {
  // (broken link)
  // 0xFE0F is an emoji variation selector in which the emoji will be rendered
  // as an image.
  if (c == 0xFE0F) {
    return true;
  }
  // Otherwise EMOJI_PRESENTATION should match all other image emojis
  // (broken link)
#if IMP_PLATFORM(WASM)
  return EM_ASM_INT(
      {
        // clang-format off
        return new RegExp('\\\\p{Emoji_Presentation}', 'u').test(
            String.fromCodePoint($0)) ? 1 : 0;
        // clang-format on
      },
      c);
#elif IMP_PLATFORM(DESKTOP)
  return u_hasBinaryProperty(c, UCHAR_EMOJI_PRESENTATION);
#else
  return false;
#endif
}

bool ContainsEmoji(absl::string_view text) {
  if (text.empty()) {
    return false;
  }
  const char* s = text.data();
  int32_t length = text.length();
  int32_t si = 0;
  UChar32 c;
  while (si < length) {
    U8_NEXT(s, si, length, c);
    if (IsEmoji(c)) {
      return true;
    }
  }
  return false;
}

bool IsRtl(UChar32 c) { return c >= 0x0590 && c < 0x0700; }

bool ContainsRtl(absl::string_view text) {
  const char* s = text.data();
  int32_t length = text.length();
  int32_t si = 0;
  while (si < length) {
    UChar32 c;
    U8_NEXT(s, si, length, c);
    if (IsRtl(c)) {
      return true;
    }
  }
  return false;
}

std::vector<Chunk> GetChunks(absl::string_view text, bool force_non_separable) {
  std::vector<Chunk> chunks;

  // Skia doesn't support rendering mixed direction text, so don't combine
  // mixed direction text into a single chunk for Desktop.
  // TODO: (broken link) - Fixed mixed direction text rendering on Desktop.
  if (
#if !IMP_PLATFORM(DESKTOP)
      IsMixedRtl(text) ||
#endif
      force_non_separable) {
    chunks.push_back(Chunk{.chunk_text = std::string(text),
                           .codepoint_count = 1,
                           .is_separable = false});
    return chunks;
  }
  // Split up the text into runs of separable and non-separable texts. For the
  // sections that are non-separable, treat and render it as if it were a
  // single glyph chunk.
  // NOTE: This routine depends on text NOT being empty, which is ensured at
  // the beginning of this function.
  assert(!text.empty());
  const uint8_t* s = reinterpret_cast<const uint8_t*>(text.data());
  int32_t length = text.length();
  int32_t si = 0;
  UChar32 c, peek;
  // U8_NEXT post increments si, so when peeking we call U8_GET on the same si
  // value.
  U8_NEXT(s, si, length, c);
  U8_GET(s, 0, si, length, peek);
  bool separable_chunk = IsSeparable(c) && !IsEmojiSequenceModifier(peek);
  int32_t chunk_start = 0;
  int32_t chunk_end = 0;
  while (chunk_end < length) {
    chunk_end = si;
    if (separable_chunk) {
      int codepoint_count = 1;
      while (si < length) {
        U8_NEXT(s, si, length, c);
        U8_GET(s, 0, si, length, peek);
        //  If the character is not separable, then stop. However, if the
        //  character is separable, and the next character is an emoji sequence
        //  modifier, then the character before it should also be not separable.
        if (!IsSeparable(c) || IsEmojiSequenceModifier(peek)) {
          break;
        }
        codepoint_count++;
        chunk_end = si;
      }
      absl::string_view substr =
          text.substr(chunk_start, chunk_end - chunk_start);
      chunks.push_back(Chunk{.chunk_text = std::string(substr),
                             .codepoint_count = codepoint_count,
                             .is_separable = true});

    } else {
      while (si < length) {
        U8_NEXT(s, si, length, c);
        U8_GET(s, 0, si, length, peek);
        //  If the character is separable, then stop. However, if the character
        //  following it is an emoji sequence modifier, then the current
        //  character should be treated as not separable to account for ZWJ
        //  emoji sequences and emoji variation selectors.
        if (IsSeparable(c) && !IsEmojiSequenceModifier(peek)) {
          break;
        }
        chunk_end = si;
      }
      absl::string_view substr =
          text.substr(chunk_start, chunk_end - chunk_start);
      chunks.push_back(Chunk{.chunk_text = std::string(substr),
                             .codepoint_count = 1,
                             .is_separable = false});
    }
    separable_chunk = !separable_chunk;
    chunk_start = chunk_end;
  }
  return chunks;
}

#if !IMP_PLATFORM(DESKTOP)

bool IsDirectionless(UChar32 c) { return c == 0x0020; }

bool IsMixedRtl(absl::string_view text) {
  const char* s = text.data();
  int32_t length = text.length();
  int32_t si = 0;
  bool contains_rtl = false;
  bool contains_ltr = false;
  while (si < length) {
    UChar32 c;
    U8_NEXT(s, si, length, c);
    if (IsRtl(c)) {
      contains_rtl = true;
    } else if (!IsDirectionless(c)) {
      contains_ltr = true;
    }
    if (contains_rtl && contains_ltr) {
      return true;
    }
  }
  return false;
}

#endif

}  // namespace imp
