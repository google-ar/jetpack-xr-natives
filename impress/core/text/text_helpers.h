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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_HELPERS_H_

#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "unicode/utypes.h"
#include "core/config.h"

namespace imp {

// A chunk of text that can be broken up into glyphs.
struct Chunk {
  // The text of the chunk
  std::string chunk_text;

  // The count of codepoints within the given text or 1 if the chunk is
  // not separable.
  int codepoint_count;

  // Whether the chunk can be divided into multiple glyphs. If false, the
  // entire chunk is treated as one glyph.
  bool is_separable;
};

// Returns true if the string contains non-separable script. Note that this
// doesn't check if it contains an emoji, in which this should be used in
// conjunction with ContainsEmoji.
bool ContainsNonSeparableScript(absl::string_view text);

// Returns true if the character can be correctly rendered on its own without
// the context of its neighbors. Characters that are part of required ligatures
// (as in Arabic and Hindi) would fail this test, while latin characters can be
// rendered independently and placed next to each other to get the same
// appearance as rendering them together.
bool IsSeparable(UChar32 c);

// Returns true if the character is a modifier for an emoji sequence. This can
// be ZWJ emoji sequences or emoji variation selectors. These generally join or
// modify the prior character(s).
bool IsEmojiSequenceModifier(UChar32 c);

// Returns true if the character is an emoji. This function is used on platforms
// that don't support glyphs (wasm/desktop). The icu library inflates binary
// size significantly so we opt for a native solution for wasm.
bool IsEmoji(UChar32 c);
// Returns true if the string contains an emoji.
bool ContainsEmoji(absl::string_view text);

// Returns true if the character is rendered right-to-left.
bool IsRtl(UChar32 c);

// Returns true if the string should be rendered right-to-left.
bool ContainsRtl(absl::string_view text);

// Returns the chunks of adjacent codepoints within text that have the same
// separability requirement. This is expected to be used on platforms that
// don't support glyphs (WASM/Desktop).
std::vector<Chunk> GetChunks(absl::string_view text, bool force_non_separable);

#if !IMP_PLATFORM(DESKTOP)

// Returns true if the character has no inherent ltr/rtl direction. This
// leans toward false negatives by only considering space characters as
// directionless. Several other punctuation characters are directionless but
// it is difficult to define a comprehensive unicode range for them.
bool IsDirectionless(UChar32 c);

// Returns true if the text contains both rtl and ltr characters.
bool IsMixedRtl(absl::string_view text);

#endif

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_HELPERS_H_
