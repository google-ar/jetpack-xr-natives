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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_CONSTANTS_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_CONSTANTS_H_

#include <cstdint>

namespace imp {

// Determines how text is laid out horizontally when DrawText is called.
enum class TextHorizontalAlignment : uint8_t {
  // Aligned normally to the left of the text fill.
  kLeft = 0,
  // Aligned normally to the center of the text.
  kCenter = 1,
  // Aligned normally to the right of the text fill.
  kRight = 2,
  // The x position passed into DrawText is (as close as possible to) the
  // exact leftmost pixel of the given text, accounting for both its stroke
  // and the initial left offset of the first character. May not be exact on
  // all platforms. Matches the behavior of DrawGlyph.
  kLeftExtent = 3,
  // The x position passed into DrawText is (as close as possible to) the
  // right-most pixel of this specific text, accounting for both its stroke
  // and the last right offset of the last character. May not be exact on all
  // platforms.
  kRightExtent = 4,
};

// Determines how text is laid out vertically when DrawText is called.
// Additional information about this can be found here:
// https://proandroiddev.com/android-and-typography-101-5f06722dd611
enum class TextVerticalAlignment : uint8_t {
  // The y position passed into DrawText is the top of the text.
  kAscent = 0,
  // The y position passed into DrawText is the center of the text.
  kCenter = 1,
  // The y position passed into DrawText is the baseline of the text. Some
  // characters will extend down below this.
  kBaseline = 2,
  // The y position passed into DrawText is the bottom of the text.
  // There may be some visual space between the bottom and where the text
  // actually starts depending on what characters are drawn, since the descent
  // is the lowest point of the lowest glyph.
  kDescent = 3,
  // The y position passed into DrawText is (as close as possible to) the
  // highest pixel of the given text, accounting for its stroke. May not be
  // exact on all platforms. Matches the behavior of DrawGlyph.
  kTopExtent = 4,
  // The y position passed into DrawText is (as close as possible to) the
  // lowest pixel of this specific text, accounting for its stroke. May not be
  // exact on all platforms.
  kBottomExtent = 5,
  // The y position passed into DrawText is (as close as possible to) the
  // topmost pixel of the given text, accounting for its stroke, and
  // additionally generally aligned to the "height" of the font.
  //
  // Clients of Impress probably shouldn't use this.
  //
  // See also TextMetrics::font_origin_y and TextMetrics::font_size_y.
  kAtlas = 6,
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_CONSTANTS_H_
