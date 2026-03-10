/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_GLYPHS_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_GLYPHS_H_

#include <cstddef>
#include <cstdint>

#include "absl/types/span.h"
#include "core/common/ref_counter.h"
#include "core/common/typed_id.h"
#include "core/common/typed_set_vector.h"
#include "core/math/vec.h"

namespace imp {

struct TextGlyph {
  enum Fields {
    // Used to track how many references to this glyph there are.
    kGlyphRef,
    // The atlas origin (xy) and size (zw).
    kAtlasOriginAndSize,
    // The actual origin (xy) and size (zw).
    kActualOriginAndSize,
    // The UV origin (xy) and size (zw).
    kUVOriginAndSize,
    // The advance width of the glyph.
    kAdvanceWidth,
  };

  using ArrayType = StructureOfArrays<RefCounter::Ref,  // kGlyphRef
                                      float4,           // kAtlasOriginAndSize
                                      float4,           // kActualOriginAndSize
                                      float4,           // kUVOriginAndSize
                                      float             // kAdvanceWidth
                                      >;
  union Proxy {
    template <size_t E>
    using Field = ArrayType::Field<E>;

    // All union members have an identical storage type.
    Field<kGlyphRef> glyph_ref;
    Field<kAtlasOriginAndSize> atlas_origin_and_size;
    Field<kActualOriginAndSize> actual_origin_and_size;
    Field<kUVOriginAndSize> uv_origin_and_size;
    Field<kAdvanceWidth> advance_width;
  };
};
using TextGlyphId = TypedId<TextGlyph, uint32_t>;

// Information about glyphs needed to render them and lay out relative to
// other glyphs in a string.
class TextGlyphs {
 public:
  TextGlyphs() = default;

  // Movable but not copyable
  TextGlyphs(TextGlyphs&& rhs) noexcept = default;
  TextGlyphs& operator=(TextGlyphs&& rhs) noexcept = default;

  TextGlyphs(TextGlyphs const& rhs) = delete;
  TextGlyphs& operator=(TextGlyphs const& rhs) = delete;

  bool IsEmpty() const { return text_glyph_arrays_.empty(); }
  void Reserve(size_t size) { text_glyph_arrays_.reserve(size); }
  size_t Size() const { return text_glyph_arrays_.size(); }
  void PushBack(RefCounter::Ref glyph_ref, float4 atlas_origin_and_size,
                float4 actual_origin_and_size, float4 uv_origin_and_size,
                float advance_width) {
    text_glyph_arrays_.push_back(glyph_ref, atlas_origin_and_size,
                                 actual_origin_and_size, uv_origin_and_size,
                                 advance_width);
  }

  absl::Span<const float> AdvanceWidths() const {
    return text_glyph_arrays_.RawSpan<TextGlyph::kAdvanceWidth>();
  }

  absl::Span<const float4> ActualOriginsAndSizes() const {
    return text_glyph_arrays_.RawSpan<TextGlyph::kActualOriginAndSize>();
  }

  absl::Span<const float4> AtlasOriginsAndSizes() const {
    return text_glyph_arrays_.RawSpan<TextGlyph::kAtlasOriginAndSize>();
  }

  absl::Span<const float4> UVOriginsAndSizes() const {
    return text_glyph_arrays_.RawSpan<TextGlyph::kUVOriginAndSize>();
  }

 private:
  // Structure of Arrays that contains all the information about each glyph
  // needed to render and lay out the text.
  TypedSetVector<TextGlyph> text_glyph_arrays_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_GLYPHS_H_
