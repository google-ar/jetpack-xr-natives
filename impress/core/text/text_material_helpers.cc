// Copyright 2026 Google LLC
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

#include "core/text/text_material_helpers.h"

#include "core/view/utils/proto/view_config.proto.imp.h"

namespace imp {

bool UseSlicedMaterial(const imp::ViewConfig& view_config) {
  if (view_config.glyph_atlas_texture_size ==
      imp::ViewConfig::GlyphAtlasTextureSize::SIZE256X256X8X8) {
    return true;
  }
  if (view_config.experimental_feature_flags.Value()
          .glyph_atlas_use_bitmap_surface_provider.Value()) {
    return true;
  }
  return false;
}

}  // namespace imp
