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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_OPTIONS_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_OPTIONS_H_

#include <cstdint>
#include <optional>

#include "absl/strings/str_format.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"

namespace imp {

// Generation options are parameters to change the generation of textures
// loaded from the AssetManager.
struct TextureGenerationOptions {
  using Format = ::filament::Texture::InternalFormat;

  // Optionally specify the number of mipmap levels to generate at runtime.
  // If no value is given, mipmaps are not generated.
  std::optional<uint8_t> generated_mipmap_levels = {};

  // Optional override to the default format in the ImageAsset object.
  std::optional<Format> texture_format_override = {};

  template <typename Sink>
  friend void AbslStringify(Sink& sink,
                            const TextureGenerationOptions& options) {
    absl::Format(&sink, "(%hhu, %hu)",
                 options.generated_mipmap_levels.value_or(1),
                 options.texture_format_override.value_or(Format::SRGB8_A8));
  }
};

// Sampler options are parameters to change the sampling behavior of textures
// created by the TextureFactory.
struct TextureSamplerOptions {
  using MinFilter = filament::TextureSampler::MinFilter;
  using MagFilter = filament::TextureSampler::MagFilter;
  using WrapMode = filament::TextureSampler::WrapMode;
  using SamplerType = filament::Texture::Sampler;

  // Optional override to the default sampler type.
  std::optional<SamplerType> sampler_type = std::nullopt;

  // Determines how texture coordinates outside of [0,1] are handled.
  WrapMode wrap_mode = WrapMode::CLAMP_TO_EDGE;

  // Sampling method used when a texel covers multiple pixels.
  MagFilter mag_filter = MagFilter::LINEAR;

  // Sampling method used when a pixel covers multiple texels.
  MinFilter min_filter = MinFilter::LINEAR;

  // Adds extra texture samples to improve textures which are displayed at an
  // oblique angle to the camera. Should be a power-of-two. The default is 1.
  // The maximum permissible value is 128.
  float anisotropy = 1.0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_OPTIONS_H_
