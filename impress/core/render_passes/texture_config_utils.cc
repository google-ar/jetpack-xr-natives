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

#include "core/render_passes/texture_config_utils.h"

#include "core/common/log.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/render_passes/texture_config.proto.imp.h"

namespace imp {
filament::Texture::InternalFormat FormatFromTexture(
    TextureConfig::Format format, TextureConfig::Format auto_format) {
  if (format == TextureConfig::AUTO) {
    format = auto_format;
  }

  switch (format) {
    case TextureConfig::RGBA8:
      return filament::Texture::InternalFormat::RGBA8;
    case TextureConfig::RGB8:
      return filament::Texture::InternalFormat::RGB8;
    case TextureConfig::RG8:
      return filament::Texture::InternalFormat::RG8;
    case TextureConfig::R8:
      return filament::Texture::InternalFormat::R8;
    case TextureConfig::RGBA32F:
      return filament::Texture::InternalFormat::RGBA32F;
    case TextureConfig::R32F:
      return filament::Texture::InternalFormat::R32F;
    case TextureConfig::RGBA16F:
      return filament::Texture::InternalFormat::RGBA16F;
    case TextureConfig::DEPTH24:
      return filament::Texture::InternalFormat::DEPTH24;
    case TextureConfig::DEPTH32F:
      return filament::Texture::InternalFormat::DEPTH32F;
    default:
      // This should never happen.
      IMP_LOG(imp::FATAL) << "Unsupported texture format in TexturePipelineRenderer.";
      return filament::Texture::InternalFormat::RGBA8;
  }
}
}  // namespace imp
