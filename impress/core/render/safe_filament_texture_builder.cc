// Copyright 2025 Google LLC
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

#include "core/render/safe_filament_texture_builder.h"

#include <cstddef>
#include <cstdint>
#include <string>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/libs/utils/include/utils/StaticString.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

// Declared as non-defaulted, but default-implemented to maintain strict
// `noexcept` requirement and replicate Filament's TextureBuilder API.
// (broken link)
SafeFilamentTextureBuilder::SafeFilamentTextureBuilder() noexcept = default;
SafeFilamentTextureBuilder::~SafeFilamentTextureBuilder() noexcept = default;
SafeFilamentTextureBuilder::SafeFilamentTextureBuilder(
    SafeFilamentTextureBuilder const& rhs) noexcept = default;
SafeFilamentTextureBuilder::SafeFilamentTextureBuilder(
    SafeFilamentTextureBuilder&& rhs) noexcept = default;
SafeFilamentTextureBuilder& SafeFilamentTextureBuilder::operator=(
    SafeFilamentTextureBuilder const& rhs) noexcept = default;
SafeFilamentTextureBuilder& SafeFilamentTextureBuilder::operator=(
    SafeFilamentTextureBuilder&& rhs) noexcept = default;

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::width(
    uint32_t width) noexcept {
  width_ = width;
  builder_.width(width);
  return *this;
}

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::height(
    uint32_t height) noexcept {
  height_ = height;
  builder_.height(height);
  return *this;
}

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::depth(
    uint32_t depth) noexcept {
  depth_ = depth;
  builder_.depth(depth);
  return *this;
}

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::levels(
    uint8_t levels) noexcept {
  levels_ = levels;
  builder_.levels(levels);
  return *this;
}

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::sampler(
    Sampler target) noexcept {
  sampler_ = target;
  builder_.sampler(target);
  return *this;
}

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::format(
    InternalFormat format) noexcept {
  format_ = format;
  builder_.format(format);
  return *this;
}

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::usage(
    Usage usage) noexcept {
  usage_ = usage;
  builder_.usage(usage);
  return *this;
}

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::swizzle(
    Swizzle r, Swizzle g, Swizzle b, Swizzle a) noexcept {
  is_swizzled_ = true;
  builder_.swizzle(r, g, b, a);
  return *this;
}

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::name(
    const char* /*absl_nonnull*/ name, size_t len) noexcept {
  name_ = std::string(name, len);
  builder_.name(name, len);
  return *this;
}

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::name(
    utils::StaticString const& name) noexcept {
  name_ = std::string(name.data(), name.size());
  builder_.name(name);
  return *this;
}

SafeFilamentTextureBuilder::Builder&
SafeFilamentTextureBuilder::external() noexcept {
  external_ = true;
  builder_.external();
  return *this;
}

SafeFilamentTextureBuilder::Builder& SafeFilamentTextureBuilder::import(
    intptr_t id) noexcept {
  id_ = id;
  builder_.import(id);
  return *this;
}

absl::StatusOr<filament::Texture* /*absl_nonnull*/>
SafeFilamentTextureBuilder::build(filament::Engine& engine) noexcept {
  // Replication of all checks in filament::Texture::Builder::build
  // without actually panicking

  if (sampler_ != Sampler::SAMPLER_EXTERNAL) {
    if (!filament::Texture::isTextureFormatSupported(engine, format_)) {
      return absl::InvalidArgumentError(
          "Texture format is not supported on this platform");
    }

    if (width_ == 0 || height_ == 0) {
      return absl::InvalidArgumentError("Texture has invalid dimensions");
    }
  }

  const bool is_protected_textures_supported =
      filament::Texture::isProtectedTexturesSupported(engine);
  const bool use_protected_memory =
      static_cast<bool>(usage_ & Usage::PROTECTED);

  if (use_protected_memory && !is_protected_textures_supported) {
    return absl::InvalidArgumentError(
        "Protected textures are not supported on this platform");
  }

  const size_t max_texture_dimension =
      filament::Texture::getMaxTextureSize(engine, sampler_);
  const size_t max_texture_depth =
      (sampler_ == Sampler::SAMPLER_2D_ARRAY ||
       sampler_ == Sampler::SAMPLER_CUBEMAP_ARRAY)
          ? filament::Texture::getMaxArrayTextureLayers(engine)
          : max_texture_dimension;
  if (width_ > max_texture_dimension || height_ > max_texture_dimension ||
      depth_ > max_texture_depth) {
    return absl::InvalidArgumentError("Texture dimensions out of range");
  }

  const auto validate_sampler_type = [&engine](Sampler sampler) {
    switch (sampler) {
      case Sampler::SAMPLER_2D:
      case Sampler::SAMPLER_CUBEMAP:
      case Sampler::SAMPLER_EXTERNAL:
        return absl::OkStatus();

      case Sampler::SAMPLER_3D:
      case Sampler::SAMPLER_2D_ARRAY:
        return engine.getActiveFeatureLevel() >=
                       filament::backend::FeatureLevel::FEATURE_LEVEL_1
                   ? absl::OkStatus()
                   : absl::InvalidArgumentError(absl::StrFormat(
                         "%d sampler type is not supported on this platform",
                         static_cast<uint32_t>(sampler)));
      case Sampler::SAMPLER_CUBEMAP_ARRAY:
        return engine.getActiveFeatureLevel() >=
                       filament::backend::FeatureLevel::FEATURE_LEVEL_2
                   ? absl::OkStatus()
                   : absl::InvalidArgumentError(absl::StrFormat(
                         "%d sampler type is not supported on this platform",
                         static_cast<uint32_t>(sampler)));
    }
    return absl::InvalidArgumentError("Invalid sampler type");
  };

  MP_RETURN_IF_ERROR(validate_sampler_type(sampler_));

  if (usage_ == Usage::NONE) {
    // Following logic from filament::Texture::Builder::build
    usage_ = Usage::DEFAULT;  // UPLOADABLE | SAMPLEABLE
    // Skipping all other usage updates that are done in
    // filament::Texture::Builder::build, because none of affect any panics.
  }

  const bool sampleable = static_cast<bool>(usage_ & Usage::SAMPLEABLE);
  const bool swizzled = is_swizzled_;
  const bool imported = id_ != 0;

#if defined(__EMSCRIPTEN__)
  if (swizzled) {
    return absl::InvalidArgumentError(
        "WebGL does not support texture swizzling.");
  }
#endif

  if (swizzled && !sampleable) {
    return absl::InvalidArgumentError("Swizzled texture must be SAMPLEABLE");
  }
  if (imported && !sampleable) {
    return absl::InvalidArgumentError("Imported texture must be SAMPLEABLE");
  }

  return builder_.build(engine);
}

}  // namespace imp
