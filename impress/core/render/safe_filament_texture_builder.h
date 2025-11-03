/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_SAFE_FILAMENT_TEXTURE_BUILDER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_SAFE_FILAMENT_TEXTURE_BUILDER_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "absl/base/nullability.h"
#include "absl/status/statusor.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/libs/utils/include/utils/StaticString.h"

namespace imp {

// This is drop-in replacement for filament::Texture::Builder that returns a
// StatusOr instead of crashing.
//
class SafeFilamentTextureBuilder {
  using Builder = SafeFilamentTextureBuilder;
  using PixelBufferDescriptor = filament::backend::PixelBufferDescriptor;
  using Sampler = filament::backend::SamplerType;
  using InternalFormat = filament::backend::TextureFormat;
  using CubemapFace = filament::backend::TextureCubemapFace;
  using Format = filament::backend::PixelDataFormat;
  using Type = filament::backend::PixelDataType;
  using CompressedType = filament::backend::CompressedPixelDataType;
  using Usage = filament::backend::TextureUsage;
  using Swizzle = filament::backend::TextureSwizzle;

 public:
  // Declared as non-defaulted, but default-implemented to maintain strict
  // `noexcept` requirement and replicate Filament's TextureBuilder API.
  // (broken link)
  SafeFilamentTextureBuilder() noexcept;
  SafeFilamentTextureBuilder(SafeFilamentTextureBuilder const& rhs) noexcept;
  SafeFilamentTextureBuilder(SafeFilamentTextureBuilder&& rhs) noexcept;
  ~SafeFilamentTextureBuilder() noexcept;
  SafeFilamentTextureBuilder& operator=(
      SafeFilamentTextureBuilder const& rhs) noexcept;
  SafeFilamentTextureBuilder& operator=(
      SafeFilamentTextureBuilder&& rhs) noexcept;

  // Being drop-in replacement requires to follow the same naming convention for
  // the methods
  Builder& width(uint32_t width) noexcept;
  Builder& height(uint32_t height) noexcept;
  Builder& depth(uint32_t depth) noexcept;
  Builder& levels(uint8_t levels) noexcept;
  Builder& sampler(Sampler target) noexcept;
  Builder& format(InternalFormat format) noexcept;
  Builder& usage(Usage usage) noexcept;
  Builder& swizzle(Swizzle r, Swizzle g, Swizzle b, Swizzle a) noexcept;
  Builder& name(const char* /*absl_nonnull*/  name, size_t len) noexcept;
  Builder& name(utils::StaticString const& name) noexcept;
  Builder& external() noexcept;
  Builder& import(intptr_t id) noexcept;

  absl::StatusOr<filament::Texture* /*absl_nonnull*/ > build(
      filament::Engine& engine) noexcept;

 private:
  filament::Texture::Builder builder_;

  uint32_t width_ = 1;
  uint32_t height_ = 1;
  uint32_t depth_ = 1;
  uint8_t levels_ = 1;
  Sampler sampler_ = Sampler::SAMPLER_2D;
  InternalFormat format_ = InternalFormat::RGBA8;
  Usage usage_ = Usage::NONE;
  bool is_swizzled_ = false;
  std::string name_;
  bool external_ = false;
  intptr_t id_ = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_SAFE_FILAMENT_TEXTURE_BUILDER_H_
