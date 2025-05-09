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

#include "core/render/texture_builder.h"

#include <cassert>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/image/image_contents.h"
#include "core/view/base_view.h"

namespace imp {

TextureBuilder::TextureBuilder(BaseView& view) noexcept
    : view_(&view),
      spy_(nullptr),
      builder_(filament::Texture::Builder()),
      texture_(nullptr) {
  if (auto* serializer = view_->GetSplitEngineSerializer()) {
    spy_ = serializer->CreateTextureBuilder();
  }
}

TextureBuilder::TextureBuilder(TextureBuilder&& rhs) noexcept
    : view_(rhs.view_),
      spy_(std::move(rhs.spy_)),
      builder_(std::move(rhs.builder_)),
      texture_(rhs.texture_) {}

TextureBuilder& TextureBuilder::operator=(TextureBuilder&& rhs) noexcept {
  view_ = rhs.view_;
  spy_ = std::move(rhs.spy_);
  builder_ = std::move(rhs.builder_);
  texture_ = rhs.texture_;
  return *this;
}

TextureBuilder& TextureBuilder::Width(uint32_t width) {
  assert(!texture_);

  if (spy_) {
    spy_->Width(width);
  }
  builder_.width(width);
  return *this;
}
TextureBuilder& TextureBuilder::Height(uint32_t height) {
  assert(!texture_);

  if (spy_) {
    spy_->Height(height);
  }
  builder_.height(height);
  return *this;
}
TextureBuilder& TextureBuilder::Levels(uint8_t levels) {
  assert(!texture_);

  if (spy_) {
    spy_->Levels(levels);
  }
  builder_.levels(levels);
  return *this;
}
TextureBuilder& TextureBuilder::Format(
    filament::backend::TextureFormat format) {
  assert(!texture_);

  if (spy_) {
    spy_->Format(format);
  }
  builder_.format(format);
  return *this;
}
TextureBuilder& TextureBuilder::Sampler(
    filament::backend::SamplerType sampler) {
  assert(!texture_);

  if (spy_) {
    spy_->Sampler(sampler);
  }
  builder_.sampler(sampler);
  return *this;
}

TextureBuilder& TextureBuilder::Name(absl::string_view name) {
  assert(!texture_);

  if (spy_) {
    spy_->Name(name);
  }

  name_ = name;
  builder_.name(name_.data(), name_.length());
  return *this;
}

TextureBuilder& TextureBuilder::ImageInternal(
    filament::Engine& engine, image::ImageContents& image_contents,
    std::function<void()> callback, int32_t* out_levels) {
  if (spy_) {
    spy_->Image(engine, image_contents, callback, out_levels);
    // The images will be moved, so the normal setImage API should be skipped.
    return *this;
  }

  if (!texture_) {
    texture_ = builder_.build(engine);
  }

  std::vector<filament::backend::PixelBufferDescriptor> images =
      image_contents.CreatePixelBufferDescriptorLevels(callback, false);
  for (int level = 0; level < images.size(); ++level) {
    texture_->setImage(engine, level, std::move(images[level]));
  }
  if (out_levels) {
    *out_levels = images.size();
  }

  return *this;
}

TextureBuilder& TextureBuilder::GenerateMipmaps(filament::Engine& engine) {
  if (spy_) {
    spy_->GenerateMipmaps(engine);
  }
  if (!texture_) {
    texture_ = builder_.build(engine);
  }
  texture_->generateMipmaps(engine);
  return *this;
}

filament::Texture* TextureBuilder::Build(filament::Engine& engine) {
  if (!texture_) {
    texture_ = builder_.build(engine);
  }
  if (spy_) {
    spy_->Finalize(texture_);
  }
  return texture_;
}

void TextureBuilder::Finalize(filament::Texture* texture) {
  // Do nothing. This is designed to be overridden by SplitEngineTextureBuilder.
}

}  // namespace imp
