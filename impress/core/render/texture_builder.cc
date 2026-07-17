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

#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/assets/asset_ptr.h"
#include "core/image/image_contents.h"
#include "core/render/image_asset.h"
#include "core/render/safe_filament_texture_builder.h"
#include "core/view/base_view.h"

namespace imp {

TextureBuilder::TextureBuilder(BaseView& view) noexcept
    : view_(&view), spy_(nullptr) {
  builder_ = SafeFilamentTextureBuilder();

  if (auto* serializer = view_->GetSplitEngineSerializer()) {
    spy_ = serializer->CreateTextureBuilder();
  }
}

TextureBuilder::TextureBuilder(TextureBuilder&& rhs) noexcept
    : view_(rhs.view_),
      spy_(std::move(rhs.spy_)),
      builder_(std::move(rhs.builder_)),
      texture_(std::move(rhs.texture_)),
      name_(std::move(rhs.name_)),
      image_assets_(std::move(rhs.image_assets_)),
      images_(std::move(rhs.images_)),
      generate_mipmaps_(rhs.generate_mipmaps_) {}

TextureBuilder& TextureBuilder::operator=(TextureBuilder&& rhs) noexcept {
  view_ = rhs.view_;
  spy_ = std::move(rhs.spy_);
  builder_ = std::move(rhs.builder_);
  texture_ = std::move(rhs.texture_);
  name_ = std::move(rhs.name_);
  image_assets_ = std::move(rhs.image_assets_);
  images_ = std::move(rhs.images_);
  generate_mipmaps_ = rhs.generate_mipmaps_;
  return *this;
}

TextureBuilder& TextureBuilder::Width(uint32_t width) {
  
  if (spy_) {
    spy_->Width(width);
  }
  builder_->width(width);
  return *this;
}
TextureBuilder& TextureBuilder::Height(uint32_t height) {
  
  if (spy_) {
    spy_->Height(height);
  }
  builder_->height(height);
  return *this;
}

TextureBuilder& TextureBuilder::Depth(uint32_t depth) {
  
  if (spy_) {
    spy_->Depth(depth);
  }
  builder_->depth(depth);
  return *this;
}

TextureBuilder& TextureBuilder::Levels(uint8_t levels) {
  
  if (spy_) {
    spy_->Levels(levels);
  }
  builder_->levels(levels);
  return *this;
}
TextureBuilder& TextureBuilder::Format(
    filament::backend::TextureFormat format) {
  
  if (spy_) {
    spy_->Format(format);
  }
  builder_->format(format);
  return *this;
}

TextureBuilder& TextureBuilder::Usage(filament::backend::TextureUsage usage) {
  
  if (spy_) {
    spy_->Usage(usage);
  }
  builder_->usage(usage);
  return *this;
}

TextureBuilder& TextureBuilder::Sampler(
    filament::backend::SamplerType sampler) {
  
  if (spy_) {
    spy_->Sampler(sampler);
  }
  builder_->sampler(sampler);
  return *this;
}

TextureBuilder& TextureBuilder::Name(absl::string_view name) {
  
  if (spy_) {
    spy_->Name(name);
  }

  name_ = name;
  builder_->name(name_.data(), name_.length());
  return *this;
}

TextureBuilder& TextureBuilder::Swizzle(filament::backend::TextureSwizzle r,
                                        filament::backend::TextureSwizzle g,
                                        filament::backend::TextureSwizzle b,
                                        filament::backend::TextureSwizzle a) {
  
  if (spy_) {
    spy_->Swizzle(r, g, b, a);
  }
  builder_->swizzle(r, g, b, a);
  return *this;
}

TextureBuilder& TextureBuilder::External() {
  
  if (spy_) {
    spy_->External();
  }
  builder_->external();
  return *this;
}

TextureBuilder& TextureBuilder::ImageInternal(filament::Engine& engine,
                                              AssetPtr<ImageAsset> image,
                                              int image_index) {
  
  if (spy_) {
    spy_->Image(engine, image, image_index);
    // The images will be moved, so the normal setImage API should be skipped.
    return *this;
  }

  if (!images_.empty()) {
    IMP_LOG(imp::WARNING) << "Multiple image data sources are not supported. "
                    "Overwriting previous image data sources.";
    // Remove other sources of images.
    images_.clear();
  }

  image_assets_.push_back({std::move(image), image_index});

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

  if (!image_assets_.empty()) {
    IMP_LOG(imp::WARNING) << "Multiple image data sources are not supported. "
                    "Overwriting previous image data sources.";
    // Remove other sources of images.
    image_assets_.clear();
  }

  images_ = image_contents.CreatePixelBufferDescriptorLevels(callback, false);
  if (out_levels) {
    *out_levels = images_.size();
  }

  return *this;
}

TextureBuilder& TextureBuilder::GenerateMipmaps(filament::Engine& engine) {
  
  if (spy_) {
    spy_->GenerateMipmaps(engine);
  }

  generate_mipmaps_ = true;
  return *this;
}

TextureBuilder& TextureBuilder::Import(intptr_t id) {
  
  if (spy_) {
    spy_->Import(id);
  }
  builder_->import(id);
  return *this;
}

filament::Texture* TextureBuilder::Build(filament::Engine& engine) {
  
  texture_ = builder_->build(engine);
  builder_.reset();

  if (!texture_.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to create texture, \"" << name_
               << "\": " << texture_.status().ToString();
    return nullptr;
  }

  if (!image_assets_.empty()) {
    for (auto& asset : image_assets_) {
      auto descriptors = asset.image->GetLevelDescriptors();
      for (int level = 0; level < descriptors.size(); ++level) {
        (*texture_)->setImage(
            engine, level, 0, 0, asset.index, (*texture_)->getWidth(level),
            (*texture_)->getHeight(level), 1, std::move(descriptors[level]));
      }
    }
  } else if (!images_.empty()) {
    for (int level = 0; level < images_.size(); ++level) {
      // Note: filament::Texture::setImage() can panic
      (*texture_)->setImage(engine, level, std::move(images_[level]));
    }
  }

  if (generate_mipmaps_) {
    (*texture_)->generateMipmaps(engine);
  }

  if (spy_) {
    spy_->Finalize(*texture_);
  }

  return *texture_;
}

void TextureBuilder::Finalize(filament::Texture* texture) {
  // Do nothing. This is designed to be overridden by SplitEngineTextureBuilder.
}

}  // namespace imp
