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

#include "core/render/texture_registry.h"

#include <cstddef>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "core/common/small_source_location.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"

namespace imp {

TextureRegistry::ScopedTextureRegistration::ScopedTextureRegistration(
    TextureRegistry& texture_registry, absl::string_view texture_name)
    : texture_registry_(&texture_registry), texture_name_(texture_name) {}

TextureRegistry::ScopedTextureRegistration::~ScopedTextureRegistration() {
  if (texture_registry_) {
    texture_registry_->UnregisterTexture(texture_name_);
  }
}

Texture* TextureRegistry::ScopedTextureRegistration::GetTexture() {
  if (!texture_registry_) {
    return nullptr;
  }

  return texture_registry_->GetTexture(texture_name_);
}

BorrowedTexturePtr TextureRegistry::ScopedTextureRegistration::BorrowTexture(
    SmallSourceLocation loc) {
  if (!texture_registry_) {
    return BorrowedTexturePtr();
  }

  return texture_registry_->BorrowTexture(texture_name_, loc);
}

OwnedTexturePtr TextureRegistry::ScopedTextureRegistration::Release() {
  if (!texture_registry_) {
    return OwnedTexturePtr();
  }

  if (!texture_registry_->registered_textures_.contains(texture_name_)) {
    IMP_LOG(imp::FATAL) << "Cannot release texture that has already been released: "
               << texture_name_;
  }

  OwnedTexturePtr texture =
      std::move(texture_registry_->registered_textures_[texture_name_]);
  texture_registry_->UnregisterTexture(texture_name_);
  return texture;
}

TextureRegistry::ScopedTextureRegistration::ScopedTextureRegistration(
    ScopedTextureRegistration&& other) {
  texture_registry_ = other.texture_registry_;
  texture_name_ = std::move(other.texture_name_);
  other.texture_registry_ = nullptr;
}

TextureRegistry::ScopedTextureRegistration&
TextureRegistry::ScopedTextureRegistration::operator=(
    ScopedTextureRegistration&& rhs) {
  if (texture_registry_) {
    texture_registry_->UnregisterTexture(texture_name_);
  }

  texture_registry_ = rhs.texture_registry_;
  texture_name_ = std::move(rhs.texture_name_);
  rhs.texture_registry_ = nullptr;

  return *this;
}

TextureRegistry::TextureRegistry(BaseView* view) : view_(view) {}

TextureRegistry::ScopedTextureRegistration TextureRegistry::RegisterTexture(
    absl::string_view texture_name, TexturePtr texture) {
  if (registered_textures_.count(texture_name) > 0) {
    IMP_LOG(imp::FATAL) << "Cannot register already registered texture named "
               << texture_name;
  }

  registered_textures_[std::string(texture_name)] = std::move(texture);
  return ScopedTextureRegistration(*this, texture_name);
}

TextureRegistry::ScopedTextureRegistration TextureRegistry::RegisterTexture(
    absl::string_view texture_name, OwnedTexturePtr texture) {
  if (registered_textures_.count(texture_name) > 0) {
    IMP_LOG(imp::FATAL) << "Cannot register already registered texture named "
               << texture_name;
  }

  registered_textures_.emplace(std::string(texture_name), std::move(texture));
  return ScopedTextureRegistration(*this, texture_name);
}

Texture* TextureRegistry::GetTexture(absl::string_view texture_name) {
  auto itr = registered_textures_.find(texture_name);
  if (itr == registered_textures_.end()) {
    return nullptr;
  }

  return itr.value().operator->();
}

BorrowedTexturePtr TextureRegistry::BorrowTexture(
    absl::string_view texture_name, SmallSourceLocation loc) {
  auto itr = registered_textures_.find(texture_name);
  if (itr == registered_textures_.end()) {
    return BorrowedTexturePtr();
  }

  return itr.value().Borrow(loc);
}

void TextureRegistry::UnregisterTexture(absl::string_view texture_name) {
  registered_textures_.erase(texture_name);
}

size_t TextureRegistry::GetTextureCount() const {
  return registered_textures_.size();
}

}  // namespace imp
