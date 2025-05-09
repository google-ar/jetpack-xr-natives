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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_H_

#include <cstdint>
#include <memory>
#include <string>

#include "absl/base/attributes.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Stream.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/common/invocable.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_ptr.h"
#include "core/math/vec.h"
#include "core/render/content_security_level.h"
#include "core/view/base_view.h"

namespace imp {

class TextureFactory;

// A wrapper around a filament::Texture that automatically cleans it up. These
// shouldn't be created directly, but instead created from the TextureFactory
// and accessed using TexturePtr.
class Texture {
 public:
  ~Texture();

  // Returns the name of the texture.
  //
  // Automatically set from the name of the ImageAsset if the Texture is created
  // from an ImageAsset. Otherwise, it can be set manually.
  absl::string_view GetName() const;

  // Sets the name of the texture.
  void SetName(absl::string_view name);

  // Returns the underlying filament::Texture*.
  filament::Texture* GetTexture() const;

  // Returns the underlying filament::TextureSampler.
  const filament::TextureSampler& GetSampler() const;

  // Returns the underlying filament::Stream*.
  filament::Stream* GetStream() const;

  uint2 GetSize() const;

  // Returns true if the texture is valid.
  bool IsValid() const;

  // Returns the content security level of the texture.
  ContentSecurityLevel GetContentSecurityLevel() const;

 private:
  Texture(BaseView& view, filament::Stream* stream, filament::Texture* texture,
          const filament::TextureSampler& sampler,
          ContentSecurityLevel security_level = ContentSecurityLevel::kNone);

  BaseView& view_;
  filament::Stream* stream_;
  filament::Texture* texture_;
  filament::TextureSampler sampler_;
  std::string name_;
  ContentSecurityLevel security_level_;

  friend class TextureFactory;
};

// For now, we only support move semantics and single ownership.
using TexturePtr ABSL_DEPRECATED(
    "Prefer using OwnedTexturePtr instead. See "
    "(broken link).") = std::unique_ptr<Texture>;

// Track lifetime of textures using OwnedPtr and BorrowedPtr.
using OwnedTexturePtr = OwnedPtr<Texture>;
using BorrowedTexturePtr = BorrowedPtr<Texture>;
using OwnedOrBorrowedTexturePtr = OwnedOrBorrowedPtr<Texture>;
// TODO: Pull this out to a separate header and have it with
// TextureProvider.
using TextureBorrower = Invocable<BorrowedTexturePtr(uint64_t)>;
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_H_
