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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_REGISTRY_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_REGISTRY_H_

#include <cstddef>
#include <string>

#include "absl/base/attributes.h"
#include "absl/strings/string_view.h"
#include "core/common/small_source_location.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

namespace imp {

// Registers textures by name so that they can be accessed later.
//
// Generally, this is used to registered dynamically created textures (i.e.
// CanvasSource or a render pass) so that they can be accessed in a component by
// name.
//
// The lifetime of a registered texture is controlled by the
// ScopedTextureRegistration returned when calling Register.
//
class TextureRegistry {
 public:
  // Controls the lifetime of a registered texture.
  //
  // ScopedTextureRegistration is move-only, and when it goes out of scope, the
  // registered texture is removed.
  //
  // Similar to Dispatcher::ScopedConnection.
  class ABSL_MUST_USE_RESULT ScopedTextureRegistration {
   public:
    ScopedTextureRegistration(TextureRegistry& texture_registry,
                              absl::string_view texture_name);
    ~ScopedTextureRegistration();

    ScopedTextureRegistration(const ScopedTextureRegistration&) = delete;
    ScopedTextureRegistration(ScopedTextureRegistration&& other);

    ScopedTextureRegistration& operator=(const ScopedTextureRegistration&) =
        delete;
    ScopedTextureRegistration& operator=(ScopedTextureRegistration&& rhs);

    ABSL_DEPRECATED("Use BorrowTexture instead.")
    // Returns the registered texture that this registration is controlling.
    Texture* GetTexture();

    // Returns the texture registered and owned by this TextureRegistry.
    BorrowedTexturePtr BorrowTexture(
        SmallSourceLocation loc = SmallSourceLocation::Current());

    // Releases ownership of the texture managed by this object.
    //
    // Prefer using `BorrowTexture` instead unless you are absolutely sure you
    // need to manually manage the lifetime of the texture.
    OwnedTexturePtr Release();

   private:
    TextureRegistry* texture_registry_;
    std::string texture_name_;
  };

  explicit TextureRegistry(BaseView* view);

  // Registers a texture by the name passed in.
  //
  // Calling this method transfers ownership of the texture to the registry.
  //
  // The lifetime of the texture is tied to the lifetime of the
  // ScopedTextureRegistration object returned.
  ScopedTextureRegistration RegisterTexture(absl::string_view texture_name,
                                            TexturePtr texture);
  ScopedTextureRegistration RegisterTexture(absl::string_view texture_name,
                                            OwnedTexturePtr texture);

  ABSL_DEPRECATED("Use BorrowTexture instead.")
  // Gets the texture registered by the name passed in. If there is no texture
  // registered with the given name, returns nullptr.
  Texture* GetTexture(absl::string_view texture_name);

  // Gets the texture registered by the name passed in. If there is no texture
  // registered with the given name, returns nullptr.
  BorrowedTexturePtr BorrowTexture(
      absl::string_view texture_name,
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Gets the total number of textures that are registered.
  size_t GetTextureCount() const;

 private:
  void UnregisterTexture(absl::string_view texture_name);

  BaseView* view_;
  StringMap<OwnedTexturePtr> registered_textures_;

  friend class ScopedTextureRegistration;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_REGISTRY_H_
