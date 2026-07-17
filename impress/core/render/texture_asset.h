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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_ASSET_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_ASSET_H_

#include <cstddef>
#include <memory>
#include <string>

#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/config.h"
#include "core/image/image_contents.h"
#if IMP_PLATFORM(WASM)
#include "core/image/wasm_texture_contents.h"
#endif
#include "core/render/texture_options.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
namespace imp {

class TextureAsset {
 public:
  static Future<std::unique_ptr<TextureAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future,
      TextureGenerationOptions options);

  TextureAsset(BaseView* view, absl::string_view texture_name,
               std::unique_ptr<image::ImageContents> image_contents,
               TextureGenerationOptions options);

#if IMP_PLATFORM(WASM)
  TextureAsset(BaseView* view, absl::string_view texture_name,
               WasmTextureContents texture_contents,
               TextureGenerationOptions options);
#endif
  ~TextureAsset();

  absl::string_view GetName() const { return texture_name_; }

  filament::Texture* GetFilamentTexture() const { return texture_; }
  filament::Texture* ReleaseFilamentTexture();

  size_t GetWidth() const { return texture_->getWidth(); }
  size_t GetHeight() const { return texture_->getHeight(); }

 private:
  BaseView* view_;
  std::string texture_name_;
  filament::Texture* texture_;
#if IMP_PLATFORM(WASM)
  GLuint gl_texture_id_ = 0;
#endif
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_ASSET_H_
