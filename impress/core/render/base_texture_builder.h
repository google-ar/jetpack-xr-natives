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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_BUILDER_BASE_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_BUILDER_BASE_H_

#include <cstdint>
#include <functional>

#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/image/image_contents.h"

namespace imp {

// An interface for building a filament::Texture* to support SplitEngine.
// The main concrete implementation is TextureBuilder.
class BaseTextureBuilder {
 public:
  virtual ~BaseTextureBuilder() = default;
  virtual BaseTextureBuilder& Width(uint32_t width) = 0;
  virtual BaseTextureBuilder& Height(uint32_t height) = 0;
  virtual BaseTextureBuilder& Levels(uint8_t levels) = 0;
  virtual BaseTextureBuilder& Format(
      filament::backend::TextureFormat format) = 0;
  virtual BaseTextureBuilder& Sampler(
      filament::backend::SamplerType sampler) = 0;
  BaseTextureBuilder& Image(filament::Engine& engine,
                            image::ImageContents& image_contents,
                            std::function<void()> callback,
                            int32_t* out_levels = nullptr) {
    return ImageInternal(engine, image_contents, callback, out_levels);
  }
  virtual BaseTextureBuilder& GenerateMipmaps(filament::Engine& engine) = 0;
  virtual BaseTextureBuilder& Name(absl::string_view name) = 0;
  virtual void Finalize(filament::Texture* texture) = 0;

 protected:
  virtual BaseTextureBuilder& ImageInternal(
      filament::Engine& engine, image::ImageContents& image_contents,
      std::function<void()> callback, int32_t* out_levels) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_BUILDER_BASE_H_
