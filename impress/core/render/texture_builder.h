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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_BUILDER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_BUILDER_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "absl/base/nullability.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/image/image_contents.h"
#include "core/render/base_texture_builder.h"
#include "core/render/safe_filament_texture_builder.h"
#include "core/view/base_view.h"

namespace imp {

// A wrapper for filament::Texture::Builder for SplitEngine to intercept data.
//
// Note: Width(), Height(), Levels(), Format(), and Sampler() must all be called
// prior to calling Image() or GenerateMipmaps() - this is because the Filament
// API actually has setImage() and generateMipmaps() on filament::Texture* but
// the SplitEngine wrapper can't allow that - we would have to introduce a
// wrapper for filament::Texture* that forwards to SplitEngine as well, which is
// additional boilerplate and complexity.
// TODO: Move the SplitEngine TextureBuilder spy to TextureFactory.
class TextureBuilder : public BaseTextureBuilder {
 public:
  explicit TextureBuilder(BaseView& view) noexcept;
  TextureBuilder(TextureBuilder const& rhs) noexcept = delete;
  TextureBuilder(TextureBuilder&& rhs) noexcept;
  TextureBuilder& operator=(TextureBuilder const& rhs) noexcept = delete;
  TextureBuilder& operator=(TextureBuilder&& rhs) noexcept;

  TextureBuilder& Width(uint32_t width) override;
  TextureBuilder& Height(uint32_t height) override;
  TextureBuilder& Levels(uint8_t levels) override;
  TextureBuilder& Format(filament::backend::TextureFormat format) override;
  TextureBuilder& Sampler(filament::backend::SamplerType sampler) override;
  TextureBuilder& GenerateMipmaps(filament::Engine& engine) override;
  TextureBuilder& Name(absl::string_view name) override;
  void Finalize(filament::Texture* texture) override;

  filament::Texture* /*absl_nullable*/  Build(filament::Engine& engine);

 protected:
  TextureBuilder& ImageInternal(filament::Engine& engine,
                                image::ImageContents& image_contents,
                                std::function<void()> callback,
                                int32_t* out_levels) override;

 private:
  BaseView* view_;
  std::unique_ptr<BaseTextureBuilder> spy_;
  SafeFilamentTextureBuilder builder_;
  absl::StatusOr<filament::Texture* /*absl_nonnull*/ > texture_;

  std::string name_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_BUILDER_H_
