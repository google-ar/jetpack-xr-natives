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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEXTURE_BUILDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEXTURE_BUILDER_H_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/assets/asset_ptr.h"
#include "core/image/image_contents.h"
#include "core/render/base_texture_builder.h"
#include "core/split_engine/split_engine_serializer.h"

namespace imp::split_engine {

class SplitEngineTextureBuilder : public BaseTextureBuilder {
 public:
  struct State {
    State() = default;
    State(State&& other) noexcept = default;
    State& operator=(State&& other) noexcept = default;
    State(const State&) = delete;
    State& operator=(const State&) = delete;

    ~State();

    uint64_t texture_id = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint8_t levels = 0;
    filament::backend::TextureFormat format =
        filament::backend::TextureFormat::RGB8;
    filament::Texture::Sampler sampler = filament::Texture::Sampler::SAMPLER_2D;
    bool mips = false;
    std::vector<filament::backend::PixelBufferDescriptor> image_descriptors;
    AssetPtr<ImageAsset> image;
    std::optional<std::string> name;

    bool finalized = false;
  };

  explicit SplitEngineTextureBuilder(
      SplitEngineSerializer& serializer) noexcept;
  SplitEngineTextureBuilder(SplitEngineTextureBuilder const& rhs) noexcept =
      delete;
  SplitEngineTextureBuilder(SplitEngineTextureBuilder&& rhs) noexcept;
  SplitEngineTextureBuilder& operator=(
      SplitEngineTextureBuilder const& rhs) noexcept = delete;
  SplitEngineTextureBuilder& operator=(
      SplitEngineTextureBuilder&& rhs) noexcept;

  SplitEngineTextureBuilder& Width(uint32_t width) override;
  SplitEngineTextureBuilder& Height(uint32_t height) override;
  SplitEngineTextureBuilder& Depth(uint32_t depth) override;
  SplitEngineTextureBuilder& Levels(uint8_t levels) override;
  SplitEngineTextureBuilder& Format(
      filament::backend::TextureFormat format) override;
  SplitEngineTextureBuilder& Usage(
      filament::backend::TextureUsage usage) override;
  SplitEngineTextureBuilder& Sampler(
      filament::backend::SamplerType sampler) override;
  SplitEngineTextureBuilder& GenerateMipmaps(filament::Engine& engine) override;
  SplitEngineTextureBuilder& Name(absl::string_view name) override;
  SplitEngineTextureBuilder& Swizzle(
      filament::backend::TextureSwizzle r, filament::backend::TextureSwizzle g,
      filament::backend::TextureSwizzle b,
      filament::backend::TextureSwizzle a) override;
  SplitEngineTextureBuilder& External() override;
  SplitEngineTextureBuilder& Import(intptr_t id) override;

  void Finalize(filament::Texture* texture) override;

 protected:
  SplitEngineTextureBuilder& ImageInternal(filament::Engine& engine,
                                           AssetPtr<ImageAsset> image,
                                           int image_index) override;
  SplitEngineTextureBuilder& ImageInternal(
      filament::Engine& engine, image::ImageContents& image_contents,
      std::function<void()> callback, int32_t* out_levels = nullptr) override;

 private:
  SplitEngineSerializer* serializer_;
  State state_;

  // TODO: (broken link) - delete the callback.
  // Almost all callsites are setting this callback to an empty lambda.
  // Remaining callsite uses this callback to track when the image was uploaded
  // which is clearly a duplication of PixelBufferDescriptor's callback.
  std::function<void()> images_released_callback_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEXTURE_BUILDER_H_
