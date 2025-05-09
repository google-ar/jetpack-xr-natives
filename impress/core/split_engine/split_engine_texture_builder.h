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

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "core/image/image_contents.h"
#include "core/render/base_texture_builder.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/split_engine/split_engine_texture_serializer.h"
#include "split_engine/schemas/split_engine_data_generated.h"

namespace imp::split_engine {

class SplitEngineTextureBuilder : public SplitEngineTextureSerializer,
                                  public BaseTextureBuilder {
 public:
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
  SplitEngineTextureBuilder& Levels(uint8_t levels) override;
  SplitEngineTextureBuilder& Format(
      filament::backend::TextureFormat format) override;
  SplitEngineTextureBuilder& Sampler(
      filament::backend::SamplerType sampler) override;
  SplitEngineTextureBuilder& GenerateMipmaps(filament::Engine& engine) override;
  SplitEngineTextureBuilder& Name(absl::string_view name) override;
  void Finalize(filament::Texture* texture) override;

  flatbuffers::Offset<android_xr::schemas::Texture> SerializeTexture(
      filament::Texture& texture, flatbuffers::FlatBufferBuilder& fbb) override;
  std::vector<size_t> GetTextureBufferSizes() override;

 protected:
  SplitEngineTextureBuilder& ImageInternal(
      filament::Engine& engine, image::ImageContents& image_contents,
      std::function<void()> callback, int32_t* out_levels = nullptr) override;

 private:
  using ImageParamsOffset =
      flatbuffers::Offset<android_xr::schemas::ImageParams>;
  using ImageParamsArray =
      flatbuffers::Offset<flatbuffers::Vector<ImageParamsOffset>>;
  using PixelBufferOffset =
      flatbuffers::Offset<android_xr::schemas::PixelBuffer>;
  using PixelBufferArray =
      flatbuffers::Offset<flatbuffers::Vector<PixelBufferOffset>>;

  ImageParamsArray CreateFlatbufferImageParams(
      flatbuffers::FlatBufferBuilder& fbb);
  PixelBufferArray CreateFlatbufferPixelBuffers(
      flatbuffers::FlatBufferBuilder& fbb);

  SplitEngineSerializer* serializer_;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint8_t levels_ = 0;
  filament::backend::TextureFormat format_ =
      filament::backend::TextureFormat::RGB8;
  filament::Texture::Sampler sampler_ = filament::Texture::Sampler::SAMPLER_2D;
  bool mips_ = false;
  std::vector<filament::backend::PixelBufferDescriptor> image_descriptors_;
  std::function<void()> images_released_callback_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEXTURE_BUILDER_H_
