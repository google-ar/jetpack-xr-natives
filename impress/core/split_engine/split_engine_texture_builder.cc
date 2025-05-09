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

#include "core/split_engine/split_engine_texture_builder.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Engine.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/image/image_contents.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_serializer.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

SplitEngineTextureBuilder::SplitEngineTextureBuilder(
    SplitEngineSerializer& serializer) noexcept
    : serializer_(&serializer) {}

SplitEngineTextureBuilder::SplitEngineTextureBuilder(
    SplitEngineTextureBuilder&& rhs) noexcept
    : serializer_(rhs.serializer_),
      width_(rhs.width_),
      height_(rhs.height_),
      levels_(rhs.levels_),
      format_(rhs.format_),
      sampler_(rhs.sampler_),
      mips_(rhs.mips_),
      image_descriptors_(std::move(rhs.image_descriptors_)),
      images_released_callback_(std::move(rhs.images_released_callback_)) {}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::operator=(
    SplitEngineTextureBuilder&& rhs) noexcept {
  serializer_ = rhs.serializer_;
  width_ = rhs.width_;
  height_ = rhs.height_;
  levels_ = rhs.levels_;
  format_ = rhs.format_;
  sampler_ = rhs.sampler_;
  mips_ = rhs.mips_;
  image_descriptors_ = std::move(rhs.image_descriptors_);
  images_released_callback_ = std::move(rhs.images_released_callback_);
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Width(uint32_t width) {
  width_ = width;
  return *this;
}
SplitEngineTextureBuilder& SplitEngineTextureBuilder::Height(uint32_t height) {
  height_ = height;
  return *this;
}
SplitEngineTextureBuilder& SplitEngineTextureBuilder::Levels(uint8_t levels) {
  levels_ = levels;
  return *this;
}
SplitEngineTextureBuilder& SplitEngineTextureBuilder::Format(
    filament::backend::TextureFormat format) {
  format_ = format;
  return *this;
}
SplitEngineTextureBuilder& SplitEngineTextureBuilder::Sampler(
    filament::backend::SamplerType sampler) {
  sampler_ = sampler;
  return *this;
}
SplitEngineTextureBuilder& SplitEngineTextureBuilder::Name(
    absl::string_view name) {
  // TODO: Implement this.
  return *this;
}
SplitEngineTextureBuilder& SplitEngineTextureBuilder::ImageInternal(
    filament::Engine& engine, image::ImageContents& image_contents,
    std::function<void()> callback, int32_t* out_levels) {
  image_descriptors_ =
      image_contents.CreatePixelBufferDescriptorLevels(nullptr, false);
  images_released_callback_ = callback;

  if (out_levels) {
    *out_levels = image_descriptors_.size();
  }

  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::GenerateMipmaps(
    filament::Engine& engine) {
  mips_ = true;
  return *this;
}

std::vector<size_t> SplitEngineTextureBuilder::GetTextureBufferSizes() {
  std::vector<size_t> image_buffer_sizes(image_descriptors_.size());
  for (const auto& image_descriptor : image_descriptors_) {
    image_buffer_sizes.push_back(image_descriptor.size);
  }
  return image_buffer_sizes;
}

void SplitEngineTextureBuilder::Finalize(filament::Texture* texture) {
  serializer_->AddTexture(*texture, *this);

  // We're done with image_descriptors_ so the backing buffers may now be freed.
  if (images_released_callback_) {
    images_released_callback_();
  }
}

flatbuffers::Offset<android_xr::schemas::Texture>
SplitEngineTextureBuilder::SerializeTexture(
    filament::Texture& texture, flatbuffers::FlatBufferBuilder& fbb) {
  return android_xr::schemas::CreateTexture(
      fbb, reinterpret_cast<TextureId>(&texture), width_, height_,
      static_cast<uint16_t>(format_), levels_, static_cast<uint8_t>(sampler_),
      mips_, CreateFlatbufferImageParams(fbb),
      CreateFlatbufferPixelBuffers(fbb));
}

SplitEngineTextureBuilder::ImageParamsArray
SplitEngineTextureBuilder::CreateFlatbufferImageParams(
    flatbuffers::FlatBufferBuilder& fbb) {
  int num_levels = image_descriptors_.size();
  std::vector<ImageParamsOffset> image_params(num_levels);
  for (int level = 0; level < num_levels; ++level) {
    const filament::backend::PixelBufferDescriptor& image =
        image_descriptors_[level];
    image_params[level] = android_xr::schemas::CreateImageParams(
        fbb, level, static_cast<uint8_t>(image.format),
        static_cast<uint8_t>(image.type), image.alignment, image.left,
        image.top, image.stride);
  }
  return fbb.CreateVector(image_params);
}

SplitEngineTextureBuilder::PixelBufferArray
SplitEngineTextureBuilder::CreateFlatbufferPixelBuffers(
    flatbuffers::FlatBufferBuilder& fbb) {
  int num_levels = image_descriptors_.size();
  std::vector<PixelBufferOffset> resource_datas(num_levels);
  for (int level = 0; level < num_levels; ++level) {
    resource_datas[level] = android_xr::schemas::CreatePixelBuffer(
        fbb, fbb.CreateVector(
                 static_cast<uint8_t*>(image_descriptors_[level].buffer),
                 image_descriptors_[level].size));
  }
  return fbb.CreateVector(resource_datas);
}

}  // namespace imp::split_engine
