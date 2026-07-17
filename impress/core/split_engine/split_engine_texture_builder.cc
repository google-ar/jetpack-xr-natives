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
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Engine.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "core/assets/asset_ptr.h"
#include "core/async/executor.h"
#include "core/image/image_contents.h"
#include "core/render/image_asset.h"
#include "core/split_engine/flatbuffer_size_calculator.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/split_engine/split_engine_texture_serializer.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_schema_version.h"

namespace imp::split_engine {

namespace {

class SplitEngineTextureSerializerImpl : public SplitEngineTextureSerializer {
 public:
  explicit SplitEngineTextureSerializerImpl(
      SplitEngineTextureBuilder::State state)
      : state_(std::move(state)) {
    
  }
  ~SplitEngineTextureSerializerImpl() override = default;

  flatbuffers::Offset<android_xr::schemas::Texture> SerializeTexture(
      flatbuffers::FlatBufferBuilder& fbb) const noexcept override {
    IMP_LOG(imp::INFO) << "[SplitEngineSerializer]: texture " << state_.texture_id;
    flatbuffers::Offset<flatbuffers::String> name_offset = 0;
    if (state_.name.has_value()) {
      name_offset = fbb.CreateString(*state_.name);
    }
    return android_xr::schemas::CreateTexture(
        fbb, state_.texture_id, state_.width, state_.height,
        static_cast<uint16_t>(state_.format), state_.levels,
        static_cast<uint8_t>(state_.sampler), state_.mips,
        CreateFlatbufferImageParams(fbb), CreateFlatbufferPixelBuffers(fbb),
        name_offset);
  }

  size_t GetSerializedSize() const noexcept override {
    std::vector<size_t> image_buffer_sizes(state_.image_descriptors.size());
    // TODO: (broken link) - support texture arrays in Split Engine.
    for (const auto& image_descriptor : state_.image_descriptors) {
      image_buffer_sizes.push_back(image_descriptor.size);
    }
    const size_t name_size = state_.name.has_value() ? state_.name->size() : 0;
    constexpr size_t kTextureCount = 1;
    return FlatbufferSizeCalculator()
        .AddTextureAndDependentData(image_buffer_sizes, name_size)
        .AddReferenceVector(kTextureCount)
        .AddAddTextureRequest(kTextureCount)
        .AddRequest()
        .Finish()
        .AddScratchSpace()
        .ComputeSize();
  }

 private:
  const SplitEngineTextureBuilder::State state_;

  using ImageParamsOffset =
      flatbuffers::Offset<android_xr::schemas::ImageParams>;
  using ImageParamsArray =
      flatbuffers::Offset<flatbuffers::Vector<ImageParamsOffset>>;
  using PixelBufferOffset =
      flatbuffers::Offset<android_xr::schemas::PixelBuffer>;
  using PixelBufferArray =
      flatbuffers::Offset<flatbuffers::Vector<PixelBufferOffset>>;

  ImageParamsArray CreateFlatbufferImageParams(
      flatbuffers::FlatBufferBuilder& fbb) const noexcept {
    const int num_levels = state_.image_descriptors.size();
    std::vector<ImageParamsOffset> image_params(num_levels);
    for (int level = 0; level < num_levels; ++level) {
      const filament::backend::PixelBufferDescriptor& image =
          state_.image_descriptors[level];
      image_params[level] = android_xr::schemas::CreateImageParams(
          fbb, level, static_cast<uint8_t>(image.format),
          static_cast<uint8_t>(image.type), image.alignment, image.left,
          image.top, image.stride);
    }
    return fbb.CreateVector(image_params);
  }

  PixelBufferArray CreateFlatbufferPixelBuffers(
      flatbuffers::FlatBufferBuilder& fbb) const noexcept {
    const int num_levels = state_.image_descriptors.size();
    std::vector<PixelBufferOffset> resource_datas(num_levels);
    for (int level = 0; level < num_levels; ++level) {
      resource_datas[level] = android_xr::schemas::CreatePixelBuffer(
          fbb, fbb.CreateVector(static_cast<uint8_t*>(
                                    state_.image_descriptors[level].buffer),
                                state_.image_descriptors[level].size));
    }
    return fbb.CreateVector(resource_datas);
  }
};

}  // namespace

SplitEngineTextureBuilder::SplitEngineTextureBuilder(
    SplitEngineSerializer& serializer) noexcept
    : serializer_(&serializer) {}

SplitEngineTextureBuilder::SplitEngineTextureBuilder(
    SplitEngineTextureBuilder&& rhs) noexcept
    : serializer_(rhs.serializer_),
      state_(std::move(rhs.state_)),
      images_released_callback_(std::move(rhs.images_released_callback_)) {}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::operator=(
    SplitEngineTextureBuilder&& rhs) noexcept {
  serializer_ = rhs.serializer_;
  state_ = std::move(rhs.state_);
  images_released_callback_ = std::move(rhs.images_released_callback_);
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Width(uint32_t width) {
  

  state_.width = width;
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Height(uint32_t height) {
  

  state_.height = height;
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Depth(uint32_t depth) {
  // TODO: (broken link) - support texture arrays in Split Engine.
  IMP_LOG(imp::FATAL) << "Depth is not supported in Split Engine.";
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Levels(uint8_t levels) {
  

  state_.levels = levels;
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Format(
    filament::backend::TextureFormat format) {
  

  state_.format = format;
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Usage(
    filament::backend::TextureUsage usage) {
  // TODO: Implement this.
  IMP_LOG(imp::FATAL) << "SplitEngineTextureBuilder::Usage is not implemented.";
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Sampler(
    filament::backend::SamplerType sampler) {
  

  state_.sampler = sampler;
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Name(
    absl::string_view name) {
  

  // This static assert verifies the production API level. It acts as a guard to
  // ensure developers verify whether texture naming has been promoted from an
  // experimental feature to production when the API level is bumped.
  //
  // If texture naming is now part of the production API (e.g., API level N),
  // update the condition below accordingly and consider removing this assert.
  //
  // If it remains experimental, update this assert to expect the new production
  // API level to unblock the build.
  //
  static_assert(android_xr::kSplitEngineProductionApiLevel == 1,
                "Production API level bumped. Verify if texture naming should "
                "be promoted to production or remain experimental.");
  if (serializer_->GetApiLevel() ==
      android_xr::kSplitEngineExperimentalApiLevel) {
    state_.name = name;
  } else {
    IMP_LOG(imp::WARNING) << "SplitEngineTextureBuilder::Name is not implemented.";
  }
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Swizzle(
    filament::backend::TextureSwizzle r, filament::backend::TextureSwizzle g,
    filament::backend::TextureSwizzle b, filament::backend::TextureSwizzle a) {
  // TODO: Implement this.
  IMP_LOG(imp::FATAL) << "SplitEngineTextureBuilder::Swizzle is not implemented.";
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::External() {
  // TODO: Implement this.
  IMP_LOG(imp::FATAL) << "SplitEngineTextureBuilder::External is not implemented.";
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::ImageInternal(
    filament::Engine& engine, AssetPtr<ImageAsset> image, int image_index) {
  if (image_index != 0) {
    // TODO: (broken link) - support texture arrays in Split Engine.
    IMP_LOG(imp::FATAL) << "Texture arrays are not supported in Split Engine.";
  }

  state_.image_descriptors = image->GetLevelDescriptors();
  // TODO: (broken link) - we may not need to store asset ptr in the state: even
  // if the image asset is destroyed, PixelBufferDescriptor shall remain valid.
  state_.image = std::move(image);

  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::ImageInternal(
    filament::Engine& engine, image::ImageContents& image_contents,
    std::function<void()> callback, int32_t* out_levels) {
  
  

  state_.image_descriptors =
      image_contents.CreatePixelBufferDescriptorLevels(nullptr, false);
  images_released_callback_ = callback;

  if (out_levels) {
    *out_levels = state_.image_descriptors.size();
  }

  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::GenerateMipmaps(
    filament::Engine& engine) {
  

  state_.mips = true;
  return *this;
}

SplitEngineTextureBuilder& SplitEngineTextureBuilder::Import(intptr_t id) {
  IMP_LOG(imp::FATAL) << "SplitEngineTextureBuilder::Import is not implemented.";
  return *this;
}

void SplitEngineTextureBuilder::Finalize(filament::Texture* texture) {
  
  

  state_.texture_id = SplitEngineSerializer::GetId(texture);
  state_.finalized = true;
  serializer_->SerializeTexture(
      std::make_unique<const SplitEngineTextureSerializerImpl>(
          std::move(state_)),
      [images_released_callback = std::move(images_released_callback_)]() {
        // We're done with image_descriptors_ so the backing buffers may now be
        // freed.
        if (images_released_callback) {
          images_released_callback();
        }
      });
}

SplitEngineTextureBuilder::State::~State() {
  if (image && Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    Executor::ForegroundExecutor()->ScheduleInvocable(
        [image = std::move(image)]() {
          /* AssetPtr shall be destroyed on foreground thread.*/
        });
  }
}
}  // namespace imp::split_engine
