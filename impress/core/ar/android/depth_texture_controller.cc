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

#include "core/ar/android/depth_texture_controller.h"

#include <cstdint>
#include <limits>
#include <memory>

#include "core/common/log.h"
#include "third_party/arcore/ar/core/c_api/arcore_c_api.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/common/trace.h"
#include "core/render/texture_factory.h"

namespace imp {
namespace ar {
namespace {
static constexpr absl::Duration kUnavailableDepthImageTimeout =
    absl::Milliseconds(250);
}  // namespace

// This class helps us wrap an ArImage_* in a closure that we pass to filament's
// PixelBufferDescriptor, which only supports legacy functions with no closure.
struct ClosureWrapper {
  imp::Invocable<void(void*, size_t)> closure;
  explicit ClosureWrapper(imp::Invocable<void(void*, size_t)> closure)
      : closure(std::move(closure)) {}

  static void Callback(void* buffer, size_t size, void* user) {
    ClosureWrapper* closure_wrapper = static_cast<ClosureWrapper*>(user);
    closure_wrapper->closure(std::move(buffer), std::move(size));
    delete closure_wrapper;
  }
};

DepthTextureController::DepthTextureController(filament::Engine* engine,
                                               TextureFactory* texture_factory)
    : engine_(*engine), texture_factory_(texture_factory) {}

void DepthTextureController::SetTextureCreatedHandler(
    std::function<void(imp::Texture const&)> handler) {
  texture_created_handler_ = std::move(handler);

  if (texture_) texture_created_handler_(*texture_);
}

void DepthTextureController::Update(ArSession_* session, ArFrame_* frame) {
  IMP_TRACE();
  if (absl::Now() < timeout_end_) {
    // Avoid smothering ARCore by waiting a few frames after unavailability.
    return;
  }
  ArImage_* image = nullptr;
  switch (auto result =
              ArFrame_acquireDepthImage16Bits(session, frame, &image)) {
    case AR_SUCCESS:
      if (!image) {
        IMP_LOG(imp::FATAL) << "image is null despite successful result returned from "
                      "ArFrame_acquireDepthImage16Bits";
      }

      UpdateFromArImage(session, UniqueArImage(image));
      break;

    case AR_ERROR_NOT_YET_AVAILABLE:
      // This is okay, maybe depth will be available soon.
      timeout_end_ = absl::Now() + kUnavailableDepthImageTimeout;
      break;

    default:  // Notifies of some other unaccounted error.
      IMP_LOG(imp::ERROR) << "Error acquiring ARCore depth image, result: " << result;
      break;
  }
}

void DepthTextureController::UpdateFromArImage(ArSession_* session,
                                               UniqueArImage image) {
  IMP_TRACE();
  // Checks the image timestamp in case the image is stale.
  auto timestamp_ns = std::numeric_limits<int64_t>::max();
  ArImage_getTimestamp(session, image.get(), &timestamp_ns);
  if (last_acquire_image_timestamp_ns_ &&
      *last_acquire_image_timestamp_ns_ >= timestamp_ns) {
    return;
  }
  last_acquire_image_timestamp_ns_ = timestamp_ns;

  // Checks that the image holds the expected format.
  auto format = AR_IMAGE_FORMAT_INVALID;
  ArImage_getFormat(session, image.get(), &format);
  if (format != AR_IMAGE_FORMAT_DEPTH16 && format != AR_IMAGE_FORMAT_D_16) {
    IMP_LOG(imp::FATAL) << "ARCore depth image format is expected to be D_16";
  }

  int32_t width = 0;
  int32_t height = 0;
  uint8_t const* depth_data = nullptr;
  int32_t depth_data_length = 0;
  ArImage_getWidth(session, image.get(), &width);
  ArImage_getHeight(session, image.get(), &height);
  ArImage_getPlaneData(session, image.get(), 0, &depth_data,
                       &depth_data_length);

  // Checks that the image memory is the expected size.
  auto expected_depth_data_length = width * height * sizeof(uint16_t);
  if (depth_data_length != expected_depth_data_length) {
    IMP_LOG(imp::FATAL) << "Incorrect length of depth data: " << depth_data_length
               << " expected: " << expected_depth_data_length;
  }

  // Creates or resizes the filament texture if it hasn't been created or if it
  // has been resized. It is unlikely that a resize will occur, but nothing in
  // the API prevents it from happening.
  auto is_texture_create_necessary =
      !texture_ || (texture_->GetSize() != uint2{width, height});
  if (is_texture_create_necessary) {
    texture_ = texture_factory_->CreateTexture(
        imp::TextureFactory::TextureCreationSettings{
            .width = static_cast<uint32_t>(width),
            .height = static_cast<uint32_t>(height),
            .format = filament::Texture::InternalFormat::RG8,
        });
  }

  // Copies image data into to the filament texture.
  auto pixel_buffer_descriptor = filament::Texture::PixelBufferDescriptor{
      depth_data, static_cast<size_t>(depth_data_length),
      filament::backend::PixelBufferDescriptor::PixelDataFormat::RG,
      filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE,
      &ClosureWrapper::Callback,
      // This closure takes ownership of image so that the lifetime matches
      // the lifetime of the PixelBufferDescriptor callback.
      new ClosureWrapper(
          [image = std::move(image)](void* buffer, size_t size) {})};
  texture_->GetTexture()->setImage(engine_, 0u,
                                   std::move(pixel_buffer_descriptor));

  // Issues texture creation callback if necessary.
  if (is_texture_create_necessary && texture_created_handler_)
    texture_created_handler_(*texture_);
}

}  // namespace ar
}  // namespace imp
