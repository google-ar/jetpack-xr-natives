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

#include "core/render/texture_builder.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/CallbackHandler.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/assets/asset_ptr.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/image/image_contents.h"
#include "core/render/image_asset.h"
#include "core/render/safe_filament_texture_builder.h"
#include "core/view/base_view.h"

namespace imp {

namespace {
// A CallbackHandler that simply calls the callback on the same thread that
// calls post.
class TextureReadyCallbackHandler : public filament::backend::CallbackHandler {
 public:
  void post(void* user,
            filament::backend::CallbackHandler::Callback callback) override {
    callback(user);
  };

  ~TextureReadyCallbackHandler() override = default;
};
}  // namespace

TextureBuilder::TextureBuilder(BaseView& view) noexcept
    : view_(&view), spy_(nullptr) {
  builder_ = SafeFilamentTextureBuilder();

  if (auto* serializer = view_->GetSplitEngineSerializer()) {
    spy_ = serializer->CreateTextureBuilder();
  }
}

TextureBuilder::TextureBuilder(TextureBuilder&& rhs) noexcept
    : view_(rhs.view_),
      spy_(std::move(rhs.spy_)),
      builder_(std::move(rhs.builder_)),
      texture_(std::move(rhs.texture_)),
      name_(std::move(rhs.name_)),
      image_assets_(std::move(rhs.image_assets_)),
      images_(std::move(rhs.images_)),
      generate_mipmaps_(rhs.generate_mipmaps_) {}

TextureBuilder& TextureBuilder::operator=(TextureBuilder&& rhs) noexcept {
  view_ = rhs.view_;
  spy_ = std::move(rhs.spy_);
  builder_ = std::move(rhs.builder_);
  texture_ = std::move(rhs.texture_);
  name_ = std::move(rhs.name_);
  image_assets_ = std::move(rhs.image_assets_);
  images_ = std::move(rhs.images_);
  generate_mipmaps_ = rhs.generate_mipmaps_;
  return *this;
}

TextureBuilder& TextureBuilder::Width(uint32_t width) {
  
  if (spy_) {
    spy_->Width(width);
  }
  builder_->width(width);
  return *this;
}
TextureBuilder& TextureBuilder::Height(uint32_t height) {
  
  if (spy_) {
    spy_->Height(height);
  }
  builder_->height(height);
  return *this;
}

TextureBuilder& TextureBuilder::Depth(uint32_t depth) {
  
  if (spy_) {
    spy_->Depth(depth);
  }
  builder_->depth(depth);
  return *this;
}

TextureBuilder& TextureBuilder::Levels(uint8_t levels) {
  
  if (spy_) {
    spy_->Levels(levels);
  }
  builder_->levels(levels);
  return *this;
}
TextureBuilder& TextureBuilder::Format(
    filament::backend::TextureFormat format) {
  
  if (spy_) {
    spy_->Format(format);
  }
  builder_->format(format);
  return *this;
}

TextureBuilder& TextureBuilder::Usage(filament::backend::TextureUsage usage) {
  
  if (spy_) {
    spy_->Usage(usage);
  }
  builder_->usage(usage);
  return *this;
}

TextureBuilder& TextureBuilder::Sampler(
    filament::backend::SamplerType sampler) {
  
  if (spy_) {
    spy_->Sampler(sampler);
  }
  builder_->sampler(sampler);
  return *this;
}

TextureBuilder& TextureBuilder::Name(absl::string_view name) {
  
  if (spy_) {
    spy_->Name(name);
  }

  name_ = name;
  builder_->name(name_.data(), name_.length());
  return *this;
}

TextureBuilder& TextureBuilder::Swizzle(filament::backend::TextureSwizzle r,
                                        filament::backend::TextureSwizzle g,
                                        filament::backend::TextureSwizzle b,
                                        filament::backend::TextureSwizzle a) {
  
  if (spy_) {
    spy_->Swizzle(r, g, b, a);
  }
  builder_->swizzle(r, g, b, a);
  return *this;
}

TextureBuilder& TextureBuilder::External() {
  
  if (spy_) {
    spy_->External();
  }
  builder_->external();
  return *this;
}

TextureBuilder& TextureBuilder::ImageInternal(filament::Engine& engine,
                                              AssetPtr<ImageAsset> image,
                                              int image_index) {
  
  if (spy_) {
    spy_->Image(engine, image, image_index);
    // The images will be moved, so the normal setImage API should be skipped.
    return *this;
  }

  if (!images_.empty()) {
    IMP_LOG(imp::WARNING) << "Multiple image data sources are not supported. "
                    "Overwriting previous image data sources.";
    // Remove other sources of images.
    images_.clear();
  }

  image_assets_.push_back({std::move(image), image_index});

  return *this;
}

TextureBuilder& TextureBuilder::ImageInternal(
    filament::Engine& engine, image::ImageContents& image_contents,
    std::function<void()> callback, int32_t* out_levels) {
  
  if (spy_) {
    spy_->Image(engine, image_contents, callback, out_levels);
    // The images will be moved, so the normal setImage API should be skipped.
    return *this;
  }

  if (!image_assets_.empty()) {
    IMP_LOG(imp::WARNING) << "Multiple image data sources are not supported. "
                    "Overwriting previous image data sources.";
    // Remove other sources of images.
    image_assets_.clear();
  }

  images_ = image_contents.CreatePixelBufferDescriptorLevels(callback, false);
  if (out_levels) {
    *out_levels = images_.size();
  }

  return *this;
}

TextureBuilder& TextureBuilder::GenerateMipmaps(filament::Engine& engine) {
  
  if (spy_) {
    spy_->GenerateMipmaps(engine);
  }

  generate_mipmaps_ = true;
  return *this;
}

TextureBuilder& TextureBuilder::Import(intptr_t id) {
  
  if (spy_) {
    spy_->Import(id);
  }
  builder_->import(id);
  return *this;
}

imp::Future<filament::Texture* /*absl_nullable*/ > TextureBuilder::BuildAsync(
    filament::Engine& engine) {
  
  if (!engine.isAsynchronousModeEnabled()) {
    IMP_LOG(imp::WARNING) << "Asynchronous mode is not enabled for the engine.";
    return Future<filament::Texture*>(Build(engine));
  }

  static TextureReadyCallbackHandler callback_handler;

  imp::Future<filament::Texture*> future;

  // Set the async callback on the builder
  builder_->async(
      &callback_handler,
      [weak_future = WeakFuture<filament::Texture*>(future)](
          filament::Texture* texture, void*) {
        std::optional<imp::Future<filament::Texture*>> future =
            weak_future.Lock();

        if (!texture) {
          if (future.has_value()) {
            future->Return(
                absl::InternalError("Failed to create async texture."));
          }
          return;
        }

        // If the future is already ready, it means it was cancelled right
        // before the async callback happened. It's also possible that this
        // future isn't cancelled here but downstream futures are cancelled, so
        // to ensure the texture is only queued for destruction once, we return
        // early here and let the downstream future handle the destruction.
        if (!future.has_value() || future->Ready()) {
          return;
        }

        // Otherwise, fulfill the future.
        future->Return(texture);
      },
      nullptr);

  // Call build. This returns immediately and the texture will be populated
  // asynchronously. The callback above will fire when it's done.
  texture_ = builder_->build(engine);

  builder_.reset();

  if (!texture_.ok()) {
    return texture_.status();
  }

  // We always schedule this callback on the foreground executor to ensure that
  // any work done will happen on the foreground thread and within the timeframe
  // of the executor pump.
  // Any cancellations upstream will result in this future being cancelled, so
  // we need to destroy the texture if that happens.
  future = future.Then(
      [&engine,
       texture = *texture_](absl::StatusOr<filament::Texture*> result) {
        if (result.status().code() == absl::StatusCode::kCancelled) {
          engine.destroy(texture);
        }
        return result;
      },
      FutureThenOptions{.executor = Executor::ForegroundExecutor(),
                        .executor_mode = FutureExecutorMode::kScheduleAlways});

  if (!image_assets_.empty()) {
    future = future.Then([&engine, image_assets = std::move(image_assets_)](
                             filament::Texture* texture) {
      std::vector<imp::Future<absl::Status>> futures;
      futures.reserve(image_assets.size());
      for (auto& asset : image_assets) {
        auto descriptors = asset.image->GetLevelDescriptors();
        for (int level = 0; level < descriptors.size(); ++level) {
          imp::Future<absl::Status> future;
          filament::Texture::AsyncCallId id = texture->setImageAsync(
              engine, level, /*xoffset=*/0, /*yoffset=*/0, asset.index,
              texture->getWidth(level), texture->getHeight(level), /*depth=*/1,
              std::move(descriptors[level]), &callback_handler,
              [future](filament::Texture*, void*) {
                future.Return(absl::OkStatus());
              });
          future = future.Then(
              [&engine, id](const absl::Status& status) {
                if (status.code() == absl::StatusCode::kCancelled) {
                  engine.cancelAsyncCall(id);
                }
                return status;
              },
              FutureThenOptions{
                  .executor = Executor::ForegroundExecutor(),
                  .executor_mode = FutureExecutorMode::kScheduleAlways});
          futures.push_back(std::move(future));
        }
      }
      return Future<absl::Status>::CombineList(futures).Then(
          [&engine, texture](const absl::Status& status)
              -> absl::StatusOr<filament::Texture*> {
            if (!status.ok()) {
              engine.destroy(texture);
              return status;
            }
            return texture;
          });
    });
  } else if (!images_.empty()) {
    future = future.Then([&engine, images = std::move(images_)](
                             filament::Texture* texture) mutable {
      std::vector<imp::Future<absl::Status>> futures;
      futures.reserve(images.size());
      for (int level = 0; level < images.size(); ++level) {
        imp::Future<absl::Status> future;
        filament::Texture::AsyncCallId id = texture->setImageAsync(
            engine, level, std::move(images[level]), &callback_handler,
            [future](filament::Texture*, void*) {
              future.Return(absl::OkStatus());
            });
        future = future.Then([&engine, id](const absl::Status& status) {
          if (status.code() == absl::StatusCode::kCancelled) {
            engine.cancelAsyncCall(id);
          }
          return status;
        });
        futures.push_back(std::move(future));
      }
      return Future<absl::Status>::CombineList(futures).Then(
          [&engine, texture](const absl::Status& status)
              -> absl::StatusOr<filament::Texture*> {
            if (!status.ok()) {
              engine.destroy(texture);
              return status;
            }
            return texture;
          });
    });
  }

  if (generate_mipmaps_) {
    future = future.Then([&engine](filament::Texture* texture) {
      texture->generateMipmaps(engine);
      return texture;
    });
  }

  return future;
}

filament::Texture* TextureBuilder::Build(filament::Engine& engine) {
  
  texture_ = builder_->build(engine);
  builder_.reset();

  if (!texture_.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to create texture, \"" << name_
               << "\": " << texture_.status().ToString();
    return nullptr;
  }

  if (!image_assets_.empty()) {
    for (auto& asset : image_assets_) {
      auto descriptors = asset.image->GetLevelDescriptors();
      for (int level = 0; level < descriptors.size(); ++level) {
        (*texture_)->setImage(
            engine, level, 0, 0, asset.index, (*texture_)->getWidth(level),
            (*texture_)->getHeight(level), 1, std::move(descriptors[level]));
      }
    }
  } else if (!images_.empty()) {
    for (int level = 0; level < images_.size(); ++level) {
      // Note: filament::Texture::setImage() can panic
      (*texture_)->setImage(engine, level, std::move(images_[level]));
    }
  }

  if (generate_mipmaps_) {
    (*texture_)->generateMipmaps(engine);
  }

  if (spy_) {
    spy_->Finalize(*texture_);
  }

  return *texture_;
}

void TextureBuilder::Finalize(filament::Texture* texture) {
  // Do nothing. This is designed to be overridden by SplitEngineTextureBuilder.
}

}  // namespace imp
