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

#include "core/image/decode_image.h"

#include <memory>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/config.h"
#include "core/image/image_contents.h"
#include "core/resources/resource_manager.h"

#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
#include "core/image/bitmap_factory_decode_image.h"
#include "core/image/ndk_decode_image.h"
#elif IMP_PLATFORM(IOS)
#include "core/image/stb_decode_image.h"
#include "core/image/webp_decode_image.h"
#elif IMP_PLATFORM(WASM)
#include "core/image/wasm_decode_image.h"
#else
#include "core/image/stb_decode_image.h"
#endif

namespace imp::image {

Future<std::unique_ptr<ImageContents>> DecodeImage(
    const imp::Context& context, absl::string_view name,
    imp::resources::Resource resource) {
  absl::StatusOr<std::unique_ptr<ImageContents>> image;
#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
  // Do not use IMP_PLATFORM(ANDROID_API30) here because it relies on
  // __ANDROID_API__, which is the minimum API level your build should support.
  // In case of aGMM the minimum API level is 23, so NDK decoder will not be
  // compiled.
  // However, __builtin_available allows us to use API that above min_sdk target
  // if it is properly gated and supported at runtime.
  return Future<std::unique_ptr<ImageContents>>::Schedule(
      [context, name = std::string(name), resource]() {
        if (__builtin_available(android 30, *)) {
          return details::NdkDecodeImage(resource);
        } else {
          return details::BitmapFactoryDecodeImage(context, name, resource);
        }
      },
      Executor::Type::kBackground);
#elif IMP_PLATFORM(IOS)
  return Future<std::unique_ptr<ImageContents>>::Schedule(
      [name = std::string(name), resource]() {
        absl::StatusOr<std::unique_ptr<ImageContents>> image =
            details::StbDecodeImage(name, resource);
        if (!image.ok()) {
          absl::StatusOr<std::unique_ptr<ImageContents>> webp_image =
              details::WebpDecodeImage(name, resource);
          // If the WebP decoder isn't compiled in, the noop stub should return
          // kUnimplemented. In that case, keep the original error from STB.
          if (webp_image.ok() ||
              webp_image.status().code() != absl::StatusCode::kUnimplemented) {
            image = std::move(webp_image);
          }
        }
        return image;
      },
      Executor::Type::kBackground);
#elif IMP_PLATFORM(WASM)
  return details::WasmDecodeImage(name, resource);
#else
  image = details::StbDecodeImage(name, resource);
#endif
  if (image.ok()) {
    return Future<std::unique_ptr<ImageContents>>(std::move(image.value()));
  }
  return Future<std::unique_ptr<ImageContents>>(image.status());
}

}  // namespace imp::image
