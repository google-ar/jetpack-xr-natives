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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_ANDROID_DEFINES_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_ANDROID_DEFINES_H_

#include <cstdint>

namespace imp {

// Enumerates the surface view types supported by external texture surface.
// Base view is the default view, supported by both SurfaceTexture-based and
// ImageReader-based implementations. The auxiliary and depth surface view types
// are only supported in ImageReader-based workflow.
enum class SurfaceViewType : uint32_t {
  kPrimaryView = 0,
  kAuxiliaryView = 1,
  kPrimaryViewDepth = 2,
  kAuxiliaryViewDepth = 3,
};

// Maximum view dimensions to guide adaptive codec (e.g., HEVC) decoding on
// Android. This aids decoders in adjusting to view size changes, but doesn't
// restrict actual buffer allocation or image sizes produced by the ImageReader.
inline constexpr int kMaxViewWidth = 4096;
inline constexpr int kMaxViewHeight = 4096;

// Maximum images held in the ImageReader buffer queue. ImageReader's max queue
// size is 62 (64 - 2 reserved).
inline constexpr int kImageReaderBufferSize = 16;

// Maximum images explicitly kept alive to be used in the renderer thread.
inline constexpr int kMaxImagesKeptAlive = 4;

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_ANDROID_DEFINES_H_
