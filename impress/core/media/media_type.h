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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_TYPE_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_TYPE_H_

#include "absl/strings/string_view.h"

namespace imp {

enum class MediaType : int {
  kUnknown = 0,
  kImage = 1,
  kVideo = 2,
};

enum class MediaShapeType : int {
  kDefaultFlat = 0,
  kVR180 = 1,
  kFull360 = 2,
};

// The stereo mode of a media asset. The enum values are defined to match
// the values returned by media3. Please see here for more information:
// third_party/java_src/android_libs/media/libraries/common/src/main/java/androidx/media3/common/C.java
// LINT.IfChange
enum class MediaStereoMode : int {
  // Unknown stereo mode.
  kUnknown = -1,
  // Monoscopic media asset.
  kMonoscopic = 0,
  // Top-bottom stereo media asset.
  kTopBottom = 1,
  // Left-right stereo media asset.
  kLeftRight = 2,
  // Stereo media asset with separate mesh for each view.
  kStereoMesh = 3,
  // Interleaved stereo media asset with left view as primary.
  kInterleavedLeftPrimary = 4,
  // Interleaved stereo media asset with right view as primary.
  kInterleavedRightPrimary = 5,
  // Interleaved stereo media asset with left view as primary including depth.
  kInterleavedLeftPrimaryWithDepth = 6,
  // Interleaved stereo media asset with right view as primary including depth.
  kInterleavedRightPrimaryWithDepth = 7,
};
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/apibindings/ImpressApi.java,
//     //depot/google3/third_party/impress/java/com/google/ar/imp/apibindings/ImpressApiImpl.java
// )

bool HasImageExtension(absl::string_view url);

bool HasVideoExtension(absl::string_view url);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_TYPE_H_
