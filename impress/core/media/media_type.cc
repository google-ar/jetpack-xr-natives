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

#include "core/media/media_type.h"

#include "absl/strings/ascii.h"
#include "absl/strings/string_view.h"
#include "core/common/file_helpers.h"
#include "core/common/robin_set.h"

namespace imp {

bool HasImageExtension(absl::string_view url) {
  static const RobinSet<absl::string_view> kImageExtensions = {".png", ".jpeg",
                                                               ".gif"};
  return kImageExtensions.find(absl::AsciiStrToLower(
             GetExtensionFromFilename(url))) != kImageExtensions.end();
}

bool HasVideoExtension(absl::string_view url) {
  static const RobinSet<absl::string_view> kVideoExtensions = {
      ".mov", ".mpeg", ".mp4", ".avi", ".mpd", ".m3u8"};
  return kVideoExtensions.find(absl::AsciiStrToLower(
             GetExtensionFromFilename(url))) != kVideoExtensions.end();
}

}  // namespace imp
