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

#include "core/media/android/android_media_source.h"

#include <string>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "core/media/android/android_media_player.h"

namespace imp::media {

// Builds an error string from a media error.
std::string MediaErrorToString(AndroidMediaPlayer* player, int what,
                               int extra) {
  std::string error =
      absl::StrFormat("Media Player Error (what %d, extra %d: ", what, extra);
  if (!player) {
    absl::StrAppend(&error, "Error: Media Player is null.)");
    return error;
  }

  if (what == player->GetMediaErrorServerDied()) {
    absl::StrAppend(&error, "Error: ServerDied, ");
  } else if (what == player->GetMediaErrorUnknown()) {
    absl::StrAppend(&error, "Error: Unknown Media Error, ");
  } else {
    absl::StrAppend(&error, "Error: Unknown, ");
  }

  if (extra == player->MediaErrorIO()) {
    absl::StrAppend(&error, "Reason: Unknown IO Error)");
  } else if (extra == player->MediaErrorMalformed()) {
    absl::StrAppend(&error, "Reason: ServerDied)");
  } else if (extra == player->MediaErrorUnsupported()) {
    absl::StrAppend(&error, "Reason: Unsupported)");
  } else if (extra == player->MediaErrorTimedOut()) {
    absl::StrAppend(&error, "Reason: Timed Out)");
  } else if (extra == player->MediaErrorSystem()) {
    absl::StrAppend(&error, "Reason: System Error)");
  } else {
    absl::StrAppend(&error, "Reason: Unknown)");
  }

  return error;
}
}  // namespace imp::media
