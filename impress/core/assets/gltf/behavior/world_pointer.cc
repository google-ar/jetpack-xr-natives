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

#include "core/assets/gltf/behavior/world_pointer.h"

#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"

namespace imp {

namespace gltf::behavior {

namespace {

// (broken link) start
constexpr absl::string_view kActiveCameraToken = "activeCamera";
constexpr absl::string_view kExtensionsToken = "extensions";
constexpr absl::string_view kKhrVisibilityToken = "KHR_visibility";
constexpr absl::string_view kKhrVisibilityVisibleToken = "visible";
constexpr absl::string_view kNodesToken = "nodes";
constexpr absl::string_view kRotationToken = "rotation";
constexpr absl::string_view kScaleToken = "scale";
constexpr absl::string_view kSceneMatrixToken = "sceneMatrix";
constexpr absl::string_view kScenesToken = "scenes";
constexpr absl::string_view kTranslationToken = "translation";
// (broken link) end

}  // namespace

absl::StatusOr<WorldPointer> WorldPointer::FromString(absl::string_view path) {
  WorldPointer result;
  std::vector<std::string> tokens = absl::StrSplit(path, '/');

  absl::string_view type = tokens[0];
  if (type == kNodesToken && tokens.size() >= 3) {
    if (!absl::SimpleAtoi(tokens[1], &result.index)) {
      return absl::InvalidArgumentError(
          absl::StrFormat("Malformed index: %s. Expected int.", tokens[1]));
    }

    absl::string_view node_pointer_type = tokens[2];
    if (node_pointer_type == kTranslationToken) {
      result.type = WorldPointer::Type::kNodeTranslation;
    } else if (node_pointer_type == kRotationToken) {
      result.type = WorldPointer::Type::kNodeRotation;
    } else if (node_pointer_type == kScaleToken) {
      result.type = WorldPointer::Type::kNodeScale;
    } else if (node_pointer_type == kSceneMatrixToken) {
      result.type = WorldPointer::Type::kNodeSceneMatrix;
    } else if (node_pointer_type == kExtensionsToken) {
      if (tokens.size() == 5 && tokens[3] == kKhrVisibilityToken &&
          tokens[4] == kKhrVisibilityVisibleToken) {
        result.type = WorldPointer::Type::kNodeExtensionKhrVisibilityVisible;
      }
    }
  } else if (type == kScenesToken && tokens.size() == 4) {
    if (!absl::SimpleAtoi(tokens[1], &result.index)) {
      return absl::InvalidArgumentError(
          absl::StrFormat("Malformed index: %s. Expected int.", tokens[1]));
    }

    absl::string_view scene_pointer_type = tokens[2];
    if (scene_pointer_type == kActiveCameraToken) {
      absl::string_view active_camera_pointer_type = tokens[3];
      if (active_camera_pointer_type == kSceneMatrixToken) {
        result.type = WorldPointer::Type::kActiveCameraSceneMatrix;
      }
    }
  }

  if (result.type == WorldPointer::Type::kUnknown) {
    return absl::UnimplementedError(absl::StrFormat(
        "Unsupported or malformed world pointer path: %s", path));
  }

  return result;
}

}  // namespace gltf::behavior

}  // namespace imp
