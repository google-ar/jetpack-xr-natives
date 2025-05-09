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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_TYPE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_TYPE_HELPERS_H_

#include "absl/strings/string_view.h"

namespace imp::editor {

constexpr absl::string_view kIsfExt = ".isf";
constexpr absl::string_view kGlbExt = ".glb";
constexpr absl::string_view kGltfExt = ".gltf";
constexpr absl::string_view kCmatExt = ".cmat";
constexpr absl::string_view kPngExt = ".png";
constexpr absl::string_view kMaterialDefinitionExt = ".materialdefinition";

// Used to distinguish drag and drop payloads that are protos from
// editor_proto_visitor so they can be saved in the asset library.
constexpr absl::string_view kProtoDragAndDropScheme = "proto://";

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_TYPE_HELPERS_H_
