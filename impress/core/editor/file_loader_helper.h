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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_LOADER_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_LOADER_HELPER_H_

#include <variant>

#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

// An enum representation of various asset types used by the editor.
enum class FileType {
  kUnsupported,
  kGltf,
  kIsf,
  kIsfJson,
  kIsfTextproto,
  kHdrImage,
  kTexture,
  kGSplat,
};

// Returns the file type by looking at the extension of the given filename.
FileType GetFileTypeFromName(absl::string_view filename);

// Indicates how to load a file from a path when not getting the data directly
// from a Cord.
enum class LoadFileFromPathSource {
  // Indicates that the file is an asset that is already available in the
  // AssetManager.
  kAsset,
  // Indicates that the file is a local file that must be added to the
  // AssetLibrary.
  kLocalFile
};

// If an absl::Cord is provided, the file is loaded from the provided data.
//
// Otherwise, the file is loaded from the file path based on the
// LoadFileFromPathSource enum.
using LoadFileSource = std::variant<absl::Cord, LoadFileFromPathSource>;

// Helper function to load an ISF file, glb file, or ImageBasedLighting
// asset. Loading a glb file and ImageBasedLighting asset is supported by
// both a path or a Cord, while loading a scene can only be done via a path.
// Loading a glb file or an ISF file will return a valid NodeHandle to the
// model or root node, whereas loading an ImageBasedLighting asset will
// return an invalid NodeHandle as a placeholder. The placement_func
// parameter is for positioning the loaded model *prior* to the
// ModelLoadedEvent being sent. This allows for systems that listen to
// ModelLoadedEvent to reposition the model without being clobbered.
// TODO: Change how ModelLoadedEvent is sent so this is
// cleaner.
Future<absl::Status> LoadFile(BaseView& view, absl::string_view path,
                              LoadFileSource source,
                              Invocable<void(NodeHandle)> placement_func = {});

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_LOADER_HELPER_H_
