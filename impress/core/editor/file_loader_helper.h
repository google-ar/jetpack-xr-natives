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

#include <memory>
#include <optional>
#include <variant>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/editor/editor.h"
#include "core/editor/file_type_loader.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Indicates how to load a file from a path when not getting the data directly
// from a Cord.
enum class LoadAssetFileFromPathSource {
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
// LoadAssetFileFromPathSource enum.
using LoadAssetFileSource =
    std::variant<absl::Cord, LoadAssetFileFromPathSource>;

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
Future<absl::Status> LoadAssetFile(
    BaseView& view, absl::string_view path, LoadAssetFileSource source,
    Invocable<void(NodeHandle)> placement_func = {});

// Loads an asset from file and places the node at the cursor location.
// If cursor is nullopt, the center of the view is used.
void LoadAssetFileAtCursor(BaseView& view, absl::string_view filename,
                           LoadAssetFileSource source,
                           std::optional<float2> cursor = std::nullopt);

// Sends events to notify the editor that a node is being loaded.
void BeginLoadingNode(Editor& editor, BaseView& view);
// Sends events to notify the editor that a node has finished being loaded.
void EndLoadingNode(Editor& editor, BaseView& view, absl::string_view path,
                    NodeHandle node, Invocable<void(NodeHandle)> placement_func,
                    Invocable<void(NodeHandle)> post_loaded_func = {});

// Implementation of FileTypeLoader for glTF files.
class GltfFileLoader : public FileTypeLoader {
 public:
  GltfFileLoader(BaseView& view) : FileTypeLoader(view) {}

  Future<absl::Status> LoadNode(
      absl::string_view path,
      Invocable<void(NodeHandle)> placement_func) override;
};

// File Loader for binary ISF files.
class IsfFileLoader : public FileTypeLoader {
 public:
  IsfFileLoader(BaseView& view) : FileTypeLoader(view) {}

  Future<absl::Status> LoadNode(
      absl::string_view path,
      Invocable<void(NodeHandle)> placement_func) override;
};

// File Loader for text  ISF files.
class TextProtoFileLoader : public FileTypeLoader {
 public:
  TextProtoFileLoader(BaseView& view) : FileTypeLoader(view) {}

  Future<absl::Status> LoadNode(
      absl::string_view path,
      Invocable<void(NodeHandle)> placement_func) override;
};

// File Loader for ImageBasedLighting assets.
class IblFileLoader : public FileTypeLoader {
 public:
  IblFileLoader(BaseView& view) : FileTypeLoader(view) {}

  Future<absl::Status> LoadNode(
      absl::string_view path,
      Invocable<void(NodeHandle)> placement_func) override;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_LOADER_HELPER_H_
