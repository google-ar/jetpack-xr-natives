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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_TYPE_REGISTRY_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_TYPE_REGISTRY_H_

#include <memory>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/editor/file_type.h"
#include "core/editor/file_type_loader.h"
#include "core/ncsb/node_handle.h"

namespace imp {
class BaseView;
}  // namespace imp

namespace imp::editor {
using LoadAssetFunc = Invocable<Future<absl::Status>(absl::string_view path)>;

// Extensible registry of file types and corresponding load functions.
class FileTypeRegistry {
 public:
  FileTypeRegistry() = default;

  void RegisterFileTypeLoader(FileType file_type,
                              std::unique_ptr<FileTypeLoader> loader);

  void RegisterFileTypeLoader(FileType file_type,
                              LoadAssetFunc load_asset_func);

  FileTypeLoader* GetFileTypeLoaderByPath(absl::string_view path) const;

  Future<absl::Status> LoadNode(
      absl::string_view path, Invocable<void(NodeHandle)> placement_func) const;

  // The registry is not copyable.
  FileTypeRegistry(const FileTypeRegistry&) = delete;
  FileTypeRegistry& operator=(const FileTypeRegistry&) = delete;

 private:
  absl::flat_hash_map<FileType, std::unique_ptr<FileTypeLoader>>
      file_type_loaders_;
};

extern const FileType kFileTypeGltf;
extern const FileType kFileTypeIsf;
extern const FileType kFileTypeIsfTextProto;
extern const FileType kFileTypeHdrImage;
extern const FileType kFileTypeTexture;
extern const FileType kFileTypeMat;

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_TYPE_REGISTRY_H_
