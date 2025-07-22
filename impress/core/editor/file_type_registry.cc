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

#include "core/editor/file_type_registry.h"

#include <memory>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/editor/file_type.h"
#include "core/editor/file_type_loader.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {
namespace {
static constexpr absl::string_view kExtensionGltf = ".gltf";
static constexpr absl::string_view kExtensionGlb = ".glb";
static constexpr absl::string_view kExtensionIsf = ".isf";
static constexpr absl::string_view kExtensionTextProto = ".textproto";
static constexpr absl::string_view kExtensionExr = ".exr";
static constexpr absl::string_view kExtensionHdr = ".hdr";
static constexpr absl::string_view kExtensionTexturePng = ".png";
static constexpr absl::string_view kExtensionTextureJpg = ".jpg";
}  // namespace

const FileType kFileTypeGltf =
    FileType::Create({kExtensionGltf, kExtensionGlb});
const FileType kFileTypeIsf = FileType::Create({kExtensionIsf});
const FileType kFileTypeIsfTextProto = FileType::Create({kExtensionTextProto});
const FileType kFileTypeHdrImage =
    FileType::Create({kExtensionExr, kExtensionHdr});
const FileType kFileTypeTexture =
    FileType::Create({kExtensionTexturePng, kExtensionTextureJpg});

void FileTypeRegistry::RegisterFileTypeLoader(
    FileType file_type, std::unique_ptr<FileTypeLoader> file_type_loader) {
  file_type_loaders_.insert(
      std::make_pair(file_type, std::move(file_type_loader)));
}

FileTypeLoader* FileTypeRegistry::GetFileTypeLoaderByPath(
    absl::string_view path) const {
  for (const auto& [file_type, file_type_loader] : file_type_loaders_) {
    if (file_type.PathMatchesFileType(path)) {
      return file_type_loader.get();
    }
  }
  return nullptr;
}

Future<absl::Status> FileTypeRegistry::LoadNode(
    absl::string_view path, Invocable<void(NodeHandle)> placement_func) const {
  for (const auto& [file_type, file_type_loader] : file_type_loaders_) {
    if (file_type.PathMatchesFileType(path)) {
      return file_type_loader->LoadNode(path, std::move(placement_func));
    }
  }
  return Future<absl::Status>(
      absl::InternalError(absl::StrFormat("Unsupported file type: %s", path)));
}

}  // namespace imp::editor
