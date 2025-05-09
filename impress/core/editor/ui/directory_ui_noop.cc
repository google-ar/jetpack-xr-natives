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

#include "core/editor/ui/directory_ui_noop.h"

#include <string>

#include "absl/strings/string_view.h"

namespace imp::editor {

void DirectoryUiNoop::DrawDirectoriesHeader() {}

void DirectoryUiNoop::DrawDirectoriesInCurrentDirectory() {}

bool DirectoryUiNoop::IsResourceInDirectory(absl::string_view resource) const {
  return true;
}

std::string DirectoryUiNoop::GetPathInDirectory(absl::string_view name) const {
  return std::string(name);
}

std::string DirectoryUiNoop::RemoveHomeDirectoryFromPath(
    absl::string_view path) const {
  return std::string(path);
}

void DirectoryUiNoop::SaveInDirectory(absl::string_view name,
                                      absl::string_view data) {}

}  // namespace imp::editor
