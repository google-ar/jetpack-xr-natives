
// Copyright 2025 Google LLC
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
#include "core/editor/file_type.h"

#include <string>
#include <vector>

#include "absl/strings/match.h"
#include "absl/strings/string_view.h"
namespace imp::editor {

FileType FileType::Create(std::vector<absl::string_view> extensions) {
  // Registry of known file types.
  static int nextId = 0;
  FileType new_type;
  new_type.extensions_ =
      std::vector<std::string>(extensions.begin(), extensions.end());
  new_type.id_ = ++nextId;
  return new_type;
}

bool FileType::PathMatchesFileType(absl::string_view path) const {
  for (auto it = extensions_.cbegin(); it != extensions_.cend(); ++it) {
    if (absl::EndsWith(path, *it)) {
      return true;
    }
  }
  return false;
}
}  // namespace imp::editor
