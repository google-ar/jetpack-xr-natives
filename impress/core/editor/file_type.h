/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_TYPE_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_TYPE_H_

#include <string>
#include <vector>

#include "absl/strings/string_view.h"

namespace imp::editor {
// Defines a "file type" for the editor/previewer.
//
// FileTypes typically have the following:
// - A list of extensions used to auto detect the file type.
// - A generated id. The id is used internally for hashing.
//
// New FileTypes are registered with the Create() function.
//
// FileTypes can be associated with a FileTypeLoader in the FileTypeRegistry.
class FileType {
 public:
  // Create a new file type.
  //
  // The caller is responsible for ensuring that each file type is only
  // registered once.
  static FileType Create(std::vector<absl::string_view> extensions);

  template <typename H>
  friend H AbslHashValue(H h, const FileType& type) {
    return H::combine(std::move(h), type.id_);
  }
  bool operator==(const FileType& other) const { return id_ == other.id_; }

  // An identifier for this file type.
  int GetId() { return id_; }

  // Returns the extensions that this file type can have.
  const std::vector<std::string>& GetExtensions() { return extensions_; }

  // Returns true if the path matches any of the extensions for this file type.
  bool PathMatchesFileType(absl::string_view path) const;

 private:
  // Default constructor is private. Use the Create() function instead.
  FileType() = default;
  // The extensions that this file type can have.
  std::vector<std::string> extensions_;
  // An identifier for this file type.
  int id_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_TYPE_H_
