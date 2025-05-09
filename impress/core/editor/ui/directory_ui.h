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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DIRECTORY_UI_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DIRECTORY_UI_H_

#include <string>

#include "absl/strings/string_view.h"

namespace imp::editor {

class DirectoryUi {
 public:
  virtual ~DirectoryUi() {}

  virtual void DrawDirectoriesHeader() = 0;

  virtual void DrawDirectoriesInCurrentDirectory() = 0;

  virtual bool IsResourceInDirectory(absl::string_view resource) const = 0;

  virtual std::string GetPathInDirectory(absl::string_view name) const = 0;

  virtual std::string RemoveHomeDirectoryFromPath(
      absl::string_view path) const = 0;

  virtual void SaveInDirectory(absl::string_view name,
                               absl::string_view data) = 0;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DIRECTORY_UI_H_
