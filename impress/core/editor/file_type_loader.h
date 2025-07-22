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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_TYPE_LOADER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_TYPE_LOADER_H_

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/view/base_view.h"

namespace imp {
class NodeHandle;
}  // namespace imp

namespace imp::editor {

// Interface for loading a node from a file.
//
// Instances of FileTypeLoader may be associated with a FileType in the
// FileTypeRegistry.
class FileTypeLoader {
 public:
  FileTypeLoader(BaseView& view) : view_(view) {}

  virtual Future<absl::Status> LoadNode(
      absl::string_view path, Invocable<void(NodeHandle)> placement_func) = 0;

  virtual ~FileTypeLoader() = default;

  BaseView& GetView() const { return view_; }

 private:
  BaseView& view_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_FILE_TYPE_LOADER_H_
