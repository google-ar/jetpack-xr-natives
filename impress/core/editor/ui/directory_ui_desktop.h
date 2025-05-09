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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DIRECTORY_UI_DESKTOP_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DIRECTORY_UI_DESKTOP_H_

#include <memory>
#include <string>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/async/future.h"
#include "core/editor/ui/directory_ui.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"

namespace imp::editor {

class DirectoryUiDesktop : public DirectoryUi {
 public:
  static Future<std::unique_ptr<DirectoryUi>> Create(BaseView& view,
                                                     ImGuiTextFilter& filter);

  void DrawDirectoriesHeader() override;

  void DrawDirectoriesInCurrentDirectory();

  bool IsResourceInDirectory(absl::string_view resource) const override;

  std::string GetPathInDirectory(absl::string_view name) const override;

  std::string RemoveHomeDirectoryFromPath(
      absl::string_view path) const override;

  void SaveInDirectory(absl::string_view name, absl::string_view data) override;

 private:
  DirectoryUiDesktop(BaseView& view, ImGuiTextFilter& filter,
                     TexturePtr home_icon, TexturePtr folder_icon,
                     TexturePtr up_arrow_icon);

  absl::string_view GetCurrentWorkingDirectory() const;

  absl::string_view GetRepoDirectory() const;

  BaseView& view_;
  ImGuiTextFilter& filter_;

  // Impress textures for the icons.
  TexturePtr home_icon_;
  TexturePtr folder_icon_;
  TexturePtr up_arrow_icon_;

  std::string repo_directory_;
  std::string home_directory_;
  std::string current_working_directory_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DIRECTORY_UI_DESKTOP_H_
