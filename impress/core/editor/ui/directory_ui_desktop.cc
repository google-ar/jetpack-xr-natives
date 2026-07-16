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

#include "core/editor/ui/directory_ui_desktop.h"

#include <dirent.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/editor/ui/directory_ui.h"
#include "core/editor/ui/ui_helpers.h"
#include "core/editor/widgets/icons/texture_assets.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"

namespace imp::editor {

constexpr ImVec2 kButtonSize(36, 36);

Future<std::unique_ptr<DirectoryUi>> DirectoryUiDesktop::Create(
    BaseView& view, ImGuiTextFilter& filter) {
  // Load all the icons.
  Future<AssetPtr<ImageAsset>> home_icon_future =
      view.GetAssetManager().LoadImage(texture_data::kHomePng);
  Future<AssetPtr<ImageAsset>> folder_icon_future =
      view.GetAssetManager().LoadImage(texture_data::kFolderPng);
  Future<AssetPtr<ImageAsset>> up_arrow_icon_future =
      view.GetAssetManager().LoadImage(texture_data::kUpArrowPng);

  return home_icon_future.Merge(folder_icon_future, up_arrow_icon_future)
      .Then([&view, &filter](
                std::tuple<AssetPtr<imp::ImageAsset>, AssetPtr<imp::ImageAsset>,
                           AssetPtr<imp::ImageAsset>>
                    tuple) -> std::unique_ptr<DirectoryUi> {
        auto [home_icon_asset, folder_icon_asset, up_arrow_icon_asset] =
            std::move(tuple);

        OwnedTexturePtr home_icon =
            view.GetTextureFactory().CreateTexture(home_icon_asset);
        OwnedTexturePtr folder_icon =
            view.GetTextureFactory().CreateTexture(folder_icon_asset);
        OwnedTexturePtr up_arrow_icon =
            view.GetTextureFactory().CreateTexture(up_arrow_icon_asset);

        return absl::WrapUnique<DirectoryUi>(new DirectoryUiDesktop(
            view, filter, std::move(home_icon), std::move(folder_icon),
            std::move(up_arrow_icon)));
      });
}

DirectoryUiDesktop::DirectoryUiDesktop(BaseView& view, ImGuiTextFilter& filter,
                                       OwnedTexturePtr home_icon,
                                       OwnedTexturePtr folder_icon,
                                       OwnedTexturePtr up_arrow_icon)
    : view_(view),
      filter_(filter),
      home_icon_(std::move(home_icon)),
      folder_icon_(std::move(folder_icon)),
      up_arrow_icon_(std::move(up_arrow_icon)) {
  // Find the repo directory.
  // If using "blaze run" use the environment variable set by blaze.
  const char* wd = std::getenv("BUILD_WORKING_DIRECTORY");
  if (wd) {
    repo_directory_ = wd;
  } else {
    // Otherwise, get it as the current directory.
    char cwd[PATH_MAX];
    getcwd(cwd, sizeof(cwd));
    repo_directory_ = cwd;
  }

  // Find the root directory.
  if (!view_.GetContext().GetArguments().empty()) {
    const std::string kBinPrefix = "bin/";
    std::string exec_path = view_.GetContext().GetArguments().at(0);
    size_t bin_length = exec_path.find(kBinPrefix) + kBinPrefix.size();
    exec_path = exec_path.substr(bin_length, exec_path.size() - bin_length);
    std::string exec_dir = exec_path.substr(0, exec_path.find_last_of('/'));
    home_directory_ = absl::StrCat(repo_directory_, "/", exec_dir);
  } else {
    // When running in a unit test, there are no arguments so we use placeholder
    // values.
    repo_directory_ = "path/to/repo";
    home_directory_ = absl::StrCat(repo_directory_, "/home");
  }

  // Start the current directory as the root directory.
  current_working_directory_ = home_directory_;
}

void DirectoryUiDesktop::DrawDirectoriesHeader() {
  if (ImGui::ImageButton("##home", home_icon_->GetTexture(), kButtonSize)) {
    current_working_directory_ = home_directory_;
  }

  ImGui::SameLine();

  bool is_in_home_dir = false;
  if (current_working_directory_ == home_directory_) {
    is_in_home_dir = true;
    ImGui::BeginDisabled();
  }

  if (ImGui::ImageButton("##up", up_arrow_icon_->GetTexture(), kButtonSize)) {
    current_working_directory_ = current_working_directory_.substr(
        0, current_working_directory_.find_last_of('/'));
  }

  if (is_in_home_dir) {
    ImGui::EndDisabled();
  }

  ImGui::SameLine();

  std::string repo_relative_path = current_working_directory_.substr(
      repo_directory_.size() + 1,
      current_working_directory_.size() - repo_directory_.size() - 1);

  float button_half_height = kButtonSize.y * 0.5f;
  float text_quarter_height = ImGui::GetTextLineHeight() * 0.25f;
  ImGui::SetCursorPosY((button_half_height - text_quarter_height) +
                       ImGui::GetCursorPosY());
  ImGui::Text("%s", repo_relative_path.c_str());
}

void DirectoryUiDesktop::DrawDirectoriesInCurrentDirectory() {
  struct dirent** dirlist;
  int num_entries;
  num_entries =
      scandir(current_working_directory_.c_str(), &dirlist, nullptr, alphasort);
  if (num_entries > 0) {
    for (int i = 0; i < num_entries; ++i) {
      struct dirent* dir = dirlist[i];
      if (dir == nullptr) {
        continue;
      }

      std::string entry_name = dir->d_name;
      std::string full_entry_path =
          absl::StrCat(current_working_directory_, "/", entry_name);

      auto is_entry_valid = [&]() {
        if (entry_name == "." || entry_name == "..") {
          return false;
        }

        struct stat statbuf;
        int stat_result = stat(full_entry_path.c_str(), &statbuf);

        if (stat_result != 0) {
          return false;
        }

        if (!S_ISDIR(statbuf.st_mode)) {
          return false;
        }

        if (!filter_.PassFilter(entry_name.c_str())) {
          return false;
        }
        return true;
      };

      if (!is_entry_valid()) {
        free(dir);
        continue;
      }

      ImGui::TableNextColumn();

      ImGuiCenterNextHorizontally(kTableEntryImageSize.x,
                                  IncludePadding::kCell);
      if (ImGui::ImageButton(entry_name.c_str(), folder_icon_->GetTexture(),
                             kTableEntryImageSize)) {
        current_working_directory_ =
            absl::StrCat(current_working_directory_, "/", entry_name);
      }

      ImVec2 text_size = ImGui::CalcTextSize(entry_name.c_str(), nullptr, false,
                                             ImGui::GetContentRegionAvail().x);
      ImGuiCenterNextHorizontally(text_size.x, IncludePadding::kNone);
      ImGui::TextWrapped(entry_name.c_str(), "");

      free(dir);
    }
    free(dirlist);
  }
}

bool DirectoryUiDesktop::IsResourceInDirectory(
    absl::string_view resource) const {
  absl::string_view repo_relative_dir = GetDirectoryFromFilename(resource);
  std::string full_dir = absl::StrCat(repo_directory_, "/", repo_relative_dir);

  return full_dir == current_working_directory_;
}

std::string DirectoryUiDesktop::GetPathInDirectory(
    absl::string_view name) const {
  absl::string_view repo_relative_directory =
      GetCurrentWorkingDirectory().substr(
          GetRepoDirectory().size() + 1,
          GetCurrentWorkingDirectory().size() - GetRepoDirectory().size() - 1);

  std::string repo_relative_path =
      absl::StrCat(repo_relative_directory, "/", name);

  return repo_relative_path;
}

std::string DirectoryUiDesktop::RemoveHomeDirectoryFromPath(
    absl::string_view path) const {
  absl::string_view home_directory = home_directory_;
  absl::string_view repo_relative_directory =
      home_directory.substr(GetRepoDirectory().size() + 1);

  size_t last_of_index = path.find(repo_relative_directory);
  if (last_of_index != std::string::npos) {
    return std::string(
        path.substr(last_of_index + repo_relative_directory.size() + 1));
  }

  return std::string(path);
}

void DirectoryUiDesktop::SaveInDirectory(absl::string_view name,
                                         absl::string_view data) {
  std::string saved_path =
      absl::StrCat(GetCurrentWorkingDirectory(), "/", name);

  BufferAccess buffer = BufferAccess::Wrap(
      reinterpret_cast<const uint8_t*>(data.data()), data.size());

  absl::Status save_result = SaveBinary(saved_path, buffer);
  if (!save_result.ok()) {
    IMP_LOG(imp::ERROR) << "Unable to save: " << save_result;
  }
}

absl::string_view DirectoryUiDesktop::GetCurrentWorkingDirectory() const {
  return current_working_directory_;
}

absl::string_view DirectoryUiDesktop::GetRepoDirectory() const {
  return repo_directory_;
}

}  // namespace imp::editor
