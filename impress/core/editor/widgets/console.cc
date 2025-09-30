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

#include "core/editor/widgets/console.h"

#include <string>
#include <tuple>
#include <utility>

#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "dear_imgui/imgui.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/platform_helpers.h"
#include "core/editor/widgets/icons/texture_assets.h"
#include "core/render/texture_asset.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"

namespace imp::editor {

constexpr absl::string_view kConsoleClearLabel = "Clear";
constexpr char kConsoleFilterLabel[] = "Filter By";
constexpr absl::string_view kConsoleAllLabel = "All";
constexpr absl::string_view kConsoleInfoLabel = "Info";
constexpr absl::string_view kConsoleWarningLabel = "Warning";
constexpr absl::string_view kConsoleErrorLabel = "Error";
constexpr absl::string_view kConsoleListBoxLabel = "##log-list";
// This is the y position of the scroll value when a new log has been added.
constexpr float kAutoScrollValue = 1.0f;
// Size of the image shown next to the logs. This helps to differentiate the
// type of log.
const ImVec2 kLogImageSize = ImVec2(20.0f, 20.0f);
// Different color for 'Filter By' to differentiate from clickable buttons.
const ImVec4 kFilterByColor = {1.0f, 1.0f, 1.0f, 0.6f};
// Maximum number of logs to store in the console. This is to prevent the
// console from growing indefinitely. After this number, the oldest logs will be
// overwritten.
const int kMaxLogs = 10000;

Console::Console(BaseView& view) : view_(view) {
  // TODO: Console log frequently crashes in split engine mode.
  if (!view_.GetSplitEngineSerializer()) {
    output::AddExternalLogHandler(this, HandleLog);
    output::Configure(true);
  }
  // Load the images for the log icons.
  Future<AssetPtr<imp::TextureAsset>> info_icon_future =
      view_.GetAssetManager().LoadTexture(texture_data::kInfoPng);
  Future<AssetPtr<imp::TextureAsset>> warning_icon_future =
      view_.GetAssetManager().LoadTexture(texture_data::kWarningPng);
  Future<AssetPtr<imp::TextureAsset>> error_icon_future =
      view_.GetAssetManager().LoadTexture(texture_data::kErrorPng);
  info_icon_future.Merge(warning_icon_future, error_icon_future)
      .Then([this](std::tuple<AssetPtr<imp::TextureAsset>,
                              AssetPtr<imp::TextureAsset>,
                              AssetPtr<imp::TextureAsset>>
                       tuple) mutable {
        std::tie(info_icon_, warning_icon_, error_icon_) = std::move(tuple);
      })
      .KeptBy(&view_);
}

Console::~Console() { output::RemoveExternalLogHandler(this); }

void Console::DrawImGui() {
  DrawLogSettings();
  DrawLogs();
}

void Console::HandleLog(void* context, output::OutputKind kind,
                        absl::string_view log) {
  if (log.empty()) {
    return;
  }

  Console* console_ui_widget = reinterpret_cast<Console*>(context);
  // Treat Fatal and Max errors are normal errors in the console.
  if (kind == output::OutputKind::kFatal || kind == output::OutputKind::kMax)
    kind = output::OutputKind::kError;
  {
    absl::MutexLock lock(&console_ui_widget->logs_mutex_);
    if (console_ui_widget->all_logs_.size() > kMaxLogs) {
      console_ui_widget
          ->log_count_[console_ui_widget
                           ->all_logs_[console_ui_widget->log_start_index_]
                           .output_kind]--;
      console_ui_widget->all_logs_[console_ui_widget->log_start_index_] =
          ConsoleLog{std::string(log), kind};
      console_ui_widget->log_start_index_++;
      if (console_ui_widget->log_start_index_ >= kMaxLogs) {
        console_ui_widget->log_start_index_ = 0;
      }
    } else {
      console_ui_widget->all_logs_.push_back(
          ConsoleLog{std::string(log), kind});
    }
    console_ui_widget->log_count_[kind]++;
    console_ui_widget->new_log_added_ = true;
  }
}

void Console::DrawLogSettings() {
  ImGui::TextColored(kFilterByColor, "%s", kConsoleFilterLabel);
  ImGui::SameLine();
  ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(0.5f, 0.5f));
  if (ImGui::Selectable(kConsoleAllLabel.data(), !filter_log_, 0,
                        ImGui::CalcTextSize(kConsoleAllLabel.data()))) {
    filter_log_ = false;
  }
  ImGui::SameLine();
  if (ImGui::Selectable(kConsoleInfoLabel.data(),
                        filter_log_ && filter_by_ == output::OutputKind::kInfo,
                        0, ImGui::CalcTextSize(kConsoleInfoLabel.data()))) {
    filter_log_ = true;
    filter_by_ = output::OutputKind::kInfo;
  }
  ImGui::SameLine();
  if (ImGui::Selectable(
          kConsoleWarningLabel.data(),
          filter_log_ && filter_by_ == output::OutputKind::kWarning, 0,
          ImGui::CalcTextSize(kConsoleWarningLabel.data()))) {
    filter_log_ = true;
    filter_by_ = output::OutputKind::kWarning;
  }
  ImGui::SameLine();
  if (ImGui::Selectable(kConsoleErrorLabel.data(),
                        filter_log_ && filter_by_ == output::OutputKind::kError,
                        0, ImGui::CalcTextSize(kConsoleErrorLabel.data()))) {
    filter_log_ = true;
    filter_by_ = output::OutputKind::kError;
  }
  ImVec2 clear_button_size = ImGui::CalcTextSize(kConsoleClearLabel.data());
  ImGui::SameLine(ImGui::GetContentRegionAvail().x - clear_button_size.x -
                  ImGui::GetStyle().WindowPadding.x * 2.0f);
  if (ImGui::Selectable(kConsoleClearLabel.data(), false, 0,
                        clear_button_size)) {
    ClearLogs();
  }
  ImGui::PopStyleVar();
}

void Console::DrawLogs() {
  ImVec2 rect_max = ImGui::GetItemRectMax();
  ImGui::PushItemWidth(rect_max.x);

  bool is_docking_layout =
      ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_DockingEnable;
  ImVec2 list_box_size =
      is_docking_layout
          ? ImVec2(-1.0f,
                   -1.0f)  // Negative values means to fill the available space.
          : ImVec2(0.0f, 0.0f);
  bool list_box_header =
      ImGui::BeginListBox(kConsoleListBoxLabel.data(), list_box_size);
  {
    absl::MutexLock lock(&logs_mutex_);
    if (all_logs_.empty()) {
      ImGui::Text("No logs to display.");
      ImGui::EndListBox();
      return;
    }
    int i = log_start_index_;
    while (true) {
      if (!filter_log_ || all_logs_.at(i).output_kind == filter_by_) {
        switch (all_logs_.at(i).output_kind) {
          case output::OutputKind::kInfo:
            if (info_icon_) {
              ImGui::Image(info_icon_->GetFilamentTexture(), kLogImageSize);
            }
            break;
          case output::OutputKind::kWarning:
            if (warning_icon_) {
              ImGui::Image(warning_icon_->GetFilamentTexture(), kLogImageSize);
            }
            break;
          case output::OutputKind::kError:
          default:
            if (error_icon_) {
              ImGui::Image(error_icon_->GetFilamentTexture(), kLogImageSize);
            }
            break;
        }
        ImGui::SameLine();
        ImGui::TextWrapped(all_logs_[i].message.c_str());
      }

      i++;
      if (i >= all_logs_.size()) {
        i = 0;
      }
      if (i == log_start_index_) {
        break;
      }
    }
  }
  if (list_box_header) {
    if (new_log_added_) {
      ImGui::SetScrollHereY(kAutoScrollValue);
      new_log_added_ = false;
    }
    ImGui::EndListBox();
  }
}

void Console::ClearLogs() {
  absl::MutexLock lock(&logs_mutex_);
  all_logs_.clear();
  log_start_index_ = 0;
  log_count_[output::OutputKind::kInfo] = 0;
  log_count_[output::OutputKind::kWarning] = 0;
  log_count_[output::OutputKind::kError] = 0;
}

}  // namespace imp::editor
