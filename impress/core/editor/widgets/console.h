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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_CONSOLE_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_CONSOLE_H_

#include <cstddef>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "core/common/platform_helpers.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp::editor {

// Window at the bottom of the screen showing Impress info, warning, and error
// logs. Useful for identifying problems with the model parsing and loading.
class Console : public Widget, public imp::Rememberer {
 public:
  explicit Console(BaseView& view);
  ~Console() override;
  absl::string_view GetName() const override { return "Console"; }
  void DrawImGui() override;

 private:
  // Handles forwarding a log from imp output to the console widget.
  static void HandleLog(void* context, output::OutputKind kind,
                        absl::string_view log);
  // Helper function for rendering the log settings.
  void DrawLogSettings();
  // Helper function for rendering the logs.
  void DrawLogs();
  // Clears all logs from the all_logs_ list.
  void ClearLogs();
  // An object that encapsulates the message and type of log.
  struct ConsoleLog {
    std::string message;
    output::OutputKind output_kind;
  };

  BaseView& view_;
  // Stores a list of all of the logs; it is used to easily keep track of their
  // ordering.
  absl::Mutex logs_mutex_;
  std::vector<ConsoleLog> all_logs_ ABSL_GUARDED_BY(logs_mutex_) = {};
  // The all_logs_ vector acts as a circular buffer, so this is the index of the
  // first log in the buffer. When max logs are reached, early logs are
  // overwritten.
  size_t log_start_index_ ABSL_GUARDED_BY(logs_mutex_) = 0;
  // Stores the count of each type of log to use for setting the ListBox size.
  tsl::robin_map<output::OutputKind, int> log_count_
      ABSL_GUARDED_BY(logs_mutex_) = {{output::OutputKind::kInfo, 0},
                                      {output::OutputKind::kWarning, 0},
                                      {output::OutputKind::kError, 0}};
  // Current selected log in the console.
  int selected_item_index_ = 0;
  // Impress textures for the icons.
  imp::TexturePtr info_icon_;
  imp::TexturePtr warning_icon_;
  imp::TexturePtr error_icon_;
  // Keep track if a new log has been added in the previous frame. This is used
  // to force the console to scroll down to the new log.
  bool new_log_added_ = false;
  // Stores the filtering log settings.
  bool filter_log_ = false;
  output::OutputKind filter_by_;
  float header_height_;
  float inspector_height_;
  float hierarchy_height_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_CONSOLE_H_
