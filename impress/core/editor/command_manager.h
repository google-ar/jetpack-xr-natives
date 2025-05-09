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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_COMMAND_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_COMMAND_MANAGER_H_

#include <memory>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/editor/command.h"

namespace imp::editor {

// CommmandManager performs and records actions done in the Impress Editor.
//
// The recorded actions can be undone and redone.
class CommandManager {
 public:
  // Creates a command of type CommandT with the args passed in. Then
  // immediately performs the command.
  template <typename CommandT, typename... Args>
  void PerformCommand(Args&&... args);

  // Undoes the most recently performed command.
  void Undo();

  // Performs the most recently undone command.
  void Redo();

 private:
  // Insert new command.
  void PushCommand(std::unique_ptr<Command> command);
  bool is_command_manager_running_ = false;
  std::vector<std::unique_ptr<Command>> undo_commands_;
  std::vector<std::unique_ptr<Command>> redo_commands_;
};

template <typename CommandT, typename... Args>
void CommandManager::PerformCommand(Args&&... args) {
  // Changes triggered by CommandManager itself are not recorded.
  if (!is_command_manager_running_) {
    std::unique_ptr<Command> command =
        std::make_unique<CommandT>(std::forward<Args>(args)...);
    if (absl::Status status = command->Perform(); !status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to perform command: " << status.ToString();
      return;
    }
    PushCommand(std::move(command));
  }
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_command_manager_H_
