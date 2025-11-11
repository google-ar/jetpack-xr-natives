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

#include "core/editor/command_manager.h"

#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/editor/command.h"

namespace imp::editor {

void CommandManager::PushCommand(std::unique_ptr<Command> command) {
  undo_commands_.push_back(std::move(command));
  redo_commands_.clear();
}

void CommandManager::Undo() {
  is_command_manager_running_ = true;
  // TODO Discard while loop.
  // Make invalid undo/redo command FATAL and return.
  while (!undo_commands_.empty()) {
    const auto undo_command_iter = undo_commands_.end() - 1;
    absl::Status status = undo_command_iter->get()->Undo();
    if (status.ok()) {
      redo_commands_.push_back(std::move(*undo_command_iter));
      undo_commands_.erase(undo_command_iter);
      break;
    } else {
      undo_commands_.pop_back();
    }
  }
  is_command_manager_running_ = false;
}

void CommandManager::Redo() {
  is_command_manager_running_ = true;
  // TODO Discard while loop.
  // Make invalid undo/redo command FATAL and return.
  while (!redo_commands_.empty()) {
    const auto redo_command_iter = redo_commands_.end() - 1;
    absl::Status status = redo_command_iter->get()->Perform();
    if (status.ok()) {
      undo_commands_.push_back(std::move(*redo_command_iter));
      redo_commands_.erase(redo_command_iter);
      break;
    } else {
      redo_commands_.pop_back();
      IMP_LOG(imp::INFO) << "One redo command skipped due to: " << status.ToString();
    }
  }
  is_command_manager_running_ = false;
}

}  // namespace imp::editor
