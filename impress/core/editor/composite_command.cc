/*
 * Copyright 2026 Google LLC
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

#include "core/editor/composite_command.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "core/editor/command.h"

namespace imp::editor {

CompositeCommand::CompositeCommand(
    std::vector<std::unique_ptr<Command>> commands)
    : commands_(std::move(commands)) {}

CompositeCommand::~CompositeCommand() = default;

absl::Status CompositeCommand::Perform() {
  for (const std::unique_ptr<Command>& command : commands_) {
    const absl::Status status = command->Perform();
    if (!status.ok()) {
      return status;
    }
  }
  return absl::OkStatus();
}

absl::Status CompositeCommand::Undo() {
  for (auto it = commands_.rbegin(); it != commands_.rend(); ++it) {
    const absl::Status status = (*it)->Undo();
    if (!status.ok()) {
      return status;
    }
  }
  return absl::OkStatus();
}

}  // namespace imp::editor
