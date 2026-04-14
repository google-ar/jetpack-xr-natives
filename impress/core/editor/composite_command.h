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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPOSITE_COMMAND_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPOSITE_COMMAND_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "core/editor/command.h"

namespace imp::editor {

// A command that executes a sequence of other commands.
// Commands are executed in the order they are added to the composite command.
// Undo executes the commands in the reverse order they are added.
// Allows us to capture a large number of commands in a single undo/redo step.
class CompositeCommand : public Command {
 public:
  explicit CompositeCommand(std::vector<std::unique_ptr<Command>> commands);

  ~CompositeCommand() override;

  // Executes all commands in order.
  absl::Status Perform() override;

  // Undoes all commands in reverse order.
  absl::Status Undo() override;

 private:
  std::vector<std::unique_ptr<Command>> commands_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPOSITE_COMMAND_H_
