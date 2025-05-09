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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_FUNCTION_COMMAND_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_FUNCTION_COMMAND_H_

#include <type_traits>
#include <utility>

#include "absl/status/status.h"
#include "core/common/invocable.h"
#include "core/editor/command.h"

namespace imp::editor {

// Defines a command that wraps functions for performing and undoing an action.
class FunctionCommand : public Command {
 public:
  template <typename PerformFn, typename UndoFn>
  FunctionCommand(PerformFn perform_fn, UndoFn undo_fn);

  absl::Status Perform() override;
  absl::Status Undo() override;

 private:
  imp::Invocable<absl::Status()> perform_fn_;
  imp::Invocable<absl::Status()> undo_fn_;
};

template <typename PerformFn, typename UndoFn>
FunctionCommand::FunctionCommand(PerformFn perform_fn, UndoFn undo_fn) {
  using PerformResultT = std::invoke_result_t<PerformFn>;
  using UndoResultT = std::invoke_result_t<UndoFn>;
  static_assert(std::is_same_v<PerformResultT, absl::Status> ||
                    std::is_void_v<PerformResultT>,
                "Perform function must return absl::Status or void.");
  static_assert(
      std::is_same_v<UndoResultT, absl::Status> || std::is_void_v<UndoResultT>,
      "Undo function must return absl::Status or void.");

  if constexpr (std::is_void_v<PerformResultT>) {
    perform_fn_ = [perform_fn = std::move(perform_fn)]() {
      perform_fn();
      return absl::OkStatus();
    };
  } else {
    perform_fn_ = std::move(perform_fn);
  }
  if constexpr (std::is_void_v<UndoResultT>) {
    undo_fn_ = [undo_fn = std::move(undo_fn)]() {
      undo_fn();
      return absl::OkStatus();
    };
  } else {
    undo_fn_ = std::move(undo_fn);
  }
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_FUNCTION_COMMAND_H_
