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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_NODE_VALUE_COMMAND_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_NODE_VALUE_COMMAND_H_

#include <type_traits>

#include "absl/status/status.h"
#include "core/common/invocable.h"
#include "core/editor/command.h"
#include "core/editor/editor_touch.h"
#include "core/ncsb/node_handle.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::editor {

// A command that applies a value to a node in the editor.
//
// The command will also ensure that the editor tracks that the node was
// modified by the editor via SceneMetadata so that it can be correctly saved
// out, and clear that tracking on undo.
template <typename T>
class NodeValueCommand : public Command {
 public:
  template <typename ApplyValueFn>
  NodeValueCommand(NodeHandle node, T old_value, T new_value,
                   ApplyValueFn apply_value_fn);

  absl::Status Perform() override;

  absl::Status Undo() override;

 private:
  NodeHandle node_;
  T old_value_;
  T new_value_;
  Invocable<absl::Status(NodeHandle, T)> apply_value_fn_;
  UndoEditorTouchFn undo_editor_touch_fn_;
};

template <typename T>
template <typename ApplyValueFn>
NodeValueCommand<T>::NodeValueCommand(NodeHandle node, T old_value, T new_value,
                                      ApplyValueFn apply_value_fn)
    : node_(node), old_value_(old_value), new_value_(new_value) {
  using ResultT = std::invoke_result_t<ApplyValueFn, NodeHandle, T>;
  static_assert(
      std::is_same_v<ResultT, absl::Status> || std::is_void_v<ResultT>,
      "Apply value function must return absl::Status or void.");
  if constexpr (std::is_void_v<ResultT>) {
    apply_value_fn_ = [apply_value_fn](NodeHandle node, T value) {
      apply_value_fn(node, value);
      return absl::OkStatus();
    };
  } else {
    apply_value_fn_ = std::move(apply_value_fn);
  }
}

template <typename T>
absl::Status NodeValueCommand<T>::Perform() {
  if (!node_) {
    return absl::FailedPreconditionError(
        "Unable to perform command: Node is invalid.");
  }

  undo_editor_touch_fn_ = EditorTouch(node_);

  MP_RETURN_IF_ERROR(apply_value_fn_(node_, new_value_));

  return absl::OkStatus();
}

template <typename T>
absl::Status NodeValueCommand<T>::Undo() {
  if (!node_) {
    return absl::FailedPreconditionError(
        "Unable to undo command: Node is invalid.");
  }

  MP_RETURN_IF_ERROR(apply_value_fn_(node_, old_value_));

  if (undo_editor_touch_fn_) {
    undo_editor_touch_fn_();
    undo_editor_touch_fn_ = {};
  }

  return absl::OkStatus();
}
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_NODE_VALUE_COMMAND_H_
