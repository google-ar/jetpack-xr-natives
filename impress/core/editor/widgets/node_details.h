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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_NODE_DETAILS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_NODE_DETAILS_H_

#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/common/rememberer.h"
#include "core/editor/command_manager.h"
#include "core/editor/widget.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Display the basic details of a node (transform, enabled/disabled).
class NodeDetails : public editor::Widget, public imp::Rememberer {
 public:
  explicit NodeDetails(BaseView& base_view);
  void DrawImGui() override;
  absl::string_view GetName() const override { return "Node"; }
  bool HasContent() const override;

 private:
  // Saves the active node to file.
  absl::Status Save();

  BaseView& view_;
  CommandManager& command_manager_;
  NodeHandle active_node_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_NODE_DETAILS_H_
