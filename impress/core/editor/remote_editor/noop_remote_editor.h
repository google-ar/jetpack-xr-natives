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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_NOOP_REMOTE_EDITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_NOOP_REMOTE_EDITOR_H_

#include "core/editor/remote_editor/remote_editor.h"
#include "core/editor/remote_editor/remote_editor_info.h"
#include "core/view/base_view.h"

namespace imp::editor {

// A RemoteEditor implementation that does nothing, for platforms where
// remote editing is not supported.
class NoopRemoteEditor : public RemoteEditor {
 public:
  explicit NoopRemoteEditor(BaseView& view) : RemoteEditor(view) {}

  bool HasConfigChanged(
      const RemoteEditorInfo::RemoteUiConfig& config) const override {
    return false;
  }

 protected:
  void OnStart(const RemoteEditorInfo::RemoteUiConfig& config) override {
    running_ = false;  // Ensure it stays off since it's a no-op.
  }
  void OnStop() override {}
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_NOOP_REMOTE_EDITOR_H_
