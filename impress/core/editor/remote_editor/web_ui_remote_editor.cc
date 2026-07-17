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

#include "core/editor/remote_editor/web_ui_remote_editor.h"

#include <variant>

#include "core/editor/remote_editor/remote_editor.h"
#include "core/editor/remote_editor/remote_editor_info.h"
#include "core/view/base_view.h"

namespace imp::editor {

WebUiRemoteEditor::WebUiRemoteEditor(BaseView& view) : RemoteEditor(view) {}

bool WebUiRemoteEditor::HasConfigChanged(
    const RemoteEditorInfo::RemoteUiConfig& config) const {
  return config_ != config;
}

bool WebUiRemoteEditor::IsDisplayModeSwitchingAllowed() const {
  return std::holds_alternative<
      imp::editor::RemoteEditorInfo::RemoteEditorStreamingConfig>(config_);
}

void WebUiRemoteEditor::OnStart(
    const RemoteEditorInfo::RemoteUiConfig& config) {
  config_ = config;
  OnWebUiStart();
}

void WebUiRemoteEditor::OnStop() { OnWebUiStop(); }

}  // namespace imp::editor
