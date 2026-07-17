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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_WEB_UI_REMOTE_EDITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_WEB_UI_REMOTE_EDITOR_H_

#include "core/editor/remote_editor/remote_editor.h"
#include "core/editor/remote_editor/remote_editor_info.h"
#include "core/view/base_view.h"

namespace imp::editor {

// A specialized RemoteEditor for streaming a Web-based UI.
class WebUiRemoteEditor : public RemoteEditor {
 public:
  explicit WebUiRemoteEditor(BaseView& view);
  ~WebUiRemoteEditor() override = default;

  bool HasConfigChanged(
      const RemoteEditorInfo::RemoteUiConfig& config) const override;

  bool IsDisplayModeSwitchingAllowed() const override;

 protected:
  void OnStart(const RemoteEditorInfo::RemoteUiConfig& config) final;
  void OnStop() final;

  // Overridden by derived classes to perform specific start/stop logic.
  virtual void OnWebUiStart() {}
  virtual void OnWebUiStop() {}

  RemoteEditorInfo::RemoteUiConfig config_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_WEB_UI_REMOTE_EDITOR_H_
