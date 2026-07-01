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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_MATERIAL_EDITOR_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_MATERIAL_EDITOR_WIDGET_H_

#include <string>

#include "absl/strings/string_view.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/render/material_registry.h"
#include "core/scene_handles/material_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/render/material_definition.proto.imp.h"

namespace imp::editor {

// Widget for creating and editing materials in the Impress Editor.
class MaterialEditorWidget : public editor::Widget, public Rememberer {
 public:
  explicit MaterialEditorWidget(BaseView& view);
  ~MaterialEditorWidget() override = default;

  void DrawImGui() override;
  absl::string_view GetName() const override { return "Material Editor"; }

 private:
  void DrawMaterialTarget();

  BaseView& view_;
  MaterialRegistry& material_registry_;

  MaterialHandle target_handle_;
  std::string previous_url_;
  // The proto we are editing
  MaterialDefinition material_definition_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_MATERIAL_EDITOR_WIDGET_H_
