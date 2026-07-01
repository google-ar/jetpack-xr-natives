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

#include "core/editor/widgets/material_editor_widget.h"

#include <string>
#include <vector>

#include "core/common/log.h"
#include "dear_imgui/imgui.h"
#include "core/common/invocable.h"
#include "core/editor/editor_field_control.h"
#include "core/editor/editor_proto_visitor.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/materials/material.h"
#include "core/render/material_registry.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/render/material_definition.proto.imp.h"

namespace imp::editor {
namespace {
constexpr ImVec4 kWarningColor = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
}  // namespace

MaterialEditorWidget::MaterialEditorWidget(BaseView& view)
    : view_(view),
      material_registry_(view.GetRegistry().GetOrCreate<MaterialRegistry>()) {}

void MaterialEditorWidget::DrawImGui() { DrawMaterialTarget(); }

void MaterialEditorWidget::DrawMaterialTarget() {
  EditorFieldControl::ShowDefaultControl("Material", &target_handle_, nullptr,
                                         editor::EditorControlFlags::kDefault,
                                         &view_);

  if (!target_handle_.GetMaterial()) {
    ImGui::TextColored(kWarningColor, "Material not loaded");
    return;
  }

  if (previous_url_ != target_handle_.GetUrl()) {
    const MaterialDefinition* def =
        material_registry_.GetMaterialDefinition(target_handle_.GetUrl());
    if (!def) {
      ImGui::TextColored(kWarningColor, "Material definition not available");
      return;
    }
    material_definition_ = *def;
    previous_url_ = target_handle_.GetUrl();
  }
  ImGui::Separator();

  ImGui::Text("Shader url: %s", material_definition_.asset.c_str());

  ImGui::Separator();
  // Use EditorProtoVisitor to draw the proto fields
  EditorProtoVisitor<MaterialDefinition> visitor(material_definition_, &view_);
  material_definition_.Visit(visitor, 0, nullptr);

  ImGui::Separator();
  if (ImGui::Button("Save")) {
    auto& registry = view_.GetRegistry().GetOrCreate<MaterialRegistry>();
    absl::Status status = registry.UpdateMaterialParameters(
        target_handle_.GetUrl(), material_definition_,
        imp::Invocable<void(BorrowedMaterialPtr)>(
            [this](BorrowedMaterialPtr material) {
              std::vector<MaterialDefinition::Parameter> params;
              params.reserve(material_definition_.parameters.size());
              for (const auto& p : material_definition_.parameters) {
                params.push_back(p);
              }
              MaterialFactory::SetMaterialParameters(
                  view_, material.operator->(), params)
                  .KeptBy(this);
            }));
    if (!status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to update material parameters: " << status;
    }
    // TODO: Sync the changes with .imp.material file on disk.
  }
}

}  // namespace imp::editor
