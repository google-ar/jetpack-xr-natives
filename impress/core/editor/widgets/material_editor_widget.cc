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

#include <functional>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/types/variant.h"
#include "dear_imgui/imgui.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/libs/math/include/math/vec2.h"
#include "filament/libs/math/include/math/vec3.h"
#include "filament/libs/math/include/math/vec4.h"
#include "core/common/invocable.h"
#include "core/editor/editor_field_control.h"
#include "core/editor/events.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/editor/widgets/asset_thumbnail_provider.h"
#include "core/editor/widgets/window/window_configuration.h"
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
      material_registry_(view.GetRegistry().GetOrCreate<MaterialRegistry>()) {
  edit_material_connection_ =
      view_.GetDispatcher().Connect([this](const EditMaterialEvent& event) {
        target_handle_.AssignMaterial(event.material_url, event.material);
        absl::StatusOr<std::reference_wrapper<WindowConfiguration>>
            window_config_or = view_.GetRegistry().Get<WindowConfiguration>();
        if (window_config_or.ok()) {
          window_config_or->get().SetWindowVisibility(
              GetName(), WindowConfiguration::WindowVisibility::kVisible,
              /*request_focus=*/true);
        }
      });
}

void MaterialEditorWidget::DrawImGui() { DrawMaterialTarget(); }

void MaterialEditorWidget::DrawMaterialTarget() {
  EditorFieldControl::ShowDefaultControl("Name: ", &target_handle_, nullptr,
                                         editor::EditorControlFlags::kDefault,
                                         &view_, /*show_edit_button=*/false);

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

  EditorFieldControl::ShowDefaultControl(
      "Shader: ", &material_definition_.asset, nullptr,
      editor::EditorControlFlags::kDisplayLabel, &view_, /*is_url=*/true);

  ImGui::Separator();

  DrawParametersUI();

  ImGui::Separator();
  if (ImGui::Button("Save")) {
    MaterialRegistry& registry =
        view_.GetRegistry().GetOrCreate<MaterialRegistry>();
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

void MaterialEditorWidget::DrawParametersUI() {
  ImGui::Text("Parameters:");
  for (auto& param : material_definition_.parameters) {
    ImGui::Spacing();
    ImGui::PushID(param.name.c_str());

    if (param.image_asset_texture()) {
      std::string* path = param.mutable_image_asset_texture();
      DrawTextureParameterUI(param, path);
    } else if (param.registered_texture()) {
      std::string* name = param.mutable_registered_texture();
      DrawTextureParameterUI(param, name);
    } else {
      ImGui::Text("%s", param.name.c_str());
      ImGui::SameLine();
      absl::visit(
          [](auto& val_content) {
            using T = std::decay_t<decltype(val_content)>;
            if constexpr (std::is_same_v<T, std::monostate>) {
              // Do nothing
            } else if constexpr (std::is_same_v<T, std::string>) {
              // Handled by image_asset_texture() and registered_texture()
            } else if constexpr (std::is_same_v<T, float>) {
              ImGui::InputFloat("##float", &val_content);
            } else if constexpr (std::is_same_v<
                                     T,
                                     filament::math::details::TVec2<float>>) {
              ImGui::InputFloat2("##float2", &val_content.x);
            } else if constexpr (std::is_same_v<
                                     T,
                                     filament::math::details::TVec3<float>>) {
              ImGui::InputFloat3("##float3", &val_content.x);
            } else if constexpr (std::is_same_v<
                                     T,
                                     filament::math::details::TVec4<float>>) {
              ImGui::InputFloat4("##float4", &val_content.x);
            } else if constexpr (std::is_same_v<T, int>) {
              ImGui::InputInt("##int", &val_content);
            } else if constexpr (std::is_same_v<T, unsigned int>) {
              ImGui::InputScalar("##uint", ImGuiDataType_U32, &val_content);
            } else if constexpr (std::is_same_v<T, bool>) {
              ImGui::Checkbox("##bool", &val_content);
            }
          },
          param.val);
    }
    ImGui::PopID();
  }
}

void MaterialEditorWidget::DrawTextureParameterUI(
    MaterialDefinition::Parameter& param, std::string* path) {
  EditorFieldControl::ShowDefaultControl(param.name.c_str(), path, nullptr,
                                         editor::EditorControlFlags::kDefault,
                                         &view_, /*is_url=*/true);

  absl::StatusOr<std::reference_wrapper<AssetThumbnailProvider>> provider_or =
      view_.GetRegistry().Get<AssetThumbnailProvider>();
  if (provider_or.ok()) {
    filament::Texture* thumbnail =
        provider_or.value().get().GetThumbnailForResource(*path);
    if (thumbnail) {
      // Extend the indent for more visual separation from the parameter names.
      const float indent_width = ImGui::GetStyle().IndentSpacing * 2.0f;
      ImGui::Indent(indent_width);
      ImGui::Image(thumbnail, ImVec2(64, 64));
      ImGui::Text("Size: %zux%zu", thumbnail->getWidth(),
                  thumbnail->getHeight());
      ImGui::Unindent(indent_width);
    }
  }
}

}  // namespace imp::editor
