// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/editor/widgets/environment_light_editor.h"

#include <optional>
#include <string>

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/editor/layout/helpers.h"
#include "core/lighting/environment_light.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/math/almost_equal.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"
#include "core/view/framework/lighting/light_manager.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/misc/cpp/imgui_stdlib.h"
#include "filament/filament/include/filament/IndirectLight.h"

namespace imp::editor {

EnvironmentLightEditor::EnvironmentLightEditor(BaseView& view) : view_(view) {}

void EnvironmentLightEditor::DrawImGui() {
  EnvironmentLight* environment_light =
      view_.GetLightManager().GetEnvironmentLight();
  if (environment_light) {
    const ImageBasedLightingAsset* reflection_ibl_asset =
        environment_light->GetReflectionIblAsset()
            ? environment_light->GetReflectionIblAsset()->Get()
            : nullptr;
    std::string ibl_asset_url;
    if (ImGui::CollapsingHeader("Reflection Asset Source:",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (reflection_ibl_asset && reflection_ibl_asset->GetAssetUrl()) {
        ibl_asset_url = std::string(*reflection_ibl_asset->GetAssetUrl());
        ImGui::InputText(
            editor::GenerateUniqueImGuiLabel("ImageBasedLightingAsset",
                                             &ibl_asset_url,
                                             EditorControlFlags::kDisplayLabel)
                .c_str(),
            &ibl_asset_url,
            ImGuiInputTextFlags_ReadOnly | ImGuiInputTextFlags_AutoSelectAll);
      } else {
        ImGui::Text("Dynamic Lighting Data");
      }
    }

    const ImageBasedLightingAsset* sh_irradiance_ibl_asset =
        environment_light->GetShIrradianceIblAsset()
            ? environment_light->GetShIrradianceIblAsset()->Get()
            : nullptr;
    std::string sh_irradiance_asset_url;
    if (ImGui::CollapsingHeader("Irradiance Spherical Harmonics Asset Source:",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (sh_irradiance_ibl_asset && sh_irradiance_ibl_asset->GetAssetUrl()) {
        sh_irradiance_asset_url =
            std::string(*sh_irradiance_ibl_asset->GetAssetUrl());
        ImGui::InputText(
            editor::GenerateUniqueImGuiLabel("ImageBasedLightingAsset",
                                             &sh_irradiance_asset_url,
                                             EditorControlFlags::kDisplayLabel)
                .c_str(),
            &sh_irradiance_asset_url,
            ImGuiInputTextFlags_ReadOnly | ImGuiInputTextFlags_AutoSelectAll);
      } else {
        ImGui::Text("Dynamic Lighting Data");
      }
    }

    if (ImGui::CollapsingHeader("Settings:", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (environment_light->GetIndirectLight()) {
        // Intensity field
        float intensity = environment_light->GetIndirectLight()->getIntensity();
        if (ImGui::InputFloat(
                editor::GenerateUniqueImGuiLabel("Intensity", &intensity,
                                                 EditorControlFlags::kDefault)
                    .c_str(),
                &intensity)) {
          environment_light->SetIntensity(intensity);
        }

        // Rotation field
        quatf current_rotation =
            environment_light->GetIndirectLight()->getRotation().toQuaternion();

        if (rotation_field_.DrawFields("Rotation", current_rotation)) {
          environment_light->SetRotation(rotation_field_.GetCurrentRotation());
        }

        ImGui::Unindent();
      } else {
        ImGui::Text("Missing Indirect Light");
      }
    }
  } else {
    ImGui::Text("Missing Environment Light");
  }
}

}  // namespace imp::editor
