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

#include "core/editor/widgets/materials_widget.h"

#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/materials/material.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_renderer.h"

namespace imp::editor {
namespace {
// The label to use in the UI for materials that do not have a name set.
constexpr absl::string_view kUnamedMaterialLabel = "<material>";

void DrawMaterials(std::vector<Material*> materials) {
  for (Material* material : materials) {
    ImGui::Indent();
    std::string material_name = material->GetName();
    if (material_name.empty()) {
      material_name = std::string(kUnamedMaterialLabel);
    }
    ImGui::Selectable(material_name.c_str());
    ImGui::Unindent();
  }
}
}  // namespace

MaterialsWidget::MaterialsWidget(BaseView& view) : view_(view) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  editor.GetDispatcher().Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        // We only support single selection for the materials widget.
        active_node_ =
            view_.GetRegistry().Get<Editor>()->get().GetSingleSelectedNode();
      },
      this);
}

bool MaterialsWidget::HasContent() const {
  if (!active_node_) {
    return false;
  }

  ComponentHandle<GltfRenderer> gltf_renderer =
      active_node_->GetComponent<GltfRenderer>();
  if (!gltf_renderer || gltf_renderer->GetMaterials().empty()) {
    return false;
  }

  return true;
}

void MaterialsWidget::DrawImGui() {
  ComponentHandle<GltfRenderer> gltf_renderer =
      active_node_->GetComponent<GltfRenderer>();
  if (gltf_renderer) {
    DrawMaterials(gltf_renderer->GetMaterials());
  }
}

}  // namespace imp::editor
