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

#include "core/view/utils/render_state_validator.h"

#include <cstddef>

#include "core/common/log.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/common/platform_helpers.h"
#include "core/ncsb/component_manager.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/render/material.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"

namespace imp {

RenderStateValidator::RenderStateValidator(BaseView& view)
    : Updater(view), view_(view) {}

void RenderStateValidator::Update(const FrameTime& frame_time) {
  filament::Engine& engine = *BaseView::GetSharedEngine();

  ComponentManager& component_manager = view_.GetComponentManager();
  component_manager.ForEach<MeshRenderer>([&engine](MeshRenderer* comp) {
    // Ignore inactive components.
    if (!comp->IsActive()) {
      return;
    }

    // For each material used by a MeshRenderer, check to see if it has any
    // invalid filament textures assigned to it.
    for (size_t primitive_index = 0;
         primitive_index < comp->GetPrimitiveCount(); primitive_index++) {
      Material* material = comp->GetMaterial(primitive_index);
      for (auto& [name, texture] : material->GetUnownedFilamentTextures()) {
        if (!engine.isValid(texture)) {
          IMP_LOG(imp::FATAL)
              << "MeshRenderer has invalid texture assigned to parameter "
              << name << " on material " << material->GetName() << "("
              << material->GetFilamentMaterialInstance()->getName()
              << ") on node " << imp::ToString(comp->GetNode());
        }
      }
    }
  });
}

}  // namespace imp
