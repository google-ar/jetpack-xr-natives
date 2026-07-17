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

#include "core/view/framework/display_layer/display_layer_manager.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Scene.h"
#include "core/common/trace.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/groups_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_manager.h"

namespace imp {

DisplayLayerManager::DisplayLayer::DisplayLayer(absl::string_view name,
                                                absl::string_view group_name,
                                                BaseView& base_view)
    : name(name), group_name(group_name) {
  absl::Status status =
      filament_view.Setup(base_view.GetHost()->GetEngine(), std::string(name));
  filament_view.Get()->setPostProcessingEnabled(false);
  filament_view.Get()->setShadowingEnabled(false);
}

DisplayLayerManager::DisplayLayer::~DisplayLayer() {
  filament_view.Cleanup(BaseView::GetSharedEngine());
}

DisplayLayerManager::DisplayLayerManager(BaseView& view) : view_(view) {}

DisplayLayerManager::~DisplayLayerManager() { RemoveAllLayers(); }

void DisplayLayerManager::CreateLayer(absl::string_view name,
                                      absl::string_view group_name) {
  // layers of the same name will be merged.
  if (FindLayerByName(name) == layers_.end()) {
    layers_.push_back(std::make_unique<DisplayLayer>(name, group_name, view_));
  } else {
    IMP_LOG(imp::WARNING) << "DisplayLayer: " + std::string(name) + " already exists";
  }
}

void DisplayLayerManager::RemoveLayer(absl::string_view name) {
  DisplayLayers::iterator it = FindLayerByName(name);
  if (it == layers_.end()) {
    it->get()->filament_view.Cleanup(view_.GetHost()->GetEngine());
    layers_.erase(it);
    IMP_LOG(imp::WARNING) << "DisplayLayer: " << name << " removed";
  }
}

void DisplayLayerManager::RenderLayers() {
  IMP_TRACE();
  for (auto& layer : layers_) {
    if (layer->is_enabled) {
      // use camera and viewport of Main Scene as default
      // TODO use camera & viewport of secondary camera if needed.
      if (layer->camera_override) {
        layer->filament_view.Get()->setCamera(
            layer->camera_override->GetCamera());
      } else {
        layer->filament_view.Get()->setCamera(
            view_.GetCameraManager().GetCamera()->GetCamera());
      }
      const filament::Viewport& viewport =
          view_.GetHost()->GetView()->getViewport();
      layer->filament_view.Get()->setViewport(viewport);
      layer->filament_view.Get()->setRenderTarget(
          view_.GetHost()->GetView()->getRenderTarget());

      filament::Scene* scene =
          view_.GetGroupsManager().GetScene(layer->group_name);

      if (scene && scene->getRenderableCount() > 0) {
        layer->filament_view.Get()->setScene(scene);
        view_.GetHost()->PerformRender(layer->filament_view.Get());
      }
    }
  }
}

void DisplayLayerManager::MoveForward(absl::string_view name) {
  DisplayLayers::iterator it = FindLayerByName(name);
  if (it == layers_.end()) {
    IMP_LOG(imp::WARNING) << "DisplayLayer: " << name << " not found!";
  } else if (it != layers_.begin()) {
    std::iter_swap(it - 1, it);
  }
}

void DisplayLayerManager::MoveBackward(absl::string_view name) {
  DisplayLayers::iterator it = FindLayerByName(name);
  if (it == layers_.end()) {
    IMP_LOG(imp::WARNING) << "DisplayLayer: " << name << " not found!";
  } else if (it != layers_.end() - 1) {
    std::iter_swap(it, it + 1);
  }
}

void DisplayLayerManager::SetLayerEnabled(absl::string_view name,
                                          bool is_enabled) {
  DisplayLayers::iterator it = FindLayerByName(name);
  if (it == layers_.end()) {
    IMP_LOG(imp::WARNING) << "DisplayLayer: " << name << " not found!";
  } else {
    it->get()->is_enabled = is_enabled;
  }
}

bool DisplayLayerManager::GetLayerEnabled(absl::string_view name) const {
  auto it = std::find_if(layers_.begin(), layers_.end(),
                         [name](auto& layer) { return layer->name == name; });
  if (it == layers_.end()) {
    IMP_LOG(imp::WARNING) << "DisplayLayer: " << name << " not found!";
  } else {
    return it->get()->is_enabled;
  }
  return false;
}

void DisplayLayerManager::SetCamera(absl::string_view name,
                                    ComponentHandle<CameraComponent> camera) {
  DisplayLayers::iterator it = FindLayerByName(name);
  if (it == layers_.end()) {
    IMP_LOG(imp::WARNING) << "DisplayLayer: " << name << " not found!";
  } else {
    it->get()->camera_override = camera;
  }
}

DisplayLayerManager::DisplayLayers::iterator
DisplayLayerManager::FindLayerByName(absl::string_view name) {
  return std::find_if(layers_.begin(), layers_.end(),
                      [name](auto& layer) { return layer->name == name; });
}

filament::View* DisplayLayerManager::GetFilamentView(absl::string_view name) {
  DisplayLayers::iterator it = FindLayerByName(name);
  if (it == layers_.end()) {
    IMP_LOG(imp::WARNING) << "DisplayLayer: " << name << " not found!";
    return nullptr;
  } else {
    return it->get()->filament_view.Get();
  }
}

}  // namespace imp
