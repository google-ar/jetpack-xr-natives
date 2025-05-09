/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_DISPLAY_LAYER_DISPLAY_LAYER_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_DISPLAY_LAYER_DISPLAY_LAYER_MANAGER_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/ncsb/component_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/window/filament_view.h"

namespace imp {

// Manages an ordered list of layers that render groups of nodes.
//
// Each layer represents a different render pass that is drawn in-order after
// the main pass that draws the Node::kMainGroupName group.
//
// A layer specifies which nodes to render by group name. Which groups a node
// is in is controlled by Node::SetGroups.
//
// By default, there are no layers automatically created by Impress except when
// running the Impress Editor, which creates a layer to render editor specific
// objects on top of everything else.
class DisplayLayerManager {
 public:
  explicit DisplayLayerManager(BaseView& view);
  ~DisplayLayerManager();

  // Create a new layer to display content in certain Group.
  void CreateLayer(absl::string_view name, absl::string_view group_name);

  // Remove a layer.
  void RemoveLayer(absl::string_view name);

  // Called be each frame to render all layers.
  void RenderLayers();

  // Adjust sequence of layers.
  void MoveForward(absl::string_view name);
  void MoveBackward(absl::string_view name);

  // Turn on/off a layer.
  void SetLayerEnabled(absl::string_view name, bool is_enabled);
  bool GetLayerEnabled(absl::string_view name) const;

  int GetNumberOfLayers() const { return layers_.size(); }
  absl::string_view GetLayerName(int index) const {
    return layers_[index]->name;
  }

  // Sets the camera to use for a layer. Pass {} to revert to the main camera.
  void SetCamera(absl::string_view name,
                 ComponentHandle<CameraComponent> camera);

  // this is removing any default layers that may exist to have a clean slate
  // for the test.
  void RemoveAllLayers() { layers_.clear(); }

  // Returns the FilamentView for a layer.
  filament::View* GetFilamentView(absl::string_view name);

 private:
  struct DisplayLayer {
    // A DisplayLayer is responsible to show contents in conresponding Group.
    // Camera and viewports are available through its FilamentView member.
    DisplayLayer(absl::string_view name, absl::string_view group_name,
                 BaseView& base_view);

    DisplayLayer(DisplayLayer&& other) = delete;
    DisplayLayer& operator=(DisplayLayer&& other) = delete;
    DisplayLayer(const DisplayLayer&) = delete;
    DisplayLayer& operator=(const DisplayLayer&) = delete;

    ~DisplayLayer();
    std::string name;
    std::string group_name;
    window::detail::FilamentView filament_view;
    ComponentHandle<CameraComponent> camera_override;
    bool is_enabled = true;
  };
  using DisplayLayers = std::vector<std::unique_ptr<DisplayLayer>>;

  DisplayLayers::iterator FindLayerByName(absl::string_view name);

  std::vector<std::unique_ptr<DisplayLayer>> layers_;
  BaseView& view_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_DISPLAY_LAYER_DISPLAY_LAYER_MANAGER_H_
