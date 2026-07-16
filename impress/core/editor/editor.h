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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_H_

#include <memory>
#include <optional>

#include "absl/container/flat_hash_set.h"
#include "core/camera/camera_component.h"
#include "core/editor/editor_info.h"
#include "core/editor/editor_plugin.h"
#include "core/geometry/shapes/rect.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/system.h"
#include "core/view/framework/assets/gltf_asset.h"

namespace imp::editor {

class WidgetUiSystem;
class AssetLibrary;
class EventInjector;

// Provides a suite of features to inspect, debug, and edit the Impress engine.
// The editor is available in all builds with --define=IMPEL_DEV_RUNTIME=1.
class Editor : public System {
 protected:
  explicit Editor(BaseView* view) : System(view) {}

 public:
  // Initializes the Editor. This function must to be called after Editor is
  // constructed, otherwise Editor::SetEnabled() will throw a fatal error.
  //
  // This is split out from the constructor logic because the Editor is
  // constructed in the Registry, and Initialize() constructs objects which
  // will look for the Editor in the Registry.
  virtual void Initialize() = 0;

  // Returns true if the editor UI is enabled.
  virtual bool IsEnabled() const = 0;
  // Enables or disables the editor UI.
  virtual void SetEnabled(bool enabled) = 0;

  // Adds a node as a child to the editor root. This node will go inactive if
  // the editor is disabled and will be excluded from the editor hierarchy UI.
  virtual void AddNode(NodeHandle node) = 0;
  // Removes a node previously added to the editor with AddNode(node).
  virtual void RemoveNode(NodeHandle node) = 0;

  // Sends a NodeSelectionChangedEvent in the editor dispatcher for the given
  // node.
  // * If `multi_selection_enabled` is true:
  //     - The node is added to the current selection.
  //     - If the node is already selected, it is deselected.
  // * Otherwise:
  //     - The node becomes the *only* selected node.
  // When the node is invalid, deselect all nodes.
  virtual void SelectNode(
      NodeHandle node, EditorInfo::SelectionMode selection_mode =
                           EditorInfo::SelectionMode::kSingleNode) noexcept = 0;

  // Returns the selected nodes.
  virtual const absl::flat_hash_set<NodeHandle>&
  GetSelectedNodes() noexcept = 0;

  // Returns the single selected node.
  //
  // If there are multiple selected nodes, this function will return an invalid
  // node handle.
  virtual NodeHandle GetSingleSelectedNode() noexcept = 0;

  // Switches to the isometric Editor camera+input mode.
  // A camera separate from the default app camera is spawned and positioned
  // based on the average position of nodes in the scene.
  // In this mode, editor-specific visualizers and widgets will rendered over
  // the scene and app input will be disabled.
  //
  // Only if EditorPlugin::GetCameraConfiguration() is
  // kEditorAndAppCameraDefault, this function will be effective.
  virtual void SwitchToEditorMode() = 0;

  // Switches to the app camera+input mode.
  // The isometric Editor camera will be hidden and editor-specific
  // visualizers and widgets will be hidden. Input forwarding will be returned
  // to the app.
  virtual void SwitchToAppMode() = 0;

  // Returns true if the Editor is in EditMode.
  //
  // EditMode is the default mode in Sandbox builds. Press the Play button to
  // exist EditMode.
  //
  // When in EditMode, Components won't actually run unless one of the following
  // conditions are met:
  // 1. FooComponent::kRunInEditMode is set to true.
  // 2. FooComponent::kExcludeFromEditor is set to true.
  // 3. The component doesn't contain IsfInfo and is therefore not editable.
  //
  // Outside of Sandbox builds, there is no UI for switching to and from
  // EditMode.
  virtual EditorInfo::RunMode GetRunMode() const = 0;

  // Returns the current DisplayMode.
  //
  // Native screen is the default mode. In the editor, there is no UI for
  // switching to and from NativeScreen/RemoteScreen.
  virtual EditorInfo::DisplayMode GetDisplayMode() const = 0;

  // Sets the current DisplayMode.
  virtual void SetDisplayMode(EditorInfo::DisplayMode display_mode) = 0;

  // Sets if the Editor is in EditMode.
  //
  // EditMode is the default mode in Sandbox builds. Press the Play button to
  // exist EditMode.
  //
  // When in EditMode, Components won't actually run unless one of the following
  // conditions are met:
  // 1. FooComponent::kRunInEditMode is set to true.
  // 2. FooComponent::kExcludeFromEditor is set to true.
  // 3. The component doesn't contain IsfInfo and is therefore not editable.
  //
  // Outside of Sandbox builds, there is no UI for switching to and from
  // EditMode.
  virtual void SetInEditMode(bool in_edit_mode) = 0;

  // Returns true if the Editor is paused.
  //
  // When paused, Component, ComponentSystem, and Updater objects will not be
  // updated.
  //
  // Outside of Sandbox builds, there is no UI for switching pausing/resume in
  // the editor.
  virtual bool IsPaused() const = 0;

  // Sets if the Editor is paused.
  //
  // When paused, Component, ComponentSystem, and Updater objects will not be
  // updated.
  //
  // Outside of Sandbox builds, there is no UI for switching pausing/resume in
  // the editor.
  virtual void SetPaused(bool paused) = 0;

  // Steps the Editor forward by one frame.
  //
  // This is useful for stepping through the scene when paused.
  virtual void StepNextFrame() = 0;

  // Returns if the editor has frames to step.
  virtual bool HasFramesToStep() = 0;

  // Adds a node to a tracked list of sandbox nodes. These nodes will not be
  // modified, but will be exempt from being recreated when the Editor
  // transitions to and from EditMode.
  virtual void AddSandboxNode(NodeHandle node) = 0;
  // Removes a node previously added with AddSandboxNode(node).
  virtual void RemoveSandboxNode(NodeHandle node) = 0;

  // Returns the WidgetUiSystem for the Editor so you can add your own widgets.
  virtual WidgetUiSystem& GetWidgetUiSystem() noexcept = 0;

  // Returns the Editor-specific Dispatcher.
  virtual Dispatcher& GetDispatcher() noexcept = 0;

  // Returns the availability of the cameras under the configuration of the
  // Editor.
  virtual EditorPlugin::CameraConfiguration GetCameraConfiguration()
      const noexcept = 0;

  // Returns the Editor root Node.
  virtual NodeHandle GetEditorRoot() noexcept = 0;

  // Returns the Editor Camera.
  virtual ComponentHandle<CameraComponent> GetCamera() noexcept = 0;

  // Returns the App camera in AppCamera mode, and the Editor camera in
  // EditorCamera mode.
  virtual ComponentHandle<CameraComponent> GetActiveCamera() noexcept = 0;

  // Returns the Event Injector.
  virtual EventInjector& GetEventInjector() noexcept = 0;

  // Returns the Asset Library.
  virtual AssetLibrary* GetAssetLibrary() noexcept = 0;

  // Returns the GltfAsset::LoadOptions to use when loading glTF files.
  virtual GltfAsset::LoadOptions GetGltfLoadOptions() const noexcept = 0;

  // Sets the screen-space rect of the 3D viewport widget.
  // This is used to correctly transform pointer coordinates for selection and
  // navigation.
  virtual void SetViewportRect(std::optional<Rect> rect) = 0;

  // Returns the screen-space rect of the 3D viewport widget.
  virtual std::optional<Rect> GetViewportRect() const = 0;
};

// Gets the Editor from the Registry. If no Editor is present in the Registry,
// it first creates an Editor then stores it in the Registry.
//
// An EditorPlugin may be passed in to extend Editor functionality. The
// EditorPlugin may only be passed to the Editor during construction. If an
// Editor already exists in the Registry and an EditorPlugin is passed in, this
// function will fatal.
// TODO: Remove is_sandbox parameter and access client_api
// directly, which requires refactoring client_api to be outside of framework.
Editor& GetOrCreateEditor(BaseView* view,
                          std::unique_ptr<EditorPlugin> plugin = nullptr,
                          bool is_sandbox = false);

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_H_
