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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_INFO_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_INFO_H_

#include "core/common/registry.h"

namespace imp::editor {

// Provides information about the current state of the Impress Editor.
//
// An EditorInfo object is accessible via the Impress Registry when the Editor
// exists.
//
// This is useful to access Editor information from places that otherwise can't
// access the actual Editor due to circular dependencies.
struct EditorInfo {
  enum class RunMode {
    kPlayMode,
    kEditMode,
    kSwitchingToPlayMode,
    kSwitchingToEditMode
  };

  // Selection mode for the editor.
  enum class SelectionMode {
    // In this mode, only a single node can be selected at any time. Selecting
    // a new node will deselect any previously selected node.
    kSingleNode,
    // In this mode, multiple nodes can be selected simultaneously. Subsequent
    // selections add to the current set of selected nodes.
    kMultipleNodes,
  };

  // Indicates whether the editor is being displayed on a native or remote
  // screen.
  enum class DisplayMode { kNativeScreen, kRemoteScreen };

  // Indicates the platform and build configuration of the application. This is
  // used to determine how the editor should be configured.
  enum class PlatformMode {
    kDefault,
    // The application is an XR application using SplitEngine.
    kXrSplitEngineApp,
  };

  virtual ~EditorInfo() = default;

  virtual bool IsEnabled() const = 0;

  // Returns the current RunMode.
  //
  // Edit mode is the default mode in Sandbox builds. Press the Play button to
  // switch from edit mode to play mode. The switch may be asynchronous because
  // the scene graph is re-created and Setup functions are run.
  //
  // When in edit mode, Components won't run unless one of the following
  // conditions are met:
  // 1. FooComponent::kRunInEditMode is set to true.
  // 2. FooComponent::kExcludeFromEditor is set to true.
  // 3. The component doesn't contain IsfInfo and is therefore not editable.
  //
  // Outside of Sandbox builds, there is no UI for switching to and from
  // EditMode.
  virtual RunMode GetRunMode() const = 0;

  // Returns the current DisplayMode. Native screen is the default mode.
  virtual DisplayMode GetDisplayMode() const = 0;

  // Returns true if the Editor is paused.
  //
  // When paused, Component, ComponentSystem, and Updater objects will not be
  // updated.
  //
  // Outside of Sandbox builds, there is no UI for switching pausing/resume in
  // the editor.
  virtual bool IsPaused() const = 0;

  // Returns true if the Editor has frames to step through.
  virtual bool HasFramesToStep() const = 0;
};

// Helper that returns true if the editor is enabled and either in edit mode or
// switching to edit mode.
bool IsInEditMode(Registry& registry);

// Helper that returns true if the editor is enabled and paused.
bool IsPaused(Registry& registry);

// Helper that returns true if the editor is enabled has frames to step through.
bool HasFramesToStep(Registry& registry);

// Helper that returns true if the editor is enabled and either in edit mode,
// switching to edit mode, or paused.
bool IsInEditModeOrPaused(Registry& registry);

// Helper that returns true if the editor is enabled and either in draft/edit
// mode, switching to draft/edit mode, or paused and has no frames to step
// through.
// This means that any updaters should not run unless they are otherwise allowed
// to run by other rules.
bool ShouldNotUpdate(Registry& registry);

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_INFO_H_
