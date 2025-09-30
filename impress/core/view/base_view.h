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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_BASE_VIEW_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_BASE_VIEW_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/common/enum_flags.h"
#include "core/common/pass_key.h"
#include "core/common/rememberer.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/node_flag.h"
#include "core/render/base_renderable_manager.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/scripting/script_message_handler_provider.h"
#include "core/view/utils/frame_time.h"
#include "core/view/utils/proto/view_config.proto.imp.h"
#include "core/view/view_hooks.h"
#include "core/window/filament_host.h"
#include "core/window/window_rotation.h"

namespace sceneform {
// TODO: temporarily shim 'Flags' into this namespace
template <typename T>
using Flags = imp::Flags<T>;
}  // namespace sceneform

namespace imp {

// These forward declarations are discouraged, but they make it possible for
// us to avoid having a circular dependency between View and many other things
// in the impress framework.
class AnimationSystem;
class AssetManager;
class UpdateSystem;
class CameraManager;
class CollisionManager;
class ComponentManager;
class Context;
class Device;
class Dispatcher;
class GestureManager;
class InputManager;
class LightManager;
class MeshFactory;
class NodeHandle;
class PathManager;
class SceneSystem;
class DisplayLayerManager;
class SkinningSystem;
class TextureFactory;
class TextureRegistry;
class MaterialFactory;
class EnvironmentLightFactory;
class GroupsManager;
class Monitor;
class Registry;
class ViewHost;
class ViewState;

#if IMP_RUNTIME(DEV)
namespace editor {
class EditorPlugin;
}
#endif

// BaseView is an interface implemented by imp::View, which is the entry point
// into building an application with the impress framework. It also owns and
// provides access to the various systems and managers in the impress framework
// (as opposed to having globals, singletons, or a registry).
//
// Separating the interface (BaseView) from the implementation (View) allows us
// to prevent circular dependencies.
class BaseView : public Rememberer,
                 public scripting::ScriptMessageHandlerProvider,
                 protected ViewHooks {
 public:
  // Creates a new Node owned by this View and returns a NodeHandle to provide
  // access to it. The Node will be destroyed if the View is destroyed.
  virtual NodeHandle CreateNode() = 0;

  // Destroys a Node owned by this view. If the NodeHandle is invalid, does
  // nothing.
  virtual void DestroyNode(NodeHandle node) = 0;

  // Iterate over all Nodes in this view, including disabled or inactive nodes.
  virtual void ForEachNode(std::function<void(NodeHandle)>&& fn) = 0;

  // Iterate over a subset of all Nodes in this view. Only nodes that meet all
  // flags will be included.
  // i.e. ForEachNode(fn, NodeFlags::kIsEnabled | NodeFlags::kIsRoot)
  // will only include enabled root nodes.
  virtual void ForEachNode(std::function<void(NodeHandle)>&& fn,
                           NodeFlag filter) = 0;

  // Returns the total number of nodes associated with this view.
  virtual std::size_t GetNodeCount() const = 0;

  // Returns the ComponentManager that is owned by this View.
  // The ComponentManager contains all components for all Nodes that are part
  // of this View and can be used to access them directly.
  virtual ComponentManager& GetComponentManager() noexcept = 0;

  // Returns the asset manager used to load and cache assets (i.e. gltf files).
  virtual AssetManager& GetAssetManager() noexcept = 0;

  // Returns the InputManager.
  virtual InputManager& GetInputManager() noexcept = 0;

  // Returns the camera manager used to access the camera for this view.
  virtual CameraManager& GetCameraManager() noexcept = 0;

  // Returns the light manager used to control the environment map
  // and default directional light.
  virtual LightManager& GetLightManager() noexcept = 0;

  // Returns the GestureManager.
  virtual GestureManager& GetGestureManager() noexcept = 0;

  virtual UpdateSystem& GetUpdateSystem() noexcept = 0;

  // Returns the system responsible for handling ISF files.
  virtual SceneSystem& GetSceneSystem() noexcept = 0;

  // Returns display layers manager.
  virtual DisplayLayerManager& GetDisplayLayerManager() noexcept = 0;

  // Returns the manager that controls testing collisions across an entire
  // scene.
  virtual CollisionManager& GetCollisionManager() noexcept = 0;

  // Returns the PathManager, which maintains information on node relationships.
  virtual PathManager& GetPathManager() noexcept = 0;

  // Returns a factory to create textures.
  virtual TextureFactory& GetTextureFactory() noexcept = 0;

  // Returns a registry that maps textures by name.
  virtual TextureRegistry& GetTextureRegistry() noexcept = 0;

  // Returns a factory to create meshes.
  virtual MeshFactory& GetMeshFactory() noexcept = 0;

  // Returns the MaterialFactory used to load materials.
  virtual MaterialFactory& GetMaterialFactory() noexcept = 0;

  // Returns the EnvironmentLightFactory used to load IBLs.
  virtual EnvironmentLightFactory& GetEnvironmentLightFactory() noexcept = 0;

  // Returns information about all groups that currently exist.
  virtual GroupsManager& GetGroupsManager() noexcept = 0;

  // Returns device information, i.e. dpi value.
  virtual Device& GetDevice() noexcept = 0;

  // Returns a registry that maps objects of any type by type.
  //
  // This can be used to simplify dependency injection by registering optional
  // or client systems/features with the view.
  //
  // The Registry is empty by default and only contains what users add to it.
  virtual Registry& GetRegistry() noexcept = 0;
  virtual const Registry& GetRegistry() const noexcept = 0;

  // Provides a context from the view.
  virtual const Context& GetContext() const = 0;

  // Returns information about the current frame time.
  virtual const FrameTime& GetFrameTime() const = 0;

  // Returns the size of the view being rendered to in UI pixels.
  virtual uint2 GetSize() const = 0;

  // Returns the margins of the viewport being rendered to in UI pixels.
  virtual uint4 GetMargins() const = 0;

  // Returns the orientation of the view being rendered.
  virtual window::WindowRotation GetDisplayRotation() const = 0;

  virtual NodeHandle AttachEntityToView(utils::Entity entity) = 0;

  // Returns the FilamentHost object that contains the filament engine
  // underlying this View.
  // Do not use this API unless you understand the underlying details of
  // filament.
  virtual window::FilamentHost* GetHost() = 0;

  // Returns the filament engine config. Any subclass can provide its own
  // configuration. However, please note that multiple instances of Impress on
  // the same thread share the same filament::Engine, so whichever instance is
  // created first will determine the config that's used.
  virtual filament::Engine::Config GetEngineConfig() const {
    return filament::Engine::Config{.disableParallelShaderCompile = true};
  }

  virtual ViewConfig GetConfig() const { return {}; }

  // Returns the desired Filament feature level.
  // Filament may use a lower feature level depending on device support.
  // ES2-only devices are only supported on feature level 0.
  // ES3+ device support is guaranteed up to feature level 1. Higher feature
  // levels require more hardware features that may not be present on all
  // ES3 devices.
  virtual filament::backend::FeatureLevel GetMaximumEngineFeatureLevel() const {
    return filament::backend::FeatureLevel::FEATURE_LEVEL_1;
  }

  // Returns true if the rendering thread should begin paused. This is an
  // experimental feature and has several caveats. See Engine::setPaused for
  // more information.
  virtual bool ShouldStartPaused() const { return false; }

  // When rendering using OpenGL, determines if a shared Gl context should be
  // passed into filament. This is necessary for ARCore and C9, but can be
  // turned off in some use cases where it isn't required.
  virtual bool ShouldUseSharedGlContext() const { return true; }

  // All View subclasses running in a process share a filament Engine
  static filament::Engine* GetSharedEngine();
  static void SetSharedEngine(filament::Engine* engine);

  // Create a filament::View that is tracked so that it can be accessed via
  // GetFilamentViews().
  virtual filament::View* CreateFilamentView() = 0;

  // Destroys a filament::View that was created by CreateFilamentView().
  virtual void DestroyFilamentView(filament::View* view) = 0;

  // Returns a span of all filament::Views that were created by
  // CreateFilamentView().
  virtual absl::Span<filament::View*> GetFilamentViews() = 0;

  // Entry point for dev-mode customization (requires IMP_DEV_RUNTIME=1 or
  // manual registration of a DevModeExtension). Called during ImGui render, so
  // clients can create ImGui widgets as well as debug geometry.
  virtual void RenderDev() {}

  // Returns the Monitor for performance monitoring
  virtual Monitor* GetMonitor() noexcept = 0;

  // Returns the wrapper for filament::RenderableManager that should be used for
  // compatibility with SplitEngine.
  virtual BaseRenderableManager& GetRenderableManager() = 0;

  // Sets the RenderableManager to create and manage Filament renderables.
  // This needs to be set prior to creating a SplitEngineSerializer.
  virtual void SetRenderableManager(
      std::unique_ptr<BaseRenderableManager> renderable_manager) = 0;

  // Returns the SplitEngineSerializer if running in split mode.
  virtual split_engine::SplitEngineSerializer* GetSplitEngineSerializer() = 0;

  // Sets the SplitEngineSerializer to serialize the scene to remote Impress.
  virtual void SetSplitEngineSerializer(
      std::unique_ptr<split_engine::SplitEngineSerializer> serializer) = 0;

  // Indicates if 'local mode' is enabled, using client-provided materials (raw
  // or builtin) instead of baked-in backend materials. This allows for rapid
  // material iteration and supports split-engine Vanilla Android, where the
  // split engine backend renderer is absent.

  // Note that this should be only used in the split engine frontend.
  virtual bool AreSplitEngineMaterialsInLocalMode() const = 0;

  // Enables or disables precise translation mode that uses doubles instead of
  // floats. This is disabled by default.
  //
  // Internally, this calls
  // filament::TransformManager::setAccurateTranslationsEnabled.
  void SetPreciseTranslationEnabled(bool enabled);

  // Returns whether the precise translation mode is enabled.
  //
  // Internally, this calls
  // filament::TransformManager::isAccurateTranslationsEnabled.
  bool IsPreciseTranslationEnabled() const;

  // Returns the name of this view.
  virtual absl::string_view GetTitle() const = 0;

  // Manually advance the foreground executor with a timeout, while tracking how
  // long it takes.
  //
  // This is normally done automatically when the view advances.
  virtual void AdvanceForegroundExecutor() = 0;

  // Notifies the view and connected native systems of updates to transient
  // transition parameters, which are caused by e.g. device rotation on iPad.
  // See `ViewTransitionParametersChangedEvent` in view_events.h for an
  // explanation of these fields.
  virtual void UpdateTransitionParameters(
      float2 transition_scale_adjustment,
      float transition_counter_rotation) = 0;

  ViewHooks& GetViewHooks(PassKey<ViewState> key) { return *this; }

#if IMP_RUNTIME(DEV)
  // Called once by ViewHost to instantiate the editor plugin.
  virtual std::unique_ptr<imp::editor::EditorPlugin> CreateEditorPlugin() {
    return nullptr;
  }
#endif

  ~BaseView() override {}
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_BASE_VIEW_H_
