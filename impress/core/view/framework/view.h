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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_VIEW_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_VIEW_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/context.h"
#include "core/common/invocable.h"
#include "core/common/registry.h"
#include "core/input/input_manager.h"
#include "core/lighting/environment_light_factory.h"
#include "core/math/vec.h"
#include "core/ncsb/component_manager.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/groups_manager.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_attachment_manager.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/path_manager.h"
#include "core/ncsb/update_system.h"
#include "core/render/base_renderable_manager.h"
#include "core/render/shader_cache_system.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_registry.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/display_layer/display_layer_manager.h"
#include "core/view/framework/gestures/gesture_manager.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/framework/lighting/light_manager.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/scene/scene_system.h"
#include "core/view/scripting/script_message_handler.h"
#include "core/view/scripting/script_message_handler_provider.h"
#include "core/view/utils/default_view_config.h"
#include "core/view/utils/device.h"
#include "core/view/utils/frame_time.h"
#include "core/view/utils/proto/view_config.proto.imp.h"
#include "core/window/filament_host.h"
#include "core/window/window_rotation.h"

namespace imp {

// A View is the entry point into building an application with the impress
// framework. The View contains the scene graph of nodes, the rendering engine,
// and all other required systems for using the framework.
//
// Make a subclass of View and override View::Setup() to start adding app logic.
//
// See ../../../samples/simple/simple_view.cc for an example usage.
class View : public BaseView {
 public:
  // Creates a View of type T. T must be a subclass of View.
  template <typename T>
  static std::unique_ptr<T> Create(const std::string& title);
  // Creates a View with a passed Context. Currently test-only.
  template <typename T>
  static std::unique_ptr<T> Create(const std::string& title,
                                   std::unique_ptr<Context> context);

  // Creates a view using the app-defined imp::client_api::CreateView function
  // called with a provided identifier, associates it with the given context,
  // and applies the ViewConfig to the View.
  static std::unique_ptr<View> CreateClient(std::unique_ptr<Context> context,
                                            const std::string& identifier,
                                            const ViewConfig& config);

  // Creates a view using the app-defined imp::client_api::CreateView function
  // called with a provided identifier and associates it with the given context.
  static std::unique_ptr<View> CreateClient(std::unique_ptr<Context> context,
                                            const std::string& identifier);

  // Creates a view using the app-defined imp::client_api::CreateView function
  // called with a default identifier and associates it with the given context.
  static std::unique_ptr<View> CreateClient(std::unique_ptr<Context> context);

  // Called by the view host it is resumed. This should only be called by
  // ViewState or testing::ViewFixture
  void Resume() override;

  // Called by the view host it is paused. This should only be called by
  // ViewState or testing::ViewFixture
  void Pause() override;

  // Creates a new Node owned by this View and returns a NodeHandle to provide
  // access to it. The Node will be destroyed if the View is destroyed.
  NodeHandle CreateNode() override;

  // Destroys a Node owned by this view. If the NodeHandle is invalid, does
  // nothing.
  void DestroyNode(NodeHandle node) override;

  // Iterate over all Nodes in this view, including disabled or inactive nodes.
  void ForEachNode(std::function<void(NodeHandle)>&& fn) override;

  // Iterate over a subset of all Nodes in this view. Only nodes that meet all
  // flags will be included.
  // i.e. ForEachNode(fn, NodeFlags::kIsEnabled | NodeFlags::kIsRoot) will only
  // include enabled root nodes.
  void ForEachNode(std::function<void(NodeHandle)>&& fn,
                   NodeFlag filter) override;

  // Returns the total number of nodes associated with this view.
  std::size_t GetNodeCount() const override;

  // Returns the ComponentManager that is owned by this View.
  // The ComponentManager contains all components for all Nodes that are part
  // of this View and can be used to access them directly.
  ComponentManager& GetComponentManager() noexcept override;

  // Returns the asset manager used to load and cache assets (i.e. gltf files).
  AssetManager& GetAssetManager() noexcept override;
  InputManager& GetInputManager() noexcept override;

  // Returns the camera manager used to access the camera for this view.
  CameraManager& GetCameraManager() noexcept override;

  // Returns the light manager used to control the environment map
  // and default directional light.
  LightManager& GetLightManager() noexcept override;

  GestureManager& GetGestureManager() noexcept override;

  // Returns an object used to send and dispatch events throughout this view.
  Dispatcher& GetDispatcher() noexcept override;

  // Returns the system that handles updating all components that declare an
  // Update method. Typically, this does not need to be accessed directly.
  UpdateSystem& GetUpdateSystem() noexcept override;

  // Returns the system responsible for handling ISF files.
  SceneSystem& GetSceneSystem() noexcept override;

  // Returns the display layer manager for render different groups
  DisplayLayerManager& GetDisplayLayerManager() noexcept override;

  // Returns collision manager for testing collisions across an entire
  // scene.
  CollisionManager& GetCollisionManager() noexcept override;

  // Returns the PathManager, which maintains information on node relationships.
  PathManager& GetPathManager() noexcept override;

  // Returns a factory to create textures.
  TextureFactory& GetTextureFactory() noexcept override;

  // Returns a registry that maps textures by name.
  TextureRegistry& GetTextureRegistry() noexcept override;

  // Returns a factory to create meshes.
  MeshFactory& GetMeshFactory() noexcept override;

  // Returns a factory to create materials.
  MaterialFactory& GetMaterialFactory() noexcept override;

  // Returns a factory to create EnvironmentLights.
  EnvironmentLightFactory& GetEnvironmentLightFactory() noexcept override;

  // Returns information about all  groups that currently exist.
  GroupsManager& GetGroupsManager() noexcept override;

  // Returns device information, i.e. dpi value.
  Device& GetDevice() noexcept override;

  // Returns a registry that maps objects of any type by type.
  //
  // This can be used to simplify dependency injection by registering optional
  // or client systems/features with the view.
  //
  // The Registry is empty by default and only contains what users add to it.
  Registry& GetRegistry() noexcept override;
  const Registry& GetRegistry() const noexcept override;

  // Returns the FilamentHost object that contains the filament engine
  // underlying this View.
  // Do not use this API unless you understand the underlying details of
  // filament.
  window::FilamentHost* GetHost() override { return host_; }

  // Attaches a filament entity to this View if it isn't already and returns a
  // NodeHandle to it. The entity can be null and will return an invalid
  // NodeHandle.
  // Do not use this API unless you understand the underlying details of
  // filament.
  NodeHandle AttachEntityToView(utils::Entity entity) override;

  // Provides a context from the view.
  const Context& GetContext() const override { return *context_; }

  // Returns information about the current frame time.
  const FrameTime& GetFrameTime() const override { return frame_time_; }

  // Returns the dimensions of the view being rendered to in UI pixels.
  uint2 GetSize() const override;
  // Returns the margins of the viewport being rendered to in UI pixels.
  uint4 GetMargins() const override;

  window::WindowRotation GetDisplayRotation() const override;

  void SetScriptMessageHandler(
      scripting::ScriptMessageHandler* script_message_handler) override {
    script_message_handler_ = script_message_handler;
  }

  scripting::ScriptMessageHandler* GetScriptMessageHandler() const override {
    return script_message_handler_;
  }

  void SetScriptEndpoint(void* script_endpoint) override {
    // Do nothing. View subclasses that want to use scripting should implement
    // this. See samples/scripting for an example.
  }

  Monitor* GetMonitor() noexcept override { return host_->GetMonitor(); }

  BaseRenderableManager& GetRenderableManager() override {
    return *renderable_manager_;
  }

  void SetRenderableManager(
      std::unique_ptr<BaseRenderableManager> renderable_manager) override {
    renderable_manager_ = std::move(renderable_manager);
  }

  split_engine::SplitEngineSerializer* GetSplitEngineSerializer() override {
    return split_engine_serializer_.get();
  }

  void SetSplitEngineSerializer(
      std::unique_ptr<split_engine::SplitEngineSerializer>
          split_engine_serializer) override {
    split_engine_serializer_ = std::move(split_engine_serializer);
  }

  bool AreSplitEngineMaterialsInLocalMode() const override;

  absl::string_view GetTitle() const override { return title_; }

  // When using a background executor that requires explicit pumping, drains it
  // with a timeout.
  //
  // Normally, this does nothing because the background executor advances
  // automatically on a background thread. However in some cases (i.e. WASM with
  // threading disabled) the background executor is explicitly pumped.
  void AdvanceBackgroundExecutor();

  // Manually advance the foreground executor with a timeout, while tracking how
  // long it takes.
  //
  // This is normally done automatically when the view advances.
  void AdvanceForegroundExecutor() override;

  // Notifies the view and connected native systems of updates to transient
  // transition parameters, which are caused by e.g. device rotation on iPad.
  // See `ViewTransitionParametersChangedEvent` in view_events.h for an
  // explanation of these fields.
  void UpdateTransitionParameters(float2 transition_scale_adjustment,
                                  float transition_counter_rotation) override;

  filament::View* CreateFilamentView() override;
  void DestroyFilamentView(filament::View* view) override;
  absl::Span<filament::View*> GetFilamentViews() override;

  ~View() override;

 protected:
  View(ViewConfig view_config = kDefaultViewConfig);

  // Runs the main loop of View.  This should only be called by ViewState or
  // testing::ViewFixture.
  void Advance(absl::Duration delta_time);

  // Override in subclass to implement shared logic between the app and editor.
  virtual void RegisterComponents() {}

  // Override in subclass to implement setup logic.
  //
  // For example, loading models, creating nodes, adding components.
  //
  // The impress view isn't fully initialized until Setup is called, so setup
  // logic should be done here instead of in the constructor.
  virtual void Setup() {}

  // Override in subclass to implement sandbox-specific setup logic.
  //
  // Example, positioning the editor camera to some non-default location.
  virtual void SetupSandbox();

  // Override in subclass to implement Cleanup logic.
  //
  // It's recommended to do cleanup work here instead of in the destructor
  // because by the time the destructor is called, the view's internal systems
  // will have already been cleaned up and will no longer be usable (i.e. all
  // nodes and components will be destroyed, AssetManager won't be usable).
  //
  // This is also preferred over the destructor to have symmetry with Setup and
  // to have a place to do cleanup where virtual methods can safely be called.
  //
  // Immediately after this method is called, the view's internal systems will
  // be cleaned up and then the view will be destructed.
  virtual void Cleanup() {}

  // Override this function to respond to a PointerHitEvent.
  virtual void OnPointerHitEvent(const PointerHitEvent& event) {}

  // Callback after the physical display has been rotated.
  virtual void OnDisplayRotationChanged(window::WindowRotation rotation) {}

  // Called when the window dimensions have changed. `dimensions` contains the
  // new width and height of the window in pixels. `margins` contains the
  // left / right / top / bottom margins of the viewport.
  virtual void OnResized(uint2 dimensions, uint4 margins) {}

  // Called once every frame.
  // TODO: make this non-virtual when we add event dispatching.
  virtual void Update(const FrameTime& frame_time) {}

  // Called during rendering, for rendering to render targets. If a client
  // wants to use a renderer object, they must override this.
  // TODO: Rename this method and related methods to OnPreRender to
  // make the naming match OnPostRender and be more general.
  virtual void OnOffscreenRender(filament::Renderer*) {}

  // Called after rendering, e.g. for saving out screenshots.
  virtual void OnPostRender() {}

  // Called after the frame is complete.
  virtual void OnPostFrame() {}

  ViewConfig GetConfig() const override { return view_config_; }

 private:
  using RenderResultFlags = window::FilamentHost::RenderResultFlags;

  void OnHostCreated(window::FilamentHost* host) override;

  void OnHostSetup(uint2 dimensions,
                   Invocable<void(bool)> additional_setup_function) override;

  void OnHostCleanup() override;

  void OnHostSetDisplayRotation(window::WindowRotation rotation) override;

  void OnHostResize(uint2 dimensions, uint4 margins,
                    float2 subpixel_ratio) override;

  void OnHostPreUpdate(
      window::FilamentHost* host, absl::Duration last_vsync_time,
      absl::Duration next_vsync_time,
      window::FilamentHost::UpdateStageFlags* out_flags,
      absl::optional<absl::Duration>* out_time_until_retry) override;

  void OnHostUpdate(
      window::FilamentHost* host, absl::Duration last_vsync_time,
      absl::Duration next_vsync_time,
      const window::FilamentHost::UpdateStageFlags& update_flags) override;

  void OnHostPostUpdate() override;

  void OnHostPreRender(window::FilamentHost* host) override;

  void OnHostPostRender() override;

  void OnHostSecondaryViewRender() override;

  void OnHostPostFrame() override;

  void OnHostOffscreenRender(filament::Renderer* renderer) override;

  void OnHostMultiPassRender() override;

  void ApplyViewConfig();

  std::string title_;

  // Non-owning pointer to the filament host.
  // TODO: Look at removing this pointer entirely to completely get
  // rid of the circular relationship between the host and the view. Instead, we
  // can probably pass the filament::Scene and filament::View into imp::View. We
  // already have access to the engine without the host.
  window::FilamentHost* host_;

  std::unique_ptr<Context> context_;

  // Note: split_engine_serializer_ needs to be above all the singleton systems
  // below so that it is destroyed after them (destruction is in reverse order).
  // Otherwise, any systems that call GetSplitEngineSerializer() trigger msan
  // errors because this field has already been destroyed.
  std::unique_ptr<split_engine::SplitEngineSerializer> split_engine_serializer_;

  ViewConfig view_config_;
  // A list of filament::Views that were created by CreateFilamentView().
  // This is used to apply global state to all user views.
  std::vector<filament::View*> filament_views_;

  // This is responsible for creating and destroying nodes, helping to manage
  // the relationship between filament entities and the view along with their
  // lifetime.
  imp_internal::NodeAttachmentManager node_attachment_manager_;

  Dispatcher dispatcher_;
  // Declare UpdateSystem early so that it is one of the last things destroyed
  // when the View is destroyed. This is because Updaters must be destroyed
  // before the UpdateSystem is.
  UpdateSystem update_system_;
  TextureRegistry texture_registry_;
  ComponentManager component_manager_;
  SceneSystem scene_system_;
  ShaderCacheSystem shader_cache_system_;
  DisplayLayerManager display_layer_manager_;
  std::unique_ptr<AssetManager> asset_manager_;
  CameraManager camera_manager_;
  LightManager light_manager_;
  GestureManager gesture_manager_;
  InputManager input_manager_;
  CollisionManager collision_manager_;
  PathManager path_manager_;
  std::unique_ptr<TextureFactory> texture_factory_;
  MeshFactory mesh_factory_;
  MaterialFactory material_factory_;
  EnvironmentLightFactory environment_light_factory_;
  std::unique_ptr<GroupsManager> groups_manager_;
  std::unique_ptr<BaseRenderableManager> renderable_manager_;
  FrameTime frame_time_;
  Device device_;
  Registry registry_;
  uint2 size_;
  uint4 margins_;
  window::WindowRotation window_rotation_;
  absl::Duration time_since_last_asset_manager_cache_cleanup_ =
      absl::ZeroDuration();

  scripting::ScriptMessageHandler* script_message_handler_;
};

template <typename T>
std::unique_ptr<T> View::Create(const std::string& title) {
  return View::Create<T>(title, std::make_unique<Context>());
}

template <typename T>
std::unique_ptr<T> View::Create(const std::string& title,
                                std::unique_ptr<Context> context) {
  // TODO: Look at removing the methods View::Create and
  // View::CreateClient.
  auto view = std::make_unique<T>();
  view->title_ = title;
  view->context_ = std::move(context);
  return view;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_VIEW_H_
