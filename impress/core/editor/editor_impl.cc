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

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/Viewport.h"
#include "core/assets/gltf/gltf_asset.h"
#include "core/assets/gltf/gltf_audio_extension.h"
#include "core/assets/gltf/gltf_interactivity_extension.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/camera/camera_manager.h"
#include "core/common/registry.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/editor/camera_defaults.h"
#include "core/editor/command_manager.h"
#include "core/editor/components/camera_rotate.h"
#include "core/editor/components/camera_translate.h"
#include "core/editor/components/camera_zoom.h"
#include "core/editor/components/grid.h"
#include "core/editor/components/temporary_editor_metadata.h"
#include "core/editor/editor.h"
#include "core/editor/editor_clipboard.h"
#include "core/editor/editor_constants.h"
#include "core/editor/editor_info.h"
#include "core/editor/editor_input_handler.h"
#include "core/editor/editor_plugin.h"
#include "core/editor/editor_style.h"
#include "core/editor/events.h"
#include "core/editor/file_loader_helper.h"
#include "core/editor/file_type_registry.h"
#include "core/editor/layout/editor_panel_ids.h"
#include "core/editor/layout/layout_composer.h"
#include "core/editor/layout/layout_config.proto.imp.h"
#include "core/editor/selection_controller.h"
#include "core/editor/selection_controller_impl.h"
#include "core/editor/visualizers/camera_visualizer.h"
#include "core/editor/visualizers/light_visualizer.h"
#include "core/editor/visualizers/visualizer_manager.h"
#include "core/editor/widget.h"
#include "core/editor/widget_layout_info.h"
#include "core/editor/widget_ui_system.h"
#include "core/editor/widgets/asset_library.h"
#include "core/editor/widgets/component_ui.h"
#include "core/editor/widgets/console.h"
#include "core/editor/widgets/debug_draw_widget.h"
#include "core/editor/widgets/editor_mode_toggle.h"
#include "core/editor/widgets/environment_light_editor.h"
#include "core/editor/widgets/event_injector.h"
#include "core/editor/widgets/filament_view_settings_widget.h"
#include "core/editor/widgets/file_drag_and_drop.h"
#include "core/editor/widgets/hierarchy.h"
#include "core/editor/widgets/materials_widget.h"
#include "core/editor/widgets/node_details.h"
#include "core/editor/widgets/performance/performance_window.h"
#include "core/editor/widgets/settings_widget.h"
#include "core/editor/widgets/toggle_camera.h"
#include "core/editor/widgets/transform.h"
#include "core/editor/widgets/vertex_select_widget.h"
#include "core/editor/widgets/viewport/viewport_helpers.h"
#include "core/editor/widgets/viewport/viewport_render_target.h"
#include "core/editor/widgets/viewport/viewport_widget.h"
#include "core/editor/widgets/visualize_bounds.h"
#include "core/editor/widgets/visualize_colliders.h"
#include "core/editor/widgets/visualize_origins.h"
#include "core/editor/widgets/window/window_widget.h"
#include "core/geometry/shapes/rect.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/input/pointer_event.h"
#include "core/lighting/light_component.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_data.proto.imp.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_state.proto.imp.h"
#include "core/view/framework/display_layer/display_layer_manager.h"
#include "core/view/framework/gestures/gesture_manager.h"
#include "core/view/framework/gestures/tap_gesture.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/framework/lighting/light_manager.h"
#include "core/view/framework/scene/scene_reference.h"
#include "core/view/framework/scene/scene_system.h"
#include "core/window/filament_host.h"

#if IMP_PLATFORM(ANDROID)
#include "core/editor/components/world_space_editor_ui.h"
#include "core/view/platforms/android/wrappers/imp_lifecycle_callback.h"
#include "core/view/utils/string_map.h"
#include "split_engine/subspace_events.h"
#endif

#if IMP_PLATFORM(DESKTOP) || IMP_PLATFORM(WASM)

#endif

namespace imp::editor {

namespace {
// Number of fingers required to toggle the editor.
constexpr int kToggleEditorPointerCount = 3;
// Select the platform-dependent layout.
constexpr LayoutConfig kDefaultLayoutConfig =
#if (IMP_PLATFORM(ANDROID) || IMP_PLATFORM(IOS))
    kDefaultMobileLayoutConfig;
#else
    kDefaultDesktopLayoutConfig;
#endif
}  // namespace

// The real implementation of the Impress editor.
class EditorImpl : public Editor {
 public:
  // Indicates which camera is used.
  enum class CameraMode { kApp, kEditor };
  // Indicates where the input should be routed to.
  enum class InputMode { kApp, kEditor };
  // A state of the Editor.
  struct EditorState {
    CameraMode camera_mode;
    InputMode input_mode;
  };

  explicit EditorImpl(BaseView* view, std::unique_ptr<EditorPlugin> plugin,
                      bool is_sandbox);
  void Initialize() override;

  bool IsEnabled() const override;
  void SetEnabled(bool enabled) override;
  void SwitchToEditorMode() override;
  void SwitchToAppMode() override;

  // The Editor nodes and environment.
  void AddNode(NodeHandle node) override;
  void RemoveNode(NodeHandle node) override;
  void SelectNode(NodeHandle node,
                  EditorInfo::SelectionMode selection_mode) override;
  const absl::flat_hash_set<NodeHandle>& GetSelectedNodes() override;
  NodeHandle GetSingleSelectedNode() override;
  void AddSandboxNode(NodeHandle node) override;
  void RemoveSandboxNode(NodeHandle node) override;
  NodeHandle GetEditorRoot() override;

  // Sets which camera is used.
  void SetCameraMode(CameraMode camera_mode);
  // Returns the availability of the cameras.
  EditorPlugin::CameraConfiguration GetCameraConfiguration() const override;
  ComponentHandle<CameraComponent> GetCamera() override;
  ComponentHandle<CameraComponent> GetActiveCamera() override;

  // Sets which input mode is used.
  void SetInputMode(InputMode input_mode);
  Dispatcher& GetDispatcher() override;

  // Handles simulation state.
  EditorInfo::RunMode GetRunMode() const override;
  EditorInfo::DisplayMode GetDisplayMode() const override;
  void SetDisplayMode(EditorInfo::DisplayMode display_mode) override;
  void SetInEditMode(bool in_edit_mode) override;
  bool IsPaused() const override;
  void SetPaused(bool paused) override;
  void StepNextFrame() override;
  bool HasFramesToStep() override;

  WidgetUiSystem& GetWidgetUiSystem() override;
  AssetLibrary* GetAssetLibrary() override;
  EventInjector& GetEventInjector() override;
  GltfAsset::LoadOptions GetGltfLoadOptions() const override;

  // Sets the rect of the viewport in pixels.
  void SetViewportRect(std::optional<Rect> rect) override;
  // Returns the rect of the viewport in pixels.
  std::optional<Rect> GetViewportRect() const override;

  void SetUseLegacyCameraControls(bool use_legacy) override;
  bool UseLegacyCameraControls() const override;

 private:
  // Used to provide access to EditorInformation without needing to access the
  // actual full Editor. This is useful to avoid circular dependencies with the
  // Editor class.
  class Info : public EditorInfo {
   public:
    explicit Info(EditorImpl& editor);

    bool IsEnabled() const override;
    EditorInfo::RunMode GetRunMode() const override;
    bool IsPaused() const override;
    bool HasFramesToStep() const override;
    EditorInfo::DisplayMode GetDisplayMode() const override;

   private:
    EditorImpl& editor_;
  };

  // Connects to the given Dispatcher to listen for events which may trigger the
  // switching of the active CameraMode.
  void EnableCameraModeToggling(Dispatcher& dispatcher);

  // Connects to the given Dispatcher to listen for events which may trigger the
  // enabling or disabling of Editor.
  void EnableEditorToggling(Dispatcher& dispatcher);

  // Enables Undo/Redo functionality keyboard triggers.
  void EnableUndoAndRedo(Dispatcher& dispatcher);

  // Enables EditorClipboard functionality (cut/copy/paste).
  void EnableEditorClipboard(Dispatcher& dispatcher);

  // Registers a handler for dealing with changes made through "Settings" menu.
  void RegisterEditorSettingChangedEventHandler();

  // Forwards UpdateSystem::PreComponentsUpdateEvent and
  // UpdateSystem::PostComponentsUpdateEvent from the app dispatcher to the
  // Editor Dispatcher.
  void EnableUpdateSystemEventForwarding();

  // Switches between CameraModes. Should only be used under the
  // CameraConfiguration::kEditorAndAppCameraDefault, will switch InputMode as
  // well.
  void ToggleCameraMode();

  // Updates the active camera if it's valid.
  void UpdateActiveCamera();

#if IMP_PLATFORM(ANDROID)
  // TODO: Find a way to distinguish between SplitEngine app on
  // mobile and Xr.
  void SpawnWorldSpaceEditor();
#endif  // IMP_PLATFORM(ANDROID)

  std::vector<NodeHandle> GetAllOverlappingNodes(Pointer p);

  // Places the Editor camera at the average position of all nodes in the
  // View, ignoring LightComponents, editor nodes, and the app camera.
  void InitializeCameraPosition();

  // Registers common widgets and initializes the WidgetUISystem.
  void InitializeWidgetUiSystem();

  // Applies the given setting of camera mode and input mode to the Editor.
  void ApplyEditorState(EditorState editor_state);

  // Used in Sandbox mode to store the NodeData for the state of the scene at
  // the time of switching to Play mode so that we can restore it when switching
  // back to Edit mode.
  struct BackupNodeData {
    std::string path;
    NodeData node_data;
    SceneSystem::MetadataMode metadata_mode;
  };

  std::unique_ptr<EditorPlugin> plugin_;
  Dispatcher dispatcher_;
  WidgetUiSystem widget_ui_system_;
  std::unique_ptr<ViewportRenderTarget> viewport_widget_render_target_;
  std::optional<Rect> viewport_widget_rect_;
  float2 last_pixel_ratio_ = {-1.0f, -1.0f};
  RobinSet<Widget*> selected_node_component_widgets_;
  NodeHandle editor_root_node_;
  EditorState current_state_;
  // Used to store the state of the Editor when it is disabled.
  EditorState saved_state_;
  NodeHandle grid_;
  ComponentHandle<CameraComponent> camera_;
  bool camera_position_initialized_;
  CommandManager& command_manager_;
  AssetLibrary* asset_library_;
  EventInjector* event_injector_;
  std::unique_ptr<VisualizerManager> visualizer_manager_;
  std::unique_ptr<GestureManager> gesture_manager_;
  Dispatcher::ScopedConnection gesture_manager_connection_;
  bool is_editor_input_handler_in_use_ = false;
  bool enabled_;
  bool initialized_ = false;
  bool is_sandbox_ = false;
  EditorInfo::RunMode run_mode_ = EditorInfo::RunMode::kPlayMode;
  bool is_paused_ = false;
  // default to native screen
  EditorInfo::DisplayMode display_mode_ =
      EditorInfo::DisplayMode::kNativeScreen;
  bool has_frames_to_step_ = false;
  std::vector<BackupNodeData> backup_node_data_;
  std::vector<NodeHandle> sandbox_nodes_;
  GltfAsset::LoadOptions gltf_load_options_;
  EditorInfo::PlatformMode platform_mode_ = EditorInfo::PlatformMode::kDefault;
  bool use_legacy_camera_controls_ = true;
};

EditorImpl::EditorImpl(BaseView* view, std::unique_ptr<EditorPlugin> plugin,
                       bool is_sandbox)
    : Editor(view),
      plugin_(std::move(plugin)),
      widget_ui_system_(view, true,
                        std::make_unique<LayoutComposer>(kDefaultLayoutConfig)),
      editor_root_node_(NodeHandle()),
      current_state_({CameraMode::kApp, InputMode::kApp}),
      camera_position_initialized_(false),
      command_manager_(view->GetRegistry().GetOrCreate<CommandManager>()),
      enabled_(false),
      is_sandbox_(is_sandbox) {
  viewport_widget_render_target_ = std::make_unique<ViewportRenderTarget>(
      *view->GetHost()->GetEngine(), uint2{1, 1});

  // In sandbox builds, start off in edit mode.
  if (is_sandbox_) {
    run_mode_ = EditorInfo::RunMode::kEditMode;
  }

  view->GetRegistry().Register<EditorInfo>(std::make_unique<Info>(*this));
  FileTypeRegistry& file_type_registry =
      view->GetRegistry().GetOrCreate<FileTypeRegistry>();
  file_type_registry.RegisterFileTypeLoader(
      kFileTypeGltf, std::make_unique<GltfFileLoader>(*view));
  file_type_registry.RegisterFileTypeLoader(
      kFileTypeIsf, std::make_unique<IsfFileLoader>(*view));
  file_type_registry.RegisterFileTypeLoader(
      kFileTypeHdrImage, std::make_unique<IblFileLoader>(*view));
#if IMP_RUNTIME(DEV)
  file_type_registry.RegisterFileTypeLoader(
      kFileTypeIsfTextProto, std::make_unique<TextProtoFileLoader>(*view));
#endif
}

void EditorImpl::Initialize() {
  if (initialized_) {
    IMP_LOG(imp::INFO) << "Initialize() has already called. Ignoring this call...";
    return;
  }
  GetView().GetRegistry().Register<SelectionController>(
      std::make_unique<SelectionControllerImpl>(&GetView()));
  editor_root_node_ = GetView().CreateNode();
  editor_root_node_->SetEnabled(false);
#if IMP_RUNTIME(DEV)
  editor_root_node_->SetAsEditorStaging(true);
#endif

  // TODO: Find a way to distinguish between SplitEngine app on
  // mobile and Xr.
  if (GetView().GetSplitEngineSerializer()) {
    platform_mode_ = EditorInfo::PlatformMode::kXrSplitEngineApp;
  }

  InitializeWidgetUiSystem();

  if (platform_mode_ != EditorInfo::PlatformMode::kXrSplitEngineApp) {
    visualizer_manager_ = std::make_unique<VisualizerManager>(&GetView());
    visualizer_manager_->RegisterVisualizer<LightComponent, LightVisualizer>();
    if (GetCameraConfiguration() !=
        EditorPlugin::CameraConfiguration::kEditorCameraOnly) {
      // When the app camera presents, showing the camera visualization.
      visualizer_manager_
          ->RegisterVisualizer<CameraComponent, CameraVisualizer>();
    }
  }

  // TODO Declare the dependencies within each extensions instead
  // of require the implementer to declare them.
  GetView()
      .GetComponentManager()
      .GetComponentSystem<GltfRenderer>()
      .RegisterExtensionWithDependency<GltfInteractivityExtension,
                                       GltfAudioExtension>();

  // Register TemporaryEditorMetadata so it can be saved/loaded with the scene.
  GetView()
      .GetSceneSystem()
      .RegisterComponentIsfInfo<TemporaryEditorMetadata>();

  // Set the default light and camera components as authored so that the full
  // component widgets are available in edit mode.
  //
  // This isn't needed outside of sandbox builds, because in that case we are
  // always in play mode, and all component widgets always show up in play mode
  // since that is appropriate for the debugging use case of the editor.
  if (is_sandbox_) {
    auto camera_metadata = GetView()
                               .GetCameraManager()
                               .GetCamera()
                               ->GetNode()
                               ->GetOrAddComponent<SceneMetadata>();
    camera_metadata->SetComponentAuthored(
        CameraComponent::IsfInfo::kTypeUrlHash, true);
    if (GetView().GetLightManager().IsDefaultLoadEnabled()) {
      auto light_metadata = GetView()
                                .GetLightManager()
                                .GetOrCreateDefaultDirectionalLight()
                                ->GetNode()
                                ->GetOrAddComponent<SceneMetadata>();
      light_metadata->SetComponentAuthored(
          LightComponent::IsfInfo::kTypeUrlHash, true);
    }
  }

  // Build the Editor camera pivot.
  NodeHandle pivot = GetView().CreateNode();
  pivot->SetName("editor-camera-pivot");
  AddNode(pivot);

  // Build the Editor camera.
  NodeHandle camera_node = GetView().CreateNode();
  camera_node->SetName("editor-camera");
  camera_node->SetParent(pivot);
  camera_ = camera_node->AddComponent<CameraComponent>();
  camera_->SetNearAndFarClip(CameraDefaults::kNearClipPlane,
                             CameraDefaults::kFarClipPlane);
  camera_node->AddComponent<CameraRotate>(pivot, CameraDefaults::kPitch,
                                          CameraDefaults::kYaw);
  camera_node->AddComponent<CameraTranslate>(pivot);
  camera_node->AddComponent<CameraZoom>(pivot);

  if (platform_mode_ != EditorInfo::PlatformMode::kXrSplitEngineApp) {
    // An overlaying helper layer for 3d widgets and visualizers, e.g. transform
    // widget shows here.
    GetView().GetDisplayLayerManager().CreateLayer(kOverlayGroup,
                                                   kOverlayGroup);
    GetView().GetDisplayLayerManager().SetCamera(kOverlayGroup, camera_);

    // Build the Editor grid.
    grid_ = GetView().CreateNode();
    grid_->SetName("grid");
    grid_->AddComponent<Grid>().KeptBy(grid_);
    AddNode(grid_);
  }

  Dispatcher& app_dispatcher = GetView().GetDispatcher();

  // Allow editor toggling from both App camera mode (app Dispatcher) and Editor
  // camera mode (Editor Dispatcher).
  EnableEditorToggling(app_dispatcher);
  EnableEditorToggling(dispatcher_);

  // Allow camera mode toggling from both App camera mode (app Dispatcher) and
  // Editor camera mode (Editor Dispatcher).
  EnableCameraModeToggling(app_dispatcher);
  EnableCameraModeToggling(dispatcher_);

  // Enable undo/redo of changes to the scene made in the editor.
  // TODO: Remove the connection to app dispatcher when
  // successfully prevent the user to change node properties when app dispatcher
  // handles the input.
  EnableUndoAndRedo(app_dispatcher);
  EnableUndoAndRedo(dispatcher_);

  EditorClipboard& editor_clipboard =
      GetView().GetRegistry().GetOrCreate<EditorClipboard>(&GetView());
  editor_clipboard.EnableDispatcherEvents(dispatcher_);
  editor_clipboard.EnableDispatcherEvents(app_dispatcher);

  // Allows loading mesh data on CPU.
  // Allows Vertex Selection functionality for meshes loaded on CPU.
  RegisterEditorSettingChangedEventHandler();

  // Forward UpdateSystem::PreComponentsUpdateEvent& and
  // UpdateSystem::PostComponentsUpdateEvent& to Editor dispatcher.
  EnableUpdateSystemEventForwarding();

#if IMP_PLATFORM(ANDROID)
  if (platform_mode_ == EditorInfo::PlatformMode::kXrSplitEngineApp) {
    SpawnWorldSpaceEditor();
  }
#endif

  IMP_LOG(imp::INFO) << "Camera Controls:";
  IMP_LOG(imp::INFO) << "Left Click & Drag to rotate the camera.";
  IMP_LOG(imp::INFO) << "Right Click & Drag to move the camera.";
  IMP_LOG(imp::INFO) << "Rotate the scroll wheel to zoom the camera in & out.";
  initialized_ = true;

  if (plugin_) {
    // If an EditorPlugin is present, invoke the OnEditorInitialized() callback
    // and get a LayoutComposer.
    plugin_->OnEditorInitialized();
    std::unique_ptr<LayoutComposer> layout_composer =
        plugin_->CreateLayoutComposer();
    if (layout_composer) {
      widget_ui_system_.SetLayoutComposer(std::move(layout_composer));
    }
  }

  // Setup the ImGui style for the editor.
  // If the extension doesn't exist, then ImGui hasn't been setup so this can't
  // be done. This happens in unit tests.
  if (GetView().GetHost()->TryGetExtension() != nullptr) {
    SetupEditorImGuiStyle();
  }

#if IMP_ENABLE_EDITOR_ON_STARTUP
  SetEnabled(true);
  SwitchToEditorMode();
#else
  SetEnabled(false);
#endif
}

bool EditorImpl::IsEnabled() const { return enabled_; }

void EditorImpl::SetEnabled(bool enabled) {
  if (!initialized_) {
    IMP_LOG(imp::FATAL) << "A call to Initialize() is required before using the Editor.";
  }

  enabled_ = enabled;
  widget_ui_system_.SetEnabled(enabled);

  SetViewportRect(std::nullopt);

#if IMP_PLATFORM(ANDROID)
  absl::StatusOr<std::reference_wrapper<ImpLifeCycleCallback>> callback =
      GetView().GetRegistry().Get<ImpLifeCycleCallback>();
  if (callback.ok()) {
    callback.value().get().OnEditorEnabled(enabled_);
  }
#endif

  if (enabled) {
    ApplyEditorState(saved_state_);
  } else {
    saved_state_ = current_state_;
    ApplyEditorState({CameraMode::kApp, InputMode::kApp});
#if IMP_RUNTIME(DEV)
    GetView().SetSizeOverride(std::nullopt);
#endif
  }

  GetView().GetDispatcher().Send(EditorEnabledEvent(enabled));
}

void EditorImpl::SwitchToEditorMode() {
  if (!enabled_) {
    return;
  }
  ApplyEditorState({CameraMode::kEditor, InputMode::kEditor});
}

void EditorImpl::SwitchToAppMode() {
  if (!enabled_) {
    return;
  }
  ApplyEditorState({CameraMode::kApp, InputMode::kApp});
}

void EditorImpl::ApplyEditorState(EditorState editor_state) {
  // Make sure the camera mode is valid for the current configuration.
  CameraMode camera_mode = (GetCameraConfiguration() ==
                            EditorPlugin::CameraConfiguration::kAppCameraOnly)
                               ? CameraMode::kApp
                               : editor_state.camera_mode;
  camera_mode = (GetCameraConfiguration() ==
                 EditorPlugin::CameraConfiguration::kEditorCameraOnly)
                    ? CameraMode::kEditor
                    : camera_mode;
  SetCameraMode(camera_mode);
  SetInputMode(editor_state.input_mode);
}

void EditorImpl::ToggleCameraMode() {
  if (!enabled_) {
    return;
  }
  ApplyEditorState((current_state_.camera_mode == CameraMode::kApp)
                       ? EditorState{CameraMode::kEditor, InputMode::kEditor}
                       : EditorState{CameraMode::kApp, InputMode::kApp});
}

void EditorImpl::InitializeCameraPosition() {
  NodeHandle app_camera = GetView().GetCameraManager().GetCamera()->GetNode();
  float3 average_position(0, 0, 0);
  int32_t count = 0;
  GetView().ForEachNode(
      [this, app_camera, &average_position, &count](NodeHandle node) {
        if (node == app_camera) {
          return;
        }
        if (node->GetComponent<LightComponent>()) {
          return;
        }
        PathManager& path_manager = GetView().GetPathManager();
        if (path_manager.IsAncestorOf(editor_root_node_, node)) {
          return;
        }
        average_position += node->GetWorldPosition();
        count++;
      },
      NodeFlags::kIsRoot);

  if (count > 0) {
    average_position /= count;
  }

  camera_->GetNode()->GetParent()->SetWorldPosition(average_position);
  camera_->GetNode()->SetWorldPosition(average_position +
                                       CameraDefaults::kStartingPosition);
  camera_position_initialized_ = true;
}

void EditorImpl::InitializeWidgetUiSystem() {
  BaseView& view = GetView();

  widget_ui_system_.AddWidget<ViewportWidget>(
      WidgetLayoutInfo(PanelId::kViewport,
                       WidgetPresence::kOnlyIn2DLargeScreen),
      *this, view, viewport_widget_render_target_.get());

  widget_ui_system_.AddWidget<Hierarchy>(
      WidgetLayoutInfo(PanelId::kSceneWindow), view);
  if (GetCameraConfiguration() ==
      EditorPlugin::CameraConfiguration::kEditorAndAppCameraDefault) {
    widget_ui_system_.AddWidget<ToggleCamera>(
        WidgetLayoutInfo(PanelId::kToolBar), view);
  }

  widget_ui_system_.AddWidget<NodeDetails>(
      WidgetLayoutInfo(PanelId::kDetailsWindow), view);
  widget_ui_system_.AddWidget<Transform>(
      WidgetLayoutInfo(PanelId::kDetailsWindow), view);
  widget_ui_system_.AddWidget<MaterialsWidget>(
      WidgetLayoutInfo(PanelId::kDetailsWindow), view);
  widget_ui_system_.AddWidget<ComponentUi>(
      WidgetLayoutInfo(PanelId::kDetailsWindow), view,
      /*component_widgets_panel_id=*/
      WidgetLayoutInfo(PanelId::kDetailsWindow));

  // TODO: Console widget is not supported in SplitEngineApp yet,
  // since it uses filament textures.
  if (platform_mode_ != EditorInfo::PlatformMode::kXrSplitEngineApp) {
    widget_ui_system_.AddWidget<Console>(WidgetLayoutInfo(PanelId::kTabBar),
                                         view);
  }

  asset_library_ = widget_ui_system_.AddWidget<AssetLibrary>(
      WidgetLayoutInfo(PanelId::kTabBar, WidgetPresence::kOnlyIn2DLargeScreen),
      view);
  widget_ui_system_.AddWidget<SettingsWidget>(
      WidgetLayoutInfo(PanelId::kMenuBar, WidgetPresence::kOnlyIn2DLargeScreen),
      GetView());
  widget_ui_system_.AddWidget<WindowWidget>(
      WidgetLayoutInfo(PanelId::kMenuBar, WidgetPresence::kOnlyIn2DLargeScreen),
      GetView());

  widget_ui_system_.AddWidget<VisualizeBounds>(
      WidgetLayoutInfo(PanelId::kFreeform), view);
  widget_ui_system_.AddWidget<VisualizeColliders>(
      WidgetLayoutInfo(PanelId::kFreeform), view);
  widget_ui_system_.AddWidget<VisualizeOrigins>(
      WidgetLayoutInfo(PanelId::kFreeform), view);
  widget_ui_system_.AddWidget<FileDragAndDrop>(
      WidgetLayoutInfo(PanelId::kFreeform), view);
  widget_ui_system_.AddWidget<PerformanceWindow>(
      WidgetLayoutInfo(PanelId::kFreeform, WidgetPresence::kAlways,
                       WidgetVisibility::kHidden),
      view);
  widget_ui_system_.AddWidget<EnvironmentLightEditor>(
      WidgetLayoutInfo(PanelId::kFreeform, WidgetPresence::kAlways,
                       WidgetVisibility::kHidden),
      view);
  widget_ui_system_.AddWidget<FilamentViewSettingsWidget>(
      WidgetLayoutInfo(PanelId::kFreeform, WidgetPresence::kAlways,
                       WidgetVisibility::kHidden),
      view);
  event_injector_ = widget_ui_system_.AddWidget<EventInjector>(
      WidgetLayoutInfo(PanelId::kFreeform, WidgetPresence::kAlways,
                       WidgetVisibility::kHidden),
      view);

  widget_ui_system_.AddWidget<DebugDrawWidget>(WidgetLayoutInfo(
      PanelId::kTabBar, WidgetPresence::kAlways, WidgetVisibility::kHidden));

  // Edit mode is only available in the Impress sandbox.
  if (is_sandbox_) {
    widget_ui_system_.AddWidget<EditorModeToggle>(
        WidgetLayoutInfo(PanelId::kToolBar), view);
  }
}

void EditorImpl::SetCameraMode(CameraMode camera_mode) {
  // Return early if the desired camera is already in use.
  if (current_state_.camera_mode == camera_mode) {
    return;
  }

  switch (camera_mode) {
    case CameraMode::kEditor:
      // Initialize editor camera position and put the editor camera in use.
      if (!camera_position_initialized_) {
        InitializeCameraPosition();
      }
      GetView().GetHost()->SetEditorCameraOverride({}, camera_->GetCamera());
      current_state_.camera_mode = CameraMode::kEditor;

      // Enables the lighting, grid, and all Nodes under the editor root Node,
      // and visualize them.
      editor_root_node_->SetEnabled(true);
      GetView().GetDisplayLayerManager().SetLayerEnabled(kOverlayGroup, true);

      break;
    case CameraMode::kApp:
      GetView().GetHost()->SetEditorCameraOverride({}, nullptr);
      current_state_.camera_mode = CameraMode::kApp;

      // Disables the lighting, grid, visualizers and all Nodes under the editor
      // root Node.
      editor_root_node_->SetEnabled(false);
      GetView().GetDisplayLayerManager().SetLayerEnabled(kOverlayGroup, false);
  }

  // Ensure the active camera's projection matches the current viewport.
  UpdateActiveCamera();
}

void EditorImpl::SetInputMode(editor::EditorImpl::InputMode input_mode) {
  // Return early if desired input mode is already in use.
  if (current_state_.input_mode == input_mode) {
    return;
  }
  current_state_.input_mode = input_mode;

  switch (input_mode) {
    case editor::EditorImpl::InputMode::kEditor:
      // Use the editor-specific Dispatcher.
      // Support gestures in Editor input mode.
      if (!gesture_manager_) {
        gesture_manager_ = std::make_unique<GestureManager>(&dispatcher_);
      }
      // Store the connection to later disconnect it.
      gesture_manager_connection_ =
          dispatcher_.Connect([this](const PointerHitEvent& event) {
            gesture_manager_->OnPointerHitEvent(event);
          });

      // Clean up the previous input handler added by the Editor.
      if (is_editor_input_handler_in_use_) {
        GetView().GetInputManager().PopInputHandler();
      }
      // Editor will take over the input.
      GetView().GetInputManager().PushInputHandler(
          std::make_unique<EditorInputHandler>(&GetView(), dispatcher_));
      is_editor_input_handler_in_use_ = true;
      break;
    case editor::EditorImpl::InputMode::kApp:
      // Disconnect the Editor input.
      gesture_manager_connection_.Disconnect();
      if (is_editor_input_handler_in_use_) {
        GetView().GetInputManager().PopInputHandler();
      }
      is_editor_input_handler_in_use_ = false;

      // Push a specific input handler that can use the Editor camera, if in
      // Editor camera only mode.
      if (GetCameraConfiguration() ==
          EditorPlugin::CameraConfiguration::kEditorCameraOnly) {
        // Use an app dispatcher what uses the editor camera.
        GetView().GetInputManager().PushInputHandler(
            std::make_unique<EditorInputHandler>(&GetView(),
                                                 GetView().GetDispatcher()));
        is_editor_input_handler_in_use_ = true;
      }
  }
}

void EditorImpl::EnableCameraModeToggling(Dispatcher& dispatcher) {
  // Toggle the camera mode on Tab.
  dispatcher.Connect(
      [this](const imp::KeyboardEvent& event) {
        if (event.type == KeyboardEventType::kOnUp &&
            event.key.code == VirtualKeyCode::VK_TAB) {
          ToggleCameraMode();
        }
      },
      this);
  // Toggle the camera on ToggleCameraEvent.
  dispatcher.Connect(
      [this](const ToggleCameraEvent& event) { ToggleCameraMode(); }, this);
}

void EditorImpl::EnableEditorToggling(Dispatcher& dispatcher) {
  // Toggle the Editor on F1.
  dispatcher.Connect(
      [this](const imp::KeyboardEvent& event) {
        if (event.type == KeyboardEventType::kOnUp &&
            event.key.code == VirtualKeyCode::VK_F1) {
          SetEnabled(!enabled_);
        }
      },
      this);
  // Toggle the Editor with 3 finger tap.
  dispatcher.Connect(
      [this](const imp::TapGesture::TapEvent& event) mutable {
        if (event.pointer_count == kToggleEditorPointerCount) {
          SetEnabled(!enabled_);
        }
      },
      this);
}

void EditorImpl::EnableUndoAndRedo(Dispatcher& dispatcher) {
  dispatcher.Connect(
      [this](const imp::KeyboardEvent& event) {
        if (!enabled_ || event.type != KeyboardEventType::kOnDown) {
          return;
        }
        if (event.key.code == VirtualKeyCode::VK_z &&
            HasKeyModifier(KeyModifier::CTRL_OR_GUI, event.key.modifiers)) {
          command_manager_.Undo();
        }
        if (event.key.code == VirtualKeyCode::VK_y &&
            HasKeyModifier(KeyModifier::CTRL_OR_GUI, event.key.modifiers)) {
          command_manager_.Redo();
        }
      },
      this);
}

void EditorImpl::RegisterEditorSettingChangedEventHandler() {
  dispatcher_.Connect(
      [this](const EditorSettingChangedEvent& event) {
        if (event.vertex_selection_enabled.has_value()) {
          if (*event.vertex_selection_enabled) {
            widget_ui_system_.AddWidget<VertexSelectWidget>(
                WidgetLayoutInfo(PanelId::kFreeform), GetView(), dispatcher_,
                editor_root_node_);
          } else {
            widget_ui_system_.RemoveWidget<VertexSelectWidget>();
          }
        }

        std::optional<bool> load_mesh_data_on_cpu_enabled =
            event.load_mesh_data_on_cpu_enabled;
        if (load_mesh_data_on_cpu_enabled.has_value()) {
          if (*load_mesh_data_on_cpu_enabled) {
            gltf_load_options_.collider_mode =
                GltfState::ColliderMode::GLTF_COLLIDER_TRIANGLES_PER_MESH;
          } else {
            gltf_load_options_.collider_mode =
                GltfState::ColliderMode::GLTF_COLLIDER_BOUNDS_PER_MESH_DEFAULT;
          }
        }

        std::optional<bool> bvh_mesh_collision_acceleration_enabled =
            event.bvh_mesh_collision_acceleration_enabled;
        if (bvh_mesh_collision_acceleration_enabled.has_value()) {
          if (*bvh_mesh_collision_acceleration_enabled) {
            gltf_load_options_.collider_mode = GltfState::ColliderMode::
                GLTF_COLLIDER_MESH_COLLISION_ACCELERATOR;
          } else {
            gltf_load_options_.collider_mode =
                GltfState::ColliderMode::GLTF_COLLIDER_TRIANGLES_PER_MESH;
          }
        }
      },
      this);
}

void EditorImpl::EnableUpdateSystemEventForwarding() {
  GetView().GetDispatcher().Connect(
      [this](const UpdateSystem::PreComponentsUpdateEvent& event) {
        dispatcher_.Send(event);
      },
      this);
  GetView().GetDispatcher().Connect(
      [this](const UpdateSystem::PostComponentsUpdateEvent& event) {
        dispatcher_.Send(event);
        // The component update pass has ended, so the frame has been stepped.
        if (run_mode_ == EditorInfo::RunMode::kPlayMode && is_paused_ &&
            has_frames_to_step_) {
          has_frames_to_step_ = false;
        }
      },
      this);
}

// TODO: Make it so that EditorUI is not attached to the first
// subspace root that is created so that the editor can select objects from
// multiple subspaces.
#if IMP_PLATFORM(ANDROID)
void EditorImpl::SpawnWorldSpaceEditor() {
  imp::editor::LayoutConfig world_layout = imp::editor::kDefaultXrLayoutConfig;
  GetView()
      .GetRegistry()
      .Get<imp::editor::Editor>()
      ->get()
      .GetWidgetUiSystem()
      .SetLayoutComposer(
          std::make_unique<imp::editor::LayoutComposer>(world_layout));

  StringMap<imp::float3> canvas_position_map = {
      {imp::editor::PanelIdToString(imp::editor::PanelId::kSceneWindow),
       kScenePanelSphericalLocation},
      {imp::editor::PanelIdToString(imp::editor::PanelId::kDetailsWindow),
       kDetailsPanelSphericalLocation},
      {imp::editor::PanelIdToString(imp::editor::PanelId::kTabBar),
       kTabBarSphericalLocation}};

  imp::NodeHandle split_engine_ui_node = GetEditorRoot()->CreateChildNode();
  split_engine_ui_node
      ->AddComponent<imp::editor::WorldSpaceEditorUi>(
          kDefaultWorldLayoutCanvasSize, canvas_position_map)
      .KeptBy(this);
  split_engine_ui_node->SetEnabled(true);
}
#endif  // IMP_PLATFORM(ANDROID)

EditorInfo::RunMode EditorImpl::GetRunMode() const { return run_mode_; }

EditorInfo::DisplayMode EditorImpl::GetDisplayMode() const {
  return display_mode_;
}

void EditorImpl::SetDisplayMode(EditorInfo::DisplayMode display_mode) {
  if (display_mode_ == display_mode) return;
  display_mode_ = display_mode;

  if (display_mode_ == EditorInfo::DisplayMode::kRemoteScreen) {
    // Do not use the viewport override in Remote Editor.
    // We continue rendering on the device rather than a viewport widget.
#if IMP_RUNTIME(DEV)
    GetView().SetSizeOverride(std::nullopt);
#endif
    SetViewportRect(std::nullopt);
  }
}

void EditorImpl::SetInEditMode(bool in_edit_mode) {
  if (!is_sandbox_) return;

  // If we're currently switching modes, return early.
  if (run_mode_ == EditorInfo::RunMode::kSwitchingToEditMode ||
      run_mode_ == EditorInfo::RunMode::kSwitchingToPlayMode) {
    return;
  }

  // Determine the target run mode and the switch mode.
  EditorInfo::RunMode run_mode = in_edit_mode ? EditorInfo::RunMode::kEditMode
                                              : EditorInfo::RunMode::kPlayMode;
  EditorInfo::RunMode switch_mode =
      in_edit_mode ? EditorInfo::RunMode::kSwitchingToEditMode
                   : EditorInfo::RunMode::kSwitchingToPlayMode;

  // If we're already in the target run mode, return early.
  if (run_mode_ == run_mode) {
    return;
  }

  // Entering Play mode works by backing up the entire
  // scene graph as NodeData so that it can be recreated as a way to revert the
  // nodes to their original state when Stop is pressed.

  // Used to assign additional tracking information for nodes that will be
  // destroyed and recreated.
  int64_t next_node_id = 0;
  std::function<void(NodeHandle)> add_metadata_recursive =
      [&next_node_id, &add_metadata_recursive](NodeHandle node) {
        // Add or update TemporaryEditorMetadata with a unique ID for this
        // session.
        auto editor_metadata =
            node->GetOrAddComponent<TemporaryEditorMetadata>();
        editor_metadata->SetId(next_node_id++);

        auto scene_metadata = node->GetOrAddComponent<SceneMetadata>();
        scene_metadata->SetComponentAuthored(
            TemporaryEditorMetadata::IsfInfo::kTypeUrlHash, true);

        for (NodeHandle child : node->GetChildren()) {
          add_metadata_recursive(child);
        }
      };

  // Used to gather all the nodes that must be destroyed when switching modes.
  std::vector<NodeHandle> to_destroy;
  GetView().ForEachNode(
      [this, &to_destroy, in_edit_mode,
       &add_metadata_recursive](NodeHandle node) {
        // Don't recreate editor nodes.
        if (node == editor_root_node_) {
          return;
        }

#if IMP_RUNTIME(DEV)
        if (node->IsEditorStaging()) {
          return;
        }
#endif

        // Don't recreate sandbox nodes.
        for (const auto& sandbox_node : sandbox_nodes_) {
          if (node == sandbox_node) {
            return;
          }
        }

        // Skip the default camera.
        // TODO: What happens when the main camera has been
        // re-parented? Or when the user overrides the main camera?
        if (node ==
            GetView().GetCameraManager().GetDefaultCamera()->GetNode()) {
          return;
        }

        // Skip the default light.
        if (GetView().GetLightManager().GetDefaultDirectionalLight() &&
            node == GetView()
                        .GetLightManager()
                        .GetDefaultDirectionalLight()
                        ->GetNode()) {
          return;
        }

        // If we are entering Play mode, then we must save all the root nodes as
        // NodeData so that they can be restored to their original state when
        // stopping.
        if (!in_edit_mode) {
          // Add additional metadata to the nodes being destroyed.
          add_metadata_recursive(node);

          // Handle nodes that don't have metadata as well for things like
          // default cameras and lights that aren't created by the editor.
          auto scene_metadata = node->GetComponent<SceneMetadata>();
          SceneSystem::MetadataMode metadata_mode =
              scene_metadata ? SceneSystem::MetadataMode::kInclude
                             : SceneSystem::MetadataMode::kExclude;
          SceneSystem::SaveMode save_mode =
              scene_metadata ? SceneSystem::SaveMode::kAuthoredContent
                             : SceneSystem::SaveMode::kFull;

          std::string path;
          auto scene_reference = node->GetComponent<SceneReference>();
          if (scene_reference) {
            path = scene_reference->GetAssetUrl();
          }

          absl::StatusOr<NodeData> node_data =
              GetView().GetSceneSystem().SaveToData(node, save_mode);
          if (!node_data.ok()) {
            IMP_LOG(imp::FATAL) << "Unable to play scene because node failed to save "
                       << node_data.status();
          }
          backup_node_data_.push_back({path, *node_data, metadata_mode});
        }

        to_destroy.push_back(node);
      },
      NodeFlags::kIsRoot);

  // Gather the IDs of the selected nodes so that they can be reselected after
  // the nodes are recreated.
  // Also, identify if any selected nodes are about to be destroyed (either
  // directly or as descendants) and deselect them to prevent
  // SelectionController from holding invalid handles.
  std::vector<int64_t> selected_node_ids;
  absl::flat_hash_set<NodeHandle> currently_selected_nodes = GetSelectedNodes();
  selected_node_ids.reserve(currently_selected_nodes.size());
  absl::flat_hash_set<NodeHandle> roots_to_destroy(to_destroy.begin(),
                                                   to_destroy.end());

  for (NodeHandle node : currently_selected_nodes) {
    if (auto metadata = node->GetComponent<TemporaryEditorMetadata>()) {
      selected_node_ids.push_back(metadata->GetId());
    }

    NodeHandle current = node;
    while (current) {
      if (roots_to_destroy.contains(current)) {
        SelectNode(node, EditorInfo::SelectionMode::kMultipleNodes);
        break;
      }
      current = current->GetParent();
    }
  }

  // Destroy the nodes and re-create from the NodeData
  //
  // This is done even when entering Play mode instead of just using the already
  // existing nodes because many of the components haven't had Setup called on
  // them yet, and destroying then re-creating the scene is the easiest way to
  // get all the Setup methods to run with the correct dependency order.
  for (NodeHandle node : to_destroy) {
    GetView().DestroyNode(node);
  }

  // If the active camera is destroyed, then set the camera back to the default.
  if (!GetView().GetCameraManager().GetCamera().IsValid()) {
    GetView().GetCameraManager().SetCamera(
        GetView().GetCameraManager().GetDefaultCamera());
  }

  // Wait until after destroying nodes to set the flag so that it doesn't impact
  // the life cyle methods of destroyed nodes.
  run_mode_ = switch_mode;

  // Recreate the scene from the backed up NodeData.
  Future<absl::Status> result(absl::OkStatus());
  for (const BackupNodeData& backup_node_data : backup_node_data_) {
    result = result.Combine(GetView().GetSceneSystem().LoadScene(
        backup_node_data.node_data, backup_node_data.path,
        SceneSystem::LoadSceneOptions{.metadata_mode =
                                          backup_node_data.metadata_mode}));
  }

  // Chain the reselection logic to run after loading completes.
  result = result.Then([this, selected_node_ids = std::move(selected_node_ids)](
                           absl::Status status) -> absl::Status {
    if (!status.ok()) {
      return status;
    }

    GetView().GetComponentManager().ForEach<TemporaryEditorMetadata>(
        [this, &selected_node_ids](TemporaryEditorMetadata* metadata) {
          if (!metadata) {
            return;
          }
          for (int64_t id : selected_node_ids) {
            if (metadata->GetId() == id) {
              // Use kMultipleNodes to add the restored node to the selection.
              // This preserves the selection of any persistent nodes (e.g.
              // default lights) that were not destroyed. Using kSingleNode
              // would clear the selection of these persistent nodes.
              // The SelectionController lazily cleans up the invalid handles of
              // the destroyed nodes.
              SelectNode(metadata->GetNode(),
                         EditorInfo::SelectionMode::kMultipleNodes);
              break;
            }
          }
        });

    // Mark the metadata as not authored so that it doesn't get saved to disk
    GetView().ForEachNode([](NodeHandle node) {
      if (node->GetComponent<TemporaryEditorMetadata>()) {
        if (auto scene_metadata = node->GetComponent<SceneMetadata>()) {
          scene_metadata->SetComponentAuthored(
              TemporaryEditorMetadata::IsfInfo::kTypeUrlHash, false);
        }
      }
    });
    return status;
  });

  // If we are entering edit mode, then we must remove the backup NodeData.
  // Next time Play mode is entered, new backup NodeData will be created.
  if (in_edit_mode) {
    backup_node_data_.clear();
  }

  // Also, always unpause when entering edit mode.
  if (in_edit_mode) {
    SetPaused(false);
  }

  // If plugin camera mode is linked, playing the editor automatically switches
  // to the app camera.
  if (GetCameraConfiguration() ==
      EditorPlugin::CameraConfiguration::kEditorCameraOnly) {
    if (in_edit_mode) {
      SetInputMode(InputMode::kEditor);
    } else {
      SetInputMode(InputMode::kApp);
    }
  }

  // After we finish re-creating the scene, update the run mode to the target.
  result.Then([this, run_mode](absl::Status status) { run_mode_ = run_mode; })
      .KeptBy(&GetView());
}

bool EditorImpl::IsPaused() const { return is_paused_; }

void EditorImpl::SetPaused(bool is_paused) { is_paused_ = is_paused; }

void EditorImpl::StepNextFrame() { has_frames_to_step_ = true; }

bool EditorImpl::HasFramesToStep() { return has_frames_to_step_; }

void EditorImpl::SetViewportRect(std::optional<Rect> rect) {
  const float2 pixel_ratio = editor::GetPhysicalPixelRatio(GetView());

  // Return early if the pixel ratio and viewport rect has not changed.
  // The pixel ratio check is in case you drag the window to a display with a
  // different pixel density.
  if (imp::AlmostEqual(pixel_ratio, last_pixel_ratio_) &&
      viewport_widget_rect_ == rect) {
    return;
  }

  last_pixel_ratio_ = pixel_ratio;

  std::optional<filament::Viewport> viewport = std::nullopt;
  filament::RenderTarget* render_target = nullptr;

  if (rect) {
    render_target = viewport_widget_render_target_->GetRenderTarget();

    const float2 size = rect->half_extent * 2.0f * pixel_ratio;
    viewport = filament::Viewport{0, 0, static_cast<uint32_t>(size.x),
                                  static_cast<uint32_t>(size.y)};
  }

  GetView().GetHost()->SetEditorViewportOverride({}, viewport);
  GetView().GetHost()->SetEditorRenderTargetOverride({}, render_target);

  // Ensure the active camera's projection matches the viewport aspect ratio.
  if (viewport_widget_rect_ != rect) {
    viewport_widget_rect_ = rect;
    UpdateActiveCamera();
  }
}

std::optional<Rect> EditorImpl::GetViewportRect() const {
  return viewport_widget_rect_;
}

void EditorImpl::UpdateActiveCamera() {
  ComponentHandle<CameraComponent> camera = GetActiveCamera();
  if (!camera.IsValid()) return;
  camera->OnIsfStateChanged();
}

void EditorImpl::AddNode(NodeHandle node) {
  if (node->GetParent()) {
    IMP_LOG(imp::FATAL) << "Only top-level editor nodes should be added to the editor.";
  }
  node->SetParent(editor_root_node_);
}
void EditorImpl::RemoveNode(NodeHandle node) { node->SetParent(NodeHandle()); }

void EditorImpl::AddSandboxNode(NodeHandle node) {
  auto it = std::find(sandbox_nodes_.begin(), sandbox_nodes_.end(), node);
  if (it != sandbox_nodes_.end()) {
    return;
  }
  sandbox_nodes_.push_back(node);
}

void EditorImpl::RemoveSandboxNode(NodeHandle node) {
  auto it = std::find(sandbox_nodes_.begin(), sandbox_nodes_.end(), node);
  if (it != sandbox_nodes_.end()) {
    sandbox_nodes_.erase(it);
  }
}

Dispatcher& EditorImpl::GetDispatcher() { return dispatcher_; }

EditorPlugin::CameraConfiguration EditorImpl::GetCameraConfiguration() const {
  if (plugin_) {
    return plugin_->GetCameraConfiguration();
  }
  return EditorPlugin::CameraConfiguration::kEditorAndAppCameraDefault;
}

NodeHandle EditorImpl::GetEditorRoot() { return editor_root_node_; }

ComponentHandle<CameraComponent> EditorImpl::GetCamera() { return camera_; }

ComponentHandle<CameraComponent> EditorImpl::GetActiveCamera() {
  if (current_state_.camera_mode == CameraMode::kEditor) {
    return camera_;
  } else {
    return GetView().GetCameraManager().GetCamera();
  }
}

WidgetUiSystem& EditorImpl::GetWidgetUiSystem() { return widget_ui_system_; }

AssetLibrary* EditorImpl::GetAssetLibrary() { return asset_library_; }

EventInjector& EditorImpl::GetEventInjector() { return *event_injector_; }

void EditorImpl::SelectNode(NodeHandle node,
                            EditorInfo::SelectionMode selection_mode) {
  GetView().GetRegistry().Get<SelectionController>()->get().TrySelectNode(
      node, selection_mode);
}

const absl::flat_hash_set<NodeHandle>& EditorImpl::GetSelectedNodes() {
  return GetView()
      .GetRegistry()
      .Get<SelectionController>()
      ->get()
      .GetSelectedNodes();
}

NodeHandle EditorImpl::GetSingleSelectedNode() {
  const absl::flat_hash_set<NodeHandle>& selected_nodes = GetSelectedNodes();
  if (selected_nodes.size() != 1) {
    return NodeHandle();
  }
  return *selected_nodes.begin();
}

GltfAsset::LoadOptions EditorImpl::GetGltfLoadOptions() const {
  return gltf_load_options_;
}

void EditorImpl::SetUseLegacyCameraControls(bool use_legacy) {
  use_legacy_camera_controls_ = use_legacy;
}

bool EditorImpl::UseLegacyCameraControls() const {
  return use_legacy_camera_controls_;
}

Editor& GetOrCreateEditor(BaseView* view, std::unique_ptr<EditorPlugin> plugin,
                          bool is_sandbox) {
  if (plugin && view->GetRegistry().Get<editor::Editor>().ok()) {
    IMP_LOG(imp::FATAL)
        << "An EditorPlugin may not be injected if an Editor already exists in "
           "the Registry.";
  }
  editor::Editor& editor = view->GetRegistry().GetOrRegister<editor::Editor>(
      [view, plugin = std::move(plugin), is_sandbox]() mutable {
        return std::make_unique<editor::EditorImpl>(view, std::move(plugin),
                                                    is_sandbox);
      });

  editor.Initialize();  // Initialize the Editor if not already initialized.

  if (is_sandbox) {
    editor.SetEnabled(true);
    editor.SwitchToEditorMode();
  }

  return editor;
}

EditorImpl::Info::Info(EditorImpl& editor) : editor_(editor) {}

bool EditorImpl::Info::IsEnabled() const { return editor_.enabled_; }

EditorInfo::RunMode EditorImpl::Info::GetRunMode() const {
  return editor_.GetRunMode();
}

bool EditorImpl::Info::IsPaused() const { return editor_.IsPaused(); }

bool EditorImpl::Info::HasFramesToStep() const {
  return editor_.HasFramesToStep();
}

EditorInfo::DisplayMode EditorImpl::Info::GetDisplayMode() const {
  return editor_.GetDisplayMode();
}

}  // namespace imp::editor
