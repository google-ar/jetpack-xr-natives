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

#include "core/view/framework/view.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/TransformManager.h"
#include "filament/libs/utils/include/utils/EntityManager.h"
#include "core/async/executor.h"
#include "core/async/future_common.h"
#include "core/common/invocable.h"
#include "core/common/trace.h"
#include "core/math/vec.h"
#include "core/monitor/duration_measurement.h"
#include "core/monitor/frame_loop_watcher.h"
#include "core/monitor/monitor_helpers.h"
#include "core/monitor/scoped_duration_measurement.h"
#include "core/ncsb/component_manager.h"
#include "core/ncsb/node_attachment_manager.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/path_manager.h"
#include "core/ncsb/update_phase.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/client_api.h"
#include "core/view/framework/display_layer/display_layer_manager.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/framework/render/renderable_manager_wrapper.h"
#include "core/view/utils/default_view_config.h"
#include "core/view/utils/frame_time.h"
#include "core/view/utils/proto/view_config.proto.imp.h"
#include "core/view/utils/render_setting_utils.h"
#include "core/view/view_events.h"
#include "core/window/filament_host.h"
#include "core/window/window_rotation.h"

namespace imp {

namespace {

// In the app-editor case, the app camera needs an initial position.
constexpr float3 kAppCameraPositionInAppEditorMode = {0, 0, 3};
constexpr absl::Duration kAssetManagerCacheCleanupInterval = absl::Seconds(4);
using UpdateStageFlags = window::FilamentHost::UpdateStageFlags;

}  // namespace

View::View(ViewConfig config)
    : title_("View"),
      context_(),
      split_engine_serializer_(nullptr),
      view_config_(std::move(config)),
      node_attachment_manager_(this),
      dispatcher_(),
      update_system_(this),
      texture_registry_(this),
      component_manager_(this),
      scene_system_(this),
      shader_cache_system_(this),
      display_layer_manager_(*this),
      asset_manager_(nullptr),
      camera_manager_(this),
      light_manager_(*this),
      gesture_manager_(&dispatcher_),
      input_manager_(std::make_unique<PointerInputHandler>(this)),
      collision_manager_(this),
      path_manager_(this),
      mesh_factory_(*this),
      material_factory_(this),
      environment_light_factory_(this),
      frame_time_(absl::Now()),
      device_(),
      size_(),
      window_rotation_(window::WindowRotation::kRotation0),
      asset_manager_cache_cleanup_interval_(kAssetManagerCacheCleanupInterval) {
  if (view_config_.cache_config.has_value() &&
      view_config_.cache_config->cache_cleanup_interval_seconds.has_value()) {
    asset_manager_cache_cleanup_interval_ = absl::Seconds(
        view_config_.cache_config->cache_cleanup_interval_seconds.value());
  }
}

View::~View() {
  // Clear out the SplitEngineSerializer before destroying the other members to
  // avoid any serialization as the view is being destroyed. All app contents
  // should be cleared when the split engine bridge is destroyed.
  split_engine_serializer_ = nullptr;
}

std::unique_ptr<View> View::CreateClient(std::unique_ptr<Context> context) {
  // TODO: Look at removing the methods View::Create and
  // View::CreateClient.
  return CreateClient(std::move(context), "default");
}

std::unique_ptr<View> View::CreateClient(std::unique_ptr<Context> context,
                                         const std::string& identifier) {
  IMP_TRACE();
  // TODO: Look at removing the methods View::Create and
  // View::CreateClient.
  // Pass the context through the ViewHost directly instead of doing it through
  // this method.
  std::unique_ptr<View> result = imp::client_api::CreateView(identifier);
  result->context_ = std::move(context);
  return result;
}

// This method needs to duplicate the logic of CreateClient above as the View
// constructor may already have a custom ViewConfig passed in via CreateView.
// TODO: Having two ways of setting ViewConfig isn't great and
// creates confusion, so we should figure out a way to set the ViewConfig
// that works for everyone.
std::unique_ptr<View> View::CreateClient(std::unique_ptr<Context> context,
                                         const std::string& identifier,
                                         const ViewConfig& config) {
  IMP_TRACE();
  // TODO: Look at removing the methods View::Create and
  // View::CreateClient.
  // Pass the context through the ViewHost directly instead of doing it through
  // this method.
  std::unique_ptr<View> result = imp::client_api::CreateView(identifier);
  result->context_ = std::move(context);
  result->view_config_ = config;
  return result;
}

NodeHandle View::CreateNode() {
  utils::Entity entity = utils::EntityManager::get().create();
  return AttachEntityToView(entity);
}

void View::DestroyNode(NodeHandle node) {
  if (!node) {
    return;
  }

  // First, get this node and all of its children sorted depth-first.
  std::vector<NodeHandle> nodes_to_destroy;
  nodes_to_destroy = GetPathManager().GetDescendants(node);
  nodes_to_destroy.push_back(node);

  // Then, remove every component from every node being destroyed.
  // All components of the same type are removed in depth first order.
  //
  // The order of types can be defined using CleanupDependencies and
  // CleanupDependents.
  GetComponentManager().RemoveAllFromNodes(nodes_to_destroy);

  // Tell the serializer about the nodes being destroyed.
  // This is done before the nodes are actually destroyed to ensure that the
  // serializer can access the node's children to determine dependencies
  // correctly.
  if (split_engine_serializer_) {
    for (NodeHandle node_to_destroy : nodes_to_destroy) {
      split_engine_serializer_->DestroyNode(node_to_destroy.GetEntity());
    }
  }

  for (NodeHandle node_to_destroy : nodes_to_destroy) {
    node_attachment_manager_.Destroy(node_to_destroy);
  }
}

void View::ForEachNode(std::function<void(NodeHandle)>&& fn) {
  node_attachment_manager_.ForEach(std::move(fn));
}

void View::ForEachNode(std::function<void(NodeHandle)>&& fn, NodeFlag filter) {
  node_attachment_manager_.ForEach(std::move(fn), filter);
}

std::size_t View::GetNodeCount() const {
  return node_attachment_manager_.GetCount();
}

NodeHandle View::AttachEntityToView(utils::Entity entity) {
  return node_attachment_manager_.Attach(entity);
}

ComponentManager& View::GetComponentManager() noexcept {
  return component_manager_;
}

UpdateSystem& View::GetUpdateSystem() noexcept { return update_system_; }

SceneSystem& View::GetSceneSystem() noexcept { return scene_system_; }

DisplayLayerManager& View::GetDisplayLayerManager() noexcept {
  return display_layer_manager_;
}

CollisionManager& View::GetCollisionManager() noexcept {
  return collision_manager_;
}

PathManager& View::GetPathManager() noexcept { return path_manager_; }

AssetManager& View::GetAssetManager() noexcept { return *asset_manager_; }

CameraManager& View::GetCameraManager() noexcept { return camera_manager_; }

LightManager& View::GetLightManager() noexcept { return light_manager_; }

GestureManager& View::GetGestureManager() noexcept { return gesture_manager_; }

InputManager& View::GetInputManager() noexcept { return input_manager_; }

Dispatcher& View::GetDispatcher() noexcept { return dispatcher_; }

TextureFactory& View::GetTextureFactory() noexcept { return *texture_factory_; }

TextureRegistry& View::GetTextureRegistry() noexcept {
  return texture_registry_;
}

MeshFactory& View::GetMeshFactory() noexcept { return mesh_factory_; }

MaterialFactory& View::GetMaterialFactory() noexcept {
  return material_factory_;
}

EnvironmentLightFactory& View::GetEnvironmentLightFactory() noexcept {
  return environment_light_factory_;
}

GroupsManager& View::GetGroupsManager() noexcept { return *groups_manager_; }

Device& View::GetDevice() noexcept { return device_; }

Registry& View::GetRegistry() noexcept { return registry_; }
const Registry& View::GetRegistry() const noexcept { return registry_; }

uint2 View::GetSize() const { return size_; }

uint4 View::GetMargins() const { return margins_; }

window::WindowRotation View::GetDisplayRotation() const {
  return window_rotation_;
}

void View::UpdateTransitionParameters(float2 transition_scale_adjustment,
                                      float transition_counter_rotation) {
  GetHost()->EnsureNextRenderCompletes();
  ViewTransitionParametersChangedEvent event;
  event.size_scale = transition_scale_adjustment;
  event.counter_rotation = transition_counter_rotation;
  GetDispatcher().Send(event);
}

filament::View* View::CreateFilamentView() {
  filament::View* view = GetSharedEngine()->createView();
  filament_views_.push_back(view);
  GetDispatcher().Send(FilamentViewCreatedEvent(view));
  return view;
}

void View::DestroyFilamentView(filament::View* view) {
  filament_views_.erase(
      std::remove(filament_views_.begin(), filament_views_.end(), view),
      filament_views_.end());
  GetSharedEngine()->destroy(view);
}

absl::Span<filament::View*> View::GetFilamentViews() {
  return absl::MakeSpan(filament_views_);
}

void View::Advance(absl::Duration delta_time) {
  IMP_TRACE_NAME("View::Advance");

  DurationMeasurement delta_time_measurement(GetMonitor(), kViewFrameTime);
  delta_time_measurement.AddSample(delta_time);

  ScopedDurationMeasurement advance_duration_measurement(GetMonitor(),
                                                         kViewAdvance);

  frame_time_.Update(delta_time);

  time_since_last_asset_manager_cache_cleanup_ += delta_time;
  if (time_since_last_asset_manager_cache_cleanup_ >
      asset_manager_cache_cleanup_interval_) {
    GetAssetManager().ClearUnused();
    time_since_last_asset_manager_cache_cleanup_ = absl::ZeroDuration();
  }

  update_system_.Update(UpdatePhase::kStart, frame_time_);

  if (!imp::client_api::GetIsAppSandboxTarget()) {
    IMP_TRACE_BLOCK("View::Update");
    Update(frame_time_);
  }

  // Note: This no-ops when dev mode is not available.
  {
    IMP_TRACE_NAME("View::QueueImGuiCommandBlock");
    GetHost()->QueueImGuiCommandBlock(
        [this]() { dispatcher_.Send(ImGuiPreRenderEvent()); });
  }

  // If the background executor must be manually advanced, do it now.
  // Note: By default Impress uses a ThreadPoolExecutor which doesn't need to be
  // advanced. However, when WASM is run with pthreads disabled, then Impress is
  // single-threaded and the background executors is explicitly advanced here.
  AdvanceBackgroundExecutor();

  AdvanceForegroundExecutor();

  input_manager_.Update();
  update_system_.Update(UpdatePhase::kPreDefault, frame_time_);
  update_system_.Update(UpdatePhase::kDefault, frame_time_);
  update_system_.Update(UpdatePhase::kPostDefault, frame_time_);

  update_system_.Update(UpdatePhase::kEnd, frame_time_);
}

void View::AdvanceBackgroundExecutor() {
  imp::Executor* ex = Executor::BackgroundExecutor();

  // Note: By default Impress uses a ThreadPoolExecutor which doesn't need to be
  // pumped. However, when WASM is run with pthreads disabled, then Impress is
  // single-threaded and the background executor is explicitly advanced here.
  if (!ex || !ex->IsPumpingRequired()) {
    return;
  }

  IMP_TRACE();

  auto start = absl::Now();
  float background_executor_timeout_ms =
      view_config_.background_executor_timeout_ms.value_or(
          kBackgroundExecutorTimeoutMs);
  size_t tasks_run = ex->DrainWithTimeout(absl::Microseconds(
      static_cast<int>(background_executor_timeout_ms * 1000)));

  if (tasks_run == 0) {
    return;
  }

  auto actual_time = absl::Now() - start;
  DurationMeasurement background_executor_duration(GetMonitor(),
                                                   kBackgroundExecutorTiming);
  background_executor_duration.AddSample(actual_time);
}

void View::AdvanceForegroundExecutor() {
  IMP_TRACE();

  imp::Executor* ex = Executor::ForegroundExecutor();
  assert(ex);
  auto start = absl::Now();
  float foreground_executor_timeout_ms =
      view_config_.foreground_executor_timeout_ms.value_or(
          kForegroundExecutorTimeoutMs);
  size_t tasks_run = ex->DrainWithTimeout(absl::Microseconds(
      static_cast<int>(foreground_executor_timeout_ms * 1000)));

  if (tasks_run == 0) {
    return;
  }

  auto actual_time = absl::Now() - start;
  DurationMeasurement foreground_executor_duration(GetMonitor(),
                                                   kForegroundExecutorTiming);
  foreground_executor_duration.AddSample(actual_time);
}

void View::OnHostCreated(window::FilamentHost* host) {
  if (!context_) {
    context_ = std::make_unique<Context>();
  }

  host_ = host;

  // Must be created after the host is assigned.
  asset_manager_ =
      std::make_unique<AssetManager>(this, view_config_.cache_config);

  //  Request a Histogram with lower bounds of 6 to 68 ms for the total time
  //  between frames.
  DurationMeasurement::AddHistogram(*host_->GetMonitor(), imp::kViewFrameTime,
                                    absl::Milliseconds(6),
                                    absl::Milliseconds(2), 31);

  //  Request a small Histogram of the time spent in View::Advance
  ScopedDurationMeasurement::AddHistogram(
      *host_->GetMonitor(), imp::kViewAdvance, absl::Milliseconds(4),
      absl::Milliseconds(8), 4);

  // Request a Histogram of the time spent in the foreground executor bounded
  // from 0ms to 24ms.
  DurationMeasurement::AddHistogram(
      *host_->GetMonitor(), kForegroundExecutorTiming, absl::ZeroDuration(),
      absl::Milliseconds(4), 6);
}

void View::OnHostSetup(uint2 dimensions,
                       Invocable<void(bool)> additional_setup_function) {
  IMP_TRACE();
  size_ = dimensions;
  margins_ = {0, 0, 0, 0};

  ApplyViewConfig();

  GetDispatcher().Connect(
      [this](const PointerHitEvent& event) {
        this->GetGestureManager().OnPointerHitEvent(event);
        this->OnPointerHitEvent(event);
      },
      this);

  groups_manager_ =
      std::make_unique<GroupsManager>(this, GetHost()->GetScene());
  if (!renderable_manager_) {
    renderable_manager_ = std::make_unique<RenderableManagerWrapper>(*this);
  }
  texture_factory_ = std::make_unique<TextureFactory>(*this);
  scene_system_.RegisterDefaultComponentsIsfInfo();
  camera_manager_.InitializeDefaultCamera();
  light_manager_.Setup();
  shader_cache_system_.Setup();

  // Provides the kFramePresented measurement in the monitor.
  registry_.GetOrCreate<FrameLoopWatcher>(*this);

  if (additional_setup_function) {
    // Impress Editor creation and registry needs to happens here, since the
    // Impress Editor depends on the preceding set up code. An Invocable is used
    // instead of directly calling Editor code in order to circumvent circular
    // dependencies. See ViewHost::ViewState for how this Invocable is used.
    additional_setup_function(imp::client_api::GetIsAppSandboxTarget());
  }

  RegisterComponents();

  if (!imp::client_api::GetIsAppSandboxTarget()) {
    IMP_TRACE_BLOCK("Setup");
    Setup();
  } else {
    SetupSandbox();
  }
}

void View::OnHostCleanup() {
  GetDispatcher().Send(ViewCleanupEvent());
  Cleanup();

  light_manager_.Cleanup();

  // Removes all components from all nodes in cleanup order.
  component_manager_.DetachAll();

  // Destroys all nodes.
  node_attachment_manager_.Cleanup();

  ClearRemembered();

  GetRegistry().Clear();

  // Destroys all component pools.
  //
  // This is done after the registry is destroyed to ensure that
  // ComponentHandle::IsValid will correctly return false without crashing if
  // done from within a Registry objects destructor. At that point, the
  // components will have all been removed, but it should still be possible to
  // check if the ComponentHandle is valid.
  component_manager_.DestroyPools();

  asset_manager_->Cleanup();
}

void View::Resume() { GetDispatcher().Send(ViewResumedEvent()); }

void View::Pause() { GetDispatcher().Send(ViewPausedEvent()); }

void View::OnHostSetDisplayRotation(window::WindowRotation rotation) {
  window_rotation_ = rotation;
  ViewRotationChangedEvent event;
  event.rotation = rotation;
  GetDispatcher().Send(event);
  OnDisplayRotationChanged(rotation);
}

void View::OnHostResize(uint2 dimensions, uint4 margins,
                        float2 subpixel_ratio) {
  size_ = dimensions;
  margins_ = margins;
  device_.SetPhysicalPixelRatio(subpixel_ratio);
  ViewSizeChangedEvent event;
  event.size = dimensions;
  event.margins = margins;
  GetDispatcher().Send(event);
  OnResized(dimensions, margins);
}

void View::OnHostPreUpdate(
    window::FilamentHost* host, absl::Duration last_vsync,
    absl::Duration next_vsync, UpdateStageFlags* out_flags,
    absl::optional<absl::Duration>* out_time_until_retry) {
  IMP_TRACE();

  if (split_engine_serializer_ &&
      !split_engine_serializer_->ReadyForNextFrame()) {
    // App is only allowed to create a limited amount of split engine buffers to
    // avoid endless memory overrun. After that, it must re-use a buffer once
    // the system indicates it has become available. If we can't re-use or
    // create a buffer, then it implies the system is too far behind and we must
    // skip the frame. and wait until the system catches up.
    out_flags->SetFlag(UpdateStageFlags::kSkipFrame);
    IMP_LOG(imp::ERROR) << "Skipping a frame: The SplitEngineSerializer is not ready.";
    return;
  }

  // Send the pre frame update event, which gives the receiver of the event
  // an opportunity to indicate this frame should be skipped.
  ViewPreFrameUpdateEvent view_pre_frame_update_event(
      [this, out_flags,
       out_time_until_retry](absl::optional<absl::Duration> time_until_retry) {
        if (!GetHost()->IsNextRenderRequired()) {
          IMP_TRACE_BLOCK("Frame-Cancel");
          out_flags->SetFlag(UpdateStageFlags::kSkipFrame);
          if (out_time_until_retry) {
            *out_time_until_retry = time_until_retry;
          }
        }
      });
  GetDispatcher().Send(view_pre_frame_update_event);
}

void View::OnHostUpdate(window::FilamentHost* host, absl::Duration last_vsync,
                        absl::Duration next_vsync,
                        const UpdateStageFlags& update_flags) {
  absl::Duration delta_time = next_vsync - last_vsync;

  // Set the out flag which indicates to filament host that rendering should
  // be skipped. Then, return early.
  // TODO Update this to use kSkipAdvance instead
  if (update_flags.HasFlag(UpdateStageFlags::kSkipFrame)) {
    GetDispatcher().Send(ViewSkippedFrameEvent());
    frame_time_.Accumulate(delta_time);
    return;
  }

  // Don't bother advancing the view if no time has elapsed.
  // We still don't skip rendering the frame in this case, because this is used
  // to force a render for scuba tests.
  //
  // However, if we've accumulated time without advancing from a previous
  // attempt to render that ended up being skipped, then we don't skip this
  // frame so that the accumulated time can be applied.
  if (delta_time == absl::ZeroDuration() && !frame_time_.HasAccumulatedTime()) {
    return;
  }

  // We aren't skipping the frame, advance the view.
  // TODO Rename this to DoUpdate or something closer to "Update"
  Advance(delta_time);
}

// TODO Implement this
void View::OnHostPostUpdate() {
  GetDispatcher().Send(ViewPostFrameUpdateEvent());
}

// TODO Implement this
void View::OnHostPreRender(window::FilamentHost* host) {}

void View::OnHostMultiPassRender() {
  this->GetDisplayLayerManager().RenderLayers();
}

void View::OnHostPostRender() {
  GetDispatcher().Send(ViewPostRenderEvent());
  OnPostRender();
}

void View::OnHostSecondaryViewRender() {
  GetDispatcher().Send(ViewSecondaryRenderEvent());
}

void View::OnHostPostFrame() {
  GetDispatcher().Send(ViewPostFrameEvent());
  OnPostFrame();
}

void View::OnHostOffscreenRender(filament::Renderer* renderer) {
  GetDispatcher().Send(ViewPreRenderEvent(renderer));
  OnOffscreenRender(renderer);
}

void View::SetupSandbox() {
  // Set the camera to point at the origin in the editor case, as the normal
  // app Setup() function won't run and thus won't position the camera.
  GetCameraManager().GetCamera()->GetNode()->SetWorldPosition(
      kAppCameraPositionInAppEditorMode);
}

void View::ApplyViewConfig() {
  if (view_config_.main_view_render_settings.has_value()) {
    OverrideViewRenderSettings(GetHost()->GetView(),
                               &(*view_config_.main_view_render_settings),
                               GetSharedEngine());
  }

  switch (view_config_.default_lighting_loading) {
    case ViewConfig::DefaultLightingLoading::DEFAULT_LIGHTING_LOADING_DISABLED:
      light_manager_.DisableDefaultLoad();
      break;
    case ViewConfig::DefaultLightingLoading::DEFAULT_LIGHTING_LOADING_ENABLED:
    case ViewConfig::DefaultLightingLoading::
        DEFAULT_LIGHTING_LOADING_UNSPECIFIED:
      break;
  }

  switch (view_config_.shader_caching_mode) {
    case ViewConfig::ShaderCachingMode::SHADER_CACHING_MODE_DISABLED:
    case ViewConfig::ShaderCachingMode::SHADER_CACHING_MODE_UNSPECIFIED:
      shader_cache_system_.DisableShaderCaching();
      break;
    case ViewConfig::ShaderCachingMode::SHADER_CACHING_MODE_ENABLED:
      shader_cache_system_.EnableShaderCaching();
      break;
  }

  if (view_config_.enable_synchronous_future_cancellation.value_or(false)) {
    FutureFlags::EnableSynchronousFutureCancellation();
  }
}

bool View::AreSplitEngineMaterialsInLocalMode() const {
#if IMP_USE_LOCAL_SPLIT_ENGINE_MATERIALS
  return true;
#else
  return split_engine_serializer_ == nullptr;
#endif
}

}  // namespace imp
