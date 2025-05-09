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

#include "core/view/ar/ar_scene_controller.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "core/ar/ar_trackable.h"
#include "core/ar/ar_trackable_handle.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/plane.h"
#include "core/common/trace.h"
#include "core/common/tuple_helpers.h"
#include "core/monitor/monitor_helpers.h"
#include "core/monitor/scoped_duration_measurement.h"
#include "core/view/ar/collision/ar_collision_system.h"
#include "core/view/ar/collision/collider_ar_plane.h"
#include "core/view/ar/collision/collider_ar_point.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/view_events.h"
#include "core/window/window_rotation.h"

namespace imp {

namespace {
// Holds the type of a thing without instantiating it.
template <typename T>
struct ComponentType {
  using type = T;
};
// Creates a tuple of types.
template <typename... Components>
std::tuple<ComponentType<Components>...> MakeTypeTupleHelper() {
  return std::tuple<ComponentType<Components>...>();
}
}  // namespace

ArSceneController::ArSceneController(BaseView* view, std::string dataset_path,
                                     bool enable_ar_plane_collision)
    : System(view),
      previous_tracking_state_(ar::TrackingState::kStopped),
      should_display_geometry_change_(true),
      running_(false),
      skip_duplicate_frames_(true),
      is_interrupted_(false),
      enable_ar_plane_collision_(enable_ar_plane_collision) {}

ArSceneController::~ArSceneController() {
  // Cancel the session-creation future to avoid a crash where the future lambda
  // hangs onto an unsafe reference to this.
  ar_session_future_.Cancel();

  if (enable_ar_plane_collision_) {
    GetComponentManager().ForEach<ColliderArPlane>(
        [this](ColliderArPlane* plane) {
          GetView().DestroyNode(plane->GetNode());
        });
  }
  GetComponentManager().ForEach<ColliderArPoint>(
      [this](ColliderArPoint* point) {
        GetView().DestroyNode(point->GetNode());
      });
  GetComponentManager().ForEach<ColliderArMagicalSurfacePoint>(
      [this](ColliderArMagicalSurfacePoint* point) {
        GetView().DestroyNode(point->GetNode());
      });
  GetComponentManager().ForEach<ArAnchorComponent>(
      [this](ArAnchorComponent* anchor) {
        GetView().DestroyNode(anchor->GetNode());
      });

  auto& collision_manager = GetView().GetCollisionManager();
  if (enable_ar_plane_collision_) {
    collision_manager
        .RemoveCollisionSystem<ColliderArPlane, ArCollisionSystem>();
  }
  collision_manager.RemoveCollisionSystem<ColliderArPoint, ArCollisionSystem>()
      .RemoveCollisionSystem<ColliderArMagicalSurfacePoint,
                             ArCollisionSystem>();
  if (!local_trackable_map_.empty()) {
    auto& view = GetView();
    for (auto it : local_trackable_map_) {
      view.DestroyNode(it.second);
    }
  }
}

Future<absl::Status> ArSceneController::ArAvailability(
    const BaseView* view, const ar::ArSessionConfig& config) {
  return ar::ArSession::CanCreate(view, config);
}

imp::Future<std::weak_ptr<ar::ArSession>> ArSceneController::Setup(
    const ar::ArSessionConfig& config) {
  return Setup(GetView().GetCameraManager().GetCamera(), config);
}

imp::Future<std::weak_ptr<ar::ArSession>> ArSceneController::Setup(
    ComponentHandle<CameraComponent> camera,
    const ar::ArSessionConfig& config) {
  IMP_TRACE();
  ar_session_future_ =
      ar::ArSession::Create(&GetView(), config)
          .Then([this, camera](std::shared_ptr<ar::ArSession> session) mutable
                -> std::weak_ptr<ar::ArSession> {
            IMP_TRACE_BLOCK("Then");
            SetupInternal(camera, session);
            return session;
          });
  return ar_session_future_;
}

void ArSceneController::SetupInternal(ComponentHandle<CameraComponent> camera,
                                      std::shared_ptr<ar::ArSession> session) {
  IMP_TRACE();
  assert(camera);
  camera_ = camera;
  ar_session_ = session;
#if IMP_RUNTIME(DEV)
  session->EnableDebugVisualization(&GetView());
#endif
  // Enable AR plane/point collision.
  constexpr float kGuessedDistance = 2.0f;

  //  Request a Histogram with lower bounds of 0 to 32 ms for ar frame
  //  calculation time.
  ScopedDurationMeasurement::AddHistogram(
      *GetView().GetMonitor(), imp::kArSessionUpdate, absl::ZeroDuration(),
      absl::Milliseconds(2), 16);

  auto& collision_manager = GetView().GetCollisionManager();

  if (enable_ar_plane_collision_) {
    collision_manager.AddCollisionSystem<ColliderArPlane, ArCollisionSystem>(
        &GetView(), session.get());
  }
  // Configures AR Point collision for instant placement.
  collision_manager
      .AddCollisionSystem<ColliderArPoint, ArCollisionSystem>(
          &GetView(), session.get(), ar::HitMode::kDistanceGuess,
          kGuessedDistance)
      .AddCollisionSystem<ColliderArMagicalSurfacePoint, ArCollisionSystem>(
          &GetView(), session.get(), ar::HitMode::kMagicalSurfacePoint);

  camera_renderer_node_ = GetView().CreateNode();
  pre_frame_update_connection_ = GetView().GetDispatcher().Connect(
      [this](const ViewPreFrameUpdateEvent& ev) {
        IMP_TRACE_BLOCK("PreFrameUpdate");
        if (!Update() && skip_duplicate_frames_) {
          ev.SkipFrame(absl::Milliseconds(2));
        }
      });
  post_frame_update_connection_ =
      GetView().GetDispatcher().Connect([this](const ViewPostRenderEvent& ev) {
        IMP_TRACE_BLOCK("PostRender");
        OnPostRender();
      });
  size_changed_connection_ =
      GetView().GetDispatcher().Connect([this](const ViewSizeChangedEvent& ev) {
        should_display_geometry_change_ = true;
      });
  rotation_changed_connection_ =
      GetView().GetDispatcher().Connect([this](const ViewSizeChangedEvent& ev) {
        should_display_geometry_change_ = true;
      });
  transition_parameters_changed_connection_ = GetView().GetDispatcher().Connect(
      [this](const ViewTransitionParametersChangedEvent& ev) {
        should_display_geometry_change_ = true;
        ar_session_->SetTransitionParameters(ev.size_scale,
                                             ev.counter_rotation);
      });

  running_ = true;
}

// Helper functor for updating a tuple of trackable types. Applies scene updates
// for each trackable type in a tuple.
struct UpdateForEachType {
  explicit UpdateForEachType(ArSceneController* ar_scene_controller,
                             bool in_enable_ar_plane_collision)
      : scene_controller(ar_scene_controller),
        enable_ar_plane_collision(in_enable_ar_plane_collision) {}

  template <typename T>
  void operator()(T&&) {
    using TrackableComponent = typename std::remove_reference<T>::type::type;
    constexpr bool is_anchor_type =
        std::is_same_v<TrackableComponent, ArAnchorComponent>;
    scene_controller->GetView()
        .GetComponentManager()
        .ForEach<TrackableComponent>(
            [this](TrackableComponent* trackable_component) {
              if (!is_anchor_type &&
                  (!trackable_component->GetTrackable().IsValid() ||
                   trackable_component->GetTrackingState() ==
                       ar::TrackingState::kStopped)) {
                // Destroys the node if the trackable has lost tracking or was
                // subsumed, but only if it's not an anchor component.
                scene_controller->GetView().DestroyNode(
                    trackable_component->GetNode());
              } else if (trackable_component->GetTrackingState() ==
                         ar::TrackingState::kTracking) {
                // Updates the trackable component.
                trackable_component->UpdateLocation();
              }
            });

    // Updates the collision system for the current type after the components
    // have been updated.
    constexpr bool is_plane_type =
        std::is_same_v<TrackableComponent, ColliderArPlane>;

    if constexpr (!is_anchor_type) {
      if (!is_plane_type || enable_ar_plane_collision) {
        // Gets the collision system for the current type.
        auto* collision_system =
            static_cast<ArCollisionSystem<TrackableComponent>*>(
                scene_controller->GetView()
                    .GetCollisionManager()
                    .GetCollisionSystem<TrackableComponent>());
        // Updates the collision system.
        collision_system->Update();
      }
    }
  }

  ArSceneController* scene_controller;
  bool enable_ar_plane_collision;
};

void ArSceneController::UpdateArTrackables() {
  if (!SessionReady()) {
    return;
  }

  UpdateForEachType updater(this, enable_ar_plane_collision_);
  if (enable_ar_plane_collision_) {
    auto types =
        MakeTypeTupleHelper<ColliderArPlane, ColliderArPoint,
                            ColliderArMagicalSurfacePoint, ArAnchorComponent>();
    ForEachTupleElement(types, updater);
  } else {
    auto types =
        MakeTypeTupleHelper<ColliderArPoint, ColliderArMagicalSurfacePoint,
                            ArAnchorComponent>();
    ForEachTupleElement(types, updater);

    // Update local planes
    auto it = local_trackable_map_.begin();
    while (it != local_trackable_map_.end()) {
      if (!it->first.IsValid()) {
        it = local_trackable_map_.erase(it);
        continue;
      }
      switch (it->first->GetTrackingState()) {
        case ar::TrackingState::kStopped: {
          it = local_trackable_map_.erase(it);
          break;
        }
        case ar::TrackingState::kTracking: {
          // Updates the trackable component.
          it->second->GetComponent<imp::ColliderArPlane>()->UpdateLocation();
          [[fallthrough]];
        }
        case ar::TrackingState::kPaused: {
          ++it;
          break;
        }
      }
    }
  }
}

void ArSceneController::SetIsInterrupted(bool next_interrupted) {
  is_interrupted_ = next_interrupted;
  ArInterruptionEvent event;
  event.is_now_interrupted = next_interrupted;
  GetView().GetDispatcher().Send(event);
}

bool ArSceneController::Update() {
  // Return true if the session is not ready or we aren't running.
  // This is so we only skip frames when Ar is actually running and we don't
  // have a new frame. Otherwise, we may skip frames simply when
  // ArSceneController exists but is paused / not setup.
  if (!SessionReady() || !running_) {
    return true;
  }

  bool next_interrupted = ar_session_->IsSessionInterrupted();
  if (next_interrupted != is_interrupted_) {
    SetIsInterrupted(next_interrupted);
  }
  if (is_interrupted_) {
    return true;
  }

  if (should_display_geometry_change_) {
    uint2 size = GetView().GetSize();
    ar_session_->SetDisplayGeometry(GetView().GetDisplayRotation(), size.x,
                                    size.y, camera_->GetNearClip(),
                                    camera_->GetFarClip());
  }

  {
    ScopedDurationMeasurement ar_session_update(GetView().GetMonitor(),
                                                kArSessionUpdate);
    if (!ar_session_->Update()) {
      ar_session_update.CancelSample();
      return false;
    }
  }

  UpdateArTrackables();
  UpdateCameraStream();

  const absl::optional<mat4f>& camera_node_matrix =
      ar_session_->camera_node_matrix();
  if (camera_node_matrix.has_value()) {
    camera_->SetCameraTransformOverride(ar_session_->model_matrix());
    camera_->GetNode()->SetLocalTrs(*camera_node_matrix);
  } else {
    camera_->GetNode()->SetLocalTrs(ar_session_->model_matrix());
  }

  // TODO: Just use GetDisplayGeometryChanged here. It is not
  // implemented in iOS, so we still need to check
  // should_display_geometry_change_ as well.
  if (should_display_geometry_change_ ||
      ar_session_->GetDisplayGeometryChanged()) {
    camera_->SetProjectionMatrix(ar_session_->projection_matrix());
  }

  should_display_geometry_change_ = false;

  // Send an event if the tracking state has changed to tracking.
  ar::TrackingState tracking_state = ar_session_->GetCameraTrackingState();
  if (previous_tracking_state_ != tracking_state &&
      tracking_state == ar::TrackingState::kTracking) {
    GetView().GetDispatcher().Send(TrackingAcquiredEvent());
  }
  previous_tracking_state_ = tracking_state;

  return true;
}

void ArSceneController::RenderDev() {
  if (!SessionReady()) {
    return;
  }
  ar_session_->RenderDev();
}

void ArSceneController::OnPostRender() {
  if (!SessionReady()) {
    return;
  }
  ar_session_->OnPostRender();
}

void ArSceneController::Pause() {
  if (!SessionReady()) {
    return;
  }

  if (is_interrupted_) {
    // Terminate our notion of interruption, since we're pausing the session.
    SetIsInterrupted(false);
  }

  ar_session_->Pause();
  running_ = false;
}

void ArSceneController::Resume() {
  if (!SessionReady()) {
    return;
  }

  ar_session_->Resume();
  running_ = true;
}

void ArSceneController::SetVisible(bool visible) {
  camera_renderer_node_->SetEnabled(visible);
}

std::string ArSceneController::GetArSessionId() {
  if (!SessionReady()) return "";

  return ar_session_->GetDebugSessionId();
}

void ArSceneController::UpdateCameraStream() {
  if (!SessionReady()) {
    return;
  }
#if !IMP_PLATFORM(DESKTOP)
  // TODO: clean up this delay until the texture is ready.
  if (!ar_session_->camera_texture()) {
    // Don't do anything until the camera texture is ready.
    return;
  }

  if (!camera_renderer_future_) {
    camera_renderer_future_ =
        camera_renderer_node_->AddComponent<imp::ArCameraRenderer>(
            ar_session_.get());
  }

  // TODO: Replace use of should_display_geometry_change_ by
  // adding a ArSession::HasDisplayGeometryChanged method. In ArCore, that
  // should call through to this method:
  // (broken link)
  if (should_display_geometry_change_ && camera_renderer_future_ &&
      camera_renderer_future_->Ready()) {
    absl::StatusOr<ComponentHandle<ArCameraRenderer>> camera_renderer =
        camera_renderer_future_->Get();
    
    (*camera_renderer)->RebuildMesh();
  }

#endif
}

std::weak_ptr<ar::ArSession> ArSceneController::GetSession() const {
  if (!SessionReady()) {
    IMP_LOG(imp::FATAL) << "AR Session not ready.";
  }
  return ar_session_;
}

bool ArSceneController::SessionReady() const {
  return ar_session_future_.Ready() && ar_session_ != nullptr;
}

void ArSceneController::SetPlacementMode(
    imp::ar::ArSessionConfig::PlacementMode mode) {
  if (!SessionReady()) {
    IMP_LOG(imp::ERROR)
        << "AR Session not ready for updating instant mode and magical surface "
           "mode.";
    return;
  }
  ar_session_->SetPlacementMode(mode);
}

NodeHandle ArSceneController::GetNodeFromTrackable(ar::ArTrackableId id) {
  auto trackable_handle = ar_session_->GetTrackableHandle<ar::ArPlane>(id);

  if (!trackable_handle ||
      trackable_handle->GetTrackingState() == ar::TrackingState::kStopped)
    return {};

  auto it = local_trackable_map_.find(trackable_handle);
  imp::NodeHandle hit_node;

  if (it == local_trackable_map_.end()) {
    hit_node = GetView().CreateNode();
    hit_node->SetLocalTrs(trackable_handle->GetTransform());

    hit_node->AddComponent<imp::ColliderArPlane>(trackable_handle);
    local_trackable_map_.insert(std::make_pair(trackable_handle, hit_node));
  } else {
    hit_node = it.value();
  }
  return hit_node;
}

std::vector<RayHit> ArSceneController::ArPlaneRaycast(const Ray& world_ray,
                                                      bool only_closest) {
  ar::ArTrackableId best_id;
  float best_distance = std::numeric_limits<float>::max();
  const ar::ArPlane* best_plane = nullptr;
  float3 best_intersection_point;
  std::vector<RayHit> hits;
  ar_session_->VisitTrackables<imp::ar::ArPlane>(
      [&best_id, &best_distance, &world_ray, &best_plane,
       &best_intersection_point, only_closest, &hits,
       this](const ar::ArTrackableId& id, const ar::ArPlane& ar_plane) {
        if (ar_plane.GetTrackingState() != ar::TrackingState::kTracking) return;
        const mat4f& plane_transform = ar_plane.GetTransform();
        const float3 out = normalize((plane_transform * float4(kUp, 0.f)).xyz);
        const float3 pos = (plane_transform * float4(kZero3, 1.f)).xyz;
        const auto plane = Plane(out, dot(out, pos));
        float3 intersection_point = kZero3;
        if (collision::PlaneIntersectsRay(plane, world_ray,
                                          &intersection_point) ==
            collision::Result::kDoesNotIntersect)
          return;

        if (!ar_plane.IsPointInPlanePolygon(intersection_point)) return;

        float plane_distance = length(intersection_point - world_ray.origin);
        if (only_closest) {
          if (plane_distance >= best_distance) return;

          best_id = id;
          best_plane = &ar_plane;
          best_distance = plane_distance;
          best_intersection_point = intersection_point;
        } else {
          hits.push_back(RayHit(plane_distance, plane_transform.toQuaternion(),
                                intersection_point, GetNodeFromTrackable(id)));
        }
      });

  if (only_closest) {
    if (!best_id.IsValid()) return {};
    hits.push_back(
        RayHit(best_distance, best_plane->GetTransform().toQuaternion(),
               best_intersection_point, GetNodeFromTrackable(best_id)));
  } else {
    std::sort(hits.begin(), hits.end(),
              [](RayHit& a, RayHit& b) { return a.distance < b.distance; });
  }
  return hits;
}

bool ArSceneController::IsReadyToRender() const {
  if (!SessionReady()) {
    return false;
  }
  return ar_session_->IsReadyToRender();
}

}  // namespace imp
