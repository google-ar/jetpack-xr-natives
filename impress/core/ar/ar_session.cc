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

#include "core/ar/ar_session.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "core/ar/ar_session_native.h"
#include "core/ar/playback/ar_session_native_playback.h"
#include "core/async/executor.h"
#include "core/common/filament_helpers.h"
#include "core/common/trace.h"
#include "core/common/tuple_helpers.h"
#include "core/config.h"
#include "core/math/quat.h"

#if IMP_RUNTIME(DEV)
#include "core/collision/collision_helpers.h"
#include "core/common/debug_draw.h"
#include "imp.h"
#endif

namespace imp {
namespace ar {

namespace {

#if IMP_RUNTIME(DEV)
// A helper struct for managing debug drawing.
class DebugHelperImpl : public DebugHelper {
 public:
  DebugHelperImpl(BaseView* view) : view_(view) {}
  BaseView* GetView() { return view_; }

 private:
  BaseView* view_;
};

// Adds a debug representation of each plane in the Ar session.
void VisualizeArPlanes(DebugHelperImpl* debug_helper,
                       const std::vector<ArTrackableHandle<ArPlane>>& planes) {
  // Ray from the center of the camera.
  Ray world_ray = debug_helper->GetView()
                      ->GetCameraManager()
                      .GetCamera()
                      ->WorldRayFromClipPoint(kZero2);

  // Intersects the ray with the AR planes and stores the result. Stores the
  // ray_hit info along with the plane that was hit.
  using plane_hit = std::pair<ArTrackableHandle<ArPlane>, RayHit>;
  std::vector<plane_hit> hits;
  for (const ArTrackableHandle<ArPlane>& ar_plane : planes) {
    if (ar_plane->GetTrackingState() != TrackingState::kTracking) {
      continue;
    }
    float3 plane_normal =
        normalize((ar_plane->GetTransform() * float4(kUp, 0.0f)).xyz);
    float3 plane_point =
        (ar_plane->GetTransform() * float4(ar_plane->GetVertices()[0], 1.0f))
            .xyz;
    Plane plane(plane_normal, dot(plane_normal, plane_point));
    float3 collision_point;
    if (collision::PlaneIntersectsRay(plane, world_ray, &collision_point) ==
        collision::Result::kDoesIntersect) {
      // Stores the hit for sorting.
      hits.push_back(std::make_pair(
          ar_plane, RayHit(length2(world_ray.origin - collision_point),
                           ar_plane->GetTransform().toQuaternion(),
                           collision_point, NodeHandle())));
    }
  }
  // Sorts the hits, the plane nearest the camera will be first in the list.
  std::sort(hits.begin(), hits.end(),
            [](const plane_hit& a, const plane_hit& b) {
              return a.second.distance < b.second.distance;
            });

  // Finds the first plane where the collision point is in the polygon.
  ArTrackableHandle<ArPlane> looking_at_plane;
  for (const plane_hit& hit : hits) {
    if (hit.first->IsPointInPlanePolygon(hit.second.world_point)) {
      looking_at_plane = hit.first;
      // Draws the hit point, which is at the center of the cameras view.
      debug_draw::Global().BoxLines(
          Box{hit.second.world_point, {0.01f, 0.01f, 0.01f}},
          debug_draw::Color(-1));
      break;
    }
  }

  // Draws each plane in polygnal form, highlighting the top-most plane whos
  // polygon encompasses the cameras center of view.
  int point_color = 0;
  for (const ArTrackableHandle<ArPlane>& ar_plane : planes) {
    if (ar_plane->GetTrackingState() != TrackingState::kTracking) {
      continue;
    }
    auto plane_orientation = ar_plane->GetTransform();

    // Changes the color for each ar_plane.
    debug_draw::Color color = debug_draw::ColorFromIndex(point_color);
    if (looking_at_plane == ar_plane) {
      // The top most ar_plane being looked at turns white
      color = debug_draw::Color(-1);
    }

    // Draws the polygon.
    float3 p0 = ar_plane->GetVertices()[ar_plane->GetVertices().size() - 1];
    for (size_t i = 0; i < ar_plane->GetVertices().size(); ++i) {
      float3 p1 = ar_plane->GetVertices()[i];
      debug_draw::Global().Line((plane_orientation * p0).xyz,
                                (plane_orientation * p1).xyz, color);
      p0 = p1;
    }

    // Draws the polygon points as boxes.
    for (const auto& point : ar_plane->GetVertices()) {
      debug_draw::Global().BoxLines(
          Box{(plane_orientation * point).xyz, {0.01f, 0.01f, 0.01f}}, color);
    }
    point_color++;
  }
}
#endif

// Helper struct that performs the trackables map updates.
struct UpdateReferences {
  explicit UpdateReferences(TrackableMapTuple* trackable_map)
      : trackable_map(trackable_map) {}
  template <typename TrackableList>
  constexpr void operator()(TrackableList&& trackable_list) {
    // Gets the map that corresponds to the trackable lists value type.
    using TrackableType =
        typename std::remove_reference<TrackableList>::type::value_type;
    auto& map = std::get<TrackableMap<TrackableType>>(*trackable_map);

    // Iterates over each trackable of the 'TrackableType' and updates the
    // map reference.
    for (const auto& trackable : trackable_list) {
      if (trackable.GetTrackingState() == TrackingState::kStopped) {
        map.erase(trackable.GetId());
      } else {
        map[trackable.GetId()] = trackable;
      }
    }
  }

  TrackableMapTuple* trackable_map;
};
}  // namespace

Future<absl::Status> ArSession::CanCreate(const BaseView* view,
                                          const ArSessionConfig& config) {
  if (config.session_type == ar::ArSessionConfig::SessionType::kPlayback) {
    // The Playback session is always supported.
    return Future<absl::Status>(absl::OkStatus());
  }

  CanCreateArSessionFn& can_create_override_fn =
      GetCanCreateArSessionOverrideFn();
  if (can_create_override_fn) {
    return can_create_override_fn(view)
               ? Future<absl::Status>(absl::OkStatus())
               : Future<absl::Status>(absl::UnavailableError(
                     "Override function returned false"));
  }
  return ArSessionNative::CanCreate(view, config);
}

Future<std::unique_ptr<ArSession>> ArSession::Create(
    BaseView* view, const ArSessionConfig& config) {
  IMP_TRACE();
  return CanCreate(view, config)
      .Then([view, config]() -> Future<std::unique_ptr<ArSession>> {
        IMP_TRACE_BLOCK("Then");
        CreateArSessionFn& create_override_fn = GetCreateArSessionOverrideFn();
        if (create_override_fn) {
          return create_override_fn(view, config);
        }

        if (config.session_type ==
            ar::ArSessionConfig::SessionType::kPlayback) {
          return Future<std::unique_ptr<ArSession>>(absl::WrapUnique<ArSession>(
              new ArSession(std::make_unique<ArSessionNativePlayback>(view))));
        }

        return ArSessionNative::Create(view, config)
            .Then([](std::unique_ptr<ArSessionNative> ar_session_native) {
              IMP_TRACE_BLOCK("Then");
              return absl::WrapUnique<ArSession>(
                  new ArSession(std::move(ar_session_native)));
            });
      });
}

CanCreateArSessionFn& ArSession::GetCanCreateArSessionOverrideFn() {
  // It's best to store static data structures as a static pointer inside of
  // a function, per (broken link) and (broken link).
  static CanCreateArSessionFn* stored_can_create_fn =
      new CanCreateArSessionFn();
  return *stored_can_create_fn;
}

CreateArSessionFn& ArSession::GetCreateArSessionOverrideFn() {
  // It's best to store static data structures as a static pointer inside of
  // a function, per (broken link) and (broken link).
  static CreateArSessionFn* stored_create_fn = new CreateArSessionFn();
  return *stored_create_fn;
}

void ArSession::SetCreateArSessionOverride(
    CanCreateArSessionFn can_create_session_fn,
    CreateArSessionFn create_session_fn) {
  CanCreateArSessionFn& can_create_override_fn =
      GetCanCreateArSessionOverrideFn();
  can_create_override_fn = can_create_session_fn;
  CreateArSessionFn& create_override_fn = GetCreateArSessionOverrideFn();
  create_override_fn = create_session_fn;
}

ArSession::ArSession(std::unique_ptr<ArSessionNative> session_native)
    : session_native_(std::move(session_native)),
      last_submitted_timestamp_(absl::InfinitePast()) {}

void ArSession::Pause() { session_native_->Pause(); }

void ArSession::Resume() { session_native_->Resume(); }

bool ArSession::Update() {
  IMP_TRACE();
  assert(Executor::CurrentExecutor() == Executor::ForegroundExecutor());

  absl::optional<ArFrame> frame =
      session_native_->Update(last_submitted_timestamp_);
  if (!frame) {
    return false;
  }

  UpdateTrackableManager(frame->GetUpdatedTrackablesTuple());
  frame_ = std::move(frame);
  return true;
}

void ArSession::RenderDev() {
#if IMP_RUNTIME(DEV)
  if (debug_helper_) {
    VisualizeArPlanes(static_cast<DebugHelperImpl*>(debug_helper_.get()),
                      GetTrackables<ArPlane>());
  }
#endif
}

void ArSession::OnPostRender() {
  if (frame_.has_value()) {
    last_submitted_timestamp_ = frame_->timestamp();
  }
}

std::string ArSession::GetDebugSessionId() const {
  uint4 debug_id = session_native_->GetDebugSessionId();

  // Best guess when time is tight (debug builds always give me 0-0-0-0)
  return absl::StrFormat("%08x-%04x-%04x-%04x-%04x%08x", debug_id.x,
                         debug_id.y >> 16, debug_id.y & 0xFFFF,
                         debug_id.z >> 16, debug_id.z & 0xFFFF, debug_id.w);
}

void ArSession::GetCameraTextureUVs(
    const absl::Span<const float2> normalized_screen_coordinates,
    std::vector<float2>* uvs_out) const {
  return session_native_->GetCameraTextureUVs(normalized_screen_coordinates,
                                              uvs_out);
}
bool ArSession::GetDisplayGeometryChanged() const {
  return session_native_->GetDisplayGeometryChanged();
}

void ArSession::SetDisplayGeometry(window::WindowRotation orientation,
                                   int width, int height, float near,
                                   float far) {
  return session_native_->SetDisplayGeometry(orientation, width, height, near,
                                             far);
}

void ArSession::SetTransitionParameters(float2 view_scale,
                                        float counter_rotation) {
  return session_native_->SetTransitionParameters(view_scale, counter_rotation);
}

std::vector<ArHitResult> ArSession::HitTest(
    float2 screen_pos, absl::optional<float> guessed_distance_meters) {
  IMP_TRACE();
  TrackableTuple new_trackables;
  std::vector<ArHitResult> results = session_native_->HitTest(
      screen_pos, guessed_distance_meters, &new_trackables);
  UpdateTrackableManager(new_trackables);
  return results;
}

std::vector<ArHitResult> ArSession::HitTestRay(
    const Ray& ray, absl::optional<float> guessed_distance) {
  IMP_TRACE();
  TrackableTuple new_trackables;
  auto results = session_native_->HitTestRay(ray, &new_trackables);
  UpdateTrackableManager(new_trackables);

  // TODO: does this make sense? Get ARCore team guidance on this.
  auto& map = std::get<TrackableMap<ArPoint>>(trackable_manager_.GetTuple());
  if (results.empty() && !map.empty() && guessed_distance) {
    auto guessed_point = ray.origin + ray.direction * guessed_distance.value();
    float best_score = 1.0e3f;
    const ArPoint* best = nullptr;
    for (auto& pair : map) {
      const ArPoint& point = pair.second;
      if (point.GetTrackingState() != TrackingState::kTracking ||
          point.GetTrackingMethod() == ArPoint::TrackingMethod::kUnavailable)
        continue;

      // Score this hit
      float3 translation, scale;
      quatf rotation;
      imp::Decompose(point.GetTransform(), &translation, &rotation, &scale);
      if (!IsYUp(rotation)) continue;
      float distance = length(translation - guessed_point);
      if (distance < best_score) {
        best_score = distance;
        best = &point;
      }
    }
    if (best) {
      quatf rotation;
      float3 translation, scale;
      imp::Decompose(best->GetTransform(), &translation, &rotation, &scale);
      results.push_back(ArHitResult(rotation, translation,
                                    length(ray.origin - translation),
                                    best->GetId()));
    }
  }

  return results;
}

absl::StatusOr<ArTrackableHandle<ArAnchor>> ArSession::CreateAnchor(
    float3 position, quatf rotation, absl::optional<ArTrackableId> id) {
  IMP_TRACE();
  absl::StatusOr<ArAnchor> anchor =
      session_native_->CreateAnchor(position, rotation, id);

  MP_RETURN_IF_ERROR(anchor.status());

  // Adds/Updates the anchor in managed trackables.
  trackable_manager_.AddOrUpdateTrackable(*anchor, anchor->GetId());
  return ArTrackableHandle<ArAnchor>(&trackable_manager_, anchor->GetId());
}

absl::StatusOr<ArTrackableHandle<ArAnchor>> ArSession::CreateAnchor(
    double latitude_degrees, double longitude_degrees,
    double wgs84_relative_altitude_meters, quatf rotation) {
  IMP_TRACE();
  absl::StatusOr<ArAnchor> anchor =
      session_native_->CreateAnchor(latitude_degrees, longitude_degrees,
                                    wgs84_relative_altitude_meters, rotation);
  MP_RETURN_IF_ERROR(anchor.status());

  // Adds/Updates the anchor in managed trackables.
  trackable_manager_.AddOrUpdateTrackable(*anchor, anchor->GetId());
  return ArTrackableHandle<ArAnchor>(&trackable_manager_, anchor->GetId());
}

ArTrackableHandle<ArAnchor> ArSession::CreateEnvironmentProbeAnchor(
    float3 position, quatf rotation) {
  absl::StatusOr<ArAnchor> anchor =
      session_native_->CreateEnvironmentProbeAnchor(position, rotation);
  

  // Adds/Updates the anchor in managed trackables.
  ArTrackableId anchor_id = anchor->GetId();
  trackable_manager_.AddOrUpdateTrackable(*anchor, anchor_id);
  return ArTrackableHandle<ArAnchor>(&trackable_manager_, anchor_id);
}

void ArSession::DestroyAnchor(ArTrackableHandle<ArAnchor> anchor) {
  IMP_TRACE();
  if (anchor) {
    session_native_->DestroyAnchor(*anchor);
    // Cleans up the managed anchor.
    trackable_manager_.ClearTrackable<ArAnchor>(anchor->GetId());
  }
}

void ArSession::EnableDebugVisualization(BaseView* view) {
#if IMP_RUNTIME(DEV)
  debug_helper_ = std::make_unique<DebugHelperImpl>(view);
#endif
}

void ArSession::DisableDebugVisualization() {
#if IMP_RUNTIME(DEV)
  debug_helper_.reset();
#endif
}

bool ArSession::IsSessionInterrupted() const {
  return session_native_->IsSessionInterrupted();
}

bool ArSession::IsReadyToRender() const {
  return session_native_->IsReadyToRender();
}

void ArSession::UpdateTrackableManager(const TrackableTuple& updates) {
  IMP_TRACE();
  UpdateReferences updater(&trackable_manager_.GetTuple());
  // Apply new frame updates to the managed trackables
  imp::ForEachTupleElement(updates, updater);
}

}  // namespace ar
}  // namespace imp
