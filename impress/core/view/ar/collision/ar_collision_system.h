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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_AR_COLLISION_AR_COLLISION_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_AR_COLLISION_AR_COLLISION_SYSTEM_H_

#include <vector>

#include "core/common/log.h"
#include "absl/types/optional.h"
#include "core/ar/ar_session.h"
#include "core/ar/ar_trackable.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/common/robin_map.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/system.h"
#include "core/view/ar/collision/collider_ar_magical_surface_point.h"
#include "core/view/ar/collision/collider_ar_plane.h"
#include "core/view/ar/collision/collider_ar_point.h"
#include "core/view/framework/collision/collision_system.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

// ArCollisionSystem manages testing rays against AR objects in the scene.
template <typename Collider>
class ArCollisionSystem : public BaseCollisionSystem {
 public:
  ArCollisionSystem(BaseView* view, ar::ArSession* ar_session,
                    ar::HitMode hit_mode = ar::HitMode::kRealDepth,
                    float guessed_distance_meters = 2.0f)
      : BaseCollisionSystem(view),
        ar_session_(ar_session),
        hit_mode_(hit_mode),
        guessed_distance_meters_(guessed_distance_meters) {}
  size_t GetColliderCount() override;
  // Tests colliders in the scene against a ray, doesn't apply filtering.
  void Intersect(const Ray& world_ray, std::vector<RayHit>* out_intersections);
  void Intersect(float2 screen_pos, std::vector<RayHit>* out_intersections);
  // Tests colliders in the scene against a ray, applies collision mask filter
  // to colliders.
  void Intersect(const Ray& world_ray, Flags<CollisionMask> mask,
                 std::vector<RayHit>* out_intersections) override;
  void Intersect(float2 screen_pos, Flags<CollisionMask> mask,
                 std::vector<RayHit>* out_intersections) override;

  // Tests colliders in the scene against a ray, applies collision mask filter
  // to colliders.
  void IntersectPrecise(const DoubleRay& world_ray, Flags<CollisionMask> mask,
                        std::vector<DoubleRayHit>* out_intersections) override;
  void IntersectPrecise(float2 screen_pos, Flags<CollisionMask> mask,
                        std::vector<DoubleRayHit>* out_intersections) override;

  // Updates the guessed distance of collision points.
  void SetDistanceGuess(float guessed_distance_meters) {
    hit_mode_ = ar::HitMode::kDistanceGuess;
    guessed_distance_meters_ = guessed_distance_meters;
  }

  void ClearDistanceGuess() { hit_mode_ = ar::HitMode::kRealDepth; }

  void Update();

 private:
  void IntersectHelper(const Ray& world_ray, absl::optional<float2> screen_pos,
                       Flags<CollisionMask> mask,
                       std::vector<RayHit>* out_intersections);

  ar::ArSession* ar_session_;
  ar::HitMode hit_mode_;
  float guessed_distance_meters_;
  RobinMap<ar::ArTrackableHandle<typename Collider::TrackableType>, NodeHandle>
      trackable_map_;
};

template <typename Collider>
size_t ArCollisionSystem<Collider>::GetColliderCount() {
  return trackable_map_.size();
}

template <typename Collider>
void ArCollisionSystem<Collider>::Intersect(
    float2 screen_pos, std::vector<RayHit>* out_intersections) {
  Intersect(screen_pos, ToFlags(CollisionMask::kAll), out_intersections);
}

template <typename Collider>
void ArCollisionSystem<Collider>::Intersect(
    const Ray& world_ray, std::vector<RayHit>* out_intersections) {
  Intersect(world_ray, ToFlags(CollisionMask::kAll), out_intersections);
}

template <typename Collider>
void ArCollisionSystem<Collider>::Intersect(
    const Ray& world_ray, Flags<CollisionMask> mask,
    std::vector<RayHit>* out_intersections) {
  IntersectHelper(world_ray, absl::nullopt, mask, out_intersections);
}

template <typename Collider>
void ArCollisionSystem<Collider>::Intersect(
    float2 screen_pos, Flags<CollisionMask> mask,
    std::vector<RayHit>* out_intersections) {
  auto world_ray = details::GetWorldRayFromPixelPosition(GetView(), screen_pos);
  IntersectHelper(world_ray, screen_pos, mask, out_intersections);
}

template <typename Collider>
void ArCollisionSystem<Collider>::IntersectPrecise(
    const DoubleRay& world_ray, Flags<CollisionMask> mask,
    std::vector<DoubleRayHit>* out_intersections) {
  IMP_LOG(imp::WARNING) << "Intersect precise is not supported.";
}

template <typename Collider>
void ArCollisionSystem<Collider>::IntersectPrecise(
    float2 screen_pos, Flags<CollisionMask> mask,
    std::vector<DoubleRayHit>* out_intersections) {
  IMP_LOG(imp::WARNING) << "Intersect precise is not supported.";
}

template <typename TrackableHandle>
bool IsTrackableActive(const TrackableHandle& trackable_handle) {
  return trackable_handle->GetTrackingState() != ar::TrackingState::kStopped;
}

template <typename Collider>
void ArCollisionSystem<Collider>::IntersectHelper(
    const Ray& world_ray, absl::optional<float2> screen_position,
    Flags<CollisionMask> mask, std::vector<RayHit>* out_intersections) {
  using TrackableType = typename Collider::TrackableType;

  if constexpr (ar::IsPoint<TrackableType>()) {
    // Prevents creating superfluous Ar points.
    if (!mask.Test(CollisionMask::kColliderArPoint)) {
      return;
    }
  }

  if constexpr (ar::IsMagicalSurfacePoint<TrackableType>()) {
    if (!mask.Test(CollisionMask::kColliderArMagicalSurfacePoint)) {
      return;
    }
  }

  // TODO: clean this up to only make one of two calls.
  // Gathers AR collisions for a ray.
  std::vector<ar::ArHitResult> hit_results;
  if (screen_position) {
    hit_results =
        ar_session_->HitTest(screen_position.value(), guessed_distance_meters_);
  } else {
    hit_results = ar_session_->HitTestRay(world_ray, guessed_distance_meters_);
  }

  // Creates a node for each new trackable handle.
  for (auto& hit_result : hit_results) {
    if (auto trackable_handle = ar_session_->GetTrackableHandle<TrackableType>(
            hit_result.GetTrackableId());
        trackable_handle && IsTrackableActive(trackable_handle)) {
      auto iter = trackable_map_.find(trackable_handle);
      NodeHandle trackable_node;
      if (iter == trackable_map_.end()) {
        trackable_node = GetView().CreateNode();
        trackable_node->SetLocalTrs(trackable_handle->GetTransform());
        trackable_node->AddComponent<Collider>(trackable_handle);
        trackable_map_.insert(std::make_pair(trackable_handle, trackable_node));
      } else {
        trackable_node = iter->second;
      }
      if (trackable_node &&
          trackable_node->GetComponent<Collider>()->TestCollisionFlags(mask)) {
        RayHit out_intersection =
            RayHit(hit_result.GetDistance(), hit_result.GetRotation(),
                   hit_result.GetWorldHitPoint(), trackable_node);
        out_intersections->push_back(out_intersection);
      }
    }
  }
}

template <typename Collider>
void ArCollisionSystem<Collider>::Update() {
  // Removes nodes that have been destroyed.
  for (auto iter = trackable_map_.begin(); iter != trackable_map_.end();
       ++iter) {
    NodeHandle node = iter->second;
    if (!node) {
      trackable_map_.erase(iter);
    }
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_AR_COLLISION_AR_COLLISION_SYSTEM_H_
