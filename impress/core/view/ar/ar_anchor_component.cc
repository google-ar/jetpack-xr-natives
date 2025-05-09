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

#include "core/view/ar/ar_anchor_component.h"

#include <functional>
#include <memory>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/common/registry.h"
#include "core/view/ar/ar_scene_controller.h"
#include "core/view/ar/collision/collider_ar_plane.h"
#include "core/view/ar/collision/collider_ar_point.h"
#include "core/view/base_view.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace {

using ar::ArSession;
using ar::ArTrackableId;

static constexpr float kAnchorPositionThreshold = 0.001f;
static constexpr float kAnchorRotationThreshold = 0.001f;

}  // namespace

std::weak_ptr<ArSession> ArAnchorComponent::GetArSessionFromRegistry() {
  absl::StatusOr<std::reference_wrapper<ArSceneController>>
      ar_scene_controller =
          GetNode()->GetView().GetRegistry().Get<ArSceneController>();
  if (!ar_scene_controller.ok()) {
    IMP_LOG(imp::FATAL) << "Attempt to create an ArAnchor with an invalid AR session: "
               << ar_scene_controller.status();
  }
  return ar_scene_controller->get().GetSession();
}

std::shared_ptr<ArSession> ArAnchorComponent::LockArSession() {
  std::shared_ptr<imp::ar::ArSession> ptr = ar_session_.lock();
  if (!ptr) {
    IMP_LOG(imp::FATAL) << "Cannot acquire AR session pointer.";
  }
  return ptr;
}

void ArAnchorComponent::Setup() { Setup(GetArSessionFromRegistry()); }

void ArAnchorComponent::Setup(std::weak_ptr<ArSession> ar_session) {
  Setup(ar_session, GetNode()->GetWorldPosition(),
        GetNode()->GetWorldRotation());
}

void ArAnchorComponent::Setup(const RayHit& hit) {
  Setup(GetArSessionFromRegistry(), hit);
}

void ArAnchorComponent::Setup(std::weak_ptr<ArSession> ar_session,
                              const RayHit& hit) {
  absl::optional<ArTrackableId> attachment_id;
  if (auto plane = hit.node->GetComponent<ColliderArPlane>()) {
    attachment_id = plane->GetTrackableId();
  } else if (auto point = hit.node->GetComponent<ColliderArPoint>()) {
    attachment_id = point->GetTrackableId();
  }
  Setup(ar_session, hit.world_point, hit.world_orientation, attachment_id);
}

void ArAnchorComponent::Setup(float3 position, quatf rotation,
                              absl::optional<ArTrackableId> attachment_id) {
  Setup(GetArSessionFromRegistry(), position, rotation, attachment_id);
}

void ArAnchorComponent::Setup(std::weak_ptr<ArSession> ar_session,
                              float3 position, quatf rotation,
                              absl::optional<ArTrackableId> attachment_id) {
  ar_session_ = ar_session;
  CreateAnchor(position, rotation, attachment_id);
}

void ArAnchorComponent::Setup(double latitude_degrees, double longitude_degrees,
                              double wgs84_relative_altitude_meters,
                              quatf rotation) {
  Setup(GetArSessionFromRegistry(), latitude_degrees, longitude_degrees,
        wgs84_relative_altitude_meters, rotation);
}

void ArAnchorComponent::Setup(std::weak_ptr<ArSession> ar_session,
                              double latitude_degrees, double longitude_degrees,
                              double wgs84_relative_altitude_meters,
                              quatf rotation) {
  ar_session_ = ar_session;
  absl::Status create_anchor_result = AttachToAnchor(
      LockArSession()->CreateAnchor(latitude_degrees, longitude_degrees,
                                    wgs84_relative_altitude_meters, rotation));
  if (!create_anchor_result.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to create anchor: " << create_anchor_result;
  }
}

void ArAnchorComponent::Cleanup() {
  absl::StatusOr<std::shared_ptr<ArSession>> ar_session_ptr = LockArSession();
  if (ar_session_ptr.ok()) {
    ar_session_ptr->get()->DestroyAnchor(trackable_);
  }
}

void ArAnchorComponent::MoveAnchor(const RayHit& hit) {
  absl::optional<ArTrackableId> attachment_id;
  if (auto plane = hit.node->GetComponent<ColliderArPlane>()) {
    attachment_id = plane->GetTrackableId();
  } else if (auto point = hit.node->GetComponent<ColliderArPoint>()) {
    attachment_id = point->GetTrackableId();
  }
  MoveAnchor(hit.world_point, hit.world_orientation, attachment_id);
}

void ArAnchorComponent::MoveAnchor(
    float3 position, quatf rotation,
    absl::optional<ArTrackableId> attachment_id) {
  // Destroys the old anchor if it isn't already at the desired position.
  if (trackable_) {
    auto current = imp::Transform<float>(GetNode()->GetWorldTrs());
    const float delta_position = length(position - current.translation);
    const float delta_rotation = fabs(1 - dot(rotation, current.rotation));
    if ((!attachment_id.has_value() || attachment_id == attachment_id_) &&
        delta_position < kAnchorPositionThreshold &&
        delta_rotation < kAnchorRotationThreshold) {
      // Anchor already has this position, rotation, and identifier.
      return;
    }
    LockArSession()->DestroyAnchor(trackable_);
  }
  // Creates the new anchor.
  return CreateAnchor(position, rotation, attachment_id);
}

void ArAnchorComponent::AttachToId(ar::ArTrackableId attachment_id) {
  MoveAnchor(GetNode()->GetWorldPosition(), GetNode()->GetWorldRotation(),
             attachment_id);
}

void ArAnchorComponent::CreateAnchor(
    float3 position, quatf rotation,
    absl::optional<ArTrackableId> attachment_id) {
  std::shared_ptr<ArSession> ar_session_ptr = LockArSession();

  absl::Status create_anchor_result = AttachToAnchor(
      ar_session_ptr->CreateAnchor(position, rotation, attachment_id),
      attachment_id);
  if (create_anchor_result.ok()) {
    return;
  }
  if (attachment_id.has_value()) {
    // Automatically retry with no attachment id.
    create_anchor_result =
        AttachToAnchor(ar_session_ptr->CreateAnchor(position, rotation, {}));
  }
  if (!create_anchor_result.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to create anchor: " << create_anchor_result;
  }
}

absl::Status ArAnchorComponent::AttachToAnchor(
    absl::StatusOr<ar::ArTrackableHandle<ar::ArAnchor>> create_anchor_result,
    absl::optional<ar::ArTrackableId> attachment_id) {
  MP_RETURN_IF_ERROR(create_anchor_result.status());

  trackable_ = *create_anchor_result;
  attachment_id_ = attachment_id;

  UpdateLocation();
  GetNode()->GetView().GetDispatcher().Send(AnchorChanged(trackable_));
  return absl::OkStatus();
}

}  // namespace imp
