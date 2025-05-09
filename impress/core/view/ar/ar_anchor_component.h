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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_AR_AR_ANCHOR_COMPONENT_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_AR_AR_ANCHOR_COMPONENT_H_

#include <optional>

#include "core/ar/ar_anchor.h"
#include "core/ar/ar_session.h"
#include "core/ar/ar_trackable_handle.h"
#include "core/ncsb/node.h"
#include "core/view/ar/base_ar_component.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

// Event sent by the ArAnchorComponent when it updates its tracked anchor.
struct AnchorChanged : public imp::Event {
  AnchorChanged() {}
  explicit AnchorChanged(ar::ArTrackableHandle<ar::ArAnchor> in_handle)
      : handle(in_handle) {}

  ar::ArTrackableHandle<ar::ArAnchor> handle;
};

class ArAnchorComponent : public BaseArComponent<ar::ArAnchor> {
 public:
  void Setup(ar::ArTrackableHandle<ar::ArSession> trackable) = delete;

  // Anchors a node at its current location.
  // Note: An ArSceneController must be present in the registry.
  void Setup();
  // Anchors a node at its current location.
  void Setup(std::weak_ptr<ar::ArSession> ar_session);

  // Anchors a node using a hit-test result.
  // Note: An ArSceneController must be present in the registry.
  void Setup(const RayHit& hit);

  // Anchors a node using a hit-test result.
  void Setup(std::weak_ptr<ar::ArSession> ar_session, const RayHit& hit);

  // Anchors a node using a position and rotation.
  // Note: An ArSceneController must be present in the registry.
  void Setup(float3 position, quatf rotation,
             absl::optional<ar::ArTrackableId> attachment_id = absl::nullopt);
  // Anchors a node using a position and rotation.
  void Setup(std::weak_ptr<ar::ArSession> ar_session, float3 position,
             quatf rotation,
             absl::optional<ar::ArTrackableId> attachment_id = absl::nullopt);

  // Anchors a node at a geo location.
  // Note: An ArSceneController must be present in the registry.
  void Setup(double latitude_degrees, double longitude_degrees,
             double wgs84_relative_altitude_meters, quatf rotation);
  // Anchors a node at a geo location.
  void Setup(std::weak_ptr<ar::ArSession> ar_session, double latitude_degrees,
             double longitude_degrees, double wgs84_relative_altitude_meters,
             quatf rotation);

  void Cleanup();

  // Relocate the anchor to a new location specified by the hit.
  void MoveAnchor(const RayHit& hit);

  // Relocate the anchor to a new location, attached to the optional trackable.
  void MoveAnchor(float3 position, quatf rotation,
                  absl::optional<ar::ArTrackableId> attachment_id = {});

  // Updates the Id that the anchor is attached to (ie the underlying AR object)
  // without changing its position or orientation.
  void AttachToId(ar::ArTrackableId attachment_id);

  // Returns the Id of the trackable the anchor is attached to.
  ar::ArTrackableId GetAttachmentId() { return *attachment_id_; }

 private:
  // Retrieves the session from the ArSceneController in the view registry.
  std::weak_ptr<ar::ArSession> GetArSessionFromRegistry();
  // Attempts to lock the weak pointer to the ArSession and returns the result.
  std::shared_ptr<ar::ArSession> LockArSession();
  // Creates the actual anchor trackable that backs this component.
  // Note: will try without the attachment ID if creating the anchor with the
  // attachment ID fails.
  void CreateAnchor(float3 position, quatf rotation,
                    absl::optional<ar::ArTrackableId> attachment_id = {});
  // Attaches to the given result of a session->CreateAnchor(...) call.
  absl::Status AttachToAnchor(
      absl::StatusOr<ar::ArTrackableHandle<ar::ArAnchor>> create_anchor_result,
      absl::optional<ar::ArTrackableId> attachment_id = {});
  // Maintains a weak ptr to the ArSession that hosts anchors.
  std::weak_ptr<ar::ArSession> ar_session_;
  // The id of the parent trackable that this component's anchors are attached.
  absl::optional<ar::ArTrackableId> attachment_id_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_AR_AR_ANCHOR_COMPONENT_H_
