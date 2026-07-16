/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_PLANE_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_PLANE_H_

#include <stdbool.h>

#include <memory>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "core/async/future.h"
#include "core/math/math.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/ramp.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"
#include "split_engine/materials/svxr_plane_material.h"

namespace svxr {

enum class PlaneTrackingState {
  kPaused,
  kStopped,
  kTracking,
};

enum class PlaneType {
  kHorizontalDownwardFacing,
  kHorizontalUpwardFacing,
  kVertical,
  kArbitrary,
};

enum class PlaneLabel {
  kUnknown,
  kWall,
  kFloor,
  kCeiling,
  kTable,
};

namespace plane_interaction_states {
// Initial state.
struct Initialized {};

// State for a completely invisible plane.
struct Hidden {};

// State for a visible plane.
struct Active {
  Ramp<float> glow;
  absl::Duration last_interaction_time;
};

// State machine type for plane.
using Machine = StateMachine<Initialized, Hidden, Active>;
}  // namespace plane_interaction_states

// Represents a detected plane that can be interacted with.
class Plane : public imp::Component,
              public plane_interaction_states::Machine::Observer {
  using float2 = imp::float2;
  using InteractionMachine = plane_interaction_states::Machine;

 public:
  Plane();
  ~Plane() override;

  // Initializes a plane component.
  imp::Future<absl::Status> Setup(PlaneTrackingState tracking_state,
                                  PlaneType type, PlaneLabel label,
                                  absl::Span<imp::float2> vertices);
  absl::Status Setup(PlaneTrackingState tracking_state, PlaneType type,
                     PlaneLabel label, absl::Span<imp::float2> vertices,
                     std::unique_ptr<android_xr::SVXRPlaneMaterial> material,
                     imp::OwnedTexturePtr texture);

  // Cleans up a plane component.
  void Cleanup();

  const imp::Rect& GetRect() const { return rect_; }

  // Returns true if a plane is determined to be active or false otherwise.
  bool ShouldBeActive(imp::float3 target_position_world) const;
  // Notifies the plane of a selected target position. Note that this is sent
  // to every plane, not just the plane that generated the target position.
  void Activate(imp::float3 target_position_world);

  bool IsRelevant() const;

  void CalculateSnappability(imp::float3 target_position_world);
  bool IsSnappable() const { return is_snappable_; }

  PlaneTrackingState GetTrackingState() const { return tracking_state_; }
  PlaneType GetType() const { return type_; }
  PlaneLabel GetLabel() const { return label_; }

  // Explicitly updated by the client, don't use Component::Update mechanism.
  void OnUpdate(const imp::FrameTime& delta_time);

  // Returns the area of overlap between this plane and another plane on the
  // XZ plane, assuming both are horizontal.
  float GetCollisionArea(const Plane& other) const;

  // Returns the total surface area of the plane on the XZ plane.
  float GetArea() const;

 private:
  // Advances the "glow" up or down based on how recent interactions have been,
  // and updates material parameters accordingly.
  InteractionMachine::OptionalState UpdateActive(
      plane_interaction_states::Active& state,
      const imp::FrameTime& delta_time);

  // Observer method.
  void OnStateChange(const InteractionMachine& machine,
                     const InteractionMachine::State& current_state,
                     const InteractionMachine::State& next_state) override;

  void CalculateArea();

  // State machine for the plane.
  InteractionMachine machine_;

  imp::ComponentHandle<imp::MeshRenderer> renderer_;

  std::unique_ptr<android_xr::SVXRPlaneMaterial> material_;
  imp::OwnedTexturePtr texture_;
  imp::Rect rect_;
  std::vector<imp::float2> vertices_;
  PlaneTrackingState tracking_state_;
  PlaneType type_;
  PlaneLabel label_;
  float area_ = 0.0f;
  bool is_snappable_ = false;
};

}  // namespace svxr
#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_PLANE_H_
