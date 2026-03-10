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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_FOOTPRINT_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_FOOTPRINT_H_

#include <stdbool.h>

#include <array>
#include <bitset>
#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/math/math.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/ramp.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"
#include "split_engine/materials/svxr_footprint_material.h"

namespace svxr {

struct FootprintInteractionStates {
  // Initial state.
  struct Initialized {
    bool is_intialize_complete = false;
  };

  // State for a completely invisible footprint.
  struct Hidden {
    std::optional<imp::float2> next_size;
    bool has_ever_been_active = false;
  };

  // State for a visible footprint.
  struct Active {
    absl::Duration last_interaction_time;
    Ramp<float> alpha;
    Ramp<float> foot_fraction;
    Ramp<imp::float2> foot_size;
    imp::float3 primary_touch_point;
    imp::float3 secondary_touch_point;
    Ramp<imp::float2> edge_touch_control;
    Ramp<imp::float4> edge_touch_response;
    Ramp<imp::float4> edge_falloff_color;
    Ramp<imp::float4> edge_cutoff_color;
    Ramp<imp::float2> fill_touch_control;
    Ramp<imp::float4> fill_touch_response;
    Ramp<imp::float4> fill_falloff_color;
    Ramp<imp::float4> fill_cutoff_color;
    bool is_footprint_primary_receiver = false;
    bool is_footprint_secondary_receiver = false;
    std::bitset<4> scale_handles_visibility;
  };

  // State machine type for footprint.
  using Machine = StateMachine<Initialized, Hidden, Active>;
};

// Represents a white rounded rectangle that appears at the base of a model.
class Footprint : public imp::Component,
                  public FootprintInteractionStates::Machine::Observer {
  using float2 = imp::float2;
  using InteractionMachine = FootprintInteractionStates::Machine;

 public:
  Footprint();
  ~Footprint() override;

  // Initializes a footprint component.
  imp::Future<absl::Status> Setup(imp::NodeHandle model_node);
  absl::Status Setup(
      imp::AssetPtr<imp::GltfAsset> footprint_asset,
      std::unique_ptr<android_xr::SVXRFootprintMaterial> edge_material,
      std::unique_ptr<android_xr::SVXRFootprintMaterial> fill_material,
      imp::AssetPtr<imp::GltfAsset> scale_handle_asset);
  // Cleans up a footprint component.
  void Cleanup();

  // Explicitly updated by the client, don't use Component::Update mechanism
  void OnUpdate(const imp::FrameTime& delta_time,
                const InteractionMode& interaction_data);

  // The sub-node that contains the authored footprint asset managed by this
  // component.
  imp::NodeHandle FootprintNode() const { return footprint_node_; }

  void OnInteractionMachineInitialized();

  void OnModelSizeChanged();

  void HandleInputEvent(const imp::float3& hit_position, bool is_primary_touch,
                        bool is_footprint_receiver);

  imp::float2 GetFootprintSize();

  virtual void SetColliderEnabled(bool is_enabled);

  enum class SnapMode {
    kNone,
    kSnappable,
    kSnappingToPlane,
    kSnappedToPlane,
    kLiftingOffPlane,
    kCooldown,
  };
  bool IsSnapMode(SnapMode snap_mode) const;
  void SetSnapMode(SnapMode snap_mode);

  // Returns true if the footprint is snapped or is in the process of snapping
  // or lifting off a plane.
  bool IsSnappedOrSnapping() const;

  bool IsScaleHandle(imp::NodeHandle node) const;
  bool IsCloseToScaleHandle(const imp::float3& world_hit_pos) const;

 private:
  // Observer method.
  void OnStateChange(const InteractionMachine& machine,
                     const InteractionMachine::State& current_state,
                     const InteractionMachine::State& next_state) override;
  // Non-trivial update methods.
  InteractionMachine::OptionalState UpdateActive(
      FootprintInteractionStates::Active& state,
      const imp::FrameTime& delta_time,
      const InteractionMode& interaction_data);
  InteractionMachine::OptionalState UpdateHidden(
      FootprintInteractionStates::Hidden& state,
      const imp::FrameTime& delta_time,
      const InteractionMode& interaction_data);
  // Helper methods.
  imp::float2 RetrieveSizeFromModel();
  void UpdateFootBonesAndBounds(float2 foot_size, float foot_fraction);
  void MaintainThickness();

  // State machine for the footprint.
  InteractionMachine machine_;
  // The node containing the model the footprint grounds.
  imp::NodeHandle model_node_;
  // Sub-node that contains the authored footprint asset.
  imp::NodeHandle footprint_node_;
  // Held reference to the GltfRenderer component for the footprint node.
  imp::ComponentHandle<imp::GltfRenderer> footprint_model_;

  std::unique_ptr<android_xr::SVXRFootprintMaterial> edge_material_;
  std::unique_ptr<android_xr::SVXRFootprintMaterial> fill_material_;

  // Holds a snapshot of the model's bounds. This helps us keep the
  // footprint's scaling relatively the same.
  imp::Box initial_model_bounds_;
  imp::float3 model_space_inner_half_extents_;

  SnapMode snap_mode_ = SnapMode::kNone;

  // The nodes containing the scale handles.
  std::array<imp::NodeHandle, 4> scale_handles_;
  std::array<imp::NodeHandle, 4> scale_handle_visuals_;
};

}  // namespace svxr
#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_FOOTPRINT_H_
