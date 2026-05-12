// Copyright 2025 Google LLC
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

#include "extensions/sceneviewerxr/ux/footprint.h"

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Box.h"
#include "extensions/sceneviewerxr/ux/gltf_bounds.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/assets/footprint_assets.h"
#include "split_engine/materials/svxr_footprint_material.h"

namespace svxr {

namespace {

// #define USE_UX_FOOTPRINT

#if defined(USE_UX_FOOTPRINT)

// These were determined manually, by messing with the inspector via babylon.
// NOTE: there seems to be a Z-flip in Blender's export; this is why the bone
// names (with P/N for Positive/Negative) don't match up with these sign bits.
constexpr imp::float3 kCorners[] = {imp::float3(+1.f, 0.f, -1.f),  //
                                    imp::float3(-1.f, 0.f, -1.f),  //
                                    imp::float3(+1.f, 0.f, +1.f),  //
                                    imp::float3(-1.f, 0.f, +1.f)};

constexpr absl::string_view kBoneNames[] = {"LowerLeft", "LowerRight",
                                            "UpperLeft", "UpperRight"};

// The corner cards are authored to be 1wu per side.  This scales to the desired
// default display size.
constexpr float kDefaultScale = 0.5f;

constexpr auto kEdgeMaterialMeshIndex = 0;
constexpr auto kFillMaterialMeshIndex = 1;

#else  // defined(USE_UX_FOOTPRINT)

constexpr auto kExpectedMeshIndex = 0;
constexpr auto kExpectedPrimitiveCount = 2;

constexpr imp::float3 kCorners[] = {
    imp::float3(-1.f, 0.f, +1.f), imp::float3(-1.f, 0.f, -1.f),
    imp::float3(+1.f, 0.f, +1.f), imp::float3(+1.f, 0.f, -1.f)};

constexpr imp::float3 kFootprintScale = imp::float3(1.f, 0.675f, 1.f);

constexpr absl::string_view kBoneNames[] = {"Bone_NN", "Bone_NP", "Bone_PN",
                                            "Bone_PP"};

// The corner cards are authored to be 1wu per side.  This scales to the desired
// default display size.
constexpr float kDefaultScale = 0.05f;

#endif  // defined(USE_UX_FOOTPRINT)
constexpr auto kCornerCount = 4;

// Used to help buff up the footprint thickness when it is very far away.
constexpr auto kDistanceBasedYBuffer = 1.5f;
constexpr auto kDistanceBufferRange = 14.f;
constexpr auto kDistanceBufferMinimum = .125f;

constexpr float kTouchAreaHalfExtentsMin = 0.06f;
constexpr float kCardInclusionFraction = 0.125f;

constexpr auto kFootSizeUpdateRampDuration = absl::Milliseconds(50);
constexpr auto kSpawnFadeInDuration = absl::Milliseconds(120);
constexpr auto kInteractFadeInDuration = absl::Milliseconds(120);
constexpr auto kUnselectFadeOutDuration = absl::Milliseconds(120);

struct ShaderParameters {
  imp::float2 edge_touch_control;
  imp::float4 edge_touch_response;
  imp::float4 edge_falloff_color;
  imp::float4 edge_cutoff_color;
  imp::float2 fill_touch_control;
  imp::float4 fill_touch_response;
  imp::float4 fill_falloff_color;
  imp::float4 fill_cutoff_color;
};

constexpr auto kParamsHidden = ShaderParameters{
    .edge_touch_control = imp::float2(0.01f, 0.4f),
    .edge_touch_response = imp::float4(0.375f, 0.375f, 2.0f, 0.0f),
    .edge_falloff_color = imp::float4(0.95f, 0.95f, 0.95f, 1.0f),
    .edge_cutoff_color = imp::float4(0.7f, 0.7f, 0.7f, 1.0f),
    .fill_touch_control = imp::kZero2,  // disabled
    .fill_touch_response = imp::float4(0.4f, 0.4f, 2.0f, 0.0f),
    .fill_falloff_color = imp::float4(0.3f, 0.3f, 0.3f, 1.0f),
    .fill_cutoff_color = imp::float4(0.3f, 0.3f, 0.3f, 1.0f),
};
constexpr auto kParamsActiveOverModel = ShaderParameters{
    .edge_touch_control = imp::float2(0.01f, 0.7f),
    .edge_touch_response = imp::float4(0.f, 0.75f, 2.0f, 0.9f),
    .edge_falloff_color = imp::float4(0.95f, 0.95f, 0.95f, 1.0f),
    .edge_cutoff_color = imp::float4(0.9f, 0.9f, 0.9f, 1.0f),
    .fill_touch_control = imp::kZero2,  // disabled
    .fill_touch_response = imp::float4(0.0f, 0.8f, 2.0f, 0.2f),
    .fill_falloff_color = imp::float4(0.45f, 0.45f, 0.45f, 1.0f),
    .fill_cutoff_color = imp::float4(0.2f, 0.2f, 0.2f, 1.0f),
};
constexpr auto kParamsActiveOverFootprint = ShaderParameters{
    .edge_touch_control = imp::float2(0.01f, 0.35f),
    .edge_touch_response = imp::float4(0.75f, 0.0f, 2.0f, 0.9f),
    .edge_falloff_color = imp::float4(0.95f, 0.95f, 0.95f, 1.0f),
    .edge_cutoff_color = imp::float4(0.7f, 0.7f, 0.7f, 1.0f),
    .fill_touch_control = imp::kZero2,  // disabled
    .fill_touch_response = imp::float4(0.8f, 0.0f, 2.0f, 0.2f),
    .fill_falloff_color = imp::float4(0.5f, 0.5f, 0.5f, 1.0f),
    .fill_cutoff_color = imp::float4(0.2f, 0.2f, 0.2f, 1.0f),
};
constexpr auto kParamsActiveNoPointer = ShaderParameters{
    .edge_touch_control = imp::float2(0.01f, 0.5f),
    .edge_touch_response = imp::float4(0.f, 0.75f, 2.0f, 0.5f),
    .edge_falloff_color = imp::float4(0.95f, 0.95f, 0.95f, 1.0f),
    .edge_cutoff_color = imp::float4(0.8f, 0.8f, 0.8f, 1.0f),
    .fill_touch_control = imp::kZero2,  // disabled
    .fill_touch_response = imp::float4(0.0f, 0.8f, 2.0f, 0.1f),
    .fill_falloff_color = imp::float4(0.3f, 0.3f, 0.3f, 1.0f),
    .fill_cutoff_color = imp::float4(0.3f, 0.3f, 0.3f, 1.0f),
};
constexpr auto kParamsIsSnappable = ShaderParameters{
    .edge_touch_control = imp::float2(0.01f, 0.5f),
    .edge_touch_response = imp::float4(1.0f, 0.75f, 2.0f, 1.0f),
    .edge_falloff_color = imp::float4(1.0f, 1.0f, 1.0f, 1.0f),
    .edge_cutoff_color = imp::float4(0.99f, 0.99f, 0.99f, 1.0f),
    .fill_touch_control = imp::kZero2,  // disabled
    .fill_touch_response = imp::float4(1.0f, 1.0f, 2.0f, 0.3f),
    .fill_falloff_color = imp::float4(1.0f, 1.0f, 1.0f, 1.0f),
    .fill_cutoff_color = imp::float4(1.0f, 1.0f, 1.0f, 1.0f),
};
constexpr auto kParamsActivePressNoPointer = kParamsActiveOverModel;

// Whether to use the pointer glow.
// When true, disables legacy method of showing a gradated glow of the pointer.
constexpr auto kDisablePointerGlow = true;

bool IsVisibleBasedOnFlags(const InteractionMode& interaction_data) {
  // All UI is invisible if SVXR is unselected.
  if (interaction_data.TestSelected(
          InteractionMode::SelectedMode::kUnselected)) {
    return false;
  }

  // If a scale or rotate is occurring, be sure to hide the footprint.
  if (interaction_data.TestTransform(InteractionMode::TransformMode::kRotate) ||
      interaction_data.TestTransform(InteractionMode::TransformMode::kScale)) {
    return false;
  } else if (interaction_data.TestTransform(
                 InteractionMode::TransformMode::kTranslate)) {
    // If we are translating, be sure to show the footprint.
    return true;
  }

  // If hovering, the footprint is visible.
  if (interaction_data.TestPointer(InteractionMode::PointerMode::kHover) ||
      interaction_data.TestPointer((InteractionMode::PointerMode::kPress))) {
    return true;
  }

  // Make footprint solid by default in selected state.
  return true;
}

}  // namespace

Footprint::Footprint()
    : machine_(FootprintInteractionStates::Initialized{}, this) {}

Footprint::~Footprint() = default;

imp::Future<absl::Status> Footprint::Setup(imp::NodeHandle model_node) {
  auto& view = GetNode()->GetView();
  model_node_ = model_node;

  auto& asset_manager = view.GetAssetManager();
  auto options = asset_manager.GetDefaultLoadOptions();
  options.vertex_access_flags = imp::GltfAsset::kPosition;

#if defined(USE_UX_FOOTPRINT)
  auto asset_future =
      asset_manager.LoadGltfAsset(imp::kFootprintScalableGlb, options);
#else   // defined(USE_UX_FOOTPRINT)
  auto asset_future = asset_manager.LoadGltfAsset(imp::kFootprintGlb, options);
#endif  // defined(USE_UX_FOOTPRINT)
  auto edge_material_future = android_xr::SVXRFootprintMaterial::Create(view);
  auto fill_material_future = android_xr::SVXRFootprintMaterial::Create(view);

  return asset_future.Merge(edge_material_future, fill_material_future)
      .Then([this](
                std::tuple<imp::AssetPtr<imp::GltfAsset>,
                           std::unique_ptr<android_xr::SVXRFootprintMaterial>,
                           std::unique_ptr<android_xr::SVXRFootprintMaterial>>
                    result) mutable -> absl::Status {
        auto& [gltf_asset, edge_material, fill_material] = result;

        return Setup(std::move(gltf_asset), std::move(edge_material),
                     std::move(fill_material));
      });
}

absl::Status Footprint::Setup(
    imp::AssetPtr<imp::GltfAsset> footprint_asset,
    std::unique_ptr<android_xr::SVXRFootprintMaterial> edge_material,
    std::unique_ptr<android_xr::SVXRFootprintMaterial> fill_material) {
  edge_material_ = std::move(edge_material);
  fill_material_ = std::move(fill_material);
  auto node = GetNode();
  footprint_node_ = GetView().CreateNode();
  footprint_node_->SetParent(node);
  auto model =
      footprint_node_->AddComponent<imp::GltfRenderer>(footprint_asset);
  auto& model_data = model->GetGltfAsset()->GetModelData();
  if (model_data.Skins().empty()) {
    return absl::InternalError("Expected Skin");
  }
  auto& skin = model_data.Skins().front();
  if (skin.sampled_joints.size() != 4) {
    return absl::InternalError(absl::StrFormat("Expected %d bones, not %d", 4,
                                               skin.sampled_joints.size()));
  }

  size_t encountered_mesh_index = 0;

  footprint_node_->GetComponent<imp::GltfScene>()->ForAllNodes(
      [this, &encountered_mesh_index](imp::NodeHandle sub_node) {
        imp::ComponentHandle<imp::GltfMesh> gltf_mesh =
            sub_node->GetComponent<imp::GltfMesh>();
        if (!gltf_mesh) return;
        size_t mesh_index = encountered_mesh_index++;

#if defined(USE_UX_FOOTPRINT)
        if (mesh_index == kEdgeMaterialMeshIndex) {
          gltf_mesh->SetMaterialOverride(edge_material_->GetMaterial());
        } else if (mesh_index == kFillMaterialMeshIndex) {
          gltf_mesh->SetMaterialOverride(fill_material_->GetMaterial());
        }
#else   // defined(USE_UX_FOOTPRINT)
        if (mesh_index == kExpectedMeshIndex &&
            gltf_mesh->GetPrimitiveCount() == kExpectedPrimitiveCount) {
          gltf_mesh->SetMaterialOverride(edge_material_->GetMaterial(), 0);
          gltf_mesh->SetMaterialOverride(fill_material_->GetMaterial(), 1);
        } else {
          imp::output::Error("Unexpected content; skipping material override");
        }
#endif  // defined(USE_UX_FOOTPRINT)
      });

  // TODO: Enable once mesh colliders work.
#if defined(PER_MESH_COLLISION)
  // Remove the default collision created for the footprint (for hit testing)
  footprint_node_->RemoveComponent<imp::BoxCollider>();
  footprint_node_->GetComponent<imp::GltfScene>()->ForAllNodes(
      [](imp::NodeHandle sub_node) {
        imp::ComponentHandle<imp::GltfMesh> gltf_mesh =
            sub_node->GetComponent<imp::GltfMesh>();
        if (gltf_mesh) {
          sub_node->RemoveComponent<imp::GltfCollider>();
          sub_node->AddComponent<imp::GltfCollider>(
              gltf_mesh, imp::GltfCollider::CollisionMode::kTriangles);
        }
      });
#endif

  footprint_node_->SetEnabled(false);

  // Position and scale the footprint correctly.

  auto local_scale = footprint_node_->GetLocalScale();
  imp::Box local_bounds =
      footprint_node_->GetOrAddComponent<GltfBounds>()->GetLocalBounds();
  auto model_root = model->GetModelRoot();
  auto scale = kFootprintScale * (imp::float3(kDefaultScale) / local_scale);
  model_root->SetLocalPosition(scale * local_bounds.halfExtent *
                               imp::float3(0.f, -1.f, 0.f));
  model_root->SetLocalScale(scale);

  // model->GetModelRoot()
  footprint_model_ = model;

#if defined(USE_UX_FOOTPRINT)
  footprint_node_->GetComponent<imp::GltfScene>()->ForAllNodes(
      [](imp::NodeHandle child_node) {
        imp::ComponentHandle<imp::GltfMesh> gltf_mesh =
            child_node->GetComponent<imp::GltfMesh>();
        if (!gltf_mesh) return;
        if (size_t morph_target_count = gltf_mesh->GetMorphTargetCount()) {
          std::vector<float> weights;
          weights.resize(morph_target_count);
          for (auto& weight : weights) {
            weight = 0.25f;
          }
          gltf_mesh->SetMorphTargetWeights(weights);
        }
      });
#endif

  return absl::OkStatus();
}

void Footprint::OnInteractionMachineInitialized() {
  UpdateFootBonesAndBounds(GetFootprintSize(), 1);
}

void Footprint::OnModelSizeChanged() {
  float2 foot_size = RetrieveSizeFromModel();

  machine_.UpdateWithAlternatives(
      [](FootprintInteractionStates::Initialized& state)
          -> InteractionMachine::OptionalState { return {}; },
      [&foot_size](FootprintInteractionStates::Hidden& state)
          -> InteractionMachine::OptionalState {
        state.next_size = foot_size;
        return {};
      },
      [&foot_size](FootprintInteractionStates::Active& state)
          -> InteractionMachine::OptionalState {
        state.foot_size.SetTarget(foot_size, kFootSizeUpdateRampDuration);
        return {};
      });
}

void Footprint::Cleanup() {
  if (footprint_node_) {
    GetView().DestroyNode(footprint_node_);
  }
}

void Footprint::OnUpdate(const imp::FrameTime& delta_time,
                         const InteractionMode& interaction) {
  machine_.UpdateWithAlternatives(
      [this](FootprintInteractionStates::Initialized& state)
          -> InteractionMachine::OptionalState {
        // Defer entering hidden state until async setup completes.
        if (!footprint_model_) return {};
        return FootprintInteractionStates::Hidden{};
      },
      [delta_time, this,
       &interaction](FootprintInteractionStates::Hidden& state)
          -> InteractionMachine::OptionalState {
        return UpdateHidden(state, delta_time, interaction);
      },
      [delta_time, this,
       &interaction](FootprintInteractionStates::Active& state)
          -> InteractionMachine::OptionalState {
        return UpdateActive(state, delta_time, interaction);
      });
}

FootprintInteractionStates::Machine::OptionalState Footprint::UpdateHidden(
    FootprintInteractionStates::Hidden& state, const imp::FrameTime& delta_time,
    const InteractionMode& interaction) {
  if (IsVisibleBasedOnFlags(interaction)) {
    // Transition to FootprintActive
    FootprintInteractionStates::Active next;
    if (!state.next_size.has_value()) {
      state.next_size = RetrieveSizeFromModel();
    }

    next.alpha.Setup(0.f);
    next.alpha.SetTarget(1.f, kSpawnFadeInDuration);

    auto setup_params = kParamsHidden;
    auto set_target_params = kParamsActiveOverModel;

    next.edge_touch_control.Setup(setup_params.edge_touch_control);
    next.edge_touch_control.SetTarget(set_target_params.edge_touch_control,
                                      kInteractFadeInDuration);
    next.fill_touch_control.Setup(setup_params.fill_touch_control);
    next.fill_touch_control.SetTarget(set_target_params.fill_touch_control,
                                      kInteractFadeInDuration);
    next.edge_touch_response.Setup(setup_params.edge_touch_response);
    next.edge_touch_response.SetTarget(set_target_params.edge_touch_response,
                                       kInteractFadeInDuration);
    next.edge_falloff_color.Setup(setup_params.edge_falloff_color);
    next.edge_falloff_color.SetTarget(set_target_params.edge_falloff_color,
                                      kInteractFadeInDuration);
    next.edge_cutoff_color.Setup(setup_params.edge_cutoff_color);
    next.edge_cutoff_color.SetTarget(set_target_params.edge_cutoff_color,
                                     kInteractFadeInDuration);
    next.fill_touch_response.Setup(setup_params.fill_touch_response);
    next.fill_touch_response.SetTarget(set_target_params.fill_touch_response,
                                       kInteractFadeInDuration);
    next.fill_falloff_color.Setup(setup_params.fill_falloff_color);
    next.fill_falloff_color.SetTarget(set_target_params.fill_falloff_color,
                                      kInteractFadeInDuration);
    next.fill_cutoff_color.Setup(setup_params.fill_cutoff_color);
    next.fill_cutoff_color.SetTarget(set_target_params.fill_cutoff_color,
                                     kInteractFadeInDuration);
    next.foot_size.Setup(state.next_size.value());
    next.foot_fraction.Setup(1.f);
    next.last_interaction_time = delta_time.GetElapsedTime();
    next.is_footprint_primary_receiver = false;
    next.is_footprint_secondary_receiver = false;
    return next;
  }
  return {};
}

FootprintInteractionStates::Machine::OptionalState Footprint::UpdateActive(
    FootprintInteractionStates::Active& state, const imp::FrameTime& delta_time,
    const InteractionMode& interaction) {
  if (!IsVisibleBasedOnFlags(interaction)) {
    //  Fade out.
    state.alpha.SetTarget(0, kUnselectFadeOutDuration);
    state.edge_touch_control.SetTarget(kParamsHidden.edge_touch_control,
                                       kUnselectFadeOutDuration);
    state.edge_touch_response.SetTarget(kParamsHidden.edge_touch_response,
                                        kUnselectFadeOutDuration);
    state.edge_falloff_color.SetTarget(kParamsHidden.edge_falloff_color,
                                       kUnselectFadeOutDuration);
    state.edge_cutoff_color.SetTarget(kParamsHidden.edge_cutoff_color,
                                      kUnselectFadeOutDuration);
    state.fill_touch_control.SetTarget(kParamsHidden.fill_touch_control,
                                       kUnselectFadeOutDuration);
    state.fill_touch_response.SetTarget(kParamsHidden.fill_touch_response,
                                        kUnselectFadeOutDuration);
    state.fill_falloff_color.SetTarget(kParamsHidden.fill_falloff_color,
                                       kUnselectFadeOutDuration);
    state.fill_cutoff_color.SetTarget(kParamsHidden.fill_cutoff_color,
                                      kUnselectFadeOutDuration);
  } else {
    bool show_pointer_on_footprint = false;
    bool is_footprint_receiver = state.is_footprint_primary_receiver ||
                                 state.is_footprint_secondary_receiver;

    ShaderParameters params;

    if (!kDisablePointerGlow) {
      if (interaction.TestGaze(InteractionMode::GazeMode::kGaze)) {
        // When we are gazing, only show the pointer if translating.
        show_pointer_on_footprint = interaction.TestTransform(
            InteractionMode::TransformMode::kTranslate);
      } else if (is_footprint_receiver) {
        // In pointer mode, show the pointer if hovering or pressing.
        show_pointer_on_footprint =
            interaction.TestPointer(InteractionMode::PointerMode::kHover) ||
            interaction.TestPointer(InteractionMode::PointerMode::kPress);
      }
      // Because UX spec has sometimes called for the footprint
      // to be solid and visible by default,
      // we need to reset the receiver flags of the state manually
      // if no pointer upon footprint was flagged this time.
      if (!show_pointer_on_footprint) {
        state.is_footprint_primary_receiver = false;
        state.is_footprint_secondary_receiver = false;
      }
    }

    if (kDisablePointerGlow) {
      if (interaction.TestPointer(InteractionMode::PointerMode::kPress) &&
          IsSnapMode(SnapMode::kSnappable)) {
        params = kParamsIsSnappable;
      } else if (interaction.TestPointer(
                     InteractionMode::PointerMode::kPress) &&
                 (is_footprint_receiver ||
                  interaction.TestGaze(InteractionMode::GazeMode::kGaze))) {
        params = kParamsActivePressNoPointer;
      } else {
        params = kParamsActiveNoPointer;
      }
    } else {  // Legacy behavior.
      if (show_pointer_on_footprint) {
        params = kParamsActiveOverFootprint;
      } else {
        params = kParamsActiveOverModel;
      }
    }

    if (state.edge_touch_control.GetTarget() != params.edge_touch_control) {
      state.edge_touch_control.SetTarget(params.edge_touch_control,
                                         kInteractFadeInDuration);
    }
    if (state.fill_touch_control.GetTarget() != params.fill_touch_control) {
      state.fill_touch_control.SetTarget(params.fill_touch_control,
                                         kInteractFadeInDuration);
    }
    if (state.edge_touch_response.GetTarget() != params.edge_touch_response) {
      state.edge_touch_response.SetTarget(params.edge_touch_response,
                                          kInteractFadeInDuration);
    }
    if (state.fill_touch_response.GetTarget() != params.fill_touch_response) {
      state.fill_touch_response.SetTarget(params.fill_touch_response,
                                          kInteractFadeInDuration);
    }
    if (state.edge_falloff_color.GetTarget() != params.edge_falloff_color) {
      state.edge_falloff_color.SetTarget(params.edge_falloff_color,
                                         kInteractFadeInDuration);
    }
    if (state.edge_cutoff_color.GetTarget() != params.edge_cutoff_color) {
      state.edge_cutoff_color.SetTarget(params.edge_cutoff_color,
                                        kInteractFadeInDuration);
    }
    if (state.fill_falloff_color.GetTarget() != params.fill_falloff_color) {
      state.fill_falloff_color.SetTarget(params.fill_falloff_color,
                                         kInteractFadeInDuration);
    }
    if (state.fill_cutoff_color.GetTarget() != params.fill_cutoff_color) {
      state.fill_cutoff_color.SetTarget(params.fill_cutoff_color,
                                        kInteractFadeInDuration);
    }
  }

  if (!state.foot_fraction.IsAtTarget() || !state.foot_size.IsAtTarget()) {
    state.foot_fraction.Step(delta_time.GetDeltaTime());
    state.foot_size.Step(delta_time.GetDeltaTime());
    UpdateFootBonesAndBounds(state.foot_size.Get(), state.foot_fraction.Get());
  }
  // Make sure the footprint height stays the same at far distances.
  MaintainThickness();

  bool alpha_stopping = false;
  if (!state.alpha.IsAtTarget()) {
    alpha_stopping = state.alpha.Step(delta_time.GetDeltaTime());
  }

  state.edge_touch_control.Step(delta_time.GetDeltaTime());
  state.edge_touch_response.Step(delta_time.GetDeltaTime());
  state.edge_falloff_color.Step(delta_time.GetDeltaTime());
  state.edge_cutoff_color.Step(delta_time.GetDeltaTime());

  state.fill_touch_control.Step(delta_time.GetDeltaTime());
  state.fill_touch_response.Step(delta_time.GetDeltaTime());
  state.fill_falloff_color.Step(delta_time.GetDeltaTime());
  state.fill_cutoff_color.Step(delta_time.GetDeltaTime());

  imp::float4 global_multiplier =
      imp::float4(imp::float3(1.f), state.alpha.Get());

  edge_material_->SetTouchControl(state.edge_touch_control.Get());
  edge_material_->SetTouchResponse(state.edge_touch_response.Get() *
                                   global_multiplier);
  edge_material_->SetFalloffColor(state.edge_falloff_color.Get());
  edge_material_->SetCutoffColor(state.edge_cutoff_color.Get());

  fill_material_->SetTouchControl(state.fill_touch_control.Get());
  fill_material_->SetTouchResponse(state.fill_touch_response.Get() *
                                   global_multiplier);
  fill_material_->SetFalloffColor(state.fill_falloff_color.Get());
  fill_material_->SetCutoffColor(state.fill_cutoff_color.Get());

  auto footprint_node = FootprintNode();
  auto primary_touch_point =
      footprint_node->WorldFromLocalPoint(state.primary_touch_point);
  auto secondary_touch_point =
      footprint_node->WorldFromLocalPoint(state.secondary_touch_point);
  edge_material_->SetPrimaryTouchPoint(primary_touch_point);
  fill_material_->SetPrimaryTouchPoint(primary_touch_point);
  edge_material_->SetSecondaryTouchPoint(secondary_touch_point);
  fill_material_->SetSecondaryTouchPoint(secondary_touch_point);

  if (alpha_stopping && state.alpha.GetTarget() == 0.f) {
    // done fading out...
    FootprintInteractionStates::Hidden next;
    next.has_ever_been_active = true;
    next.next_size.emplace(state.foot_size.Get());
    return next;
  }

  return {};
}

void Footprint::HandleInputEvent(const imp::float3& hit_position,
                                 bool is_primary_touch,
                                 bool is_footprint_receiver) {
  machine_.UpdateWithAlternatives(
      [&hit_position, is_primary_touch, is_footprint_receiver,
       this](FootprintInteractionStates::Active& state)
          -> InteractionMachine::OptionalState {
        if (is_primary_touch) {
          state.primary_touch_point =
              FootprintNode()->LocalFromWorldPoint(hit_position);
          state.is_footprint_primary_receiver = is_footprint_receiver;
        } else {
          state.secondary_touch_point =
              FootprintNode()->LocalFromWorldPoint(hit_position);
          state.is_footprint_secondary_receiver = is_footprint_receiver;
        }
        return {};
      },
      [](auto& state) -> InteractionMachine::OptionalState { return {}; });
}

imp::float2 Footprint::GetFootprintSize() {
  return machine_.ApplyWithAlternatives(
      [](FootprintInteractionStates::Active& state) {
        return state.foot_size.Get();
      },
      [this](FootprintInteractionStates::Hidden& state) {
        return state.next_size.value_or(RetrieveSizeFromModel());
      },
      [this](auto& state) { return RetrieveSizeFromModel(); });
}

void Footprint::SetColliderEnabled(bool is_enabled) {
  if (auto scene = footprint_node_->GetComponent<imp::GltfScene>()) {
    scene->ForAllNodes([is_enabled](imp::NodeHandle node) {
      if (node->GetComponent<imp::GltfCollider>().IsValid()) {
        node->GetComponent<imp::GltfCollider>()->SetEnabled(is_enabled);
      }
    });
  }
}

bool Footprint::IsSnapMode(SnapMode snap_mode) const {
  return snap_mode_ == snap_mode;
}

void Footprint::SetSnapMode(SnapMode snap_mode) {
  if (IsSnapMode(snap_mode)) {
    return;
  }
  snap_mode_ = snap_mode;
}

bool Footprint::IsSnappedOrSnapping() const {
  return IsSnapMode(SnapMode::kSnappedToPlane) ||
         IsSnapMode(SnapMode::kSnappingToPlane) ||
         IsSnapMode(SnapMode::kLiftingOffPlane);
}

void Footprint::OnStateChange(const InteractionMachine& machine,
                              const InteractionMachine::State& current_state,
                              const InteractionMachine::State& next_state) {
  if (std::holds_alternative<FootprintInteractionStates::Hidden>(next_state)) {
    // Hide the asset when entering Hidden
    footprint_model_->GetModelRoot()->SetEnabled(true);
    footprint_node_->SetEnabled(true);
  } else if (std::holds_alternative<FootprintInteractionStates::Active>(
                 next_state)) {
    const FootprintInteractionStates::Active& next_active =
        std::get<FootprintInteractionStates::Active>(next_state);
    // Show the asset when entering Active
    footprint_model_->GetModelRoot()->SetEnabled(true);
    footprint_node_->SetEnabled(true);
    // Apply the initial size specified for the footprint.
    UpdateFootBonesAndBounds(next_active.foot_size.Get(),
                             next_active.foot_fraction.Get());
  }
}

imp::float2 Footprint::RetrieveSizeFromModel() {
  if (!model_node_) {
    return imp::float2(0, 0);
  }
  if (initial_model_bounds_.isEmpty()) {
    initial_model_bounds_ =
        model_node_->GetOrAddComponent<GltfBounds>()->GetLocalBounds();
  }
  imp::float3 size =
      model_node_->GetLocalScale() * 2.0f * initial_model_bounds_.halfExtent;
  return imp::float2(size.x, size.z);
}

void Footprint::UpdateFootBonesAndBounds(float2 foot_size,
                                         float foot_fraction) {
  auto local_scale = footprint_node_->GetLocalScale();
  auto model = footprint_node_->GetComponent<imp::GltfRenderer>();

  auto actual_half_extents =
      local_scale * imp::float3(foot_size.x * 0.5f, 0, foot_size.y * 0.5f);
  auto min_half_extents =
      imp::float3(kTouchAreaHalfExtentsMin, 0, kTouchAreaHalfExtentsMin);

  auto card_inclusion =
      imp::float3(kCardInclusionFraction, 0, kCardInclusionFraction) *
      kDefaultScale;

  auto model_space_inner_half_extents =
      foot_fraction *
      (max(actual_half_extents, min_half_extents) - card_inclusion) /
      kDefaultScale;

  for (int i = 0; i < kCornerCount; ++i) {
    auto offset = kCorners[i] * model_space_inner_half_extents;

    auto bone_node = model->GetOrCreateNode(kBoneNames[i]);
    auto bone_trs = imp::Transform<float>(bone_node->GetLocalTrs());
    bone_trs.translation = offset;
    bone_node->SetLocalTrs(bone_trs.AsMat4());
  }

  model->ScheduleSkinningUpdate();

  // Extend by 1 in X and Z to account for the cards rendering the footprint.
  auto new_bounds = filament::Box{
      imp::float3(0), model_space_inner_half_extents +
                          imp::float3(1.0f - kCardInclusionFraction, 0.01f,
                                      1.0f - kCardInclusionFraction)};
  model->SetRenderBounds(new_bounds);
}

void Footprint::MaintainThickness() {
  auto local_scale = footprint_node_->GetLocalScale();
  auto model = footprint_node_->GetComponent<imp::GltfRenderer>();

  // Ensure footprint height stays the same at far distances.
  imp::Box local_bounds =
      footprint_node_->GetOrAddComponent<GltfBounds>()->GetLocalBounds();
  auto model_root = model->GetModelRoot();

  // Make footprint keep same height at any distance.
  auto camera = GetView().GetCameraManager().GetCamera();
  imp::mat4f world_from_footprint = footprint_node_->GetWorldTrs();
  imp::mat4f world_from_camera = camera->GetNode()->GetWorldTrs();
  imp::float3 camera_position = (world_from_camera * imp::kZero3).xyz;
  imp::float3 footprint_position = (world_from_footprint * imp::kZero3).xyz;
  float current_distance = length(camera_position - footprint_position);

  float decrement = (std::clamp((current_distance - kDistanceBasedYBuffer),
                                0.0f, kDistanceBufferRange)) /
                    kDistanceBufferRange;
  float distance_based_y_modifier =
      current_distance * std::max(1.f - (decrement), kDistanceBufferMinimum);
  auto scale = kFootprintScale * (imp::float3(kDefaultScale) / local_scale);
  float y_offset = scale.y * current_distance;

  scale.y *= distance_based_y_modifier;

  model_root->SetLocalPosition(y_offset * local_bounds.halfExtent.y *
                               imp::float3(0.f, -1.f, 0.f));
  model_root->SetLocalScale(scale);
}

}  // namespace svxr
