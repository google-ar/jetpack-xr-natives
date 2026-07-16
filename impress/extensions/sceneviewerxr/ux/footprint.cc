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
#include <bitset>
#include <cstdlib>
#include <memory>
#include <optional>
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
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/assets/footprint_assets.h"
#include "extensions/sceneviewerxr/ux/gltf_bounds.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
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
constexpr auto kScaleHandleInteractionDistance = 0.035f;
constexpr auto kScaleHandleAnimationSlideDistance = 0.08f;
// Adjust this to lower or raise the handle relative to the footprint plane.
constexpr float kScaleHandleYOffset = -0.03f;

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
constexpr auto kScaleHandleAnimationDuration = absl::Milliseconds(150);
constexpr auto kScaleHandleGrabAnimationDuration = absl::Milliseconds(100);
constexpr float kScaleHandleGrabScaleMultiplier = 0.85f;
constexpr int kFootprintGraceFrames = 2;

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

  // If a rotate is occurring, be sure to hide the footprint.
  if (interaction_data.TestTransform(InteractionMode::TransformMode::kRotate)) {
    return false;
  } else if (interaction_data.TestTransform(
                 InteractionMode::TransformMode::kTranslate)) {
    // If we are translating, be sure to show the footprint.
    return true;
  } else if (interaction_data.TestTransform(
                 InteractionMode::TransformMode::kScale)) {
    // If we are scaling, only show footprint/handles for 1-handed scale.
    return interaction_data.GetScaleHandle().has_value() &&
           *interaction_data.GetScaleHandle() != ScaleHandle::kTwoHanded;
  }

  // If hovering, the footprint is visible.
  if (interaction_data.TestPointer(InteractionMode::PointerMode::kHover) ||
      interaction_data.TestPointer(InteractionMode::PointerMode::kPress)) {
    return true;
  }

  // Make footprint solid by default in selected state.
  return true;
}

FootprintInteractionStates::Active CreateActiveStateWithDefaults(
    float target_alpha, absl::Duration duration, imp::float2 foot_size) {
  FootprintInteractionStates::Active next;
  next.alpha.Setup(0.f);
  next.alpha.SetTarget(target_alpha, duration);
  next.ignore_visibility_flags = true;

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

  next.foot_size.Setup(foot_size);
  next.foot_fraction.Setup(1.f);
  next.last_interaction_time = absl::ZeroDuration();
  next.is_footprint_primary_receiver = false;
  next.is_footprint_secondary_receiver = false;
  next.scale_handles_visibility.reset();
  for (int i = 0; i < 4; ++i) {
    next.scale_handle_animations[i].Setup(0.0f);
    next.scale_handle_pressed_animations[i].Setup(0.0f);
  }
  return next;
}

}  // namespace

Footprint::Footprint()
    : machine_(FootprintInteractionStates::Initialized{}, this) {}

Footprint::~Footprint() = default;

imp::Future<absl::Status> Footprint::Setup(
    imp::NodeHandle model_node, std::optional<imp::Box> initial_bounds) {
  auto& view = GetNode()->GetView();
  model_node_ = model_node;
  if (initial_bounds.has_value()) {
    initial_model_bounds_ = *initial_bounds;
  }

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
  auto scale_handle_future =
      asset_manager.LoadGltfAsset(imp::kScaleHandleGlb, options);

  return asset_future
      .Merge(edge_material_future, fill_material_future, scale_handle_future)
      .Then([this](
                std::tuple<imp::AssetPtr<imp::GltfAsset>,
                           std::unique_ptr<android_xr::SVXRFootprintMaterial>,
                           std::unique_ptr<android_xr::SVXRFootprintMaterial>,
                           imp::AssetPtr<imp::GltfAsset>>
                    result) mutable -> absl::Status {
        auto& [gltf_asset, edge_material, fill_material, scale_handle_asset] =
            result;

        return Setup(std::move(gltf_asset), std::move(edge_material),
                     std::move(fill_material), std::move(scale_handle_asset));
      });
}

absl::Status Footprint::Setup(
    imp::AssetPtr<imp::GltfAsset> footprint_asset,
    std::unique_ptr<android_xr::SVXRFootprintMaterial> edge_material,
    std::unique_ptr<android_xr::SVXRFootprintMaterial> fill_material,
    imp::AssetPtr<imp::GltfAsset> scale_handle_asset) {
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
  if (skin.sampled_joints.size() != kCornerCount) {
    return absl::InternalError(absl::StrFormat(
        "Expected %d bones, not %d", kCornerCount, skin.sampled_joints.size()));
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

  for (int corner_index = 0; corner_index < kCornerCount; ++corner_index) {
    auto handle_node = GetView().CreateNode();
    handle_node->SetName("ScaleHandle");
    handle_node->SetParent(footprint_node_);

    auto visual_node = GetView().CreateNode();
    visual_node->SetName("ScaleHandleVisual");
    visual_node->SetParent(handle_node);

    auto renderer =
        visual_node->AddComponent<imp::GltfRenderer>(scale_handle_asset);
    handle_node->AddComponent<imp::BoxCollider>(renderer->GetLocalBounds());

    // Disable visual initially
    visual_node->SetEnabled(false);

    scale_handles_[corner_index] = handle_node;
    scale_handle_visuals_[corner_index] = visual_node;

    // Ensure scene nodes are enabled if needed (on visual)
    if (auto scene = visual_node->GetComponent<imp::GltfScene>()) {
      scene->ForAllNodes([](imp::NodeHandle node) {
        if (auto collider = node->GetComponent<imp::GltfCollider>()) {
          collider->SetEnabled(true);
        }
      });
    }
  }

  return absl::OkStatus();
}

void Footprint::OnInteractionMachineInitialized() {
  machine_.UpdateWithAlternatives(
      [this](FootprintInteractionStates::Initialized& state)
          -> InteractionMachine::OptionalState {
        state.is_intialize_complete = true;
        UpdateFootBonesAndBounds(GetFootprintSize(), 1);
        return {};
      },
      [](FootprintInteractionStates::Hidden& state)
          -> InteractionMachine::OptionalState { return {}; },
      [](FootprintInteractionStates::Active& state)
          -> InteractionMachine::OptionalState { return {}; });
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

void Footprint::Show(std::optional<absl::Duration> custom_fade_duration) {
  absl::Duration fade_duration =
      custom_fade_duration.value_or(kSpawnFadeInDuration);
  machine_.UpdateWithAlternatives(
      [this, fade_duration](FootprintInteractionStates::Initialized& state)
          -> InteractionMachine::OptionalState {
        // When forced to show during initialization, set up right away.
        state.is_intialize_complete = true;
        UpdateFootBonesAndBounds(GetFootprintSize(), 1.0f);

        return CreateActiveStateWithDefaults(1.0f, fade_duration,
                                             RetrieveSizeFromModel());
      },
      [this, fade_duration](FootprintInteractionStates::Hidden& state)
          -> InteractionMachine::OptionalState {
        if (!state.next_size.has_value()) {
          state.next_size = RetrieveSizeFromModel();
        }

        return CreateActiveStateWithDefaults(1.0f, fade_duration,
                                             state.next_size.value());
      },
      [fade_duration](FootprintInteractionStates::Active& state)
          -> InteractionMachine::OptionalState {
        if (state.alpha.GetTarget() != 1.f) {
          state.alpha.SetTarget(1.f, fade_duration);
          state.ignore_visibility_flags = true;
        }
        return {};
      });
}

void Footprint::SetAlpha(float target_alpha) {
  machine_.UpdateWithAlternatives(
      [](FootprintInteractionStates::Initialized& state)
          -> InteractionMachine::OptionalState { return {}; },
      [](FootprintInteractionStates::Hidden& state)
          -> InteractionMachine::OptionalState { return {}; },
      [target_alpha](FootprintInteractionStates::Active& state)
          -> InteractionMachine::OptionalState {
        state.alpha.Setup(target_alpha);
        state.alpha.SetTarget(target_alpha, absl::ZeroDuration());
        return {};
      });
}

void Footprint::Hide() {
  machine_.UpdateWithAlternatives(
      [](FootprintInteractionStates::Initialized& state)
          -> InteractionMachine::OptionalState { return {}; },
      [](FootprintInteractionStates::Hidden& state)
          -> InteractionMachine::OptionalState { return {}; },
      [](FootprintInteractionStates::Active& state)
          -> InteractionMachine::OptionalState {
        state.alpha.SetTarget(0, kUnselectFadeOutDuration);
        state.ignore_visibility_flags = true;
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
        if (footprint_model_ && state.is_intialize_complete) {
          return FootprintInteractionStates::Hidden{};
        }
        return {};
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
    next.scale_handles_visibility.reset();
    for (int i = 0; i < 4; ++i) {
      next.scale_handle_animations[i].Setup(0.0f);
      next.scale_handle_pressed_animations[i].Setup(0.0f);
    }
    return next;
  }
  return {};
}

FootprintInteractionStates::Machine::OptionalState Footprint::UpdateActive(
    FootprintInteractionStates::Active& state, const imp::FrameTime& delta_time,
    const InteractionMode& interaction) {
  // When we are scaling via a scale handle, we are in one-handed scaling mode.
  std::optional<ScaleHandle> scale_handle = interaction.GetScaleHandle();
  bool is_one_handed_scale =
      interaction.TestTransform(InteractionMode::TransformMode::kScale) &&
      scale_handle.has_value() && *scale_handle != ScaleHandle::kTwoHanded;
  if (footprint_model_) {
    // If we are in one-handed scaling mode, we want to hide the footprint so
    // only the scale handle is visible.
    footprint_model_->SetEnabled(!is_one_handed_scale);
  }

  // Handle visibility logic
  std::bitset<kCornerCount> next_visibility =
      CalculateScaleHandleVisibility(interaction, state);
  bool handles_visible = next_visibility.any();

  bool ignore_visibility = state.ignore_visibility_flags;
  if (ignore_visibility && state.alpha.IsAtTarget()) {
    state.ignore_visibility_flags = false;
    ignore_visibility = false;
  }

  if (ignore_visibility) {
    // Skip visibility checks while programmatic animation is running.
  } else if (!IsVisibleBasedOnFlags(interaction) || handles_visible) {
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
    // Ensure the footprint is visible.
    if (state.alpha.GetTarget() != 1.f) {
      state.alpha.SetTarget(1.f, kInteractFadeInDuration);
    }
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
  edge_material_->SetFalloffColor(state.edge_falloff_color.Get() *
                                  global_multiplier);
  edge_material_->SetCutoffColor(state.edge_cutoff_color.Get() *
                                 global_multiplier);

  fill_material_->SetTouchControl(state.fill_touch_control.Get());
  fill_material_->SetTouchResponse(state.fill_touch_response.Get() *
                                   global_multiplier);
  fill_material_->SetFalloffColor(state.fill_falloff_color.Get() *
                                  global_multiplier);
  fill_material_->SetCutoffColor(state.fill_cutoff_color.Get() *
                                 global_multiplier);

  auto footprint_node = FootprintNode();
  auto primary_touch_point =
      footprint_node->WorldFromLocalPoint(state.primary_touch_point);
  auto secondary_touch_point =
      footprint_node->WorldFromLocalPoint(state.secondary_touch_point);
  edge_material_->SetPrimaryTouchPoint(primary_touch_point);
  fill_material_->SetPrimaryTouchPoint(primary_touch_point);
  edge_material_->SetSecondaryTouchPoint(secondary_touch_point);
  fill_material_->SetSecondaryTouchPoint(secondary_touch_point);

  if (alpha_stopping && state.alpha.GetTarget() == 0.f && !handles_visible) {
    // done fading out...
    FootprintInteractionStates::Hidden next;
    next.has_ever_been_active = true;
    next.next_size.emplace(state.foot_size.Get());
    return next;
  }

  // Determine if a handle is currently actively dragged for scaling
  bool is_scaling =
      interaction.TestTransform(InteractionMode::TransformMode::kScale);
  std::optional<ScaleHandle> active_handle =
      is_scaling ? interaction.GetScaleHandle() : std::nullopt;

  for (int corner_index = 0; corner_index < kCornerCount; ++corner_index) {
    if (next_visibility[corner_index] !=
        state.scale_handles_visibility[corner_index]) {
      if (next_visibility[corner_index]) {
        state.scale_handle_animations[corner_index].SetTarget(
            1.0f, kScaleHandleAnimationDuration);
      } else {
        state.scale_handle_animations[corner_index].SetTarget(
            0.0f, kScaleHandleAnimationDuration);
        // Force the grab animation to drop when visibility is lost
        state.scale_handle_pressed_animations[corner_index].SetTarget(
            0.0f, absl::ZeroDuration());
      }
    }

    bool is_this_corner_grabbed =
        (active_handle.has_value() &&
         static_cast<int>(*active_handle) == corner_index);
    if (is_this_corner_grabbed &&
        state.scale_handle_pressed_animations[corner_index].GetTarget() !=
            1.0f) {
      state.scale_handle_pressed_animations[corner_index].SetTarget(
          1.0f, kScaleHandleGrabAnimationDuration);
    } else if (!is_this_corner_grabbed &&
               state.scale_handle_pressed_animations[corner_index]
                       .GetTarget() != 0.0f) {
      state.scale_handle_pressed_animations[corner_index].SetTarget(
          0.0f, kScaleHandleGrabAnimationDuration);
    }

    if (scale_handle_visuals_[corner_index]) {
      // The handle visual should be enabled if it is targeted (animating in or
      // already fully out) OR if it is untargeted but still animating away
      // (ramp is > 0.0f).
      bool visually_active =
          next_visibility[corner_index] ||
          state.scale_handle_animations[corner_index].Get() > 0.0f;

      // Make sure our local state matches the desired visual enablement state
      // to avoid redundantly calling SetEnabled on the NdkNode
      if (scale_handle_visuals_[corner_index]->IsEnabled() != visually_active) {
        scale_handle_visuals_[corner_index]->SetEnabled(visually_active);
      }
    }
  }
  state.scale_handles_visibility = next_visibility;

  // We update the handle positions *after* determining their visibility and
  // setting up their initial animation ramps. This ensures that on the very
  // first frame a handle becomes visible, its position is calculated using
  // the new animation state (e.g. anim_t = 0.0) rather than popping at
  // 1.0 for a single frame before the ramp resets.
  // Update handle positions every frame since model_root transform changes
  if (auto model = footprint_node_->GetComponent<imp::GltfRenderer>()) {
    auto model_root = model->GetModelRoot();
    auto model_trs = model_root->GetLocalTrs();
    for (int corner_index = 0; corner_index < kCornerCount; ++corner_index) {
      state.scale_handle_animations[corner_index].Step(
          delta_time.GetDeltaTime());
      state.scale_handle_pressed_animations[corner_index].Step(
          delta_time.GetDeltaTime());

      if (!scale_handles_[corner_index]) continue;
      // 1.0f - kCardInclusionFraction accounts for the visual extent of the
      // corner card beyond the bone position.
      constexpr float kVisualExtent = 1.0f - kCardInclusionFraction;
      auto offset = kCorners[corner_index] *
                    (model_space_inner_half_extents_ +
                     imp::float3(kVisualExtent, 0.f, kVisualExtent));
      offset.y += kScaleHandleYOffset;

      float anim_t = state.scale_handle_animations[corner_index].Get();
      float clamped_t = std::clamp(anim_t, 0.0f, 1.0f);
      // The handle slides outward along a strictly 45-degree angle from the
      // resting offset. We use a fixed distance constraint
      // (kScaleHandleAnimationSlideDistance) and normalize kCorners to get the
      // pure 45-degree directional vector.
      imp::float3 slide_direction = normalize(kCorners[corner_index]);
      auto animated_offset =
          offset - (slide_direction *
                    (kScaleHandleAnimationSlideDistance * (1.0f - clamped_t)));

      // Calculate resting position and animated position in rig space
      auto resting_pos = (model_trs * imp::float4(offset, 1.f)).xyz;
      auto animated_pos = (model_trs * imp::float4(animated_offset, 1.f)).xyz;

      constexpr float kRotations[] = {
          M_PI / 2.0f,        // 90 deg  (bottom-right)
          0.0f,               // 0 deg   (bottom-left)
          M_PI,               // 180 deg (top-right)
          3.0f * M_PI / 2.0f  // 270 deg (top-left)
      };

      // Set the physical handle_node to the resting position so the BoxCollider
      // doesn't slip away from the raycast when animating
      auto handle_trs =
          imp::Transform<float>(scale_handles_[corner_index]->GetLocalTrs());
      handle_trs.translation = resting_pos;
      handle_trs.rotation = imp::quatf::fromAxisAngle(
          imp::float3(0.f, 1.f, 0.f), kRotations[corner_index]);
      scale_handles_[corner_index]->SetLocalTrs(handle_trs.AsMat4());

      // Set the visual child node to animate towards the final position
      // We calculate the delta translation, then un-rotate it by the parent's
      // yaw to align it with the visual node's internal local space.
      imp::quatf inv_handle_rot = imp::quatf::fromAxisAngle(
          imp::float3(0.f, 1.f, 0.f), -kRotations[corner_index]);
      auto visual_trs = imp::Transform<float>(
          scale_handle_visuals_[corner_index]->GetLocalTrs());
      visual_trs.translation = inv_handle_rot * (animated_pos - resting_pos);

      // Apply the grab scale effect
      float pressed_t =
          std::clamp(state.scale_handle_pressed_animations[corner_index].Get(),
                     0.0f, 1.0f);
      float grab_scale =
          imp::lerp(1.0f, kScaleHandleGrabScaleMultiplier, pressed_t);
      visual_trs.scale = imp::float3(grab_scale, grab_scale, grab_scale);

      scale_handle_visuals_[corner_index]->SetLocalTrs(visual_trs.AsMat4());
    }
  }

  UpdateFootprintReceiverGracePeriod(state);
  return {};
}

std::bitset<4> Footprint::CalculateScaleHandleVisibility(
    const InteractionMode& interaction,
    const FootprintInteractionStates::Active& state) {
  std::bitset<kCornerCount> next_visibility;
  if (interaction.TestTransform(InteractionMode::TransformMode::kScale)) {
    std::optional<ScaleHandle> handle = interaction.GetScaleHandle();
    if (handle.has_value() && *handle != ScaleHandle::kTwoHanded) {
      next_visibility[static_cast<int>(*handle)] = true;
    }
  } else if (IsVisibleBasedOnFlags(interaction) &&
             state.is_footprint_primary_receiver &&
             !interaction.TestTransform(
                 InteractionMode::TransformMode::kTranslate)) {
    imp::float3 touch_local = state.primary_touch_point;
    for (int corner_index = 0; corner_index < kCornerCount; ++corner_index) {
      if (!scale_handles_[corner_index]) continue;
      if (!scale_handle_visuals_[corner_index]) continue;

      auto model = footprint_node_->GetComponent<imp::GltfRenderer>();
      if (!model) continue;

      // We calculate the theoretical "resting" position of the handle from
      // scratch, rather than using `scale_handles_[...]->GetLocalPosition()`.
      // This is because the handle's actual localized position is animated
      // inward when it appears. If we used the animated position, the handle
      // would run away from the user's touch point, lose its hover state, and
      // instantly disappear, causing a flicker loop.
      constexpr float kVisualExtent = 1.0f - kCardInclusionFraction;
      auto offset = kCorners[corner_index] *
                    (model_space_inner_half_extents_ +
                     imp::float3(kVisualExtent, 0.f, kVisualExtent));
      imp::float3 handle_pos =
          (model->GetModelRoot()->GetLocalTrs() * imp::float4(offset, 1.f)).xyz;

      float dist = length(imp::float2(touch_local.x, touch_local.z) -
                          imp::float2(handle_pos.x, handle_pos.z));
      if (dist < kScaleHandleInteractionDistance) {
        next_visibility[corner_index] = true;
      }
    }
  }
  return next_visibility;
}

void Footprint::UpdateFootprintReceiverGracePeriod(
    FootprintInteractionStates::Active& state) {
  if (state.footprint_primary_receiver_grace_frames > 0) {
    state.footprint_primary_receiver_grace_frames--;
  }
  state.is_footprint_primary_receiver =
      (state.footprint_primary_receiver_grace_frames > 0);
  state.is_footprint_secondary_receiver = false;
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
          if (is_footprint_receiver) {
            state.footprint_primary_receiver_grace_frames =
                kFootprintGraceFrames;
          }
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
    footprint_model_->SetEnabled(true);
    footprint_node_->SetEnabled(true);
  } else if (std::holds_alternative<FootprintInteractionStates::Active>(
                 next_state)) {
    const FootprintInteractionStates::Active& next_active =
        std::get<FootprintInteractionStates::Active>(next_state);
    // Show the asset when entering Active
    footprint_model_->SetEnabled(true);
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

  model_space_inner_half_extents_ = model_space_inner_half_extents;

  for (int corner_index = 0; corner_index < kCornerCount; ++corner_index) {
    auto offset = kCorners[corner_index] * model_space_inner_half_extents;

    auto bone_node = model->GetOrCreateNode(kBoneNames[corner_index]);
    auto bone_trs = imp::Transform<float>(bone_node->GetLocalTrs());
    bone_trs.translation = offset;
    bone_node->SetLocalTrs(bone_trs.AsMat4());
  }

  if (!model->IsSkinningScheduled()) {
    model->ScheduleSkinningUpdate();
  }

  // Extend by 1 in X and Z to account for the cards rendering the footprint.
  auto new_bounds = filament::Box{
      imp::float3(0), model_space_inner_half_extents +
                          imp::float3(1.0f - kCardInclusionFraction, 0.01f,
                                      1.0f - kCardInclusionFraction)};
  model->SetRenderBounds(new_bounds);

  // Update scale handles
  // Note: Handle positions are also updated in UpdateActive to track model_root
  // animation/thickness changes
  auto model_root = model->GetModelRoot();
  auto model_trs = model_root->GetLocalTrs();
  for (int corner_index = 0; corner_index < kCornerCount; ++corner_index) {
    if (!scale_handles_[corner_index]) continue;
    auto offset = kCorners[corner_index] * model_space_inner_half_extents;
    auto handle_pos = (model_trs * imp::float4(offset, 1.f)).xyz;

    auto handle_trs =
        imp::Transform<float>(scale_handles_[corner_index]->GetLocalTrs());
    handle_trs.translation = handle_pos;
    scale_handles_[corner_index]->SetLocalTrs(handle_trs.AsMat4());
  }
}

void Footprint::MaintainThickness() {
  auto local_scale = footprint_node_->GetLocalScale();
  auto model = footprint_node_->GetComponent<imp::GltfRenderer>();

  // Ensure footprint height stays the same at far distances.
  imp::Box local_bounds =
      footprint_node_->GetComponent<GltfBounds>()->GetLocalBounds();
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

std::optional<ScaleHandle> Footprint::GetTargetedScaleHandle(
    imp::NodeHandle receiver, const imp::float3& world_hit_pos) const {
  imp::float3 local_hit_pos =
      footprint_node_->LocalFromWorldPoint(world_hit_pos);
  for (int i = 0; i < scale_handles_.size(); ++i) {
    const auto& handle = scale_handles_[i];
    if (!handle) continue;
    imp::NodeHandle node = receiver;
    while (node) {
      if (node == handle) {
        return static_cast<ScaleHandle>(i);
      }
      node = node->GetParent();
      // Optimization: Stop if we reach the footprint node.
      if (node == footprint_node_) {
        break;
      }
    }

    // Check proximity
    imp::float3 handle_pos = handle->GetLocalPosition();
    float dist = length(imp::float2(local_hit_pos.x, local_hit_pos.z) -
                        imp::float2(handle_pos.x, handle_pos.z));
    if (dist < kScaleHandleInteractionDistance)
      return static_cast<ScaleHandle>(i);
  }
  return std::nullopt;
}

}  // namespace svxr
