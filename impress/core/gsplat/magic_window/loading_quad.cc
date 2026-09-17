// Copyright 2026 Google LLC
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

#include "core/gsplat/magic_window/loading_quad.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "core/common/log.h"
#include "core/input/pointer_event_processor.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_param_value.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_factory.h"
#include "core/ncsb/node.h"
#include "core/render/mesh_renderer.h"
#include "core/split_engine/materials/split_engine_generic_material.h"
#include "core/view/utils/frame_time.h"

namespace imp {
namespace {
// Figma spec:
// (broken link)
constexpr float kDefaultFadeDurationSeconds = 0.5f;
constexpr float3 kLoadingQuadColor = {0.2392f, 0.2745f, 0.3020f};
constexpr float kMeshOpacity = 0.8f;
}  // namespace

float LoadingQuad::GetMaxOpacity() const { return kMeshOpacity; }

void LoadingQuad::Setup(imp::float2 size, float corner_radius) {
  if (GetView().GetSplitEngineSerializer() == nullptr) {
    IMP_LOG(imp::ERROR) << "Split Engine is required for LoadingQuad.";
    return;
  }

  imp::GenericMaterialSpec spec{
      imp::schemas::GenericMaterialLightingModel::Unlit,
      imp::schemas::GenericMaterialBlendMode::Transparent,
      imp::schemas::GenericMaterialDoubleSidedMode::SingleSided,
      imp::schemas::GenericMaterialDepthClearMaterial::Disabled};

  imp::split_engine::SplitEngineGenericMaterial::Create(GetView(), spec)
      .Then([this, size, corner_radius](imp::GenericMaterialPtr material) {
        OnMaterialCreated(std::move(material), size, corner_radius);
      })
      .KeptBy(this);
}

void LoadingQuad::OnMaterialCreated(imp::GenericMaterialPtr material,
                                    imp::float2 size, float corner_radius) {
  imp::CreateQuadSettings settings;
  settings.size = size;
  settings.corner_radius = corner_radius;
  settings.color = imp::float4(1.0f, 1.0f, 1.0f, 1.0f);

  renderer_ = GetNode()->AddComponent<imp::MeshRenderer>();
  renderer_->SetMesh(
      GetNode()->GetView().GetMeshFactory().CreatePanel(settings));
  material_ = std::move(material);
  material_->SetBaseColorFactor(
      imp::float4(kLoadingQuadColor, current_opacity_));
  renderer_->SetMaterial(material_->GetMaterial());
}

void LoadingQuad::Cleanup() {
  if (renderer_) {
    GetNode()->RemoveComponent<imp::MeshRenderer>();
  }
  material_.reset();
}

void LoadingQuad::Fade(float start_opacity, float end_opacity) {
  GetNode()->SetEnabled(true);
  if (!is_animating_) {
    current_opacity_ = start_opacity;
  }
  fade_start_opacity_ = current_opacity_;
  fade_end_opacity_ = end_opacity;

  // Calculate speed based on the default duration for a full transition
  // (from 0 to kMeshOpacity). This avoids arbitrary hardcoded speeds.
  const float fade_speed = kMeshOpacity / kDefaultFadeDurationSeconds;
  const float distance = std::abs(fade_end_opacity_ - fade_start_opacity_);
  fade_duration_seconds_ = distance / fade_speed;
  fade_timer_seconds_ = 0.0f;
  is_animating_ = true;
}

void LoadingQuad::Update(const imp::FrameTime& frame_time) {
  if (!is_animating_) return;
  if (material_ == nullptr) return;

  // Cap delta time to prevent sudden jumps during hitches (e.g. GMM
  // initialization)
  const float delta_seconds = std::min(frame_time.GetDeltaSeconds(), 0.1f);
  fade_timer_seconds_ += delta_seconds;

  if (std::isnan(fade_duration_seconds_) ||
      AlmostEqual(fade_duration_seconds_, 0.0f)) {
    current_opacity_ = fade_end_opacity_;
    is_animating_ = false;
  } else {
    const float progress = fade_timer_seconds_ / fade_duration_seconds_;
    if (progress >= 1.0f) {
      current_opacity_ = fade_end_opacity_;
      is_animating_ = false;
    } else {
      current_opacity_ = fade_start_opacity_ +
                         progress * (fade_end_opacity_ - fade_start_opacity_);
    }
  }

  material_->SetBaseColorFactor(
      imp::float4(kLoadingQuadColor, current_opacity_));

  if (!is_animating_ && AlmostEqual(fade_end_opacity_, 0.0f)) {
    GetNode()->SetEnabled(false);
  }
}

}  // namespace imp
