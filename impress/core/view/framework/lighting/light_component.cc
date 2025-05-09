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

#include "core/view/framework/lighting/light_component.h"

#include <algorithm>
#include <cstdint>

#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/LightManager.h"
#include "core/math/quat.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/lighting/light_state.proto.imp.h"

namespace imp {
using LightInstance = filament::LightManager::Instance;
using FilamentLightType = filament::LightManager::Type;

constexpr LightState::Type kDefaultType = LightState::DIRECTIONAL;
constexpr bool kDefaultIsShadowCastingDisabled = false;
constexpr float kDefaultFalloff = 1.0f;
constexpr int kDefaultShadowMapSize = 1024;
constexpr bool kDefaultShadowLispsm = true;
constexpr float kDefaultShadowConstantBias = 0.001f;
constexpr float kDefaultShadowNormalBias = 1.0f;

// Fix the direction of the light to forward, and depend on the direction of
// the node to control the direction of the light.
constexpr float3 kDirection = {0.0f, 0.0f, -1.0f};

const float3 LightComponent::kDefaultColor = {1.0f};
const float LightComponent::kDefaultIntensity = 100000.0f;

FilamentLightType LightStateTypeToFilamentLightType(LightState::Type type) {
  switch (type) {
    case LightState::Type::SUN:
      return FilamentLightType::SUN;
    case LightState::Type::DIRECTIONAL:
      return FilamentLightType::DIRECTIONAL;
    case LightState::Type::POINT:
      return FilamentLightType::POINT;
    case LightState::Type::FOCUSED_SPOT:
      return FilamentLightType::FOCUSED_SPOT;
    case LightState::Type::SPOT:
      return FilamentLightType::SPOT;
  }
}

LightComponent::LightComponent() {
  // Set default values for the state_.
  // May be overridden by serialized data.
  state_.type = kDefaultType;
  state_.color = kDefaultColor;
  state_.intensity = kDefaultIntensity;
  state_.is_shadow_casting_disabled = kDefaultIsShadowCastingDisabled;
  state_.falloff = kDefaultFalloff;
  state_.spot_cone_inner_outer = {filament::math::F_PI, filament::math::F_PI};
  state_.shadow_map_size = kDefaultShadowMapSize;
  state_.shadow_lispsm = kDefaultShadowLispsm;
  state_.shadow_constant_bias = kDefaultShadowConstantBias;
  state_.shadow_normal_bias = kDefaultShadowNormalBias;
}

void LightComponent::Setup() { CreateLight(); }

void LightComponent::Setup(Type type) {
  state_.type = type;
  CreateLight();
}

void LightComponent::OnActiveStatusChanged(bool is_active) {
  auto& lm = GetLightManager();
  LightInstance instance = lm.getInstance(GetEntity());
  lm.setIntensity(instance, is_active ? state_.intensity : 0);
}

void LightComponent::OnIsfStateChanged() {
  auto& lm = GetLightManager();
  float3 direction = lm.getDirection(lm.getInstance(GetEntity()));
  DestroyLight();
  CreateLight();
  lm.setDirection(lm.getInstance(GetEntity()), direction);
}

void LightComponent::Cleanup() { DestroyLight(); }

LightComponent::Type LightComponent::GetType() const { return state_.type; }

void LightComponent::SetType(Type type) {
  state_.type = type;
  auto& lm = GetLightManager();
  float3 direction = lm.getDirection(lm.getInstance(GetEntity()));
  DestroyLight();
  CreateLight();
  lm.setDirection(lm.getInstance(GetEntity()), direction);
}

void LightComponent::SetColor(const float3& color) {
  auto& lm = GetLightManager();
  LightInstance instance = lm.getInstance(GetEntity());
  lm.setColor(instance, color);
  state_.color = color;
}

const float3& LightComponent::GetColor() const { return state_.color; }

void LightComponent::SetIntensity(float intensity) {
  state_.intensity = intensity;
  if (IsActive()) {
    auto& lm = GetLightManager();
    LightInstance instance = lm.getInstance(GetEntity());
    lm.setIntensity(instance, intensity);
  }
}

float LightComponent::GetIntensity() const { return state_.intensity; }

void LightComponent::SetShadowCastingDisabled(bool is_shadow_casting_disabled) {
  auto& lm = GetLightManager();
  LightInstance instance = lm.getInstance(GetEntity());
  lm.setShadowCaster(instance, !is_shadow_casting_disabled);
  state_.is_shadow_casting_disabled = is_shadow_casting_disabled;
}

bool LightComponent::IsShadowCastingDisabled() const {
  return state_.is_shadow_casting_disabled;
}

void LightComponent::SetFalloff(float falloff) {
  auto& lm = GetLightManager();
  LightInstance instance = lm.getInstance(GetEntity());
  lm.setFalloff(instance, falloff);
  state_.falloff = falloff;
}

float LightComponent::GetFalloff() const { return state_.falloff; }

void LightComponent::SetSpotCone(const float2& inner_outer) {
  auto& lm = GetLightManager();
  LightInstance instance = lm.getInstance(GetEntity());
  lm.setSpotLightCone(instance, inner_outer.x, inner_outer.y);
  state_.spot_cone_inner_outer = inner_outer;
}

float2 LightComponent::GetSpotCone() const {
  return state_.spot_cone_inner_outer;
}

void LightComponent::SetShadowTransform(const imp::quatf& transform) {
  state_.shadow_transform = transform;
  UpdateShadowOptions();
}

imp::quatf LightComponent::GetShadowTransform() const {
  return state_.shadow_transform.value_or(imp::quatf(1.0f));
}

void LightComponent::SetShadowMapSize(int map_size) {
  state_.shadow_map_size = map_size;
  UpdateShadowOptions();
}

int LightComponent::GetShadowMapSize() const {
  return state_.shadow_map_size.value_or(kDefaultShadowMapSize);
}

void LightComponent::SetShadowConstantBias(float constant_bias) {
  state_.shadow_constant_bias = constant_bias;
  UpdateShadowOptions();
}

float LightComponent::GetShadowConstantBias() const {
  return state_.shadow_constant_bias;
}

void LightComponent::SetShadowNormalBias(float normal_bias) {
  state_.shadow_normal_bias = normal_bias;
  UpdateShadowOptions();
}

float LightComponent::GetShadowNormalBias() const {
  return state_.shadow_normal_bias;
}

void LightComponent::SetShadowFar(float shadow_far) {
  state_.shadow_far = shadow_far;
  UpdateShadowOptions();
}

float LightComponent::GetShadowFar() const { return state_.shadow_far; }

void LightComponent::SetShadowStable(bool stable) {
  state_.shadow_stable = stable;
  UpdateShadowOptions();
}

bool LightComponent::GetShadowStable() const { return state_.shadow_stable; }

void LightComponent::SetShadowLispsm(bool shadow_lispsm) {
  state_.shadow_lispsm = shadow_lispsm;
  UpdateShadowOptions();
}

bool LightComponent::GetShadowLispsm() const { return state_.shadow_lispsm; }

void LightComponent::SetShadowCascadeCount(int shadow_cascade_count) {
  state_.shadow_cascade_count = shadow_cascade_count;
  UpdateShadowOptions();
}

int LightComponent::GetShadowCascadeCount() const {
  return state_.shadow_cascade_count;
}

void LightComponent::SetShadowCascadeSplitScheme(
    LightState::ShadowCascadeSplitScheme split_scheme) {
  state_.shadow_cascade_split_scheme = split_scheme;
  UpdateShadowOptions();
}

LightState::ShadowCascadeSplitScheme
LightComponent::GetShadowCascadeSplitScheme() const {
  return state_.shadow_cascade_split_scheme;
}

void LightComponent::SetShadowCascadePracticalLambda(float lambda) {
  state_.shadow_cascade_practical_lambda = lambda;
  UpdateShadowOptions();
}

float LightComponent::GetShadowCascadePracticalLambda() const {
  return state_.shadow_cascade_practical_lambda;
}

void LightComponent::CreateLight() {
  // Can only have one light on a node.
  assert(!GetLightManager().hasComponent(GetEntity()));

  filament::LightManager::Builder(
      LightStateTypeToFilamentLightType(state_.type))
      .color(state_.color)
      .intensity(state_.intensity)
      .castShadows(!state_.is_shadow_casting_disabled)
      .falloff(state_.falloff)
      .direction(kDirection)
      .spotLightCone(state_.spot_cone_inner_outer.x,
                     state_.spot_cone_inner_outer.y)
      .shadowOptions(GetShadowOptions())
      .build(*BaseView::GetSharedEngine(), GetEntity());
}
void LightComponent::DestroyLight() { GetLightManager().destroy(GetEntity()); }

void LightComponent::UpdateShadowOptions() {
  auto& lm = GetLightManager();
  LightInstance instance = lm.getInstance(GetEntity());
  lm.setShadowOptions(instance, GetShadowOptions());
}

filament::LightManager& LightComponent::GetLightManager() const {
  return BaseView::GetSharedEngine()->getLightManager();
}

filament::LightManager::ShadowOptions LightComponent::GetShadowOptions() const {
  filament::LightManager::ShadowOptions options = {
      .mapSize = static_cast<uint32_t>(GetShadowMapSize()),
      .shadowCascades =
          static_cast<uint8_t>(std::clamp(GetShadowCascadeCount(), 1, 4)),
      .constantBias = GetShadowConstantBias(),
      .normalBias = GetShadowNormalBias(),
      .shadowFar = GetShadowFar(),
      .stable = GetShadowStable(),
      .lispsm = GetShadowLispsm(),
      .transform = GetShadowTransform()};

  float near = GetView().GetCameraManager().GetCamera()->GetNearClip();
  float far = GetView().GetCameraManager().GetCamera()->GetFarClip();
  switch (GetShadowCascadeSplitScheme()) {
    case LightState::ShadowCascadeSplitScheme::UNIFORM:
      filament::LightManager::ShadowCascades::computeUniformSplits(
          options.cascadeSplitPositions, options.shadowCascades);
      break;
    case LightState::ShadowCascadeSplitScheme::LOG:
      filament::LightManager::ShadowCascades::computeLogSplits(
          options.cascadeSplitPositions, options.shadowCascades, near, far);
      break;
    case LightState::ShadowCascadeSplitScheme::PRACTICAL:
      filament::LightManager::ShadowCascades::computePracticalSplits(
          options.cascadeSplitPositions, options.shadowCascades, near, far,
          GetShadowCascadePracticalLambda());
      break;
  }
  return options;
}

}  // namespace imp
