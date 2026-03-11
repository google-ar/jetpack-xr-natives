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

#include "core/view/framework/lighting/light_manager.h"

#include <grp.h>

#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/owned_or_unowned_memory.h"
#include "core/common/owned_ptr.h"
#include "core/lighting/environment_light.h"
#include "core/lighting/environment_light_factory.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/groups_manager.h"
#include "core/ncsb/node_handle.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/lighting/light_component.h"
#include "core/view/utils/asset.h"
#include "core/window/default_lighting.h"

namespace imp {

namespace {

constexpr float3 kDefaultDirectionalLightWorldForward = {0.0f, -1.0f, 0.0f};
constexpr float3 kDefaultDirectionalLightWorldUp = {0.0f, 0.0f, -1.0f};

constexpr absl::string_view kDefaultLightNodeName = "light";

using AutomatedLightingMode = GroupsManager::AutomatedLightingMode;
using EnvironmentLightHolder = GroupsManager::EnvironmentLightHolder;
}  // namespace

LightManager::LightManager(BaseView& view) : view_(view) {}

const AssetDefinition& LightManager::GetDefaultLightingResource() {
  return kDefaultIblZip;
}

float LightManager::GetDefaultIndirectLightIntensity() const {
  return kDefaultEnvironmentLightIntensity;
}
float LightManager::GetDefaultDirectionalLightIntensity() const {
  return kDefaultDirectionalLightIntensity;
}

void LightManager::SetEnvironmentLight(EnvironmentLightPtr environment_light,
                                       absl::string_view group_name) {
  GroupsManager& groups_manager = view_.GetGroupsManager();

  EnvironmentLightHolder environment_light_holder(
      AutomatedLightingMode::kNoEnvironmentLight);
  if (environment_light) {
    UpdateSplitEngineEnvironmentLight(
        environment_light->GetReflectionIblAsset(), group_name);
    environment_light_holder = EnvironmentLightHolder(
        OwnedOrUnownedMemory<EnvironmentLight>(std::move(environment_light)));
  }
  groups_manager.SetGroupEnvironmentLight(group_name,
                                          std::move(environment_light_holder));
}

void LightManager::SetEnvironmentLight(EnvironmentLight* environment_light,
                                       absl::string_view group_name) {
  GroupsManager& groups_manager = view_.GetGroupsManager();

  EnvironmentLightHolder environment_light_holder(
      AutomatedLightingMode::kNoEnvironmentLight);
  if (environment_light) {
    UpdateSplitEngineEnvironmentLight(
        environment_light->GetReflectionIblAsset(), group_name);
    environment_light_holder = EnvironmentLightHolder(
        OwnedOrUnownedMemory<EnvironmentLight>(std::move(environment_light)));
  }
  groups_manager.SetGroupEnvironmentLight(group_name,
                                          std::move(environment_light_holder));
}

void LightManager::SetEnvironmentLight(
    OwnedPtr<EnvironmentLight> environment_light,
    absl::string_view group_name) {
  GroupsManager& groups_manager = view_.GetGroupsManager();

  EnvironmentLightHolder environment_light_holder(
      AutomatedLightingMode::kNoEnvironmentLight);
  if (environment_light) {
    UpdateSplitEngineEnvironmentLight(
        environment_light->GetReflectionIblAsset(), group_name);
    environment_light_holder =
        EnvironmentLightHolder(std::move(environment_light));
  }
  groups_manager.SetGroupEnvironmentLight(group_name,
                                          std::move(environment_light_holder));
}

void LightManager::SetEnvironmentLight(
    BorrowedPtr<EnvironmentLight> environment_light,
    absl::string_view group_name) {
  GroupsManager& groups_manager = view_.GetGroupsManager();

  EnvironmentLightHolder environment_light_holder(
      AutomatedLightingMode::kNoEnvironmentLight);
  if (environment_light) {
    UpdateSplitEngineEnvironmentLight(
        environment_light->GetReflectionIblAsset(), group_name);
    environment_light_holder =
        EnvironmentLightHolder(std::move(environment_light));
  }
  groups_manager.SetGroupEnvironmentLight(group_name,
                                          std::move(environment_light_holder));
}

void LightManager::MirrorMainGroupEnvironmentLightToGroup(
    absl::string_view group_name) {
  if (group_name == GroupsManager::kMainGroupName) {
    return;
  }
  view_.GetGroupsManager().SetGroupEnvironmentLight(
      group_name, EnvironmentLightHolder(
                      AutomatedLightingMode::kUseMainGroupEnvironmentLight));
}

void LightManager::ClearEnvironmentLight(absl::string_view group_name) {
  if (group_name == GroupsManager::kMainGroupName) {
    if (split_engine::SplitEngineSerializer* serializer =
            view_.GetSplitEngineSerializer()) {
      serializer->ClearPreferredEnvironmentIblAsset();
    }
  }
  view_.GetGroupsManager().SetGroupEnvironmentLight(
      group_name,
      EnvironmentLightHolder(AutomatedLightingMode::kNoEnvironmentLight));
}

BorrowedPtr<EnvironmentLight> LightManager::GetGroupEnvironmentLight(
    absl::string_view group_name) {
  return view_.GetGroupsManager().GetEnvironmentLight(group_name);
}

EnvironmentLight* LightManager::GetEnvironmentLight(
    absl::string_view group_name) {
  return view_.GetGroupsManager().GetRawEnvironmentLight(group_name);
}

Future<absl::Status> LightManager::SetupDefaultLighting() {
  if (!default_ibl_) {
    default_ibl_ = view_.GetAssetManager().LoadImageBasedLighting(
        GetDefaultLightingResource());
  }

  return default_ibl_->Then(
      [this](AssetPtr<ImageBasedLightingAsset> ibl_asset) {
        SetEnvironmentLight(
            view_.GetEnvironmentLightFactory().CreateEnvironmentLight(
                ibl_asset, kDefaultEnvironmentLightIntensity));

        SetupDefaultDirectionalLight();
      });
}

void LightManager::UpdateSplitEngineEnvironmentLight(
    absl::optional<AssetPtr<ImageBasedLightingAsset>> ibl_asset,
    absl::string_view group_name, float intensity, const float3& tint) {
  if (!ibl_asset.has_value() || group_name != GroupsManager::kMainGroupName) {
    return;
  }
  if (split_engine::SplitEngineSerializer* serializer =
          view_.GetSplitEngineSerializer()) {
    serializer->SetPreferredEnvironmentIblAsset(
        *ibl_asset.value()->BorrowReflectionTexture()->GetTexture(), intensity,
        tint);
  }
}

LightManager::EnvironmentLightingStatus LightManager::GetDefaultLightingStatus()
    const {
  if (!default_lighting_status_) {
    return EnvironmentLightingStatus::kUnloaded;
  }

  return default_lighting_status_->Ready()
             ? EnvironmentLightingStatus::kReady
             : EnvironmentLightingStatus::kLoadInProgress;
}

void LightManager::DisableDefaultLoad() {
  default_load_option_ = DefaultLoadOption::kDisabled;
}

void LightManager::DisableDefaultIblLoad() {
  default_load_option_ = DefaultLoadOption::kDirectionalLightOnly;
}

bool LightManager::IsDefaultLoadEnabled() const {
  return default_load_option_ != DefaultLoadOption::kDisabled;
}

ComponentHandle<LightComponent> LightManager::GetDefaultDirectionalLight()
    const {
  return default_directional_light_;
}

ComponentHandle<LightComponent>
LightManager::GetOrCreateDefaultDirectionalLight() {
  if (!default_directional_light_) {
    SetupDefaultDirectionalLight();
  }

  return default_directional_light_;
}

void LightManager::Setup() {
  switch (default_load_option_) {
    case DefaultLoadOption::kDisabled:
      // Nothing to do.
      break;
    case DefaultLoadOption::kDirectionalLightOnly:
      SetupDefaultDirectionalLight();
      break;
    case DefaultLoadOption::kEnabled:
      default_lighting_status_ = SetupDefaultLighting();
      break;
    default:
      // This should never happen.
      IMP_LOG(imp::FATAL) << "Unknown default load option: "
                 << static_cast<int>(default_load_option_);
      break;
  }
}

void LightManager::Cleanup() {
  default_ibl_.reset();
  default_lighting_status_.reset();
}

void LightManager::EnsureLighting() {
  if ((!GetDefaultDirectionalLight() || !GetEnvironmentLight()) &&
      !default_lighting_status_) {
    default_lighting_status_ = SetupDefaultLighting();
  }
}

void LightManager::SetupDefaultDirectionalLight() {
  NodeHandle light_node = default_directional_light_
                              ? default_directional_light_->GetNode()
                              : NodeHandle();
  if (!light_node) {
    light_node = view_.CreateNode();
  }

  light_node->SetName(kDefaultLightNodeName);
  light_node->SetEnabled(true);

  light_node->SetWorldForward(kDefaultDirectionalLightWorldForward,
                              kDefaultDirectionalLightWorldUp);

  if (!default_directional_light_) {
    default_directional_light_ = light_node->AddComponent<LightComponent>();
  }

  default_directional_light_->SetShadowCastingDisabled(false);
  default_directional_light_->SetFalloff(1.0f);
  default_directional_light_->SetIntensity(kDefaultDirectionalLightIntensity);
}

void LightManager::ApplyMainGroupLighting(absl::string_view group_name) {
  MirrorMainGroupEnvironmentLightToGroup(group_name);
  AddDefaultDirectionalLightToGroup(group_name);
}

void LightManager::AddDefaultDirectionalLightToGroup(
    absl::string_view group_name) {
  if (default_directional_light_) {
    default_directional_light_->GetNode()->AddToGroup(group_name);
  }
}

}  // namespace imp
