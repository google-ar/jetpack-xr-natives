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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/IndirectLight.h"
#include "filament/filament/include/filament/Scene.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/platform_helpers.h"
#include "core/lighting/environment_light.h"
#include "core/lighting/environment_light_factory.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
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

}  // namespace

LightManager::LightManager(BaseView* view) : view_(view) {}

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
  auto it = environment_lighting_map_.find(group_name);
  if (it == environment_lighting_map_.end()) {
    IMP_LOG(imp::ERROR) << "Failed to set EnvironmentLight for group " << group_name
               << ": Group does not exist";
    return;
  }

  it.value().Set(std::move(environment_light));

  UpdateEnvironmentLight(it.value().Get(), group_name);
}

void LightManager::SetEnvironmentLight(EnvironmentLight* environment_light,
                                       absl::string_view group_name) {
  environment_lighting_map_[std::string(group_name)].Set(environment_light);

  UpdateEnvironmentLight(environment_light, group_name);
}

EnvironmentLight* LightManager::GetEnvironmentLight(
    absl::string_view group_name) {
  auto it = environment_lighting_map_.find(group_name);
  if (it == environment_lighting_map_.end()) {
    return nullptr;
  }

  return it.value().Get();
}

Future<absl::Status> LightManager::SetupDefaultLighting() {
  if (!default_ibl_) {
    default_ibl_ = view_->GetAssetManager().LoadImageBasedLighting(
        GetDefaultLightingResource());
  }

  return default_ibl_->Then(
      [this](AssetPtr<ImageBasedLightingAsset> ibl_asset) {
        SetEnvironmentLight(
            view_->GetEnvironmentLightFactory().CreateEnvironmentLight(
                ibl_asset, kDefaultEnvironmentLightIntensity));

        SetupDefaultDirectionalLight();
      });
}

void LightManager::DisableDefaultLoad() { is_default_load_enabled_ = false; }

LightManager::EnvironmentLightingStatus LightManager::GetDefaultLightingStatus()
    const {
  if (!default_lighting_status_) {
    return EnvironmentLightingStatus::kUnloaded;
  }

  return default_lighting_status_->Ready()
             ? EnvironmentLightingStatus::kReady
             : EnvironmentLightingStatus::kLoadInProgress;
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
  if (is_default_load_enabled_) {
    default_lighting_status_ = SetupDefaultLighting();
  }

  // Creates GroupLighting for main group manually as the GroupCreatedEvent for
  // main group gets sent out before LightManager::Setup().
  environment_lighting_map_[std::string(
      view_->GetGroupsManager().kMainGroupName)] =
      OwnedOrUnownedEnvironmentLight();

  group_created_event_connection_ = view_->GetDispatcher().Connect(
      [this](const GroupsManager::GroupCreatedEvent& event) {
        auto it = environment_lighting_map_.find(event.group_name);
        if (it != environment_lighting_map_.end()) {
          UpdateEnvironmentLight(it.value().Get(), event.group_name);
        } else {
          environment_lighting_map_[event.group_name] =
              OwnedOrUnownedEnvironmentLight();
        }
      });
}

void LightManager::Cleanup() {
  default_ibl_.reset();
  default_lighting_status_.reset();
  environment_lighting_map_.clear();
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
    light_node = view_->CreateNode();
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

void LightManager::UpdateEnvironmentLight(EnvironmentLight* environment_light,
                                          absl::string_view group_name) {
  filament::Scene* scene = view_->GetGroupsManager().GetScene(group_name);
  if (!scene) {
    return;
  }

  if (environment_light) {
    scene->setIndirectLight(environment_light->GetIndirectLight());
  } else {
    scene->setIndirectLight(nullptr);
  }
}

void LightManager::ApplyMainGroupLighting(absl::string_view group_name) {
  if (GetDefaultLightingStatus() ==
      EnvironmentLightingStatus::kLoadInProgress) {
    default_lighting_status_
        ->Then([this, group = std::string(group_name)]() {
          AddDefaultDirectionalLightToGroup(group);
          SetEnvironmentLight(GetEnvironmentLight(), group);
        })
        .KeptBy(view_);
  } else {
    AddDefaultDirectionalLightToGroup(group_name);
    SetEnvironmentLight(GetEnvironmentLight(), group_name);
  }
}

void LightManager::AddDefaultDirectionalLightToGroup(
    absl::string_view group_name) {
  if (default_directional_light_) {
    default_directional_light_->GetNode()->AddToGroup(group_name);
  }
}

}  // namespace imp
