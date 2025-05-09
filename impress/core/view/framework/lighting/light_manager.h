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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_LIGHTING_LIGHT_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_LIGHTING_LIGHT_MANAGER_H_

#include <memory>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/lighting/environment_light.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/view/base_view.h"
#include "core/view/framework/lighting/light_component.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/string_map.h"

namespace imp {

// Creates default lighting and provides access to change the default lighting
// of an ImpView's scene.
// TODO: Expose ability to control environment map.
// TODO: Clean up legacy lighting pipeline and transition state
// code.
class LightManager {
 public:
  explicit LightManager(BaseView* view);

  static const AssetDefinition& GetDefaultLightingResource();

  static constexpr float kDefaultDirectionalLightIntensity = 250.0f;
  static constexpr float kDefaultEnvironmentLightIntensity = 220.0f;

  // Current status of EnvironmentLight.
  enum class EnvironmentLightingStatus {
    kUnloaded,
    kLoadInProgress,
    kReady,
  };

  // Sets the EnvironmentLight for the group.
  // If group_name is not specified, the main group will be used.
  //
  // Please note that if environment_light is nullptr, the indirect light for
  // the default group will be removed.
  void SetEnvironmentLight(
      EnvironmentLightPtr environment_light,
      absl::string_view group_name = GroupsManager::kMainGroupName);
  void SetEnvironmentLight(
      EnvironmentLight* environment_light,
      absl::string_view group_name = GroupsManager::kMainGroupName);

  // Returns the current EnvironmentLight of the group.
  // If group_name is not specified, the main group will be used.
  //
  // For groups using default lighting, this could return nullptr if the default
  // lighting asset is still being loaded.
  EnvironmentLight* GetEnvironmentLight(
      absl::string_view group_name = GroupsManager::kMainGroupName);

  // Gets the default directional light for the main group.
  //
  // Only one directional light can be used at a time, see LightComponent for
  // details.
  //
  // There is nothing special about this light, it's just a LightComponent on
  // a node with some default settings. The settings of the light can be
  // modified. The light's node can be rotated or disabled. The light can also
  // be removed/destroyed and re-created by calling InitializeDefaultLighting.
  //
  // The default light has an intensity of 250.0f, is pointed downwards, and
  // the color is white.
  ComponentHandle<LightComponent> GetDefaultDirectionalLight() const;

  // Gets the default directional light for the main group. If the light not
  // created yet, creates a new directional light and returns the handle to it.
  //
  // In the case where DisableDefaultLoad() is called before Setup(),
  // LightManager will not create a default directional light. Calling this
  // function will create the directional light with default parameters.
  ComponentHandle<LightComponent> GetOrCreateDefaultDirectionalLight();

  // Gets the default intensity of indirect light.
  float GetDefaultIndirectLightIntensity() const;

  // Gets the default intensity of directional light.
  float GetDefaultDirectionalLightIntensity() const;

  // For on-demand loading of default lighting; if no lighting is loaded,
  // triggers load of default lighting
  void EnsureLighting();

  // Triggers load of the default light environment, unless DisableDefaultLoad
  // is called first.
  void Setup();
  // Destroys global lights.
  void Cleanup();

  // Returns the loading status of the default lighting.
  EnvironmentLightingStatus GetDefaultLightingStatus() const;

  // Disables default lighting loading. This needs to be called in the
  // constructor of imp::View for it to work properly.
  void DisableDefaultLoad();

  bool IsDefaultLoadEnabled() const { return is_default_load_enabled_; }

  // Applies the main group lighting to the specified group.
  //
  // This will not create new directional light or environment light, which
  // also means that the main group remains as the owner of the environment
  // light that's shared across groups.
  void ApplyMainGroupLighting(absl::string_view group_name);

 private:
  void SetupDefaultDirectionalLight();

  // Updates the indirect light of the filament::Scene associated with
  // `group_name`. If the scene doesn't exist, this will be no-op.
  void UpdateEnvironmentLight(EnvironmentLight* environment_light,
                              absl::string_view group_name);

  void AddDefaultDirectionalLightToGroup(absl::string_view group_name);

  Future<absl::Status> SetupDefaultLighting();

  BaseView* view_;

  std::optional<Future<AssetPtr<ImageBasedLightingAsset>>> default_ibl_;

  std::optional<Future<absl::Status>> default_lighting_status_;

  StringMap<OwnedOrUnownedEnvironmentLight> environment_lighting_map_;

  ComponentHandle<LightComponent> default_directional_light_;

  Dispatcher::ScopedConnection group_created_event_connection_;
  Dispatcher::ScopedConnection group_destroyed_event_connection_;

  bool is_default_load_enabled_ = true;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_LIGHTING_LIGHT_MANAGER_H_
