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

#ifndef THIRD_PARTY_IMPRESS_CORE_LIGHTING_LIGHT_COMPONENT_H_
#define THIRD_PARTY_IMPRESS_CORE_LIGHTING_LIGHT_COMPONENT_H_

#include "filament/filament/include/filament/LightManager.h"
#include "core/lighting/light_state.proto.imp.h"
#include "core/math/quat.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"

namespace imp {

// Adds a light source in the scene, such as a sun or street lights.
//
// Lights come in three flavors:
// - directional lights
// - point lights
// - spot lights
//
//
// Directional lights
// ------------------
//
// Directional lights have a direction, but don't have a position. All light
// rays are parallel and come from infinitely far away and from everywhere.
// Typically a directional light is used to simulate the sun.
//
// Directional lights will point in the forward direction of the node.
//
// Directional lights are able to cast shadows.
//
// To create a directional light use Type::DIRECTIONAL or Type::SUN, both are
// similar, but the later also draws a sun's disk in the sky and its reflection
// on glossy objects.
//
// Currently, only a single directional light is supported. If several
// directional lights are added to the scene, the dominant one will be used.
//
// Point lights
// ------------
//
// Unlike directional lights, point lights have a position but emit light in all
// directions. The intensity of the light diminishes with the inverse square of
// the distance to the light. falloff controls the distance beyond which the
// light has no more influence.
//
// A scene can have multiple point lights.
//
// Spot lights
// -----------
//
// Spot lights are similar to point lights but the light it emits is limited to
// a cone.
//
// A spot light is therefore defined by a position, a direction and inner and
// outer cones. The spot light's influence is limited to inside the outer cone.
// The inner cone defines the light's falloff attenuation.
//
// A physically correct spot light is a little difficult to use because changing
// the outer angle of the cone changes the illumination levels, as the same
// amount of light is spread over a changing volume. The coupling of
// illumination and the outer cone means that an artist cannot tweak the
// influence cone of a spot light without also changing the perceived
// illumination. It therefore makes sense to provide artists with a parameter to
// disable this coupling. This is the difference between Type::SPOT and
// Type::FOCUSED_SPOT.
class LightComponent : public Component {
 public:
  static const float kDefaultIntensity;
  static const float3 kDefaultColor;

  // Comes from light_data.proto
  using Type = LightState::Type;

  LightComponent();

  // Sets up a light either with default values (DIRECTONAL light) or serialized
  // values from light_data.proto.
  void Setup();

  // Sets up a default light of the type passed in.
  void Setup(Type type);

  // Called when the active status on this component changes.
  void OnActiveStatusChanged(bool is_active);

  // Apply changes from state info.
  void OnIsfStateChanged();

  // Destroys the light.
  void Cleanup();

  // the default is DIRECTIONAL.
  Type GetType() const;

  // Set the type of light. Note that this will destroy and recreate light.
  void SetType(Type type);

  // Sets the color of the light.
  // The color is expected to be in the linear sRGB color-space.
  // filament::Color contains some conversion helpers.
  // The default is white.
  // TODO Create an ImpView color type.
  void SetColor(const float3& color);

  // Returns the color of the light.
  // The color is returned in the linear sRGB color-space.
  // The default is white.
  // TODO Create an ImpView color type.
  const float3& GetColor() const;

  // Sets the intensity of the light, it can be negative.
  //
  // This parameter depends on the light type:
  //  - For directional lights, it specifies the illuminance in *lux*
  //    (or *lumen/m^2*).
  //  - For point lights and spot lights, it specifies the luminous power
  //    in *lumen*.
  // The default is 100000.
  void SetIntensity(float intensity);

  // Returns the light's intensity in lumen.
  //
  // This parameter depends on the light type:
  //  - For directional lights, it specifies the illuminance in *lux*
  //    (or *lumen/m^2*).
  //  - For point lights and spot lights, it specifies the luminous power
  //    in *lumen*.
  // The default is 100000.
  float GetIntensity() const;

  // Sets if this light will not cast shadows.
  // Only DIRECTIONAL and SUN lights can cast shadows.
  // The default is false.
  void SetShadowCastingDisabled(bool is_shadow_casting_disabled);

  // Returns if this light will not cast shadows.
  // Only DIRECTIONAL and SUN lights can cast shadows.
  // The default is false.
  bool IsShadowCastingDisabled() const;

  // Sets the falloff distance for point lights and spot lights.
  // The influence of the light diminishes to zero as it approaches the falloff.
  // The default is 1.
  void SetFalloff(float falloff);

  // Returns the falloff distance for point lights and spot lights.
  // The influence of the light diminishes to zero as it approaches the falloff.
  // The default is 1.
  float GetFalloff() const;

  // Sets the inner and outer cone angles for spot lights.
  // The defaults for both are PI. Does not affect other light types.
  void SetSpotCone(const float2& inner_outer);

  // Gets the inner and outer cone angles for spot lights. Does not return
  // meaningful values for other light types.
  float2 GetSpotCone() const;

  // Sets the shadow transform for directional lights. This transform is applied
  // on top of the directional light's direction.
  void SetShadowTransform(const imp::quatf& transform);

  // Gets the shadow transform for directional lights. This transform is applied
  // on top of the directional light's direction.
  imp::quatf GetShadowTransform() const;

  // Sets the size of the shadow map.  Must be a power-of-two and larger or
  // equal to 8.
  void SetShadowMapSize(int map_size);

  // Gets the size of the shadow map
  int GetShadowMapSize() const;

  // Sets the constant bias in world units (e.g. meters) by which shadows are
  // moved away from the light. 1mm by default.
  void SetShadowConstantBias(float constant_bias);

  // Returns the constant bias in world units.
  float GetShadowConstantBias() const;

  // Sets the shadow normal bias, used to move the shadow away from the fragment
  // normal. Default is 1.0.
  void SetShadowNormalBias(float normal_bias);

  // Returns the shadow normal bias.
  float GetShadowNormalBias() const;

  // Sets the distance from the camera after which shadows are clipped. Use 0.0f
  // to use the camera far distance.
  void SetShadowFar(float shadow_far);

  // Gets the distance from the camera after which shadows are clipped.
  float GetShadowFar() const;

  // If true, shadows will be optimized for stability over resolution.
  void SetShadowStable(bool stable);

  // Returns true if the shadows are optimized for stability.
  bool GetShadowStable() const;

  // If true, enable light-space perspective shadow-mapping.
  void SetShadowLispsm(bool shadow_lispsm);

  // Returns true if light-space perspective shadow-mapping is enabled.
  bool GetShadowLispsm() const;

  // Sets the number of shadow cascades as a number between [1, 4]. Values >1
  // will enable shadow cascades.
  void SetShadowCascadeCount(int shadow_cascade_count);

  // Returns the number of shadow cascades.
  int GetShadowCascadeCount() const;

  // Sets the shadow cascade split scheme.
  void SetShadowCascadeSplitScheme(
      LightState::ShadowCascadeSplitScheme split_scheme);

  // Returns the shadow cascade split scheme.
  LightState::ShadowCascadeSplitScheme GetShadowCascadeSplitScheme() const;

  // Sets the lambda value for the practical shadow cascade split scheme.
  // Ignored if shadow_cascade_split_scheme is not set to practical.
  void SetShadowCascadePracticalLambda(float lambda);

  // Returns the lambda value for the practical shadow cascade split scheme.
  float GetShadowCascadePracticalLambda() const;

 private:
  void CreateLight();
  void DestroyLight();
  void UpdateShadowOptions();

  filament::LightManager& GetLightManager() const;

  filament::LightManager::ShadowOptions GetShadowOptions() const;

  LightState state_;

 public:
  using IsfInfo = IsfInfo<&LightComponent::state_>;
  static constexpr bool kRunInEditMode = true;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_LIGHTING_LIGHT_COMPONENT_H_
