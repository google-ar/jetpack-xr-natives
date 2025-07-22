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

#include "apibindings/impress_api_view.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/filament/include/filament/View.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "apibindings/asset_animator.h"
#include "apibindings/asset_loader.h"
#include "apibindings/asset_ptr_map.h"
#include "apibindings/bindings_material.h"
#include "apibindings/bindings_object.h"
#include "apibindings/bindings_texture.h"
#include "apibindings/stereo_surface.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/lighting/environment_light.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/render/texture_options.h"
#include "core/split_engine/materials/split_engine_generic_material.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/framework/animation/gltf_animator.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_state.proto.imp.h"
#include "core/view/framework/lighting/light_manager.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/materials/water_reflection_material.h"
#include "mediapipe/framework/port/status_macros.h"

// TODO: (broken link) - Add unit tests for this with
//                     google3/third_party/impress/testing/view_fixture.h

namespace imp {
namespace {
Future<OwnedTexturePtr> LoadTextureFromPath(TextureFactory& texture_factory,
                                            AssetManager& asset_manager,
                                            AssetPtrMap& asset_ptr_map,
                                            absl::string_view path,
                                            filament::TextureSampler sampler) {
  return asset_manager.LoadImage(asset_ptr_map.GetAssetString(path))
      .Then([sampler, &texture_factory = texture_factory](
                AssetPtr<ImageAsset> image) -> absl::StatusOr<OwnedTexturePtr> {
        // Uploads the texture to the system.
        // TODO: There is an ABI compatibility issue with the
        // TextureFactory::Options struct. Note that the remote renderer might
        // be running a different version of Filament than this process was
        // compiled against, so we need to be careful about the enum values we
        // use here.
        OwnedTexturePtr texture = texture_factory.CreateTexture(
            *image, imp::TextureGenerationOptions{},
            imp::TextureSamplerOptions{
                // TextureFactory uses a single dimension wrap mode
                // whereas Filament uses separate wrap modes for
                // each dimension. We use the same wrap mode (S) for
                // both dimensions.
                .wrap_mode = sampler.getWrapModeS(),
                .mag_filter = sampler.getMagFilter(),
                .min_filter = sampler.getMinFilter(),
                .anisotropy = sampler.getAnisotropy()});
        return texture;
      });
}

absl::StatusOr<ComponentHandle<StereoSurface>> GetStereoSurface(
    int32_t node_id) {
  NodeHandle node(utils::Entity::import(node_id));
  if (!node.IsValid()) {
    return absl::InvalidArgumentError("Node is not valid.");
  }
  auto result = node->GetComponent<StereoSurface>();
  if (!result.IsValid()) {
    return absl::InvalidArgumentError("Node is not a StereoSurface.");
  }
  return result;
}
}  // namespace

void ImpressApiView::SetupImpressApiNative() {
  asset_ptr_map_ = std::make_unique<AssetPtrMap>(*this);
}

void ImpressApiView::LoadImageBasedLightingAsset(
    absl::string_view path, std::unique_ptr<AssetLoader> asset_loader) {
  asset_ptr_map_->LoadImageBasedLightingAsset(path, std::move(asset_loader));
}

void ImpressApiView::LoadImageBasedLightingAsset(
    absl::Cord data, absl::string_view key,
    std::unique_ptr<AssetLoader> asset_loader) {
  asset_ptr_map_->LoadImageBasedLightingAsset(data, key,
                                              std::move(asset_loader));
}

absl::Status ImpressApiView::ReleaseImageBasedLightingAsset(
    std::intptr_t ibl_token) {
  return asset_ptr_map_->ReleaseImageBasedLightingAsset(ibl_token);
}

void ImpressApiView::LoadGltfAsset(absl::string_view path,
                                   std::unique_ptr<AssetLoader> asset_loader) {
  asset_ptr_map_->LoadGltfAsset(path, std::move(asset_loader));
}

void ImpressApiView::LoadGltfAsset(absl::Cord data, absl::string_view key,
                                   std::unique_ptr<AssetLoader> asset_loader) {
  asset_ptr_map_->LoadGltfAsset(data, key, std::move(asset_loader));
}

absl::Status ImpressApiView::ReleaseGltfAsset(std::intptr_t gltf_token) {
  return asset_ptr_map_->ReleaseGltfAsset(gltf_token);
}

absl::StatusOr<int32_t> ImpressApiView::InstanceGltfModel(
    std::intptr_t gltf_token, bool enable_collider) {
  absl::StatusOr<AssetPtr<GltfAsset>> gltf_asset_ptr =
      asset_ptr_map_->GetStoredGltfAsset(gltf_token);

  if (!gltf_asset_ptr.ok()) {
    return absl::InvalidArgumentError("Gltf asset is not cached.");
  }

  NodeHandle node = CreateNode();

  if (!enable_collider) {
    auto render_options = GltfAsset::LoadOptions(
        {.collider_mode = GltfState::ColliderMode::NONE});
    node->AddComponent<GltfRenderer>(gltf_asset_ptr.value(), render_options);
  } else {
    node->AddComponent<GltfRenderer>(gltf_asset_ptr.value());
  }

  return node.GetEntity().getId();
}

absl::Status ImpressApiView::SetGltfModelColliderEnabled(int32_t node,
                                                         bool enable_collider) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  std::vector<ComponentHandle<GltfMesh>> gltf_meshes =
      GetPathManager().GetComponentsInDescendantsOrSelf<GltfMesh>(node_handle);

  for (ComponentHandle<GltfMesh> gltf_mesh : gltf_meshes) {
    if (enable_collider) {
      gltf_mesh->GetNode()->AddComponent<GltfCollider>(gltf_mesh);
    } else {
      gltf_mesh->GetNode()->RemoveComponent<GltfCollider>();
    }
  }

  return absl::OkStatus();
}

void ImpressApiView::AnimateGltfModel(
    int32_t node, absl::string_view animation_name, bool loop,
    std::unique_ptr<AssetAnimator> asset_animator) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    asset_animator->OnFailure("Node is not valid.");
    return;
  }
  ComponentHandle<GltfAnimator> gltf_animator =
      node_handle->GetOrAddComponent<GltfAnimator>();

  GltfAnimator::PlayCommand gltf_animation;
  // If no animation_name was supplied, default to first available animation
  if (!animation_name.empty()) {
    gltf_animation.animation = std::string(animation_name);
  }
  gltf_animation.options.looping = loop;

  absl::Status can_play = gltf_animator->CanPlay(gltf_animation);
  if (!can_play.ok()) {
    IMP_LOG(imp::ERROR) << "Cannot play animation: " << can_play.message();
    asset_animator->OnFailure("Cannot play animation.");
    return;
  }
  gltf_animator->Play(gltf_animation);
  // TODO: (broken link) - Refactor this per-node caching into a custom
  //                     AnimatorController component.
  node_to_anim_ctx_[node] = std::make_tuple(
      gltf_animator,
      std::optional<std::unique_ptr<AssetAnimator>>(std::move(asset_animator)));
}

absl::Status ImpressApiView::StopGltfModelAnimation(int32_t node) {
  NodeHandle node_handle(utils::Entity::import(node));
  if (!node_handle) {
    return absl::InvalidArgumentError("Node is not valid.");
  }
  auto it = node_to_anim_ctx_.find(node);
  if (it != node_to_anim_ctx_.end()) {
    auto& [animator, callback] = node_to_anim_ctx_[node];
    if (animator) {
      // We technically can avoid checking validity here because we don't
      // support attaching and detaching the animation component from the
      // application side, but keeping for correctness.
      animator->Stop();
    }
    // Optionally we could call the callback here, but this method implies
    // that the animation has been "cancelled,"  rather than completing.
    node_to_anim_ctx_.erase(it);
    return absl::OkStatus();
  }
  return absl::NotFoundError("Animation is not playing.");
}

int32_t ImpressApiView::CreateImpressNode() {
  return CreateNode().GetEntity().getId();
}

absl::Status ImpressApiView::DestroyImpressNode(int32_t node) {
  // If the node is animating, be sure to remove it from the Animation map.
  // Otherwise we hit an assert on the next update.
  auto unused = StopGltfModelAnimation(node);
  NodeHandle node_handle(utils::Entity::import(node));
  if (node_handle) {
    DestroyNode(node_handle);
    return absl::OkStatus();
  }
  return absl::InvalidArgumentError("Node is not valid.");
}

absl::StatusOr<int32_t> ImpressApiView::CreateStereoSurfaceEntity(
    MediaStereoMode stereo_mode, ContentSecurityLevel content_security_level,
    bool use_super_sampling) {
  NodeHandle node = CreateNode();
  absl::StatusOr<ComponentHandle<StereoSurface>> status =
      node->AddComponent<StereoSurface>(stereo_mode, content_security_level,
                                        use_super_sampling);
  if (!status.ok()) {
    return status.status();
  }
  if (!status->IsValid()) {
    return absl::InternalError("Node is not valid.");
  }
  return node.GetEntity().getId();
}

absl::Status ImpressApiView::SetStereoSurfaceEntityCanvasShape(
    int32_t node_id, StereoSurface::CanvasShape canvas_shape) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  return stereo_surface->SetCanvasShape(canvas_shape);
}

absl::StatusOr<android::Surface*>
ImpressApiView::GetSurfaceFromStereoSurfaceEntity(int32_t node_id) {
  absl::StatusOr<ComponentHandle<StereoSurface>> result =
      GetStereoSurface(node_id);
  if (!result.ok()) {
    return result.status();
  }
  return (*result)->GetSurface();
}

absl::Status ImpressApiView::SetFeatherRadiusForStereoSurfaceEntity(
    int32_t node_id, const float2& feather_radius) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetFeatherRadius(feather_radius);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetStereoModeForStereoSurfaceEntity(
    int32_t node_id, MediaStereoMode stereo_mode) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetStereoMode(stereo_mode);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetPrimaryAlphaMaskForStereoSurfaceEntity(
    int32_t node_id, int64_t alpha_mask_token) {
  OwnedOrBorrowedTexturePtr alpha_mask;
  // If the alpha mask token is kUnSetAlphaMaskToken, then the alpha mask is
  // removed.
  if (alpha_mask_token != kUnSetAlphaMaskToken) {
    alpha_mask = FromJava<BindingsTexture>(alpha_mask_token)->GetTexture();
  }
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetPrimaryAlphaMask(std::move(alpha_mask));
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetAuxiliaryAlphaMaskForStereoSurfaceEntity(
    int32_t node_id, int64_t alpha_mask_token) {
  OwnedOrBorrowedTexturePtr alpha_mask;
  // If the alpha mask token is kUnSetAlphaMaskToken, then the alpha mask is
  // removed.
  if (alpha_mask_token != kUnSetAlphaMaskToken) {
    alpha_mask = FromJava<BindingsTexture>(alpha_mask_token)->GetTexture();
  }
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetAuxiliaryAlphaMask(std::move(alpha_mask));
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetContentColorMetadataForStereoSurfaceEntity(
    int32_t node_id, MediaColorSpace color_space) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetContentColorMetadata(color_space);
  return absl::OkStatus();
}

void ImpressApiView::LoadTexture(absl::string_view path,
                                 filament::TextureSampler sampler,
                                 std::unique_ptr<AssetLoader> asset_loader) {
  LoadTextureFromPath(GetTextureFactory(), GetAssetManager(), *asset_ptr_map_,
                      path, sampler)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<OwnedTexturePtr> texture) mutable {
        if (texture.ok() && *texture) {
          // This transfers the ownership of the BindingsTexture object
          // to Java. At this point, Java is responsible for managing
          // the lifecycle of the texture object. The Java side will
          // call the DestroyNativeObject method when it is done with
          // the texture.
          // TODO: This contract is broken since the
          // DisposeAllResources method will flush resources that might still
          // be in use by the Java side without it being aware. The Java side
          // should be responsible for tracking bindings resources and
          // individually disposing them.
          std::intptr_t texture_token =
              ToJava(new BindingsTexture(*std::move(texture)));
          bindings_texture_set_.insert(texture_token);
          asset_loader->OnSuccess(texture_token);
        } else {
          asset_loader->OnFailure("Failed to load texture.");
        }
      })
      .KeptBy(this);
}

absl::StatusOr<std::intptr_t> ImpressApiView::BorrowReflectionTexture() {
  const EnvironmentLight* environment_light =
      GetLightManager().GetEnvironmentLight();
  if (!environment_light) return absl::NotFoundError("No environment light.");

  std::optional<AssetPtr<ImageBasedLightingAsset>> ibl_asset =
      environment_light->GetReflectionIblAsset();
  if (!ibl_asset.has_value())
    return absl::NotFoundError("No reflection texture.");

  BorrowedTexturePtr reflections_texture =
      (*ibl_asset)->BorrowReflectionTexture();
  // This transfers the ownership of the BindingsTexture object to
  // Java. At this point, Java is responsible for managing the lifecycle
  // of the texture object. The Java side will call the
  // DestroyNativeObject method when it is done with the texture.
  // TODO: This contract is broken since the
  // DisposeAllResources method will flush resources that might still
  // be in use by the Java side without it being aware. The Java side
  // should be responsible for tracking bindings resources and
  // individually disposing them.
  std::intptr_t reflections_texture_token =
      ToJava(new BindingsTexture(std::move(reflections_texture)));
  bindings_texture_set_.insert(reflections_texture_token);
  return reflections_texture_token;
}

absl::StatusOr<std::intptr_t> ImpressApiView::GetReflectionTextureFromIbl(
    std::intptr_t ibl_token) {
  absl::StatusOr<AssetPtr<ImageBasedLightingAsset>> ibl_asset_ptr =
      asset_ptr_map_->GetStoredIblAsset(ibl_token);

  if (!ibl_asset_ptr.ok()) {
    return absl::NotFoundError("IBL asset is not cached.");
  }

  BorrowedTexturePtr reflections_texture =
      ibl_asset_ptr.value()->BorrowSkyboxCubemap();
  // This transfers the ownership of the BindingsTexture object to
  // Java. At this point, Java is responsible for managing the lifecycle
  // of the texture object. The Java side will call the
  // DestroyNativeObject method when it is done with the texture.
  // TODO: This contract is broken since the
  // DisposeAllResources method will flush resources that might still
  // be in use by the Java side without it being aware. The Java side
  // should be responsible for tracking bindings resources and
  // individually disposing them.
  std::intptr_t reflections_texture_token =
      ToJava(new BindingsTexture(std::move(reflections_texture)));
  bindings_texture_set_.insert(reflections_texture_token);
  return reflections_texture_token;
}

void ImpressApiView::CreateWaterMaterial(
    std::unique_ptr<AssetLoader> asset_loader, bool is_alpha_map_version) {
  android_xr::WaterReflectionMaterial::Create(*this, is_alpha_map_version)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<
                    std::unique_ptr<android_xr::WaterReflectionMaterial>>
                    material) mutable {
        if (material.ok() && *material) {
          // This transfers the ownership of the BindingsMaterial object to
          // Java. At this point, Java is responsible for managing the lifecycle
          // of the material object. The Java side will call the
          // DestroyNativeObject method when it is done with the material.
          // TODO: This contract is broken since the
          // DisposeAllResources method will flush resources that might still
          // be in use by the Java side without it being aware. The Java side
          // should be responsible for tracking bindings resources and
          // individually disposing them.
          std::intptr_t material_token =
              ToJava(new BindingsMaterial(*std::move(material)));
          bindings_material_set_.insert(material_token);
          asset_loader->OnSuccess(material_token);
        } else {
          asset_loader->OnFailure(
              "Failed to create the built-in water material.");
        }
      })
      .KeptBy(this);
}

void ImpressApiView::DestroyNativeObject(std::intptr_t handle) {
  // Creating a unique_ptr and letting it go out of scope will destroy the
  // bindings object, which will free the memory it was holding.
  std::unique_ptr<BindingsObject> bindings_object(
      FromJava<BindingsObject>(handle));
}

absl::Status ImpressApiView::SetReflectionMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t reflection_map) {
  return SetWaterMaterialTextureParameter(
      water_material, reflection_map,
      [](android_xr::WaterReflectionMaterial* material,
         BorrowedTexturePtr borrowed_texture) {
        material->SetReflectionCube(borrowed_texture);
      });
}

absl::Status ImpressApiView::SetNormalMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t normal_map) {
  return SetWaterMaterialTextureParameter(
      water_material, normal_map,
      [](android_xr::WaterReflectionMaterial* material,
         BorrowedTexturePtr borrowed_texture) {
        material->SetNormalMap(borrowed_texture);
      });
}

absl::Status ImpressApiView::SetNormalTilingOnWaterMaterial(
    std::intptr_t water_material, float normal_tiling) {
  MP_ASSIGN_OR_RETURN(
      android_xr::WaterReflectionMaterial * material,
      GetMaterialFromBindingsMaterial<android_xr::WaterReflectionMaterial>(
          water_material));
  material->SetNormalTiling(normal_tiling);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetNormalSpeedOnWaterMaterial(
    std::intptr_t water_material, float normal_speed) {
  MP_ASSIGN_OR_RETURN(
      android_xr::WaterReflectionMaterial * material,
      GetMaterialFromBindingsMaterial<android_xr::WaterReflectionMaterial>(
          water_material));
  material->SetNormalSpeed(normal_speed);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetAlphaStepMultiplierOnWaterMaterial(
    std::intptr_t water_material, float alpha_step_multiplier) {
  MP_ASSIGN_OR_RETURN(
      android_xr::WaterReflectionMaterial * material,
      GetMaterialFromBindingsMaterial<android_xr::WaterReflectionMaterial>(
          water_material));
  material->SetAlphaStepMultiplier(alpha_step_multiplier);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetAlphaMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t alpha_map) {
  return SetWaterMaterialTextureParameter(
      water_material, alpha_map,
      [](android_xr::WaterReflectionMaterial* material,
         BorrowedTexturePtr borrowed_texture) {
        material->SetAlphaMap(borrowed_texture);
      });
}

absl::Status ImpressApiView::SetNormalZOnWaterMaterial(
    std::intptr_t water_material, float normal_z) {
  MP_ASSIGN_OR_RETURN(
      android_xr::WaterReflectionMaterial * material,
      GetMaterialFromBindingsMaterial<android_xr::WaterReflectionMaterial>(
          water_material));
  material->SetNormalZ(normal_z);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetNormalBoundaryOnWaterMaterial(
    std::intptr_t water_material, float normal_boundary) {
  MP_ASSIGN_OR_RETURN(
      android_xr::WaterReflectionMaterial * material,
      GetMaterialFromBindingsMaterial<android_xr::WaterReflectionMaterial>(
          water_material));
  material->SetNormalBoundary(normal_boundary);
  return absl::OkStatus();
}

void ImpressApiView::CreateGenericMaterial(
    std::unique_ptr<AssetLoader> asset_loader,
    GenericMaterialSpec generic_material_spec) {
  split_engine::SplitEngineGenericMaterial::Create(*this, generic_material_spec)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<
                    std::unique_ptr<split_engine::SplitEngineGenericMaterial>>
                    generic_material) {
        if (generic_material.ok() && *generic_material) {
          // This transfers the ownership of the BindingsMaterial object to
          // Java. At this point, Java is responsible for managing the lifecycle
          // of the material object. The Java side will call the
          // DestroyNativeObject method when it is done with the material.
          // TODO: This contract is broken since the
          // DisposeAllResources method will flush resources that might still
          // be in use by the Java side without it being aware. The Java side
          // should be responsible for tracking bindings resources and
          // individually disposing them.
          std::intptr_t material_token =
              ToJava(new BindingsMaterial(*std::move(generic_material)));
          bindings_material_set_.insert(material_token);
          asset_loader->OnSuccess(material_token);
        } else {
          asset_loader->OnFailure(
              "Failed to create the built-in generic material.");
        }
      })
      .KeptBy(this);
}

absl::Status ImpressApiView::SetBaseColorTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t base_color_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, base_color_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.base_color.emplace();
        material_parameters.base_color->texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetBaseColorUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  return material->SetBaseColorUvTransform(uv_transform);
}

absl::Status ImpressApiView::SetBaseColorFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float4& factors) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetBaseColorFactor(factors);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetMetallicRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t metallic_roughness_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, metallic_roughness_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.metallic_roughness.emplace();
        material_parameters.metallic_roughness->texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetMetallicRoughnessUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  return material->SetMetallicRoughnessUvTransform(uv_transform);
}

absl::Status ImpressApiView::SetMetallicFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetMetallicFactor(factor);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetRoughnessFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetRoughnessFactor(factor);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetNormalTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t normal_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, normal_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.normal.emplace();
        material_parameters.normal->texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetNormalUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  return material->SetNormalUvTransform(uv_transform);
}

absl::Status ImpressApiView::SetNormalFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetNormalScale(factor);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetAmbientOcclusionTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t ambient_occlusion_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, ambient_occlusion_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.ambient_occlusion.emplace();
        material_parameters.ambient_occlusion->texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetAmbientOcclusionUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  return material->SetAmbientOcclusionUvTransform(uv_transform);
}

absl::Status ImpressApiView::SetAmbientOcclusionFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetAmbientOcclusionStrength(factor);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetEmissiveTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t emissive_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, emissive_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.emissive.emplace();
        material_parameters.emissive->texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetEmissiveUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  return material->SetEmissiveUvTransform(uv_transform);
}

absl::Status ImpressApiView::SetEmissiveFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factors) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetEmissiveFactor(factors);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetClearcoatTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, clearcoat_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.clearcoat.emplace();
        material_parameters.clearcoat->intensity_texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetClearcoatNormalTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_normal_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, clearcoat_normal_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.clearcoat.emplace();
        material_parameters.clearcoat->normal_texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetClearcoatRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_roughness_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, clearcoat_roughness_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.clearcoat.emplace();
        material_parameters.clearcoat->roughness_texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetClearcoatFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factor) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetClearcoatFactors(factor);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetSheenColorTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t sheen_color_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, sheen_color_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.sheen.emplace();
        material_parameters.sheen->color_texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetSheenColorFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factors) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetSheenColorFactor(factors);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetSheenRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t sheen_roughness_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, sheen_roughness_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.sheen.emplace();
        material_parameters.sheen->roughness_texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetSheenRoughnessFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetSheenRoughnessFactor(factor);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetTransmissionTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t transmission_texture) {
  return SetGenericMaterialTextureParameter(
      generic_material, transmission_texture,
      [](split_engine::SplitEngineGenericMaterial* material,
         imp::GenericMaterialTextureParameter texture_parameter,
         imp::TextureBorrower texture_borrower) {
        imp::GenericMaterialParameters material_parameters;
        material_parameters.transmission.emplace();
        material_parameters.transmission->texture = texture_parameter;
        return material->AssignTexturesAndParams(material_parameters,
                                                 texture_borrower);
      });
}

absl::Status ImpressApiView::SetTransmissionUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  return material->SetTransmissionUvTransform(uv_transform);
}

absl::Status ImpressApiView::SetTransmissionFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetTransmissionFactor(factor);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetIndexOfRefractionOnGenericMaterial(
    std::intptr_t generic_material, float index_of_refraction) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetIndexOfRefraction(index_of_refraction);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetAlphaCutoffOnGenericMaterial(
    std::intptr_t generic_material, float alpha_cutoff) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * material,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  material->SetAlphaCutoff(alpha_cutoff);
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetMaterialOverride(int32_t node_id,
                                                 std::intptr_t material,
                                                 absl::string_view mesh_name) {
  NodeHandle model_node(utils::Entity::import(node_id));
  if (!model_node) {
    return absl::InvalidArgumentError("Node is not valid.");
  }

  BindingsMaterial* bindings_material = FromJava<BindingsMaterial>(material);
  if (!bindings_material) {
    return absl::InvalidArgumentError("Provided material handle is not valid.");
  }

  split_engine::SplitEngineMaterial* mat = bindings_material->GetBaseMaterial();

  if (!mat) {
    return absl::InternalError("BindingsMaterial contained a null pointer.");
  }

  NodeHandle mesh_node = model_node->FindByName(mesh_name);
  if (!mesh_node) {
    return absl::NotFoundError(
        absl::StrFormat("No Gltf child node named %s.", mesh_name));
  }
  ComponentHandle<GltfMesh> mesh = mesh_node->GetComponent<GltfMesh>();
  if (!mesh) {
    return absl::InvalidArgumentError("Child doesn't have a mesh.");
  }
  mesh->SetMaterialOverride(mat->GetMaterial());
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetEnvironmentLight(std::intptr_t ibl_token) {
  absl::StatusOr<AssetPtr<ImageBasedLightingAsset>> ibl_asset_ptr =
      asset_ptr_map_->GetStoredIblAsset(ibl_token);
  if (!ibl_asset_ptr.ok()) {
    return absl::NotFoundError("IBL asset is not cached.");
  }
  split_engine::SplitEngineSerializer* serializer = GetSplitEngineSerializer();
  if (serializer == nullptr) {
    return absl::InternalError("SplitEngineSerializer is not available.");
  }
  serializer->SetPreferredEnvironmentIblAsset(
      *ibl_asset_ptr.value()->BorrowReflectionTexture()->GetTexture(),
      LightManager::kDefaultEnvironmentLightIntensity, kOne3);
  GetLightManager().SetEnvironmentLight(
      GetEnvironmentLightFactory().CreateEnvironmentLight(
          ibl_asset_ptr.value(),
          LightManager::kDefaultEnvironmentLightIntensity));
  return absl::OkStatus();
}

absl::Status ImpressApiView::ClearEnvironmentLight() {
  split_engine::SplitEngineSerializer* serializer = GetSplitEngineSerializer();
  if (serializer == nullptr) {
    return absl::InternalError("SplitEngineSerializer is not available.");
  }
  serializer->ClearPreferredEnvironmentIblAsset();
  return absl::OkStatus();
}

absl::Status ImpressApiView::DisposeAllResources() {
  // The order of destruction matters - we first destroy the glTF models, then
  // the materials which might be used in the glTF models, then the textures
  // which might be used in the materials.
  absl::Status status = asset_ptr_map_->DisposeIblAssets();
  if (!status.ok()) {
    return status;
  }
  asset_ptr_map_->DestroyGltfAssetsAndInstances();
  // We destroy bindings materials and textures separately instead of
  // looping over all bindings objects because the textures are used in the
  // materials but not the other way around, so we need to destroy the textures
  // first.
  for (auto& bindings_material : bindings_material_set_) {
    DestroyNativeObject(bindings_material);
  }
  for (auto& bindings_texture : bindings_texture_set_) {
    DestroyNativeObject(bindings_texture);
  }
  return absl::OkStatus();
}

absl::Status ImpressApiView::SetImpressNodeParent(int32_t child,
                                                  int32_t parent) {
  NodeHandle child_handle(utils::Entity::import(child));
  if (!child_handle) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Child node is not valid: %d.", child));
  }
  NodeHandle parent_handle(utils::Entity::import(parent));
  if (!parent_handle) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Parent node is not valid: %d.", parent));
  }
  child_handle->SetParent(parent_handle);
  return absl::OkStatus();
}

void ImpressApiView::Setup() {
  // Setup View settings
  filament::View* filament_view = GetHost()->GetView();
  filament_view->setPostProcessingEnabled(false);
}

void ImpressApiView::Update(const FrameTime& frame_time) {
  // TODO: (broken link) - Hook into the Impress Animation Event system and drive
  //                     the callback dispatch from there, instead of polling on
  //                     Update.
  for (auto it = node_to_anim_ctx_.begin(); it != node_to_anim_ctx_.end();) {
    auto& [animator, callback] = it->second;
    if (animator && !animator->IsPlaying() && callback.has_value()) {
      callback.value()->OnComplete();
      node_to_anim_ctx_.erase(it++);
    } else {
      ++it;
    }
  }
}

absl::StatusOr<BorrowedTexturePtr> ImpressApiView::BorrowTexture(
    std::intptr_t texture_handle) {
  BindingsTexture* bindings_texture = FromJava<BindingsTexture>(texture_handle);
  if (!bindings_texture) {
    return absl::InvalidArgumentError("Provided texture handle is not valid.");
  }

  BorrowedTexturePtr borrowed_texture = bindings_texture->GetTexture();
  if (!borrowed_texture) {
    return absl::InvalidArgumentError(
        "Texture associated with handle is not valid.");
  }

  return borrowed_texture;
}

}  // namespace imp
