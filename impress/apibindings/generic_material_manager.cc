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

#include "apibindings/generic_material_manager.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/bindings_material.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/texture_manager.h"
#include "core/async/future.h"
#include "core/common/hash.h"
#include "core/common/owned_ptr.h"
#include "core/common/small_source_location.h"
#include "core/common/type_traits.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/split_engine/materials/split_engine_generic_material.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace {

class GenericMaterialManagerImpl : public GenericMaterialManager {
 public:
  explicit GenericMaterialManagerImpl(ImpressApiView& view);
  ~GenericMaterialManagerImpl() override = default;

  void CreateGenericMaterial(
      std::unique_ptr<BaseAssetLoader> asset_loader,
      GenericMaterialSpec generic_material_spec) override;
  absl::Status SetBaseColorTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t base_color_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetBaseColorUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetBaseColorFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float4& factors) override;
  absl::Status SetMetallicRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t metallic_roughness_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetMetallicRoughnessUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetMetallicFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) override;
  absl::Status SetRoughnessFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) override;
  absl::Status SetNormalTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t normal_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetNormalUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetNormalFactorOnGenericMaterial(std::intptr_t generic_material,
                                                float factor) override;
  absl::Status SetAmbientOcclusionTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t ambient_occlusion_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetAmbientOcclusionUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetAmbientOcclusionFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) override;
  absl::Status SetEmissiveTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t emissive_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetEmissiveUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetEmissiveFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factors) override;
  absl::Status SetClearcoatTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetClearcoatNormalTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_normal_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetClearcoatRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_roughness_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetClearcoatFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factor) override;
  absl::Status SetSheenColorTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t sheen_color_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetSheenColorFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factors) override;
  absl::Status SetSheenRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t sheen_roughness_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetSheenRoughnessFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) override;
  absl::Status SetTransmissionTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t transmission_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetTransmissionUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetTransmissionFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) override;
  absl::Status SetIndexOfRefractionOnGenericMaterial(
      std::intptr_t generic_material, float index_of_refraction) override;
  absl::Status SetAlphaCutoffOnGenericMaterial(std::intptr_t generic_material,
                                               float alpha_cutoff) override;

 private:
  // Returns a typed pointer to the generic material implementation.
  absl::StatusOr<split_engine::SplitEngineGenericMaterial*> GetMaterial(
      std::intptr_t material_handle);

  // Helper template to reduce boilerplate when setting a texture parameter.
  template <typename SetterFn>
  absl::Status SetTextureParameter(
      std::intptr_t generic_material, std::intptr_t texture,
      std::optional<filament::TextureSampler> sampler, SetterFn setter_fn);

  ImpressApiView& view_;
};

}  // namespace

GenericMaterialManagerImpl::GenericMaterialManagerImpl(ImpressApiView& view)
    : view_(view) {}

void GenericMaterialManagerImpl::CreateGenericMaterial(
    std::unique_ptr<BaseAssetLoader> asset_loader,
    GenericMaterialSpec generic_material_spec) {
  split_engine::SplitEngineGenericMaterial::Create(view_, generic_material_spec)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<
                    std::unique_ptr<split_engine::SplitEngineGenericMaterial>>
                    generic_material) {
        if (generic_material.ok() && *generic_material != nullptr) {
          OwnedPtr<split_engine::SplitEngineBuiltinMaterial> owned_material_ptr(
              *std::move(generic_material));
          std::intptr_t material_token = view_.ToJava(new BindingsMaterial(
              owned_material_ptr->GetMaterial(SmallSourceLocation::Current()),
              type_traits::kTypeHash<
                  split_engine::SplitEngineGenericMaterial>));
          view_.GetBindingsMaterialMap().emplace(material_token,
                                                 std::move(owned_material_ptr));
          asset_loader->OnSuccess(material_token);
        } else {
          asset_loader->OnFailure(absl::StrFormat(
              "Failed to create the built-in generic material: %s.",
              generic_material.status().message()));
        }
      })
      .KeptBy(&view_);
}

absl::StatusOr<split_engine::SplitEngineGenericMaterial*>
GenericMaterialManagerImpl::GetMaterial(std::intptr_t material_handle) {
  BindingsMaterial* bindings_material =
      view_.FromJava<BindingsMaterial>(material_handle);
  if (!bindings_material) {
    return absl::InvalidArgumentError("Provided material handle is not valid.");
  }

  HashValue material_type_hash = bindings_material->GetTypeHash();
  HashValue expected_type_hash =
      type_traits::kTypeHash<split_engine::SplitEngineGenericMaterial>;
  if (material_type_hash != expected_type_hash) {
    return absl::InvalidArgumentError(
        "Provided material handle is not of the correct type (Generic).");
  }

  auto it = view_.GetBindingsMaterialMap().find(material_handle);
  if (it == view_.GetBindingsMaterialMap().end()) {
    return absl::NotFoundError("Material handle not found in generic map.");
  }

  split_engine::SplitEngineBuiltinMaterial* base_material = &(*it->second);
  split_engine::SplitEngineGenericMaterial* derived_material =
      static_cast<split_engine::SplitEngineGenericMaterial*>(base_material);
  if (!derived_material) {
    return absl::InternalError(
        "Material type hash matched, but static_cast failed.");
  }

  return derived_material;
}

template <typename SetterFn>
absl::Status GenericMaterialManagerImpl::SetTextureParameter(
    std::intptr_t generic_material, std::intptr_t texture,
    std::optional<filament::TextureSampler> sampler, SetterFn setter_fn) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * generic_material_ptr,
      GetMaterial(generic_material));
  MP_ASSIGN_OR_RETURN(BorrowedTexturePtr borrowed_texture,
                   view_.GetTextureManager().BorrowTexture(texture));

  imp::GenericMaterialTextureParameter texture_parameter_payload;
  // This ID serves as a key for the TextureBorrower lambda. Impress will invoke
  // the lambda with this ID to retrieve the texture. As this function only sets
  // one texture, we only need one key. The key is therefore set to an arbitrary
  // value of “1”.
  uint64_t texture_id = 1;
  texture_parameter_payload.texture_id = texture_id;
  if (sampler.has_value()) {
    texture_parameter_payload.sampler = *sampler;
  }
  (void)setter_fn(
      generic_material_ptr, texture_parameter_payload,
      imp::TextureBorrower([expected_texture_id = texture_id,
                            borrowed_texture = std::move(borrowed_texture)](
                               uint64_t id) -> BorrowedTexturePtr {
        
        return borrowed_texture.WithNewLocation();
      }));
  return absl::OkStatus();
}

absl::Status GenericMaterialManagerImpl::SetBaseColorTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t base_color_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, base_color_texture, sampler,
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

absl::Status
GenericMaterialManagerImpl::SetBaseColorUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  return material->SetBaseColorUvTransform(uv_transform);
}

absl::Status GenericMaterialManagerImpl::SetBaseColorFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float4& factors) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetBaseColorFactor(factors);
  return absl::OkStatus();
}

absl::Status
GenericMaterialManagerImpl::SetMetallicRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t metallic_roughness_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, metallic_roughness_texture, sampler,
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

absl::Status
GenericMaterialManagerImpl::SetMetallicRoughnessUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  return material->SetMetallicRoughnessUvTransform(uv_transform);
}

absl::Status GenericMaterialManagerImpl::SetMetallicFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetMetallicFactor(factor);
  return absl::OkStatus();
}

absl::Status GenericMaterialManagerImpl::SetRoughnessFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetRoughnessFactor(factor);
  return absl::OkStatus();
}

absl::Status GenericMaterialManagerImpl::SetNormalTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t normal_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, normal_texture, sampler,
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

absl::Status GenericMaterialManagerImpl::SetNormalUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  return material->SetNormalUvTransform(uv_transform);
}

absl::Status GenericMaterialManagerImpl::SetNormalFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetNormalScale(factor);
  return absl::OkStatus();
}

absl::Status
GenericMaterialManagerImpl::SetAmbientOcclusionTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t ambient_occlusion_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, ambient_occlusion_texture, sampler,
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

absl::Status
GenericMaterialManagerImpl::SetAmbientOcclusionUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  return material->SetAmbientOcclusionUvTransform(uv_transform);
}

absl::Status
GenericMaterialManagerImpl::SetAmbientOcclusionFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetAmbientOcclusionStrength(factor);
  return absl::OkStatus();
}

absl::Status GenericMaterialManagerImpl::SetEmissiveTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t emissive_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, emissive_texture, sampler,
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

absl::Status
GenericMaterialManagerImpl::SetEmissiveUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  return material->SetEmissiveUvTransform(uv_transform);
}

absl::Status GenericMaterialManagerImpl::SetEmissiveFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factors) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetEmissiveFactor(factors);
  return absl::OkStatus();
}

absl::Status GenericMaterialManagerImpl::SetClearcoatTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, clearcoat_texture, sampler,
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

absl::Status
GenericMaterialManagerImpl::SetClearcoatNormalTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_normal_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, clearcoat_normal_texture, sampler,
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

absl::Status
GenericMaterialManagerImpl::SetClearcoatRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_roughness_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, clearcoat_roughness_texture, sampler,
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

absl::Status GenericMaterialManagerImpl::SetClearcoatFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factor) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetClearcoatFactors(factor);
  return absl::OkStatus();
}

absl::Status GenericMaterialManagerImpl::SetSheenColorTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t sheen_color_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, sheen_color_texture, sampler,
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

absl::Status GenericMaterialManagerImpl::SetSheenColorFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factors) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetSheenColorFactor(factors);
  return absl::OkStatus();
}

absl::Status
GenericMaterialManagerImpl::SetSheenRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t sheen_roughness_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, sheen_roughness_texture, sampler,
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

absl::Status
GenericMaterialManagerImpl::SetSheenRoughnessFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetSheenRoughnessFactor(factor);
  return absl::OkStatus();
}

absl::Status
GenericMaterialManagerImpl::SetTransmissionTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t transmission_texture,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      generic_material, transmission_texture, sampler,
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

absl::Status
GenericMaterialManagerImpl::SetTransmissionUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  return material->SetTransmissionUvTransform(uv_transform);
}

absl::Status GenericMaterialManagerImpl::SetTransmissionFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetTransmissionFactor(factor);
  return absl::OkStatus();
}

absl::Status GenericMaterialManagerImpl::SetIndexOfRefractionOnGenericMaterial(
    std::intptr_t generic_material, float index_of_refraction) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetIndexOfRefraction(index_of_refraction);
  return absl::OkStatus();
}

absl::Status GenericMaterialManagerImpl::SetAlphaCutoffOnGenericMaterial(
    std::intptr_t generic_material, float alpha_cutoff) {
  MP_ASSIGN_OR_RETURN(split_engine::SplitEngineGenericMaterial * material,
                   GetMaterial(generic_material));
  material->SetAlphaCutoff(alpha_cutoff);
  return absl::OkStatus();
}

std::unique_ptr<GenericMaterialManager> CreateGenericMaterialManager(
    ImpressApiView& view) {
  return std::make_unique<GenericMaterialManagerImpl>(view);
}

}  // namespace imp
