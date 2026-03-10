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

#include "apibindings/media_material_cache.h"

#include <memory>
#include <utility>

#include "absl/types/span.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/texture.h"
#include "split_engine/materials/jxr_media_material.h"

namespace imp {

namespace {

void SetEffectiveFeatherRadius(android_xr::JxrMediaMaterial* material,
                               MediaBlendingMode blending_mode,
                               float2 feather_radius) {
  if (material == nullptr) {
    return;
  }
  float2 effective_feather_radius =
      blending_mode == MediaBlendingMode::kOpaque ? kZero2 : feather_radius;
  material->SetFeatherRadius(effective_feather_radius);
}

}  // namespace

android_xr::JxrMediaMaterial* MediaMaterialCache::Get(
    RenderEyeTarget eye_target, MediaBlendingMode blending_mode) {
  auto it = materials_.find({eye_target, blending_mode});
  if (it == materials_.end()) {
    return nullptr;
  }
  ApplyParametersTo(it->second.get(), eye_target, blending_mode);
  return it->second.get();
}

android_xr::JxrMediaMaterial* MediaMaterialCache::Set(
    RenderEyeTarget eye_target, MediaBlendingMode blending_mode,
    MediaMaterialCache::MaterialPtr material) {
  auto& stored_material = materials_[{eye_target, blending_mode}];
  stored_material = std::move(material);
  ApplyParametersTo(stored_material.get(), eye_target, blending_mode);
  return stored_material.get();
}

template <typename Func>
void MediaMaterialCache::ApplyToMaterials(
    absl::Span<const RenderEyeTarget> targets, MediaBlendingMode blending_mode,
    Func&& func) {
  for (auto target : targets) {
    if (auto* material = Get(target, blending_mode)) {
      func(material);
    }
  }
}

void MediaMaterialCache::SetAuxiliaryAlphaMask(
    absl::Span<const RenderEyeTarget> targets, MediaBlendingMode blending_mode,
    BorrowedTexturePtr texture) {
  for (auto target : targets) {
    parameters_[target].auxiliary_alpha_mask = texture;
  }
  ApplyToMaterials(targets, blending_mode,
                   [texture](android_xr::JxrMediaMaterial* material) {
                     material->SetAuxiliaryAlphaMask(texture);
                   });
}

void MediaMaterialCache::SetPrimaryAlphaMask(
    absl::Span<const RenderEyeTarget> targets, MediaBlendingMode blending_mode,
    BorrowedTexturePtr texture) {
  for (auto target : targets) {
    parameters_[target].primary_alpha_mask = texture;
  }
  ApplyToMaterials(targets, blending_mode,
                   [texture](android_xr::JxrMediaMaterial* material) {
                     material->SetPrimaryAlphaMask(texture);
                   });
}

void MediaMaterialCache::SetStereoType(
    absl::Span<const RenderEyeTarget> targets, MediaBlendingMode blending_mode,
    MediaStereoMode stereo_mode) {
  for (auto target : targets) {
    parameters_[target].stereo_mode = stereo_mode;
  }
  ApplyToMaterials(targets, blending_mode,
                   [stereo_mode](android_xr::JxrMediaMaterial* material) {
                     material->SetStereoType(stereo_mode);
                   });
}

void MediaMaterialCache::SetContentColorMetadata(
    absl::Span<const RenderEyeTarget> targets, MediaBlendingMode blending_mode,
    const MediaColorSpace& color_space) {
  for (auto target : targets) {
    parameters_[target].color_space = color_space;
  }
  ApplyToMaterials(targets, blending_mode,
                   [color_space](android_xr::JxrMediaMaterial* material) {
                     material->SetContentColorMetadata(color_space);
                   });
}

void MediaMaterialCache::SetFeatherRadius(
    absl::Span<const RenderEyeTarget> targets, MediaBlendingMode blending_mode,
    float2 feather_radius) {
  for (auto target : targets) {
    parameters_[target].feather_radius = feather_radius;
  }
  ApplyToMaterials(
      targets, blending_mode,
      [blending_mode, feather_radius](android_xr::JxrMediaMaterial* material) {
        SetEffectiveFeatherRadius(material, blending_mode, feather_radius);
      });
}

void MediaMaterialCache::SetSubViewConfig(
    absl::Span<const RenderEyeTarget> targets, MediaBlendingMode blending_mode,
    float4 sub_view_rect_left, float4 sub_view_rect_right) {
  for (auto target : targets) {
    parameters_[target].sub_view_rect_left = sub_view_rect_left;
    parameters_[target].sub_view_rect_right = sub_view_rect_right;
  }
  ApplyToMaterials(
      targets, blending_mode,
      [sub_view_rect_left,
       sub_view_rect_right](android_xr::JxrMediaMaterial* material) {
        material->SetSubViewConfig(sub_view_rect_left, sub_view_rect_right);
      });
}

void MediaMaterialCache::SetPrimaryTexture(
    absl::Span<const RenderEyeTarget> targets, MediaBlendingMode blending_mode,
    BorrowedTexturePtr texture) {
  for (auto target : targets) {
    parameters_[target].primary_texture = texture;
  }
  ApplyToMaterials(targets, blending_mode,
                   [texture](android_xr::JxrMediaMaterial* material) {
                     material->SetPrimaryTexture(texture);
                   });
}

void MediaMaterialCache::SetAuxiliaryTexture(
    absl::Span<const RenderEyeTarget> targets, MediaBlendingMode blending_mode,
    BorrowedTexturePtr texture) {
  for (auto target : targets) {
    parameters_[target].auxiliary_texture = texture;
  }
  ApplyToMaterials(targets, blending_mode,
                   [texture](android_xr::JxrMediaMaterial* material) {
                     material->SetAuxiliaryTexture(texture);
                   });
}

void MediaMaterialCache::SetCornerRadius(
    absl::Span<const RenderEyeTarget> targets, MediaBlendingMode blending_mode,
    float2 corner_radius) {
  for (auto target : targets) {
    parameters_[target].corner_radius = corner_radius;
  }
  ApplyToMaterials(targets, blending_mode,
                   [corner_radius](android_xr::JxrMediaMaterial* material) {
                     material->SetCornerRadius(corner_radius);
                   });
}

void MediaMaterialCache::ResetTextures(BorrowedTexturePtr texture) {
  for (auto& [target, params] : parameters_) {
    params.primary_texture = texture;
    params.auxiliary_texture = texture;
    params.primary_alpha_mask = texture;
    params.auxiliary_alpha_mask = texture;
  }

  for (auto& [key, material] : materials_) {
    if (material) {
      material->SetPrimaryTexture(texture);
      material->SetAuxiliaryTexture(texture);
      material->SetPrimaryAlphaMask(texture);
      material->SetAuxiliaryAlphaMask(texture);
    }
  }
}

void MediaMaterialCache::ApplyParametersTo(
    android_xr::JxrMediaMaterial* material, RenderEyeTarget target,
    MediaBlendingMode blending_mode) {
  if (!material) {
    return;
  }
  const auto& params = parameters_[target];
  if (params.primary_texture) {
    material->SetPrimaryTexture(params.primary_texture);
  }
  if (params.auxiliary_texture) {
    material->SetAuxiliaryTexture(params.auxiliary_texture);
  }
  if (params.primary_alpha_mask) {
    material->SetPrimaryAlphaMask(params.primary_alpha_mask);
  }
  if (params.auxiliary_alpha_mask) {
    material->SetAuxiliaryAlphaMask(params.auxiliary_alpha_mask);
  }
  if (params.stereo_mode.has_value()) {
    material->SetStereoType(*params.stereo_mode);
  }
  if (params.color_space.has_value()) {
    material->SetContentColorMetadata(*params.color_space);
  }
  if (params.feather_radius.has_value()) {
    SetEffectiveFeatherRadius(material, blending_mode, *params.feather_radius);
  }
  if (params.corner_radius.has_value()) {
    material->SetCornerRadius(*params.corner_radius);
  }
  if (params.sub_view_rect_left.has_value() &&
      params.sub_view_rect_right.has_value()) {
    material->SetSubViewConfig(*params.sub_view_rect_left,
                               *params.sub_view_rect_right);
  }
}

bool MediaMaterialCache::Contains(RenderEyeTarget eye_target,
                                  MediaBlendingMode blending_mode) const {
  return materials_.contains({eye_target, blending_mode});
}

}  // namespace imp
