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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_MEDIA_MATERIAL_CACHE_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_MEDIA_MATERIAL_CACHE_H_

#include <memory>
#include <optional>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/texture.h"
#include "split_engine/materials/jxr_media_material.h"

namespace imp {

class MediaMaterialCache {
 public:
  using MaterialPtr = std::unique_ptr<android_xr::JxrMediaMaterial>;

  // Returns the material for the given key, or nullptr if not found.
  android_xr::JxrMediaMaterial* Get(RenderEyeTarget eye_target,
                                    MediaBlendingMode blending_mode);

  // Sets the material for the given key and returns a pointer to it.
  android_xr::JxrMediaMaterial* Set(RenderEyeTarget eye_target,
                                    MediaBlendingMode blending_mode,
                                    MaterialPtr material);

  // Sets the auxiliary alpha mask for the given targets and blending mode.
  void SetAuxiliaryAlphaMask(absl::Span<const RenderEyeTarget> targets,
                             MediaBlendingMode blending_mode,
                             BorrowedTexturePtr texture);

  // Sets the primary alpha mask for the given targets and blending mode.
  void SetPrimaryAlphaMask(absl::Span<const RenderEyeTarget> targets,
                           MediaBlendingMode blending_mode,
                           BorrowedTexturePtr texture);

  // Sets the stereo type for the given targets and blending mode.
  void SetStereoType(absl::Span<const RenderEyeTarget> targets,
                     MediaBlendingMode blending_mode,
                     MediaStereoMode stereo_mode);

  // Sets the content color metadata for the given targets and blending mode.
  void SetContentColorMetadata(absl::Span<const RenderEyeTarget> targets,
                               MediaBlendingMode blending_mode,
                               const MediaColorSpace& color_space);

  // Sets the feather radius for the given targets and blending mode.
  void SetFeatherRadius(absl::Span<const RenderEyeTarget> targets,
                        MediaBlendingMode blending_mode, float2 feather_radius);

  // Sets the sub view config for the given targets and blending mode.
  void SetSubViewConfig(absl::Span<const RenderEyeTarget> targets,
                        MediaBlendingMode blending_mode,
                        float4 sub_view_rect_left, float4 sub_view_rect_right);

  // Sets the primary texture for the given targets and blending mode.
  void SetPrimaryTexture(absl::Span<const RenderEyeTarget> targets,
                         MediaBlendingMode blending_mode,
                         BorrowedTexturePtr texture);

  // Sets the auxiliary texture for the given targets and blending mode.
  void SetAuxiliaryTexture(absl::Span<const RenderEyeTarget> targets,
                           MediaBlendingMode blending_mode,
                           BorrowedTexturePtr texture);

  // Sets the corner radius for the given targets and blending mode.
  void SetCornerRadius(absl::Span<const RenderEyeTarget> targets,
                       MediaBlendingMode blending_mode, float2 corner_radius);

  // Resets all textures to the provided texture.
  // This is expected to be used with the Placeholder texture.
  // This updates the cached parameters for all targets and applies the texture
  // to all existing materials in the cache.
  void ResetTextures(BorrowedTexturePtr texture);

  // Returns true if the cache contains a material for the given key.
  bool Contains(RenderEyeTarget eye_target,
                MediaBlendingMode blending_mode) const;

 private:
  template <typename Func>
  void ApplyToMaterials(absl::Span<const RenderEyeTarget> targets,
                        MediaBlendingMode blending_mode, Func&& func);

  // Applies the cached parameters for the given eye target to the material.
  void ApplyParametersTo(android_xr::JxrMediaMaterial* material,
                         RenderEyeTarget target,
                         MediaBlendingMode blending_mode);

  struct MaterialParameters {
    BorrowedTexturePtr primary_texture;
    BorrowedTexturePtr auxiliary_texture;
    BorrowedTexturePtr primary_alpha_mask;
    BorrowedTexturePtr auxiliary_alpha_mask;
    std::optional<MediaStereoMode> stereo_mode;
    std::optional<MediaColorSpace> color_space;
    std::optional<float2> feather_radius;
    std::optional<float2> corner_radius;
    std::optional<float4> sub_view_rect_left;
    std::optional<float4> sub_view_rect_right;
  };

  struct Key {
    RenderEyeTarget eye_target = {};
    MediaBlendingMode blending_mode = {};

    template <typename H>
    friend H AbslHashValue(H h, const Key& k) {
      return H::combine(std::move(h), k.eye_target, k.blending_mode);
    }

    bool operator==(const Key& other) const {
      return eye_target == other.eye_target &&
             blending_mode == other.blending_mode;
    }
  };

  absl::flat_hash_map<Key, MaterialPtr> materials_;
  absl::flat_hash_map<RenderEyeTarget, MaterialParameters> parameters_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_MEDIA_MATERIAL_CACHE_H_
