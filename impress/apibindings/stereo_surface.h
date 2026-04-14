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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_STEREO_SURFACE_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_STEREO_SURFACE_H_
#include <sys/types.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <variant>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "apibindings/media_material_cache.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/mesh_renderer.h"
#include "core/render/texture.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "split_engine/materials/jxr_media_material.h"

namespace imp {

// A component that creates an Android surface texture and renders it to a mesh.
class StereoSurface : public Component {
 public:
  struct Quad {
    float width = 1.0f;
    float height = 1.0f;
    float corner_radius = 0.0f;
  };

  struct Sphere {
    float radius = 1.0f;
  };

  struct Hemisphere {
    float radius = 1.0f;
  };

  struct StereoMesh {
    // Left eye vertex positions.
    std::vector<float> left_positions;
    // Left eye vertex texture coordinates.
    std::vector<float> left_texcoords;
    // Left eye vertex indices.
    std::optional<std::vector<uint32_t>> left_indices;
    // Right eye vertex positions.
    std::optional<std::vector<float>> right_positions;
    // Right eye vertex texture coordinates.
    std::optional<std::vector<float>> right_texcoords;
    // Right eye vertex indices.
    std::optional<std::vector<uint32_t>> right_indices;
    // Draw mode.
    filament::RenderableManager::PrimitiveType draw_mode;
  };

  struct CurvedRect {
    float width = 1.0f;
    float height = 1.0f;
    float corner_radius = 0.0f;
    float curve_radius = std::numeric_limits<float>::infinity();
  };

  using CanvasShape = std::variant<std::monostate, Quad, Sphere, Hemisphere,
                                   StereoMesh, CurvedRect>;

  absl::Status Setup(MediaStereoMode stereo_mode,
                     MediaBlendingMode blending_mode,
                     ContentSecurityLevel content_security_level,
                     bool use_super_sampling);
  void Cleanup();

  // TODO: (broken link) - The surface is returned before the material is
  // assigned. Decide if this should return a material or use some other
  // mechanism
  absl::StatusOr<android::Surface*> GetSurface();

  // Matches the values returned by media3. See Also:
  // third_party/java_src/android_libs/media/libraries/common/src/main/java/androidx/media3/common/C.java
  void SetStereoMode(MediaStereoMode stereo_mode);

  // Sets the blending mode for the surface.
  void SetBlendingMode(MediaBlendingMode blending_mode);

  // Dynamically updates the shape of the canvas.
  absl::Status SetCanvasShape(const CanvasShape& canvas_shape);

  // Dynamically enables or disables the collider.
  // The collider shape is determined by the canvas shape.
  //   - Quad -> BoxCollider
  //   - Sphere -> SphereCollider
  //   - Hemisphere -> DoNothing (Not ready yet)
  // TODO: (broken link) - Support MeshCollider for hemisphere once
  //   the bug is fixed.
  absl::Status SetColliderEnabled(bool enable_collider);

  // Sets the feather radius for the edges of the quad in UV space.
  void SetFeatherRadius(const float2& feather_radius);

  // This is needed to support android.graphics.Canvas methods.
  absl::Status SetSurfaceDimensions(int width, int height);

  void SetPrimaryAlphaMask(OwnedOrBorrowedTexturePtr alpha_mask);
  void SetAuxiliaryAlphaMask(OwnedOrBorrowedTexturePtr auxiliary_alpha_mask);

  // Configures the color space metadata for content rendered on the stereo
  // surface. When set to an unknown color space, the system will attempt a
  // best-effort color conversion. If specific color space parameters are
  // provided, these will be used to explicitly define the source color space
  // for backend color conversion.
  void SetContentColorMetadata(MediaColorSpace color_space);

  // Sets the subview rect for the stereo surface.
  void SetSubViewRects(const float4& left_rect, const float4& right_rect);

 private:
  enum class ColliderType {
    kNone,
    kUnknown,
    kPanel,
    kSphere,
    kMesh,
    // A mesh collider on a child node of the stereo surface, which is a
    // workaround for older system images.
    kWorkaroundMesh
  };

  std::unique_ptr<AndroidExternalTextureSurface> surface_;
  ComponentHandle<MeshRenderer> mesh_renderer_left_or_both_;

  NodeHandle right_eye_node_;
  ComponentHandle<MeshRenderer> mesh_renderer_right_;
  Future<absl::Status> per_eye_material_future_;

  MediaMaterialCache material_cache_;
  absl::flat_hash_map<RenderEyeTarget, Future<absl::Status>> material_futures_;
  bool is_per_eye_ = false;

  MediaStereoMode stereo_mode_;
  CanvasShape canvas_shape_;
  ColliderType collider_type_ = ColliderType::kNone;
  MediaBlendingMode blending_mode_ = MediaBlendingMode::kTransparent;
  bool use_super_sampling_ = false;

  bool GetColliderEnabled() const;
  absl::Status UpdateColliderTypeByShape(const CanvasShape& canvas_shape);
  void CleanupColliderType();

  Future<absl::Status> InitializeMaterial(RenderEyeTarget eye_target);

  Future<std::unique_ptr<android_xr::JxrMediaMaterial>> CreateJxrMediaMaterial(
      RenderEyeTarget eye_target, bool use_super_sampling,
      MediaBlendingMode blending_mode);

  // Recreates the materials for the surface. This should be called after
  // updating any material compile-time constant parameters.
  void RecreateMaterials();

  // Creates a new material for the given eye target and blending mode, or
  // returns an existing material if one already exists.
  Future<android_xr::JxrMediaMaterial*> GetOrCreateMaterial(
      RenderEyeTarget eye_target, bool use_super_sampling,
      MediaBlendingMode blending_mode);

  // True if the old workaround of creating a child node for mesh colliders
  // should be used.
  // TODO: (broken link) - Determine this value at runtime based on a system image
  // version check once the system image version API is ready. This is to
  // avoid crashes on older system images that do not support the MeshCollider
  // without this workaround.
  bool use_mesh_collider_workaround_ = true;
};

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_STEREO_SURFACE_H_
