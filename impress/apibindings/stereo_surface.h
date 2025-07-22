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

#include <memory>
#include <variant>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "split_engine/materials/jxr_media_material.h"

namespace imp {

// A component that creates an Android surface texture and renders it to a mesh.
class StereoSurface : public Component {
 public:
  struct Quad {
    float width = 1.0f;
    float height = 1.0f;
  };

  struct Sphere {
    float radius = 1.0f;
  };

  struct Hemisphere {
    float radius = 1.0f;
  };

  using CanvasShape = std::variant<std::monostate, Quad, Sphere, Hemisphere>;

  absl::Status Setup(MediaStereoMode stereo_mode,
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

  // Dynamically updates the shape of the canvas.
  absl::Status SetCanvasShape(const CanvasShape& canvas_shape);

  // Sets the feather radius for the edges of the quad in UV space.
  void SetFeatherRadius(const float2& feather_radius);

  void SetPrimaryAlphaMask(OwnedOrBorrowedTexturePtr alpha_mask);
  void SetAuxiliaryAlphaMask(OwnedOrBorrowedTexturePtr auxiliary_alpha_mask);

  // Configures the color space metadata for content rendered on the stereo
  // surface. When set to an unknown color space, the system will attempt a
  // best-effort color conversion. If specific color space parameters are
  // provided, these will be used to explicitly define the source color space
  // for backend color conversion.
  void SetContentColorMetadata(MediaColorSpace color_space);

 private:
  std::unique_ptr<AndroidExternalTextureSurface> surface_;
  std::unique_ptr<android_xr::JxrMediaMaterial> material_;
  ComponentHandle<MeshRenderer> mesh_renderer_;
  Future<absl::Status> material_future_;

  MediaStereoMode stereo_mode_;
  CanvasShape canvas_shape_;
};

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_STEREO_SURFACE_H_
