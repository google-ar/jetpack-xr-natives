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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_MESH_SURFACE_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_MESH_SURFACE_H_

#include <optional>
#include <variant>

#include "absl/status/statusor.h"
#include "core/common/robin_map.h"
#include "core/common/typed_id.h"
#include "core/common/typed_vector.h"
#include "third_party/tinyusdz/src/prim-types.hh"
#include "third_party/tinyusdz/src/usdGeom.hh"
#include "third_party/tinyusdz/src/usdShade.hh"
#include "third_party/tinyusdz/src/value-types.hh"

namespace imp::loader::details::provider_usdz {

template <typename T>
using SurfaceAttribute = std::variant<std::monostate, T, tinyusdz::Path>;

struct SurfaceTexture {
  std::optional<tinyusdz::value::AssetPath> file;
  std::optional<tinyusdz::UsdUVTexture::SourceColorSpace> source_color_space;
  std::optional<tinyusdz::UsdUVTexture::Wrap> wrap_s;
  std::optional<tinyusdz::UsdUVTexture::Wrap> wrap_t;
};
using SurfaceTextureId = TypedId<SurfaceTexture, int16_t>;

class MeshMaterial {
 public:
  using color3f = tinyusdz::value::color3f;
  using normal3f = tinyusdz::value::normal3f;

  SurfaceAttribute<color3f> diffuse_color;
  SurfaceAttribute<color3f> emissive_color;
  SurfaceAttribute<int> use_specular_workflow;
  SurfaceAttribute<color3f> specular_color;
  SurfaceAttribute<float> metallic;
  SurfaceAttribute<float> roughness;
  SurfaceAttribute<float> clearcoat;
  SurfaceAttribute<float> clearcoat_roughness;
  SurfaceAttribute<float> opacity;
  SurfaceAttribute<float> opacity_threshold;
  SurfaceAttribute<float> ior;
  SurfaceAttribute<normal3f> normal;
  SurfaceAttribute<float> displacement;
  SurfaceAttribute<float> occlusion;

  static absl::StatusOr<MeshMaterial> Extract(const tinyusdz::Prim &prim);

  RobinMap<std::string, SurfaceTextureId> surface_textures_from_name;
  TypedVector<SurfaceTexture> surface_textures;

  // Private constructed and move-only
  MeshMaterial(const MeshMaterial &) = delete;
  MeshMaterial &operator=(const MeshMaterial &rhs) = delete;
  MeshMaterial(MeshMaterial &&rhs) = default;
  MeshMaterial &operator=(MeshMaterial &&rhs) = default;

 private:
  MeshMaterial() = default;
  static absl::Status ExtractUvTexture(const tinyusdz::Prim &prim,
                                       const tinyusdz::UsdUVTexture &texture,
                                       MeshMaterial &material);
  template <typename T>
  static absl::StatusOr<SurfaceAttribute<T>> ResolveAttribute(
      const tinyusdz::TypedAttributeWithFallback<tinyusdz::Animatable<T>>
          &attribute);
};

}  // namespace imp::loader::details::provider_usdz

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_MESH_SURFACE_H_
