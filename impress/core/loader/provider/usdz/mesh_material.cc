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

#include "core/loader/provider/usdz/mesh_material.h"

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/robin_map.h"
#include "core/common/typed_id.h"
#include "core/common/typed_vector.h"
#include "third_party/tinyusdz/src/prim-types.hh"
#include "third_party/tinyusdz/src/usdShade.hh"
#include "third_party/tinyusdz/src/value-types.hh"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_usdz {

namespace {

template <typename T>
absl::StatusOr<std::optional<T>> GetTypedAttr(
    const tinyusdz::TypedAttribute<tinyusdz::Animatable<T>> &attr) {
  if (!attr.authored()) {
    return std::optional<T>{};
  }
  if (attr.is_blocked()) {
    return absl::InternalError("Blocked Attribute");
  } else if (attr.is_connection()) {
    return absl::InternalError("Connection Attribute");
  }

  auto pv = attr.get_value();

  if (!pv) {
    return absl::InternalError("Missing Attribute");
  }
  if (pv.value().is_timesamples()) {
    return absl::InternalError("Timesampled Attribute");
  }
  T a;
  if (!pv.value().get_scalar(&a)) {
    return absl::InternalError("Internal Error4");
  }
  return a;
}

template <typename T>
absl::StatusOr<std::optional<T>> GetTypedTokenAttr(
    const tinyusdz::TypedAttributeWithFallback<tinyusdz::Animatable<T>> &attr) {
  if (!attr.authored()) {
    return std::optional<T>{};
  }
  if (attr.is_blocked()) {
    return absl::InternalError("Blocked Attribute");
  } else if (attr.is_connection()) {
    return absl::InternalError("Connection Attribute");
  }
  tinyusdz::Animatable<T> pv = attr.get_value();

  if (pv.is_timesamples()) {
    return absl::InternalError("Timesampled Attribute");
  }
  T a;
  if (!pv.get_scalar(&a)) {
    return absl::InternalError("Internal Error3");
  }
  return a;
}
}  // namespace

using ::tinyusdz::Path;

template <typename T>
// static
absl::StatusOr<SurfaceAttribute<T>> MeshMaterial::ResolveAttribute(
    const tinyusdz::TypedAttributeWithFallback<tinyusdz::Animatable<T>>
        &attribute) {
  SurfaceAttribute<T> result = {};
  if (!attribute.authored()) {
    auto v = attribute.get_value();
    T a;
    if (v.get_scalar(&a)) {
      result = a;
    }
    return result;
  }
  if (attribute.is_connection()) {
    const std::vector<Path> &paths = attribute.get_connections();
    if (paths.size() == 1) {
      result = paths[0];
    } else {
      return absl::InternalError("Internal error");
    }
  } else if (attribute.is_value_empty()) {
    // nothing to do
  } else {
    auto v = attribute.get_value();

    if (v.is_timesamples()) {
      return absl::InternalError("Missing support for timesamples");
    } else if (v.is_blocked()) {
    } else if (v.is_scalar()) {
      T a;
      if (!v.get_scalar(&a)) {
        return absl::InternalError("Internal error");
      }
      result = a;
    } else {
      return absl::InternalError("Invalid animatable");
    }
  }

  if (attribute.metas().authored()) {
    return absl::InternalError("Missing metadata parsing");
  }
  return result;
}

absl::Status MeshMaterial::ExtractUvTexture(
    const tinyusdz::Prim &prim, const tinyusdz::UsdUVTexture &texture,
    MeshMaterial &material) {
  SurfaceTexture result;
  MP_ASSIGN_OR_RETURN(result.file, GetTypedAttr(texture.file));
  MP_ASSIGN_OR_RETURN(result.source_color_space,
                   GetTypedTokenAttr(texture.sourceColorSpace));
  MP_ASSIGN_OR_RETURN(result.wrap_s, GetTypedTokenAttr(texture.wrapS));
  MP_ASSIGN_OR_RETURN(result.wrap_t, GetTypedTokenAttr(texture.wrapT));

  material.surface_textures_from_name[prim.absolute_path().full_path_name()] =
      material.surface_textures.Append<SurfaceTextureId>(result);
  return absl::OkStatus();
}

// static
absl::StatusOr<MeshMaterial> MeshMaterial::Extract(const tinyusdz::Prim &prim) {
  const tinyusdz::Prim *surface_prim = nullptr;
  MeshMaterial result;

  // resolve textures
  for (const auto &child : prim.children()) {
    if (!child.is<tinyusdz::Shader>()) continue;
    const tinyusdz::Shader *shader = child.as<tinyusdz::Shader>();
    if (auto texture_value = shader->value.as<tinyusdz::UsdUVTexture>()) {
      MP_RETURN_IF_ERROR(ExtractUvTexture(child, *texture_value, result));
    } else if (auto surface_value =
                   shader->value.as<tinyusdz::UsdPreviewSurface>()) {
      if (surface_prim) return absl::InternalError("Multiple surfaces");
      surface_prim = &child;
    }
  }

  if (!surface_prim) return absl::InternalError("No surface");
  const tinyusdz::Shader *shader = surface_prim->as<tinyusdz::Shader>();
  const tinyusdz::UsdPreviewSurface &surface =
      *shader->value.as<tinyusdz::UsdPreviewSurface>();

  MP_ASSIGN_OR_RETURN(result.diffuse_color,
                   ResolveAttribute(surface.diffuseColor));
  MP_ASSIGN_OR_RETURN(result.emissive_color,
                   ResolveAttribute(surface.emissiveColor));
  MP_ASSIGN_OR_RETURN(result.use_specular_workflow,
                   ResolveAttribute(surface.useSpecularWorkflow));
  MP_ASSIGN_OR_RETURN(result.specular_color,
                   ResolveAttribute(surface.specularColor));
  MP_ASSIGN_OR_RETURN(result.metallic, ResolveAttribute(surface.metallic));
  MP_ASSIGN_OR_RETURN(result.clearcoat, ResolveAttribute(surface.clearcoat));
  MP_ASSIGN_OR_RETURN(result.clearcoat_roughness,
                   ResolveAttribute(surface.clearcoatRoughness));
  MP_ASSIGN_OR_RETURN(result.roughness, ResolveAttribute(surface.roughness));
  MP_ASSIGN_OR_RETURN(result.opacity, ResolveAttribute(surface.opacity));
  MP_ASSIGN_OR_RETURN(result.opacity_threshold,
                   ResolveAttribute(surface.opacityThreshold));
  MP_ASSIGN_OR_RETURN(result.ior, ResolveAttribute(surface.ior));
  MP_ASSIGN_OR_RETURN(result.normal, ResolveAttribute(surface.normal));
  MP_ASSIGN_OR_RETURN(result.displacement, ResolveAttribute(surface.displacement));
  MP_ASSIGN_OR_RETURN(result.occlusion, ResolveAttribute(surface.occlusion));

  return result;
}

}  // namespace imp::loader::details::provider_usdz
