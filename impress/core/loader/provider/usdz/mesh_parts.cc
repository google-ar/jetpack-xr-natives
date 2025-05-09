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

#include "core/loader/provider/usdz/mesh_parts.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/optional_error.h"
#include "core/common/platform_helpers.h"
#include "third_party/tinyusdz/src/prim-types.hh"
#include "third_party/tinyusdz/src/usdGeom.hh"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_usdz {

using ::tinyusdz::Path;

namespace {

absl::StatusOr<Path> GetMaterialPath(
    const tinyusdz::Relationship &material_binding) {
  std::optional<Path> material_path;
  if (material_binding.is_path()) {
    material_path.emplace(material_binding.targetPath);
  } else if (material_binding.is_pathvector()) {
    if (!material_binding.targetPathVector.empty()) {
      material_path.emplace(material_binding.targetPathVector[0]);
    }
  }
  if (!material_path) return Error("Failed to find material path");
  return material_path.value();
}

}  // namespace

using ::tinyusdz::Path;

absl::StatusOr<MeshParts> MeshParts::FromMesh(
    const tinyusdz::Prim &prim,
    const std::vector<int32_t> &face_vertex_counts) {
  MeshParts result;
  for (const auto &child : prim.children()) {
    if (!child.is<tinyusdz::Model>()) continue;
    const tinyusdz::Model *model = child.as<tinyusdz::Model>();
    std::vector<int32_t> mesh_face_indices;
    std::optional<Path> material_path;

    for (const auto &[name, property] : model->props) {
      const tinyusdz::Attribute &attr = property.get_attribute();
      constexpr auto kFaceIndicesPropertyName = "indices";
      constexpr auto kMaterialBindingPropertyName = "material:binding";
      constexpr auto kFamilyNamePropertyName = "familyName";
      constexpr auto kElementTypePropertyName = "elementType";

      if (name == kFaceIndicesPropertyName) {
        if (!attr.get_value(&mesh_face_indices))
          return absl::InternalError("Failed to parse face indices");
      } else if (name == kMaterialBindingPropertyName) {
        if (attr.metas().bindMaterialAs) {
          IMP_LOG(imp::ERROR) << "@-- test1: " << attr.metas().bindMaterialAs->str();
        }
        if (property.is_relationship()) {
          MP_ASSIGN_OR_RETURN(material_path,
                           GetMaterialPath(property.get_relationship()));
        }
      } else if ((name == kFamilyNamePropertyName) ||
                 (name == kElementTypePropertyName)) {
        // Ignored properties
      } else {
        return absl::InternalError(absl::StrFormat("Unknown property %s [%s]",
                                                   name, attr.type_name()));
      }
    }
    if (!mesh_face_indices.empty() && material_path) {
      size_t triangle_count = absl::c_accumulate(
          mesh_face_indices, 0ul,
          [&face_vertex_counts](size_t running_total, int32_t face_index) {
            return running_total +
                   static_cast<size_t>(face_vertex_counts[face_index] - 2);
          });

      result.face_indices.push_back(std::move(mesh_face_indices));
      result.material_paths.push_back(std::move(material_path.value()));
      result.triangle_counts.push_back(triangle_count);
    } else if (!mesh_face_indices.empty()) {
      return absl::InternalError("Missing material for part");
    }
  }

  if (result.face_indices.empty()) {
    const tinyusdz::GeomMesh *mesh = prim.as<tinyusdz::GeomMesh>();
    if (!mesh->materialBinding) return Error("childless mesh missing material");
    MP_ASSIGN_OR_RETURN(Path material_path,
                     GetMaterialPath(*mesh->materialBinding));

    size_t triangle_count = absl::c_accumulate(
        face_vertex_counts, 0ul,
        [](size_t running_total, int32_t face_vertex_count) {
          return running_total + static_cast<size_t>(face_vertex_count - 2);
        });
    std::vector<int32_t> mesh_face_indices(face_vertex_counts.size());
    std::iota(mesh_face_indices.begin(), mesh_face_indices.end(), 0);

    result.face_indices.push_back(std::move(mesh_face_indices));
    result.material_paths.push_back(std::move(material_path));
    result.triangle_counts.push_back(triangle_count);
  }
  return result;
}

}  // namespace imp::loader::details::provider_usdz
