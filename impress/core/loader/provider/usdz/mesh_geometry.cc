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

#include "core/loader/provider/usdz/mesh_geometry.h"

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "third_party/tinyusdz/src/prim-types.hh"
#include "third_party/tinyusdz/src/usdGeom.hh"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_usdz {

namespace {

// Wraps the chain of accessors in tinyusdz to simplify extracting bulk data.
template <typename T>
absl::Status Collapse(
    const tinyusdz::TypedAttribute<tinyusdz::Animatable<T>> &attr,
    T *collapsed) {
  if (auto attr_value = attr.get_value()) {
    if (attr_value.value().get_scalar(collapsed)) {
      return absl::OkStatus();
    }
  }
  return absl::InternalError("Could not collapse attribute");
}

}  // namespace

// static
absl::StatusOr<MeshGeometry> MeshGeometry::Collect(
    const tinyusdz::GeomMesh &mesh) {
  MeshGeometry result;
  MP_RETURN_IF_ERROR(Collapse(mesh.faceVertexCounts, &result.face_vertex_counts));
  MP_RETURN_IF_ERROR(
      Collapse(mesh.faceVertexIndices, &result.face_vertex_indices));

  for (const auto &[name, property] : mesh.props) {
    const tinyusdz::Attribute &attr = property.get_attribute();
    constexpr auto kTexcordPropertyName = "primvars:st";
    constexpr auto kTexcordIndicesPropertyName = "primvars:st:indices";
    constexpr auto kDisplayColorPropertyName = "primvars:displayColor";
    constexpr auto kDisplayColorIndicesPropertyName =
        "primvars:displayColor:indices";
    constexpr auto kOpacityPropertyName = "primvars:displayOpacity";
    constexpr auto kOpacityIndicesPropertyName =
        "primvars:displayOpacity:indices";
    constexpr auto kSubsetFamilyTypePropertyName =
        "subsetFamily:materialBind:familyType";
    constexpr auto kSkelGeomBindTransformPropertyName =
        "primvars:skel:geomBindTransform";
    constexpr auto kSkelJointIndicesPropertyName = "primvars:skel:jointIndices";
    constexpr auto kSkelJointWeightsPropertyName = "primvars:skel:jointWeights";
    constexpr auto kSkelJointsPropertyName = "skel:joints";
    constexpr auto kSubdivisionSchemePropertyName =
        "userProperties:USD_ATTR_subdivisionScheme";
    if (name == kTexcordPropertyName) {
      if (!attr.get_value(&result.texcoords))
        return absl::InternalError("Failed to parse texcoords");
    } else if (name == kTexcordIndicesPropertyName) {
      if (!attr.get_value(&result.texcoord_indices))
        return absl::InternalError("Failed to parse texcoords");
    } else if ((name == kDisplayColorPropertyName) ||
               (name == kDisplayColorIndicesPropertyName) ||
               (name == kOpacityPropertyName) ||
               (name == kOpacityIndicesPropertyName) ||
               (name == kSubsetFamilyTypePropertyName) ||
               (name == kSkelGeomBindTransformPropertyName) ||
               (name == kSkelJointIndicesPropertyName) ||
               (name == kSkelJointWeightsPropertyName) ||
               (name == kSkelJointsPropertyName) ||
               (name == kSubdivisionSchemePropertyName)) {
      // Ignored properties.
    } else {
      return absl::InternalError(
          absl::StrFormat("Unknown property %s (%s)", name, attr.type_name()));
    }
  }

  if (result.points.empty())
    MP_RETURN_IF_ERROR(Collapse(mesh.points, &result.points));
  if (result.normals.empty())
    MP_RETURN_IF_ERROR(Collapse(mesh.normals, &result.normals));

  return result;
}

}  // namespace imp::loader::details::provider_usdz
