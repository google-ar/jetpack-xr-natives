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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_MESH_GEOMETRY_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_MESH_GEOMETRY_H_

#include <cstdint>
#include <vector>

#include "absl/status/statusor.h"
#include "third_party/tinyusdz/src/value-types.hh"

namespace tinyusdz {
class GeomMesh;
}  // namespace tinyusdz

namespace imp::loader::details::provider_usdz {

struct MeshGeometry {
  std::vector<tinyusdz::value::point3f> points;
  std::vector<tinyusdz::value::normal3f> normals;
  std::vector<tinyusdz::value::texcoord2f> texcoords;
  std::vector<int32_t> texcoord_indices;
  std::vector<int32_t> face_vertex_counts;
  std::vector<int32_t> face_vertex_indices;

  static absl::StatusOr<MeshGeometry> Collect(const tinyusdz::GeomMesh &mesh);

  // Private constructed and move-only
  MeshGeometry(const MeshGeometry &) = delete;
  MeshGeometry &operator=(const MeshGeometry &rhs) = delete;
  MeshGeometry(MeshGeometry &&rhs) = default;
  MeshGeometry &operator=(MeshGeometry &&rhs) = default;

 private:
  MeshGeometry() = default;
};

}  // namespace imp::loader::details::provider_usdz

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_MESH_GEOMETRY_H_
