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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_MESH_PARTS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_MESH_PARTS_H_

#include "absl/status/statusor.h"
#include "core/common/paired_vector.h"
#include "core/common/typed_id.h"
#include "third_party/tinyusdz/src/prim-types.hh"

namespace imp::loader::details::provider_usdz {

struct MeshPart;
using PartId = TypedId<MeshPart, uint16_t>;
template <typename T>
using PartLookup = PairedVector<T, MeshPart>;

struct MeshParts {
  PartLookup<std::vector<int32_t>> face_indices;
  PartLookup<tinyusdz::Path> material_paths;
  PartLookup<size_t> triangle_counts;

  static absl::StatusOr<MeshParts> FromMesh(
      const tinyusdz::Prim &prim,
      const std::vector<int32_t> &face_vertex_counts);

  // Private constructed and move-only
  MeshParts(const MeshParts &) = delete;
  MeshParts &operator=(const MeshParts &rhs) = delete;
  MeshParts(MeshParts &&rhs) = default;
  MeshParts &operator=(MeshParts &&rhs) = default;

 private:
  MeshParts() = default;
};

}  // namespace imp::loader::details::provider_usdz

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_MESH_PARTS_H_
