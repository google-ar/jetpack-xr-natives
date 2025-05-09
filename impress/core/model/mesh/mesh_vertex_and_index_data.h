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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_VERTEX_AND_INDEX_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_VERTEX_AND_INDEX_DATA_H_

#include <cstdint>

#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"

namespace imp {

// A non-owning pair of MeshVertexData and MeshIndexData.
//
// NOTE: This structure does not guarantee any correspondence, nor does it own
// the underlying vertex and index data. It can be used to provide convenient
// and temporary pairing between vertex data and index data. Please note the
// difference between this structure and MeshData, and choose carefully.
struct MeshVertexAndIndexData {
  MeshVertexData* vertex_data;
  MeshIndexData* index_data;
};

// Used indicates a submesh's indices as in a range of indices in its parent
// mesh, or the whole mesh.
struct MeshRange {
  int32_t offset;
  int32_t count;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_VERTEX_AND_INDEX_DATA_H_
