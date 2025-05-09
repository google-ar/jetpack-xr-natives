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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_DESCRIPTION_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_DESCRIPTION_H_

#include "core/model/mesh/vertex_format.h"
#include "filament/filament/include/filament/IndexBuffer.h"

namespace imp {

// A description of the vertex and index buffers in a mesh.
struct MeshDescription {
  using IndexType = filament::IndexBuffer::IndexType;

  // Tests if the meshes described by two MeshDescriptions are equal.
  bool operator==(const MeshDescription& rhs) const;
  bool operator!=(const MeshDescription& rhs) const;

  // Size of the type of index.
  size_t GetIndexSize() const;

  VertexFormat vertex_format;
  IndexType index_type;
  size_t vertex_count;
  size_t index_count;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_DESCRIPTION_H_
