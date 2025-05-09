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

#include "core/model/mesh/mesh_data.h"

#include <cstddef>

#include "core/common/log.h"
#include "core/common/platform_helpers.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_index_data.h"

namespace imp {

MeshData::MeshData(const MeshDescription& description)
    : description_(description),
      vertex_data_(description),
      index_data_(description) {}

const MeshDescription& MeshData::GetDescription() const { return description_; }

MeshData::BufferDescriptor MeshData::CopyVertexData() const {
  return vertex_data_.CopyVertexData();
}

MeshData::BufferDescriptor MeshData::CopyIndexData() const {
  return index_data_.CopyIndexData();
}

MeshData::BufferDescriptor MeshData::MoveVertexData() {
  description_.vertex_count = 0;
  return vertex_data_.MoveVertexData();
}

MeshData::BufferDescriptor MeshData::MoveIndexData() {
  description_.index_count = 0;
  return index_data_.MoveIndexData();
}

void MeshData::TruncateVertices(size_t new_count) {
  vertex_data_.TruncateVertices(new_count);
  description_.vertex_count = new_count;
}

void MeshData::TruncateIndices(size_t new_count) {
  index_data_.TruncateIndices(new_count);
  description_.index_count = new_count;
}

}  // namespace imp
