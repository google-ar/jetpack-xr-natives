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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_CREATE_MESH_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_CREATE_MESH_DATA_H_

#include <cstdint>

#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
namespace imp::loader::details {

// Create mesh vertex data that is available on CPU.
MeshVertexDataPtr CreateMeshVertexData(
    const schemas::VertexBufferInfo& vertex_buffer,
    uint8_t vertex_access_flags);

// Create mesh index data that is available on CPU.
MeshIndexDataPtr CreateMeshIndexData(
    const schemas::IndexBufferInfo& index_buffer);

}  // namespace imp::loader::details
#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_CREATE_MESH_DATA_H_
