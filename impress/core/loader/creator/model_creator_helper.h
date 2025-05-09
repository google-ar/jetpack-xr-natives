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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_MODEL_CREATOR_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_MODEL_CREATOR_HELPER_H_

#include <cstdint>

#include "core/math/vec.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "filament/libs/math/include/math/vec3.h"

namespace imp::loader::details {

template <typename T>
void ApplyTypedVertexAttribute(size_t id,
                               VertexFormat::VertexAttribute vertex_attr,
                               MeshVertexData* vertex,
                               const unsigned char* data) {
  vertex->VertexAttributeAt<T>(id, vertex_attr) =
      *const_cast<T*>(reinterpret_cast<const T*>(data));
}

void ApplyVertexAttribute(VertexFormat::AttributeType attr_type, size_t id,
                          VertexFormat::VertexAttribute vertex_attr,
                          MeshVertexData* vertex, const unsigned char* data);

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_MODEL_CREATOR_HELPER_H_
