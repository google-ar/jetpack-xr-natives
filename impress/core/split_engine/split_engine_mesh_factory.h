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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_FACTORY_H_

#include <cstdint>
#include <optional>

#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "core/loader/creator/inflight_creation.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_data_generated.h"

namespace imp::split_engine {

// Creates a filament vertex buffer from an Android XR flatbuffer schema.
filament::VertexBuffer* BuildVertexBuffer(
    BaseView& view, filament::Engine& engine,
    const android_xr::schemas::VertexBufferInfo& info,
    loader::details::InflightCreation* inflight_creation,
    std::optional<uint8_t> vertex_access_flags = std::nullopt);

// Creates a filament index buffer from an Android XR flatbuffer schema.
filament::IndexBuffer* BuildIndexBuffer(
    BaseView& view, filament::Engine& engine,
    const android_xr::schemas::IndexBufferInfo& info,
    loader::details::InflightCreation* inflight_creation,
    std::optional<bool> store_index_data = std::nullopt);

// Creates a filament morph target buffer from an Android XR flatbuffer schema.
filament::MorphTargetBuffer* BuildMorphTargetBuffer(
    BaseView& view, filament::Engine& engine,
    const android_xr::schemas::MorphTargetBufferInfo& info);

// Creates CPU mesh vertex data from an Android XR flatbuffer schema.
MeshVertexDataPtr CreateMeshVertexData(
    const android_xr::schemas::VertexBufferInfo& vertex_buffer,
    uint8_t vertex_access_flags);

// Creates CPU mesh index data from an Android XR flatbuffer schema.
MeshIndexDataPtr CreateMeshIndexData(
    const android_xr::schemas::IndexBufferInfo& index_buffer);

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_FACTORY_H_
