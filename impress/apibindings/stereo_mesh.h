// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_STEREO_MESH_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_STEREO_MESH_H_

#include <cstdint>
#include <optional>
#include <string>

#include "absl/types/span.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_factory.h"
#include "core/view/base_view.h"

namespace imp {

// A helper struct for creating a stereo mesh.
struct CreateStereoMeshSettings {
  // Vertex positions.
  absl::Span<const float> positions;
  // Vertex texture coordinates.
  absl::Span<const float> texture_coordinates;
  // Vertex indices.
  std::optional<absl::Span<const uint32_t>> indices;
  // Draw mode.
  filament::RenderableManager::PrimitiveType draw_mode;
  std::optional<std::string> name = std::nullopt;
};

// Create a stereo mesh with the given settings.
// AABB will be calculated according to mesh data.
OwnedMeshPtr CreateStereoMesh(
    BaseView* view, CreateStereoMeshSettings settings,
    // Specifies if the mesh information should be
    // stored in memory, which is required for collisions.
    MeshFactory::MeshDataStorageMode data_mode =
        MeshFactory::MeshDataStorageMode::kDiscardMeshData);
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_STEREO_MESH_H_
