/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_CUSTOM_MESH_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_CUSTOM_MESH_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "absl/types/span.h"
#include "apibindings/bindings_mesh_buffer.h"
#include "apibindings/bindings_object.h"
#include "apibindings/impress_api_view.h"
#include "core/geometry/shapes/box.h"
#include "core/model/mesh/mesh.h"

namespace imp {

// A custom mesh that can be created from a MeshBuffer and a list of subsets.
class BindingsCustomMesh : public BindingsObject {
 public:
  // Defines how the indices of a subset are interpreted to form geometric
  // primitives, such as a list of triangles (kTriangles) or a triangle
  // strip (kTriangleStrip).
  enum class SubsetTopology : int {
    kTriangles = 0,
    kTriangleStrip = 1,
  };

  // A subset of the mesh buffer's indices.
  struct Subset {
    int32_t index_offset;
    int32_t index_count;
    SubsetTopology topology = SubsetTopology::kTriangles;
  };

  // Creates a custom mesh from a BindingsMeshBuffer and a list of subsets.
  // The custom mesh is created from the root mesh of the BindingsMeshBuffer.
  // The subsets are used to create submeshes of the root mesh.
  // The bounding_box is optional, and if not provided, the AABB of the
  // mesh buffer will be used.
  // The custom mesh owns the submeshes, and the submeshes will be destroyed
  // when the custom mesh is destroyed.
  // We use a shared_ptr to keep the mesh buffer alive as long as it is
  // referenced by any BindingsCustomMesh. This is because a buffer can be
  // referenced by multiple meshes, and we need to handle non-deterministic
  // destruction order in garbage-collected environments like Java/Kotlin, where
  // it's not feasible to require users to manage destruction order manually.
  BindingsCustomMesh(std::shared_ptr<BindingsMeshBuffer> buffer,
                     absl::Span<const Subset> subsets,
                     std::optional<Box> bounding_box, ImpressApiView& view);

  // Returns the submesh at the given index.
  BorrowedMeshPtr BorrowSubMeshAt(size_t index) const;

  // Returns the number of submeshes.
  size_t GetSubMeshCount() const;

 private:
  std::shared_ptr<BindingsMeshBuffer> buffer_;
  std::vector<OwnedMeshPtr> submeshes_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_CUSTOM_MESH_H_
