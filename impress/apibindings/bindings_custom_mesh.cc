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

#include "apibindings/bindings_custom_mesh.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <utility>

#include "core/common/log.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "apibindings/bindings_mesh_buffer.h"
#include "apibindings/impress_api_view.h"
#include "core/geometry/shapes/box.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_factory.h"

namespace imp {

namespace {

filament::RenderableManager::PrimitiveType ToFilamentPrimitiveType(
    BindingsCustomMesh::SubsetTopology topology) {
  switch (topology) {
    case BindingsCustomMesh::SubsetTopology::kTriangles:
      return filament::RenderableManager::PrimitiveType::TRIANGLES;
    case BindingsCustomMesh::SubsetTopology::kTriangleStrip:
      return filament::RenderableManager::PrimitiveType::TRIANGLE_STRIP;
  }
  IMP_LOG(imp::ERROR) << "Unrecognized SubsetTopology: " << static_cast<int>(topology)
             << ". Defaulting to TRIANGLES.";
  return filament::RenderableManager::PrimitiveType::TRIANGLES;
}

}  // namespace

BindingsCustomMesh::BindingsCustomMesh(
    std::shared_ptr<BindingsMeshBuffer> buffer,
    absl::Span<const Subset> subsets, std::optional<Box> bounding_box,
    ImpressApiView& view)
    : buffer_(std::move(buffer)) {
  MeshFactory factory(view);

  Box box = bounding_box.has_value() ? *bounding_box
                                     : buffer_->BorrowRootMesh()->GetAabb();

  // Create submeshes for each subset.
  for (const auto& subset : subsets) {
    // Note: MeshRenderer computes the overall AABB of the renderable by taking
    // the union of the AABBs of all its submeshes, and Filament performs
    // frustum culling on the renderable as a whole. Therefore, setting
    // individual bounding boxes per submesh does not make sense. We pass the
    // single overall custom mesh bounding box to all submeshes.
    auto submesh = factory.CreateSubMesh(
        buffer_->BorrowRootMesh(), subset.index_offset, subset.index_count, box,
        ToFilamentPrimitiveType(subset.topology));
    submeshes_.push_back(std::move(submesh));
  }
}

BorrowedMeshPtr BindingsCustomMesh::BorrowSubMeshAt(size_t index) const {
  if (index >= submeshes_.size()) {
    IMP_LOG(imp::ERROR) << "Submesh index " << index << " is out of bounds [0, "
               << submeshes_.size() << ")";
    return BorrowedMeshPtr{};
  }
  return submeshes_[index].Borrow();
}

size_t BindingsCustomMesh::GetSubMeshCount() const { return submeshes_.size(); }

}  // namespace imp
