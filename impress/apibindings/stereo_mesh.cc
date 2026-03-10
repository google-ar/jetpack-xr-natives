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

#include "apibindings/stereo_mesh.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <numeric>
#include <optional>
#include <utility>

#include "absl/base/no_destructor.h"
#include "core/common/log.h"
#include "absl/types/span.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_factory.h"
#include "core/model/mesh/vertex_format.h"
#include "core/view/base_view.h"

namespace imp {
namespace {
using VertexAttribute = VertexFormat::VertexAttribute;
using AttributeType = VertexFormat::AttributeType;

// TODO (broken link) Investigate moving this function into an impress helper
template <typename TVec>
void FillSpan(absl::Span<TVec> dest,
              absl::Span<const typename TVec::value_type> src) {
  using T = typename TVec::value_type;
  constexpr int kDimension = sizeof(TVec) / sizeof(T);
  static_assert(sizeof(TVec) % sizeof(T) == 0,
                "TVec size must be a multiple of its value_type size.");
  static_assert(sizeof(TVec) == sizeof(T) * kDimension,
                "TVec must be tightly packed with no padding.");
  static_assert(std::is_standard_layout_v<TVec>,
                "TVec must be a standard layout type for this operation.");
  static_assert(std::is_trivial_v<T>,
                "TVec::value_type must be a trivial type to be safely copied "
                "with memcpy.");
  static_assert(kDimension > 0, "Dimension must be positive.");
  if (dest.size() * sizeof(TVec) < src.size() * sizeof(T)) {
    IMP_LOG(imp::ERROR) << "Destination span is smaller than source span";
    return;
  }
  std::memcpy(dest.data(), src.data(), src.size() * sizeof(T));
}
}  // namespace

OwnedMeshPtr CreateStereoMesh(BaseView* view, CreateStereoMeshSettings settings,
                              MeshFactory::MeshDataStorageMode data_mode) {
  // A VertexFormat for float3 Pos and float2 UV as two de-interleaved buffers.
  static const absl::NoDestructor<VertexFormat> kVertexFormat([]() {
    return VertexFormat{{.attribute = VertexAttribute::POSITION,
                         .type = AttributeType::FLOAT3,
                         .attribute_group_override = 0},
                        {.attribute = VertexAttribute::UV0,
                         .type = AttributeType::FLOAT2,
                         .attribute_group_override = 1}};
  }());

  const size_t vertex_count = settings.positions.size() / 3;
  const size_t texcoord_count = settings.texture_coordinates.size() / 2;

  if (vertex_count == 0) {
    IMP_LOG(imp::ERROR) << "Custom mesh has no vertices.";
    return {};
  }

  if (settings.positions.size() % 3 != 0 ||
      settings.texture_coordinates.size() % 2 != 0) {
    IMP_LOG(imp::ERROR) << "Position count " << settings.positions.size()
               << " or texcoord count " << settings.texture_coordinates.size()
               << " is not a multiple of 3/2.";
    return {};
  }

  if (vertex_count != texcoord_count) {
    IMP_LOG(imp::ERROR) << "Custom mesh has mismatch between position count ("
               << vertex_count << ") and texcoord count (" << texcoord_count
               << ").";
    return {};
  }

  if (settings.draw_mode == MeshFactory::PrimitiveType::TRIANGLES) {
    if (settings.indices.has_value()) {
      if (settings.indices->size() % 3 != 0) {
        IMP_LOG(imp::ERROR) << "Custom mesh with TRIANGLES draw mode must have an "
                      "index count divisible by 3, but got "
                   << settings.indices->size();
        return {};
      }
    } else {
      if (vertex_count % 3 != 0) {
        IMP_LOG(imp::ERROR) << "Custom mesh with TRIANGLES draw mode and no indices "
                      "must have a vertex count divisible by 3, but got "
                   << vertex_count;
        return {};
      }
    }
  }

  const bool has_indices =
      settings.indices.has_value() && !settings.indices->empty();
  const size_t index_count =
      has_indices ? settings.indices->size() : vertex_count;

  auto mesh_data = std::make_unique<MeshData>(
      MeshDescription{*kVertexFormat, MeshDescription::IndexType::UINT,
                      vertex_count, index_count});

  FillSpan(mesh_data->Vertices<float3>(0), settings.positions);
  FillSpan(mesh_data->Vertices<float2>(1), settings.texture_coordinates);
  absl::Span<uint32_t> indices = mesh_data->Indices<uint32_t>();
  if (has_indices) {
    for (size_t i = 0; i < settings.indices->size(); ++i) {
      if (settings.indices->at(i) >= vertex_count) {
        IMP_LOG(imp::ERROR) << "Custom mesh has an index " << settings.indices->at(i)
                   << " which is out of bounds, vertex count is "
                   << vertex_count;
        return {};
      }
    }
    std::memcpy(indices.data(), settings.indices->data(),
                settings.indices->size() * sizeof(uint32_t));
  } else {
    // non-indexed primitives are not supported as filament::RenderableManager
    // always requires an index buffer.
    std::iota(indices.begin(), indices.end(), 0);
  }

  return view->GetMeshFactory().CreateByMovingMeshData(
      settings.draw_mode, std::move(mesh_data), std::nullopt, data_mode,
      settings.name);
}

}  // namespace imp
