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

#ifndef THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_BUFFER_H_
#define THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_BUFFER_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/types/span.h"
#include "core/geometry/shapes/box.h"
#include "core/line_renderer/line_mat_types.h"
#include "core/math/math.h"
#include "core/math/transform.h"
#include "core/model/mesh/mesh_data.h"

namespace imp::line_renderer {

// Buffer for line vertices.
// This class handles differences between standard tile-based lines and markup
// lines without complicating the geometry extrusion routines.
// TODO: Support markup.
template <typename VertexAttributes = LineVertexAttributes>
class LineAttributeBuffer {
 public:
  // Creates a buffer to hold tile or markup line vertices.
  // `num_vertices` and `num_indices` will be zero.
  // `max_vertices` The maximum number of vertices in the buffer. Together
  //                with `is_markup` this determines the size of the allocated
  //                buffer.
  // `max_indices` The maximum number of indices in the index buffer.
  // `unit_size` The size of a target coordinate system unit in world
  //             units. Coordinates will be divided by this amount to transform
  //             them into the output coordinate system. This is applied after
  //             the position translation.
  // `units_per_dp` World units per device-independent pixel.
  LineAttributeBuffer(int max_vertices, int max_indices, float unit_size,
                      float units_per_dp);

  LineAttributeBuffer(const LineAttributeBuffer&) = delete;
  LineAttributeBuffer& operator=(const LineAttributeBuffer&) = delete;
  LineAttributeBuffer(LineAttributeBuffer&&) = default;
  LineAttributeBuffer& operator=(LineAttributeBuffer&&) = default;

  // Adds a LineVertex to the buffer, and increments `num_vertices`.
  // Note: Does nothing if the buffer is already full.
  void AddVertex(const VertexAttributes& vertex);

  // Adds an index to the index buffer, and increments `num_indices`.
  // Note: Does nothing if the buffer is already full.
  void AddIndex(uint16_t index);

  // Replaces a LineVertex at an index in the buffer.
  // Note: This does not change span distances for markup vertices.
  // `index` Index of an existing element in the buffer.
  // `vertex` The LineVertex to set.
  void ReplaceVertexAtIndex(int index, const VertexAttributes& vertex);

  // Replaces a vertex index at an index in the buffer.
  // `index` Index of an existing element in the buffer.
  // `vertex_index` The new triangle index to assign.
  void ReplaceVertexIndexAtIndex(int index, int vertex_index);

  // Returns the vertex at an index as const reference.
  // Note: The first element is returned instead if the index is invalid (though
  // it may be uninitialized if the buffer is empty.)
  const VertexAttributes& VertexAtIndex(int index) const;

  // Returns the `i`-th index.
  uint16_t Index(int i) const;

  // Sets the span distances for the element at an index.
  // Note: Span distances are only used for markup lines.
  // `index` Index of the element (vertex) to modify.
  // `start_distance` Start distance for the span encompassing the element,
  // in world units.
  // `end_distance` End distance for the span encompassing the element, in
  // world units.
  void UpdateSpanDistancesForElement(int index, float start_distance,
                                     float end_distance);

  // Returns the size of a target coordinate system unit in world units.
  float unit_size() const { return unit_size_; }

  // Returns the world units per device-independent pixel.
  float units_per_dp() const { return units_per_dp_; }

  // Sets the span distances to use when adding new elements.
  // Note: Span distances are only used for markup lines.
  // `start_distance` Start distance to apply to new elements, in world
  // units.
  // `end_distance` End distance to apply to new elements, in world units.
  void set_span_distance(float start_distance, float end_distance) {
    span_start_distance_ = start_distance;
    span_end_distance_ = end_distance;
  }

  // Returns the span start distance that will be used when adding new
  // elements.
  float span_start_distance() const { return span_start_distance_; }

  // Returns the span end distance that will be used when adding new elements.
  float span_end_distance() const { return span_end_distance_; }

  int num_vertices() const { return num_vertices_; }
  int num_indices() const { return num_indices_; }

  void set_num_vertices(int num_vertices) {
    num_vertices_ = std::clamp(num_vertices, /*low=*/0, max_vertices_);
  }
  void set_num_indices(int num_indices) {
    num_indices_ = std::clamp(num_indices, /*low=*/0, max_indices_);
  }

  int max_vertices() const { return max_vertices_; }
  int max_indices() const { return max_indices_; }

  // Releases the mesh data and invalidates the buffer. The returned mesh data
  // contains exactly `num_vertices()` and `num_indices()`.
  // The vertex data may be transformed from the input to improve precision -
  // be sure to apply the `transform()` to the line's entity to move it back.
  imp::MeshDataPtr FinalizeAndReleaseMeshData();

  // The bounds of the vertex data.
  imp::Box GetBoundingBox() const;

  // The transform that should be applied to the vertex data.
  imp::Transform<double> GetTransform() const;

 private:
  int num_vertices_;
  int num_indices_;
  int max_vertices_;
  int max_indices_;
  imp::MeshDataPtr mesh_data_;
  absl::Span<VertexAttributes> vertices_;
  absl::Span<uint16_t> indices_;

  float unit_size_;
  float units_per_dp_;
  float span_start_distance_;
  float span_end_distance_;

  imp::Box bounds_;
};

using LineBuffer = LineAttributeBuffer<LineVertexAttributes>;

template <typename VertexAttributes>
LineAttributeBuffer<VertexAttributes>::LineAttributeBuffer(int max_vertices,
                                                           int max_indices,
                                                           float unit_size,
                                                           float units_per_dp)
    : num_vertices_(0),
      num_indices_(0),
      max_vertices_(max_vertices),
      max_indices_(max_indices),
      mesh_data_(std::make_unique<imp::MeshData>(imp::MeshDescription{
          .vertex_format = LineVertexFormat(),
          .index_type = imp::MeshDescription::IndexType::USHORT,
          .vertex_count = static_cast<size_t>(max_vertices_),
          .index_count = static_cast<size_t>(max_indices_),
      })),
      vertices_(mesh_data_->Vertices<LineVertexAttributes>()),
      indices_(mesh_data_->Indices<uint16_t>()),
      unit_size_(unit_size),
      units_per_dp_(units_per_dp),
      span_start_distance_(0),
      span_end_distance_(0) {}

template <typename VertexAttributes>
void LineAttributeBuffer<VertexAttributes>::AddVertex(
    const VertexAttributes& vertex) {
  size_t index = num_vertices();
  if (index >= max_vertices()) {
    IMP_LOG(imp::FATAL) << "Attempt to add vertex beyond the range of vertex buffer.";
  }

  vertices_[index] = vertex;
  vertices_[index].pos /= unit_size_;
  vertices_[index].distance /= units_per_dp_;
  ++num_vertices_;
  ExtendBoundsToContainPoint(bounds_, vertices_[index].pos);
}

template <typename VertexAttributes>
void LineAttributeBuffer<VertexAttributes>::AddIndex(uint16_t index) {
  size_t index_idx = num_indices();
  if (index_idx >= max_indices()) {
    IMP_LOG(imp::FATAL) << "Attempt to add index beyond the range of index buffer.";
  }

  indices_[index_idx] = index;
  ++num_indices_;
}

template <typename VertexAttributes>
void LineAttributeBuffer<VertexAttributes>::ReplaceVertexAtIndex(
    int index, const VertexAttributes& vertex) {
  if (index < 0 || index > num_vertices()) {
    IMP_LOG(imp::FATAL) << "Attempt to replace vertex at index beyond range of buffer.";
  }

  vertices_[index] = vertex;
  vertices_[index].pos /= unit_size_;
  vertices_[index].distance /= units_per_dp_;
  ExtendBoundsToContainPoint(bounds_, vertices_[index].pos);
}

template <typename VertexAttributes>
void LineAttributeBuffer<VertexAttributes>::ReplaceVertexIndexAtIndex(
    int index, int vertex_index) {
  if (index < 0 || index > num_indices()) {
    IMP_LOG(imp::FATAL) << "Attempt to replace index at index beyond range of buffer.";
  }
  indices_[index] = vertex_index;
}

template <typename VertexAttributes>
const VertexAttributes& LineAttributeBuffer<VertexAttributes>::VertexAtIndex(
    int index) const {
  if (index < num_vertices()) {
    return vertices_[index];
  }
  return vertices_[0];
}

template <typename VertexAttributes>
uint16_t LineAttributeBuffer<VertexAttributes>::Index(int i) const {
  if (i < num_indices()) {
    return indices_[i];
  }
  return indices_[0];
}

template <typename VertexAttributes>
void LineAttributeBuffer<VertexAttributes>::UpdateSpanDistancesForElement(
    int index, float start_distance, float end_distance) {
  if (index >= max_vertices()) return;
}

template <typename VertexAttributes>
imp::MeshDataPtr
LineAttributeBuffer<VertexAttributes>::FinalizeAndReleaseMeshData() {
  mesh_data_->TruncateVertices(num_vertices());
  mesh_data_->TruncateIndices(num_indices());
  return std::move(mesh_data_);
}

template <typename VertexAttributes>
imp::Box LineAttributeBuffer<VertexAttributes>::GetBoundingBox() const {
  return bounds_;
}

}  // namespace imp::line_renderer

#endif  // THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_BUFFER_H_
