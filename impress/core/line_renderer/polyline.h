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

#ifndef THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_POLYLINE_H_
#define THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_POLYLINE_H_

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <set>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

// #include "maps/shared/mapcore/common/geometry/bounds.h"
#include "core/line_renderer/point_util.h"
// #include "maps/shared/mapcore/common/spatial/mercator_point_util.h"
#include "absl/log/check.h"
#include "core/math/vec.h"

namespace imp::line_renderer {

template <typename PositionT>
class Polyline;

constexpr float kInvalidPolyspanLength = -1.f;

// Represents a span of a generic vector attribute inside a polyline. Does not
// own any storage.
template <typename T>
class PolylineAttributeSpan {
 public:
  using const_iterator = typename std::vector<T>::const_iterator;
  using reverse_iterator = typename std::vector<T>::reverse_iterator;
  using const_reverse_iterator =
      typename std::vector<T>::const_reverse_iterator;

  PolylineAttributeSpan() : base_(), begin_(base_), end_(base_) {
    ValidateIterators();
  }

  PolylineAttributeSpan(const PolylineAttributeSpan& other)
      : base_(other.base_), begin_(other.begin_), end_(other.end_) {
    ValidateIterators();
  }

  PolylineAttributeSpan(const_iterator begin, const_iterator end)
      : base_(begin), begin_(begin), end_(end) {
    ValidateIterators();
  }

  PolylineAttributeSpan(const_iterator base, const_iterator begin,
                        const_iterator end)
      : base_(base), begin_(begin), end_(end) {
    ValidateIterators();
  }

  // Returns the global offset of the local index within the base backing store.
  // `i` must be less than size() and greater than zero. If not, the index
  // returned will likely be invalid.
  int GlobalIndex(int i) const {
    ValidateIndex(i);
    return (begin_ + i) - base_;
  }

  // Returns the number of points in the span.
  int size() const { return end_ - begin_; }

  // Returns the point at the given index.
  const T& operator[](int i) const {
    ValidateIndex(i);
    return begin_[i];
  }
  const T& front() const {
    return begin_[0];
  }
  const T& back() const {
    return end_[-1];
  }

  // Iterators for range-based iteration.
  const_iterator begin() const { return begin_; }
  const_iterator end() const { return end_; }

  const_reverse_iterator rbegin() const {
    return std::reverse_iterator<const_iterator>(end_);
  }

  const_reverse_iterator rend() const {
    return std::reverse_iterator<const_iterator>(begin_);
  }

  // Returns a new sub-span defined by a slice [`begin`, `end`).
  // The backing store is unaffected.
  //
  // `begin` The index of the first point in the slice (inclusive).
  // `end` The index after the last point in the slice (exclusive).
  PolylineAttributeSpan Slice(int begin, int end) const {
    ValidateSpan(begin, end);
    return PolylineAttributeSpan(base_, begin_ + begin, begin_ + end);
  }

  PolylineAttributeSpan& operator=(const PolylineAttributeSpan& other) =
      default;

  PolylineAttributeSpan& operator=(PolylineAttributeSpan&& other) = default;

  bool operator==(const PolylineAttributeSpan& other) const {
    // Quick check - identity equality.
    if (begin_ == other.begin_ && end_ == other.end_) {
      return true;
    }

    return size() == other.size() && std::equal(begin_, end_, other.begin_);
  }

  bool operator!=(const PolylineAttributeSpan& other) const {
    return !(*this == other);
  }

 protected:
  void ValidateIndex(int i) const {
    
  }

  void ValidateSpan(int begin, int end) const {
    ValidateIndex(begin);
    if (begin != end) {
      ValidateIndex(end - 1);
    }
  }

  void ValidateIterators() const {
    
    
  }

  const_iterator base_;
  const_iterator begin_;
  const_iterator end_;
};

// Represents a span inside a polyline that consists of positions and optional
// attributes. Providing position-specific helpers including bounds,
// simplification, and interpolation. Does not own any storage.
template <typename PositionT, typename... AttributeT>
class PolylineSpan : public PolylineAttributeSpan<
                         decltype(std::conditional_t<
                                  sizeof...(AttributeT) == 0, PositionT,
                                  std::tuple<PositionT, AttributeT...>>())> {
 public:
  // The type of the each point is a tuple when there are additional attributes,
  // but it collapses to be just a PositionT when there are no attributes.
  static constexpr bool kPositionOnly = sizeof...(AttributeT) == 0;
  using PointT =
      decltype(std::conditional_t<sizeof...(AttributeT) == 0, PositionT,
                                  std::tuple<PositionT, AttributeT...>>());

  using const_iterator = typename PolylineAttributeSpan<PointT>::const_iterator;
  using reverse_iterator =
      typename PolylineAttributeSpan<PointT>::reverse_iterator;
  using const_reverse_iterator =
      typename PolylineAttributeSpan<PointT>::const_reverse_iterator;

  // Helpful accessors into the polyline position attribute.
  static const PositionT& GetPosition(const PointT& point) {
    if constexpr (kPositionOnly) {
      return point;
    } else {
      return std::get<0>(point);
    }
  }
  static PositionT& GetPosition(PointT& point) {
    if constexpr (kPositionOnly) {
      return point;
    } else {
      return std::get<0>(point);
    }
  }

  // An iterator for only the position attribute.
  class PositionIterator {
   public:
    // Iterator traits.
    using difference_type =
        typename std::iterator_traits<const_iterator>::difference_type;
    using iterator_category = std::random_access_iterator_tag;
    using value_type = PositionT;
    using pointer = const PositionT*;
    using reference = const PositionT&;

    explicit PositionIterator(const_iterator iter) : iter_(iter) {}
    PositionIterator& operator++() {
      ++iter_;
      return *this;
    }
    bool operator==(PositionIterator other) const {
      return iter_ == other.iter_;
    }
    bool operator!=(PositionIterator other) const { return !(*this == other); }
    reference operator*() const { return GetPosition(*iter_); }
    reference operator[](int i) const {
      if constexpr (kPositionOnly) {
        return iter_[i];
      } else {
        return std::get<0>(iter_[i]);
      }
    }
    difference_type operator-(const PositionIterator& other) const {
      return iter_ - other.iter_;
    }

   private:
    const_iterator iter_;
  };

  PolylineSpan()
      : PolylineAttributeSpan<PointT>(),
        cached_length_(kInvalidPolyspanLength)
  /*, cached_bounds_(kInvalidBounds<PositionT>)*/ {}

  PolylineSpan(const PolylineSpan& other) = default;

  PolylineSpan(const_iterator begin, const_iterator end)
      : PolylineAttributeSpan<PointT>(begin, end),
        cached_length_(kInvalidPolyspanLength)
  /*, cached_bounds_(kInvalidBounds<PositionT>)*/ {}

  PolylineSpan(const_iterator base, const_iterator begin, const_iterator end)
      : PolylineAttributeSpan<PointT>(base, begin, end),
        cached_length_(kInvalidPolyspanLength)
  /*, cached_bounds_(kInvalidBounds<PositionT>)*/ {}

  // Returns the position attribute.
  const PositionT& Position(const_iterator iter) const {
    return GetPosition(*iter);
  }
  const PositionT& Position(int i) const { return PositionBegin()[i]; }

  // Iterators for just position attribute.
  PositionIterator PositionBegin() const {
    return PositionIterator(this->begin_);
  }
  PositionIterator PositionEnd() const { return PositionIterator(this->end_); }

  /*
  // The bounding rectangle of this span.
  Bounds<PositionT> Bounds() const {
    if (cached_bounds_ == kInvalidBounds<PositionT>) {
      cached_bounds_ = (this->size() > 0)
                           ? mapcore::Bounds<PositionT>::FromPoints(
                                 PositionBegin(), PositionEnd())
                           : mapcore::Bounds<PositionT>{};
    }
    return cached_bounds_;
  }
  */

  // The total length of this span.
  float Length() const {
    if (cached_length_ == kInvalidPolyspanLength) {
      int segment_count = this->size() - 1;
      cached_length_ = 0;
      for (int i = 0; i < segment_count; ++i) {
        const PositionT& p0 = Position(i);
        const PositionT& p1 = Position(i + 1);
        cached_length_ += PointUtil<PositionT>::Distance(p0, p1);
      }
    }
    return cached_length_;
  }

  // Returns true if this span is a loop.
  bool IsLoop() const {
    return this->size() > 0 && this->begin_[0] == this->end_[-1];
  }

  // Returns the point along the polyline at a fraction of the distance from the
  // polyline's start to its end. All point attributes are interpolated.
  // Optionally determines rotation angle to face the point along the segment
  // containing it, in radians. An angle of 0 faces East, a.k.a. vector (1, 0).
  //
  // `s` Fractional distance from polyline start to end.
  // `out_angle` Pointer to store facing angle, in radians.
  //
  // Note that `out_angle` does not work in 3D, it exists only as an artifact of
  // porting this function from 2D. To make this work generally, we should
  // prefer to output the angle vector (tangent) instead of the angle itself
  // then let the caller figure out what angle it is relative to.
  PointT Interpolate(float s, float* out_angle = nullptr) const {
    if (out_angle) {
      *out_angle = 0;
    }
    // Handle invalid polylines with 0 or 1 points.
    if (this->size() == 0) {
      return PointT();
    } else if (this->size() == 1) {
      return *(this->end_ - 1);
    }

    // Clamp interpolation value to [0, 1].
    s = std::max(s, 0.0f);
    s = std::min(s, 1.0f);

    // Find the interpolated point on the polyline.
    PointT point = *(this->end_ - 1);
    float remaining_dist = Length() * s;
    auto iter = this->begin_;
    auto segment_end = this->end_ - 1;
    while (iter != segment_end) {
      double segment_length =
          PointUtil<PositionT>::Distance(Position(iter), Position(iter + 1));
      if (segment_length >= remaining_dist &&
          // Avoid duplicate points at the end of a line where the segment
          // length would cause a divide by zero.
          segment_length >= std::numeric_limits<double>::epsilon()) {
        float remaining_s = remaining_dist / segment_length;
        point = InterpolatePoint(iter[0], iter[1], remaining_s);
        break;
      }
      remaining_dist -= segment_length;
      ++iter;
    }

    // Calculate rotation angle to face point.
    if (out_angle) {
      using FloatVector =
          decltype(std::conditional_t<PositionT::SIZE == 2, imp::float2,
                                      imp::float3>());
      FloatVector angle_vec =
          iter != segment_end
              ? (Position(iter + 1) - Position(iter))
              : (Position(this->end_ - 1) - Position(this->end_ - 2));
      float len = length(angle_vec);
      if (len > 0) {
        // Normalize the vector.
        angle_vec = angle_vec / len;
        // Dot product between angle_vec and East-facing vector (1, 0).
        float dot = angle_vec.x;
        // Clamp to [-1, 1], just to be safe.
        s = std::max(s, -1.0f);
        s = std::min(s, 1.0f);
        *out_angle = std::acos(dot);
        if (angle_vec.y < 0) {
          // Adjust to always rotate counterclockwise.
          *out_angle = (M_PI * 2) - *out_angle;
        }
      }
    }

    return point;
  }

  // Calculates the minimum distance from line to any segment.
  float Distance(const PositionT& p) const { return sqrtf(DistanceSquared(p)); }

  float DistanceSquared(const PositionT& p) const {
    switch (this->size()) {
      case 0:
        return std::numeric_limits<float>::lowest();
      case 1:
        return PointUtil<PositionT>::DistanceSqr(Position(0), p);
      default: {
        float min_dist = std::numeric_limits<float>::max();
        auto segment_end = this->end_ - 1;
        for (auto iter = this->begin_; iter != segment_end; ++iter) {
          float dist = PointUtil<PositionT>::DistanceToLineSegmentSqr(
              p, Position(iter), Position(iter + 1),
              /*out_closest_point=*/nullptr);
          min_dist = std::min(min_dist, dist);
        }

        return min_dist;
      }
    }
  }

  // Calculates the local distances in world units along the polyline from its
  // start.
  std::vector<float> DistancesAlongPoints() const {
    std::vector<float> distances;
    PointUtil<PositionT>::template ComputeDistancesAlongPoints<float>(
        PositionBegin(), PositionEnd(), &distances);
    return distances;
  }

  PolylineSpan& operator=(const PolylineSpan& other) = default;

  PolylineSpan& operator=(PolylineSpan&& other) = default;

 protected:
  static PointT InterpolatePoint(const PointT& a, const PointT& b,
                                 float fraction) {
    if constexpr (kPositionOnly) {
      return PointUtil<PointT>::Interpolate(a, b, fraction);
    } else {
      // Apply `PointUtil::Interpolate()` element-wise to `a` and `b` and return
      // the resulting tuple.
      return std::apply(
          [&](const auto&... a_attr) {
            return std::apply(
                [&](const auto&... b_attr) {
                  return std::make_tuple(
                      PointUtil<typename std::decay<decltype(a_attr)>::type>::
                          Interpolate(a_attr, b_attr, fraction)...);
                },
                b);
          },
          a);
    }
  }

  // Use the Douglas-Peucker algorithm to simplify the line, using the
  // recursive selectVertices method to select which vertices will be
  // kept in the result line.
  void SelectVertices(float max_error_2, int interval,
                      std::set<int>* selected) const {
    int vertex_size = static_cast<int>(this->size());
    if (vertex_size <= 2) {
      return;
    }

    // Find the vertex in between the start and end vertices that is
    // farthest away from the segment formed by the first and last vertices.
    // If it is less than max_error, we're done. Otherwise, select that
    // vertex and recursively call ourselves on the two halves.
    const PositionT& pt1 = Position(this->begin_);
    const PositionT& pt2 = Position(this->end_ - 1);

    int pivot = -1;
    float max_dist_2 = max_error_2;
    // Search the inner points for the largest deviation from the line
    // formed by the end points.
    for (int i = interval; i < vertex_size - 1; i += interval) {
      float dist_2 = PointUtil<PositionT>::DistanceToLineSegmentSqr(
          Position(this->begin_ + i), pt1, pt2,
          /*out_closest_point=*/nullptr);
      if (dist_2 > max_dist_2) {
        pivot = i;
        max_dist_2 = dist_2;
      }
    }

    // Nothing worth saving in this sub-range.
    if (pivot < 0) {
      return;
    }

    // Mark this point as removed.
    selected->insert(this->GlobalIndex(pivot));

    // Recursively check the lower sub-span.
    PolylineSpan lower_span(this->base_, this->begin_,
                            this->begin_ + pivot + 1);
    lower_span.SelectVertices(max_error_2, interval, selected);

    // Recursively check the upper sub-span.
    PolylineSpan upper_span(this->base_, this->begin_ + pivot, this->end_);
    upper_span.SelectVertices(max_error_2, interval, selected);
  }

  // Creates new polyline data with a new backing store from the current Span
  // such that no point along the new polyline is further than `max_error` away
  // from the original span.
  std::vector<PointT> GetSimplifiedPoints(float max_error, int interval) const {
    int vertex_size = static_cast<int>(this->size());
    if (vertex_size <= 2) {
      // No simplification can be done if there are 2 or fewer vertices.
      return std::vector<PointT>(this->begin_, this->end_);
    }

    std::set<int> selected;
    selected.insert(0);
    selected.insert(vertex_size - 1);

    // A local span with a local base.
    PolylineSpan local_span(this->begin_, this->begin_, this->end_);
    local_span.SelectVertices(max_error * max_error, interval, &selected);

    // Build a new polyline from the selected vertices.
    std::vector<PointT> result;
    result.reserve(selected.size());
    for (int i : selected) {
      result.push_back(this->begin_[i]);
    }
    return result;
  }

  void ClearCached() {
    cached_length_ = kInvalidPolyspanLength;
    // cached_bounds_ = kInvalidBounds<PositionT>;
  }

  mutable float cached_length_;
  // mutable mapcore::Bounds<PositionT> cached_bounds_;
};

// Represents a polyline with only a position attribute, including its own
// storage.
template <typename PositionT>
class Polyline : public PolylineSpan<PositionT> {
 public:
  Polyline() { SetIterators(&points_); }

  Polyline(std::initializer_list<PositionT> points) : points_(points) {
    SetIterators(&points_);
  }

  explicit Polyline(std::vector<PositionT>&& points)
      : points_(std::move(points)) {
    SetIterators(&points_);
  }

  template <typename Iterator>
  Polyline(Iterator begin, Iterator end) : points_(begin, end) {
    SetIterators(&points_);
  }

  Polyline(const Polyline& other) : points_(other.points_) {
    SetIterators(&points_);
  }

  Polyline(Polyline&& other) noexcept : points_(std::move(other.points_)) {
    SetIterators(&points_);
  }

  // Invalidates all iterators.
  Polyline& operator=(Polyline&& other) = default;

  // Releases the contents of the vector. Invalidates all iterators.
  std::vector<PositionT> release() {
    std::vector<PositionT> released = std::move(points_);
    points_.clear();
    SetIterators(&points_);
    Polyline::ClearCached();
    return released;
  }

  // Creates a new Polyline with a new backing store from the current Span
  // such that no point along the new polyline is further than `max_error` away
  // from the original span.
  Polyline<PositionT> Simplify(float max_error, int interval) const {
    return Polyline<PositionT>(this->GetSimplifiedPoints(max_error, interval));
  }

 private:
  void SetIterators(std::vector<PositionT>* p) {
    Polyline::base_ = p->begin();
    Polyline::begin_ = p->begin();
    Polyline::end_ = p->end();
    this->ValidateIterators();
  }

  std::vector<PositionT> points_;
};

// Represents a polyline with both a position and an additional vector
// attribute, including its own storage.
template <typename PositionT, typename AttributeT>
class PolylineWithVectorAttribute : public PolylineSpan<PositionT, AttributeT> {
 public:
  PolylineWithVectorAttribute() { SetIterators(&points_); }

  PolylineWithVectorAttribute(
      std::initializer_list<std::tuple<PositionT, AttributeT>> points)
      : points_(points) {
    SetIterators(&points_);
  }

  explicit PolylineWithVectorAttribute(
      std::vector<std::tuple<PositionT, AttributeT>>&& points)
      : points_(std::move(points)) {
    SetIterators(&points_);
  }

  template <typename Iterator>
  PolylineWithVectorAttribute(Iterator begin, Iterator end)
      : points_(begin, end) {
    SetIterators(&points_);
  }

  PolylineWithVectorAttribute(const PolylineWithVectorAttribute& other)
      : points_(other.points_) {
    SetIterators(&points_);
  }

  PolylineWithVectorAttribute(PolylineWithVectorAttribute&& other) noexcept
      : points_(std::move(other.points_)) {
    SetIterators(&points_);
  }

  // Invalidates all iterators.
  PolylineWithVectorAttribute& operator=(PolylineWithVectorAttribute&& other) =
      default;

  // Releases the contents of the vector. Invalidates all iterators.
  std::vector<std::tuple<PositionT, AttributeT>> release() {
    std::vector<std::tuple<PositionT, AttributeT>> released =
        std::move(points_);
    points_.clear();
    SetIterators(&points_);
    PolylineWithVectorAttribute::ClearCached();
    return released;
  }

  // Creates a new PolylineWithVectorAttribute with a new backing store from the
  // current Span such that no point along the new polyline is further than
  // `max_error` away from the original span.
  PolylineWithVectorAttribute<PositionT, AttributeT> Simplify(
      float max_error, int interval) const {
    return PolylineWithVectorAttribute<PositionT, AttributeT>(
        this->GetSimplifiedPoints(max_error, interval));
  }

 private:
  void SetIterators(std::vector<std::tuple<PositionT, AttributeT>>* p) {
    PolylineWithVectorAttribute::base_ = p->begin();
    PolylineWithVectorAttribute::begin_ = p->begin();
    PolylineWithVectorAttribute::end_ = p->end();
    this->ValidateIterators();
  }

  std::vector<std::tuple<PositionT, AttributeT>> points_;
};

using Polyspan2D = PolylineSpan<imp::int2>;
using Polyline2D = Polyline<imp::int2>;

using Polyspan2f = PolylineSpan<imp::float2>;
using Polyline2f = Polyline<imp::float2>;

// A 3D polyline with a float3 position and a float3 normal attribute.
using Polyline3f = PolylineWithVectorAttribute<imp::float3, imp::float3>;

}  // namespace imp::line_renderer

#endif  // THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_POLYLINE_H_
