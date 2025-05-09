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

#ifndef THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_POINT_UTIL_H_
#define THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_POINT_UTIL_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <type_traits>
#include <vector>

// #include "maps/shared/mapcore/common/geometry/bounds.h"
// #include "maps/shared/mapcore/common/spatial/model.h"
// #include "absl/log/check.h"
#include "core/math/math.h"
#include "core/math/vec.h"
// #include "util/math/mathutil.h"

namespace imp::line_renderer {

// Common operations for points and line segments.
// Available in integral (PointUtil2D) or floating point (PointUtil2f) versions,
// the former should be used when dealing with world-scale geometry, and the
// latter when dealing with screen space scales. Do not attempt to use the float
// specialization to represent world-scale points, as the precision is
// insufficient. A subset of these operations are usable with 3D points -- the
// ones that only work for 2D are denoted via static asserts.
template <typename VectorT>
class PointUtil {
  // Implementation note: When doing arithmetic on points, care should be taken
  // to avoid overflow for the integral realization of this template.
  // While it is safe to perform subtraction on points within the world bounds,
  // promoting to the wider PromotedScalar _before_ subtraction allows
  // correctness over a wider out-of-world-bounds range. Promotion _must_ occur
  // before any multiplication, including dot and cross operations. If exact
  // quantities are required, components should be converted to to
  // PromotedScalar; if only a qualitative measure or sign is required then
  // conversion to float is acceptable.
 public:
  using Scalar = typename VectorT::value_type;
  using Vector = VectorT;
  // Some operations in int32 space require additional precision to give correct
  // results, while float operations can safely stay in float space and do not
  // need to be promoted.
  using PromotedScalar =
      std::conditional_t<std::is_integral_v<Scalar>, int64_t, Scalar>;

  // Returns 1 if `a`, `b`, and `c` form a counter-clockwise triangle, -1 if
  // they form a clockwise triangle, and 0 if they are collinear.
  static int Sign(const Vector &a, const Vector &b, const Vector &c) {
    static_assert(Vector::SIZE == 2, "Only supported in 2D.");
    // Promote all types to float to prevent overflow if inputs are large.
    const imp::float2 a_float{a};
    const imp::float2 b_float{b};
    const imp::float2 c_float{c};
    float crossf = cross(c_float - a_float, c_float - b_float);
    // Returns 1 if A, B, and C form a counter-clockwise triangle, -1 if they
    // form a clockwise triangle, and 0 if they are collinear.
    return (crossf > 0) ? 1 : (crossf < 0 ? -1 : 0);
  }

  // Returns true if `p` is on the segment `a`-`b`. An optional epsilon may be
  // provided for floating point inaccuracy.
  static bool SegmentContainsPoint(const Vector &a, const Vector &b,
                                   const Vector &p, float epsilon = 0.0) {
    using FloatVector =
        decltype(std::conditional_t<Vector::SIZE == 2, imp::float2,
                                    imp::float3>());
    // 
    FloatVector a_float = a;
    FloatVector b_float = b;
    FloatVector p_float = p;
    FloatVector projected = Project(p_float, a_float, b_float,
                                    /*is_line_segment=*/true);
    if (epsilon == 0.0) {
      return projected == p_float;
    } else {
      return DistanceSqr(p_float, projected) <= epsilon * epsilon;
    }
  }

  // The same as LineAndSegmentIntersect(a, b, c, d, false)
  static std::optional<Vector> LineAndSegmentIntersect(const Vector &a,
                                                       const Vector &b,
                                                       const Vector &c,
                                                       const Vector &d) {
    return LineAndSegmentIntersect(a, b, c, d, false);
  }

  // Returns the intersection of the line passing through points A and B and the
  // line segment with endpoints C and D, or std::nullopt if there is no
  // intersection. If AB and CD are collinear, an arbitrary Point on CD will be
  // returned. If `allow_out_of_segment_intersection` is `false` the
  // intersection point should belong to the segment. Otherwise it can be
  // outside.
  static std::optional<Vector> LineAndSegmentIntersect(
      const Vector &a, const Vector &b, const Vector &c, const Vector &d,
      bool allow_out_of_segment_intersection) {
    // Note that this could be implemented in 3D, but the current implementation
    // only works in 2D as an artifact of having been ported from 2D code.
    static_assert(Vector::SIZE == 2, "Only supported in 2D.");
    PromotedScalar ortho_x = PromotedScalar{a.y} - b.y;
    PromotedScalar ortho_y = PromotedScalar{b.x} - a.x;
    PromotedScalar denominator = (PromotedScalar{d.x} - c.x) * ortho_x +
                                 (PromotedScalar{d.y} - c.y) * ortho_y;

    if (denominator == 0) {
      if (Sign(a, b, c) == 0) {
        // AB and CD are colinear. Can return any point on CD.
        return c;
      }
      return std::nullopt;
    }
    double t = static_cast<double>(
                   (static_cast<PromotedScalar>(a.x) - c.x) * ortho_x +
                   (static_cast<PromotedScalar>(a.y) - c.y) * ortho_y) /
               denominator;
    if (!allow_out_of_segment_intersection && (t < 0 || t > 1)) {
      return std::nullopt;
    }
    return Vector(static_cast<Scalar>(c.x + (d.x - c.x) * t),
                  static_cast<Scalar>(c.y + (d.y - c.y) * t));
  }

  // Returns ratio in which the line passing through points A and B intersects
  // the line passing through C and D. Let O be the intersection point of
  // lines AB and CD, then returned value is CO/CD if point O lies on the ray
  // CD and -CO/CD otherwise. (e.g. it returns zero if point O coincides with
  // C and 1 if it coincides with D).
  // Returns infinity if the line and segment are parallel.
  static double SegmentsIntersectionRatio(const Vector &a, const Vector &b,
                                          const Vector &c, const Vector &d) {
    static_assert(Vector::SIZE == 2, "Only supported in 2D.");
    PromotedScalar ortho_x = PromotedScalar{a.y} - b.y;
    PromotedScalar ortho_y = PromotedScalar{b.x} - a.x;
    PromotedScalar num = (PromotedScalar{a.x} - c.x) * ortho_x +
                         (PromotedScalar{a.y} - c.y) * ortho_y;
    PromotedScalar denominator = (PromotedScalar{d.x} - c.x) * ortho_x +
                                 (PromotedScalar{d.y} - c.y) * ortho_y;
    return static_cast<double>(num) / denominator;
  }

  // Returns true if segments AB and CD intersect each other, even at just
  // a single point.
  static bool SegmentsIntersect(const Vector &a, const Vector &b,
                                const Vector &c, const Vector &d) {
    static_assert(Vector::SIZE == 2, "Only supported in 2D.");
    PromotedScalar ab_x = PromotedScalar{b.x} - a.x;
    PromotedScalar ab_y = PromotedScalar{b.y} - a.y;
    PromotedScalar cd_x = PromotedScalar{d.x} - c.x;
    PromotedScalar cd_y = PromotedScalar{d.y} - c.y;
    PromotedScalar ac_x = PromotedScalar{c.x} - a.x;
    PromotedScalar ac_y = PromotedScalar{c.y} - a.y;

    PromotedScalar denominator = cd_x * ab_y - cd_y * ab_x;
    if (denominator == 0) {
      // Either segments are parallel or one of them is zero.
      if (ac_x == 0 && ac_y == 0) {
        // A and C coincide.
        return true;
      }
      if (ac_x * ab_y - ac_y * ab_x != 0) {
        // Segments parallel but not collinear
        return false;
      }
      if (ab_x == 0 && ab_y == 0) {
        // A coincides with B
        return FallsBetween(c, d, a);
      }
      if (cd_x == 0 && cd_y == 0) {
        // C coincides with D
        return FallsBetween(a, b, c);
      }
      // Segments are collinear.
      return (FallsBetween(a, b, c) || FallsBetween(a, b, d) ||
              FallsBetween(c, d, a) || FallsBetween(c, d, b));
    }

    // t = ca * ortho(ab) / (cd * ortho(ab))
    double t = static_cast<double>((-ac_x * ab_y + ac_y * ab_x)) / denominator;
    if (t < 0 || t > 1) return false;

    // t = ac * ortho(cd) / (ab * ortho(cd))
    t = static_cast<double>(ac_x * cd_y - ac_y * cd_x) / (-denominator);
    return t >= 0 && t <= 1;
  }

  // Returns true if segment AB crosses the segment extending from P upward
  // to infinity. Segment AB is considered to be closed on the left side and
  // open on the right side. If P lies on the segment AB, this function
  // will return false.
  static bool CrossesAbove(const Vector &a, const Vector &b, const Vector &p) {
    static_assert(Vector::SIZE == 2, "Only supported in 2D.");
    // Use 64-bit locals for the later multiplications
    PromotedScalar a_x = a.x;
    PromotedScalar a_y = a.y;
    PromotedScalar b_x = b.x;
    PromotedScalar b_y = b.y;
    PromotedScalar p_x = p.x;
    PromotedScalar p_y = p.y;

    // Count vertex crossings on the left point, but not on the right.
    if (a_y <= p_y && b_y <= p_y) {
      return false;
    } else if (p_x >= a_x && p_x >= b_x) {
      return false;
    } else if (p_x < a_x && p_x < b_x) {
      return false;
    }

    if (b_x >= a_x) {
      return (p_x - a_x) * (b_y - a_y) > (p_y - a_y) * (b_x - a_x);
    } else {
      return (p_x - a_x) * (b_y - a_y) < (p_y - a_y) * (b_x - a_x);
    }
  }

  // Returns the bearing of the vector (dx, dy), in degrees clockwise from
  // North. The result will be between 0 and 360.
  static float GetBearing(Scalar dx, Scalar dy) {
    float bearing = 90.0f - imp::ToDegrees(atan2(dy, dx));
    if (bearing < 0) {
      bearing += 360;
    }
    return bearing;
  }

  // Returns the signed difference between two bearings in degrees. Returned
  // value is between -180 and 180. It's positive if b is clockwise from a and
  // negative otherwise.
  static float SignedBearingDifference(float a, float b) {
    float difference = b - a;
    while (difference > 180.0f) {
      difference -= 360.0f;
    }
    while (difference < -180.0f) {
      difference += 360.0f;
    }
    return difference;
  }

  // Computes the square of the distance in world units between the given
  // two points. Does not wrap around the meridian. Consider using
  // `ShortestPath`
  static double DistanceSqr(const Vector &a, const Vector &b) {
    Vector diff = a - b;
    return dot(diff, diff);
  }

  // Computes the distance in world units between the given two points.
  // Does not wrap around the meridian. Consider using `ShortestPath` if
  // wrapping is expected. Consider using DistanceSqr if performance is a
  // concern.
  static double Distance(const Vector &a, const Vector &b) {
    return sqrt(DistanceSqr(a, b));
  }

  // Calculates the distances to each point along `begin` to `end`.
  // `out_distances` will contain the distance to each point in world units.
  // <OutputType> A type that will represent the distance values. One of double
  //              or float.
  template <typename OutputType, typename Iterator>
  static void ComputeDistancesAlongPoints(
      Iterator begin, Iterator end, std::vector<OutputType> *out_distances) {
    if (!out_distances) return;
    size_t count = std::distance(begin, end);
    out_distances->resize(count, 0.0);

    double distance = 0.0;
    for (size_t idx = 1; idx < count; ++idx) {
      const Vector &a = begin[idx - 1];
      const Vector &b = begin[idx];

      double distance_a_to_b = Distance(a, b);
      distance += distance_a_to_b;
      (*out_distances)[idx] = static_cast<OutputType>(distance);
    }
  }

  // Returns a point interpolated between `a` (at fraction 0.0) and `b`
  // (at fraction 1.0). `fraction` represents the location of the point to
  // return on the line created by `a` and `b`. This ignores wraparound so
  // may not interpolate along the shortest path between the points.
  // Fraction is not limited to the range [0, 1]., so this may calculcate
  // extrapolations as well.
  static Vector Interpolate(const Vector &a, const Vector &b, float fraction) {
    Vector difference = b - a;
    Vector mult = difference * fraction;
    return mult + a;
  }

  // Returns the fraction along the line segment where the proint is projected.
  // `point` The point to project.
  // `a` The start point of the line segment.
  // `b` The end point of the line segment.
  static float ProjectionFraction(const Vector &point, const Vector &a,
                                  const Vector &b) {
    using FloatVector =
        decltype(std::conditional_t<Vector::SIZE == 2, imp::float2,
                                    imp::float3>());
    if (a == b) {
      return 0.f;
    }

    Vector b_minus_a = b - a;
    Vector point_minus_a = point - a;
    return dot(static_cast<FloatVector>(b_minus_a), point_minus_a) /
           dot(static_cast<FloatVector>(b_minus_a), b_minus_a);
  }

  // Returns the perpendicular projection of the given point onto the line
  // defined by `a` and `b`. `is_line_segment` if true, the projection will be
  // clamped to lie in between `a` and `b`. If false, the projection may lie
  // anywhere on the infinite line defined by those two points.
  static Vector Project(const Vector &point, const Vector &a, const Vector &b,
                        bool is_line_segment) {
    if (a == b) {
      return a;
    }

    float fraction = ProjectionFraction(point, a, b);
    if (is_line_segment) {
      if (fraction <= 0) {
        return a;
      } else if (fraction >= 1) {
        return b;
      }
    }
    return Interpolate(a, b, fraction);
  }

  // Returns the square of the distance in world units from the given point to
  // the line segment defined by `a` and `b`.  Much faster than
  // DistanceToLineSegment. out_closest_point If this pointer is non-NULL, the
  // location pointed to will be set to the closest point on the line segment.
  static double DistanceToLineSegmentSqr(const Vector &point, const Vector &a,
                                         const Vector &b,
                                         Vector *out_closest_point) {
    Vector local_closest = Project(point, a, b, /*is_line_segment=*/true);
    if (out_closest_point) {
      *out_closest_point = local_closest;
    }
    return DistanceSqr(point, local_closest);
  }

  // Computes the distance from the point to the line segment defined by
  // `a` and `b` in world units. Optionally provide `out_closest_point`
  // to retrieve the closest point on the line to the given point. Use
  // DistanceToLineSegmentSqr to avoid a call to `sqrt` in performance
  // sensitive code.
  static double DistanceToLineSegment(const Vector &point, const Vector &a,
                                      const Vector &b,
                                      Vector *out_closest_point) {
    return sqrt(DistanceToLineSegmentSqr(point, a, b, out_closest_point));
  }

  // Returns the radians between the given two vectors. Will be positive
  // and in the range [0, pi].
  static float AngleBetweenVectors(const Vector &v1, const Vector &v2) {
    using FloatVector =
        decltype(std::conditional_t<Vector::SIZE == 2, imp::float2,
                                    imp::float3>());
    float length_1 = length(static_cast<FloatVector>(v1));
    float length_2 = length(static_cast<FloatVector>(v2));
    if (imp::AlmostEqual<float>(length_1, 0) ||
        imp::AlmostEqual<float>(length_2, 0)) {
      return 0;
    }

    float dot_result =
        dot(static_cast<FloatVector>(v1), static_cast<FloatVector>(v2));
    return acos(dot_result / (length_1 * length_2));
  }

 private:
  // Returns true if the projection of P onto AB falls between A and B,
  // and false otherwise.
  static bool FallsBetween(const VectorT &a, const VectorT &b,
                           const VectorT &p) {
    // Return true iff (AB dot AP) / AB.norm2() is between 0 and 1.
    // This will be the case if (AB dot AP) is positive and less than
    // AB.norm2().
    // Length of the projection of AP onto AB is given by (AP dot AB)/||AB||
    // and the question is if 0 <= (AP dot AB)/||AB|| <= ||AB||.
    // Rearranging gives the criterion 0 <= (AP dot AB) <= ||AB||^2
    // Promoted to float to prevent overflow if ab, ap are large.
    const imp::float2 a_float{a};
    const imp::float2 b_float{b};
    const imp::float2 p_float{p};

    float ab_dot_ap = dot(b_float - a_float, p_float - a_float);
    float ab_norm2 = norm2(b_float - a_float);

    return ab_dot_ap >= 0.0 && ab_dot_ap <= ab_norm2;
  }
  PointUtil() = delete;
  ~PointUtil() = delete;
};

using PointUtil2D = PointUtil<imp::int2>;
using PointUtil2f = PointUtil<imp::float2>;

using PointUtil3D = PointUtil<imp::int3>;
using PointUtil3f = PointUtil<imp::float3>;

}  // namespace imp::line_renderer

#endif  // THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_POINT_UTIL_H_
