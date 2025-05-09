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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_DEBUG_DRAW_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_DEBUG_DRAW_H_

#include <cstdint>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/collision/collision_flags.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"

// Debug draw provides the ability to easily add simple debug geometry to a
// Filament scene in order to graphically annotate the contents of the scene
// for debugging and iteration.
namespace imp {
namespace debug_draw {

// Color values are passed as normalized sRGBA ubyte4.
using Color = filament::math::ubyte4;

enum class DebugColor {
  kRed,
  kPink,
  kPurple,
  kDeepPurple,
  kIndigo,
  kBlue,
  kLightBlue,
  kCyan,
  kTeal,
  kGreen,
  kGreenSemiHighlight,
  kGreenHighlight,
  kLightGreen,
  kLime,
  kYellow,
  kAmber,
  kOrange,
  kDeepOrange,
  kDebugColorCount,
};
constexpr size_t kDebugColorCount =
    static_cast<size_t>(DebugColor::kDebugColorCount);

// From the 600 set at material.io
static constexpr Color kDebugColors[kDebugColorCount] = {
    {0xe5, 0x39, 0x35, 0xff},  // kRed (Material Red600)
    {0xd8, 0x1b, 0x60, 0xff},  // kPink (Material Pink-600)
    {0x8e, 0x24, 0xaa, 0xff},  // kPurple (Material Purple-600)
    {0x5e, 0x35, 0xb1, 0xff},  // kDeepPurple (Material DeepPurple-600)
    {0x39, 0x49, 0xab, 0xff},  // kIndigo (Material Indigo-600)
    {0x1e, 0x88, 0xe5, 0xff},  // kBlue (Material Blue-600)
    {0x03, 0x9b, 0xe5, 0xff},  // kLightBlue (Material LightBlue-600)
    {0x00, 0xac, 0xc1, 0xff},  // kCyan (Material Cyan-600)
    {0x00, 0x89, 0x7b, 0xff},  // kTeal (Material Teal-600)
    {0x43, 0xa0, 0x47, 0xff},  // kGreen (Material Green-600)
    {0xa5, 0xd6, 0xa7, 0xff},  // kGreenSemiHighlight (Material Green-400)
    {0x66, 0xbb, 0x6a, 0xff},  // kGreenHighlight (Material Green-200)
    {0x7c, 0xb3, 0x42, 0xff},  // kLightGreen (Material LightGreen-600)
    {0xc0, 0xca, 0x33, 0xff},  // kLime (Material Lime-600)
    {0xfd, 0xd8, 0x35, 0xff},  // kYellow (Material Yellow-600)
    {0xff, 0xb3, 0x00, 0xff},  // kAmber (Material Amber-600)
    {0xfb, 0x8c, 0x00, 0xff},  // kOrange (Material Orange-600)
    {0xf4, 0x51, 0x1e, 0xff},  // kDeepOrange (Material DeepOrange-600)
};

// Convenience method
inline Color GetColor(DebugColor color) {
  return kDebugColors[static_cast<size_t>(color)];
}

// Provides a stable color for a given index.
inline Color ColorFromIndex(size_t i) {
  return kDebugColors[i % kDebugColorCount];
}

inline Color DefaultColorFromVisualizationStyle(
    VisualizationStyle visualization_style) {
  return visualization_style == VisualizationStyle::kSelected
             ? debug_draw::GetColor(debug_draw::DebugColor::kGreenHighlight)
             : (visualization_style == VisualizationStyle::kSelectedDescendent
                    ? debug_draw::GetColor(
                          debug_draw::DebugColor::kGreenSemiHighlight)
                    : debug_draw::GetColor(debug_draw::DebugColor::kGreen));
}

// TODO Move this into a helper class to do the conversion
inline Rect ScreenToClipSpace(float2 screen_size, Rect screen_rect) {
  float2 center =
      screen_rect.center / screen_size * float2{2.0f, -2.0f} - float2{1, -1};
  float2 half_extent = screen_rect.half_extent / screen_size * 2;
  return Rect{center, half_extent};
}

inline float2 ScreenToClipSpace(float2 screen_size, float2 screen_point) {
  return screen_point / screen_size * float2{2.0f, -2.0f} - float2{1, -1};
}

// Returns Vector2f(0, 0) for the normalized if vector is Vector2f(0, 0)
inline imp::float2 SafeNormalized(const imp::float2& v) {
  if (v.x == 0.0 && v.y == 0.0) {
    return v;
  } else {
    return normalize(v);
  }
}

inline imp::float2 Orthogonal(const imp::float2& v) { return {v.y, -v.x}; }

// 2D cross product in the x and y components; dots the result of cross(a, b)
// with the negative Y axis.
inline float Cross2D(const imp::float2& a, const imp::float2& b) {
  return (a.y * b.x) - (a.x * b.y);
}

float2 ShiftPointOrthogonally(float2 prev, float2 point, float2 next,
                              float distance);

std::vector<float2> ShiftPointsOrthogonally(const std::vector<float2>& points,
                                            float distance);

std::vector<float2> GenerateArc(float2 start, float2 end, bool invert);

std::vector<float2> GenerateOutline2D(const std::vector<float2>& path,
                                      float width);

std::vector<float2> To2DPath(const std::vector<float3>& path_3d);

// Geometry buffers may be submitted directly for debug drawing if a user wishes
// to render more advance geometry than is possible using the built-in debug
// draw geometry apis for lines and points.
using PositionBuffer = std::vector<float3>;
using ColorBuffer = std::vector<Color>;
using IndexBuffer = std::vector<uint16_t>;

struct Geometry {
  filament::backend::PrimitiveType type;
  PositionBuffer positions;
  ColorBuffer colors;
  IndexBuffer indices;
};

enum class VertexSpace {
  kModelView,
  kScreen,
};

// Provides the mechanism for setting up debug drawing, attaching it to a
// Filament instance, advancing the debug drawing state, and cleaning up.  Note
// that in order to make debug drawing calls as useful as possible, we require
// that there is exactly one Fixture active and in use at any given time when
// a debug drawing is used.  This way the user does not have to plumb the
// Fixture to all of the places debug drawing is used.
class Fixture final {
 public:
  // Sets up debug_drawing to be associated with the given scene.  Note that
  // only one fixture may be created at any given time.
  Fixture(filament::Engine* engine, filament::Scene* scene,
          filament::Material* debug_material = nullptr,
          filament::Material* debug_screenspace_material = nullptr);
  ~Fixture();

  // Advances debug_drawing a single frame and updates the Filament scene
  // appropriately given the current debug drawing state.
  void Advance();
};

class DrawSpace {
 public:
  // Not copyable or movable
  DrawSpace(const DrawSpace&) = delete;
  DrawSpace& operator=(const DrawSpace&) = delete;

  // Class should be stack-allocated to provide RAII-style activation and
  // submission of debug draw calls for a local coordinate space.
  void* operator new(size_t size) = delete;
  void operator delete(void* p) = delete;
  void* operator new[](size_t size) = delete;
  void operator delete[](void* p) = delete;

  // Draws a point at the given position with the given color.
  void Point(const filament::math::float3& position, const Color& color);

  // Draws a line from start to end with the given color.
  void Line(const filament::math::float3& start,
            const filament::math::float3& end, const Color& color);

  // Draws generic debug geometry by submitting vertex/color/index buffers.
  void UserDefined(debug_draw::Geometry geometry);

 protected:
  explicit DrawSpace(utils::Entity entity, uint32_t duration_frames = 1u,
                     VertexSpace vertex_space = VertexSpace::kModelView);
  ~DrawSpace();

  const utils::Entity entity_;
  const uint32_t duration_frames_;
  std::vector<Geometry> geometry_snippets_;
  VertexSpace vertex_space_;
};

// Provides scope-controlled debug drawing in the coordinate space of a
// given entity.  Note that until this object is destroyed, the draw calls
// within its scope will not be submitted, so it should be stack-allocated and
// go out of scope once debug draw calls are complete.
// Example:
//    debug_draw::Local draw_local{my_entity};
//    draw_local.Line({0.0f, 0.0f, 0.0f},
//                    {1.0f, 0.0f, 0.0f},
//                    {0xff, 0xff, 0xff, 0xff});
class Local : public DrawSpace {
 public:
  // Activates a given entity for local debug drawing which will persist for the
  // given number of frames, with a minimum of 1 frame of persistence.
  explicit Local(utils::Entity entity, uint32_t duration_frames = 1u)
      : DrawSpace(entity, duration_frames, VertexSpace::kModelView) {}

  // Draws the edges of an axis-aligned bounding box with the given color.
  void BoxLines(const filament::Box& box, const Color& color);

  // Draws the solid faces of an axis-aligned bounding box with the given color.
  void BoxFaces(const filament::Box& box, const Color& color);

  // Draws a sphere given its position and radius with lines defining its shape
  // with the given color.
  void SphereLines(const filament::math::float3& center, float radius,
                   const Color& color);

  // Draws a capsule with lines defining its shape with the given color.
  void CapsuleLines(const filament::math::float3& center, float height,
                    float radius, const Color& color);

  void MeshLines(absl::Span<const MeshVertexAndIndexData> mesh,
                 const Color& color);
};

// Provides a convenient way to draw relative to the global coordinate space.
// Example:
//    debug_draw::Global{}.Point({0.0, 0.0, 0.0}, {0xff, 0x00, 0x00, 0xff});
class Global : public Local {
 public:
  explicit Global(uint32_t duration_frames = 1u) : Local({}, duration_frames) {}
};

// Provides a convenient way to draw relative to the normalized screen (clip)
// coordinate space. (-1.0f, 1.0f) is the upper left corner and (1.0f, -1.0f) is
// the lower right corner.
// Example:
//    debug_draw::Screen{}.Line({-0.5f, 0.5f}, {0.5f, -0.5f} {0xff, 0x00, 0x00,
//    0xff});
//
// TODO: Refactor debug_draw so that it can have access to
// imp::View and then add in a new Screen class that's defined in pixel
// coordinate.
class NormalizedScreen : public DrawSpace {
 public:
  explicit NormalizedScreen(uint32_t duration_frames = 1u)
      : DrawSpace({}, duration_frames, VertexSpace::kScreen) {}

  // Draws a point at the given position with the given color.
  void Point(const filament::math::float2& position, const Color& color) {
    DrawSpace::Point(float3(position.x, position.y, 0.0f), color);
  }

  // Draws a line from start to end with the given color.
  void Line(const filament::math::float2& start,
            const filament::math::float2& end, const Color& color) {
    DrawSpace::Line(float3(start.x, start.y, 0.0f), float3(end.x, end.y, 0.0f),
                    color);
  }

  void RectLines(const Rect& rect, const Color& color);
};

}  // namespace debug_draw
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_DEBUG_DRAW_H_
