// Copyright 2024 Google LLC
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

#include "core/text/text_collider.h"

#include <optional>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/collision/collision_flags.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/plane.h"
#include "core/collision/ray.h"
#include "core/common/debug_draw.h"
#include "core/common/enum_flags.h"
#include "core/common/filament_helpers.h"
#include "core/geometry/shapes/line_segment.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/text/text_renderer.h"
#include "core/view/base_view.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/collision_system.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

namespace {
using collision::AABBIntersectsRay;
using collision::Capsule2DContainPoint;
using collision::PlaneIntersectsRay;
using collision::RayIntersection;
using collision::RectContainsPoint;
using collision::Result;
}  // namespace

absl::Status TextCollider::Setup() {
  ComponentHandle<TextRenderer> text_renderer =
      GetNode()->GetComponent<TextRenderer>();
  if (!text_renderer) {
    return absl::FailedPreconditionError(
        "Node does not contain a TextRenderer.");
  }

  text_renderer_ = text_renderer;
  collision_flags_ = imp::ToFlags(CollisionMask::kTextCollider);

  return absl::OkStatus();
}

std::optional<RayHit> TextCollider::Intersect(const Ray& ray) {
  if (!text_renderer_.IsValid()) {
    return std::nullopt;
  }
  if (text_renderer_->GetVertexDomain() == filament::VertexDomain::DEVICE) {
    return std::nullopt;
  }

  mat4f local_from_world = inverse(GetNode()->GetWorldTrs());
  Ray local_ray = ray.GetTransformed(local_from_world);
  // TODO Consider adding a helper function to compare the 2D AABB
  // with the point, instead of making 3D AABB checking.
  std::optional<RayIntersection<float>> intersection =
      AABBIntersectsRay(GetLocalBounds(), local_ray);
  if (!intersection.has_value()) {
    return std::nullopt;
  }

  if (!text_renderer_->GetTextLayoutProvider().has_value()) {
    intersection->collision_point =
        GetNode()->WorldFromLocalPoint(intersection->collision_point);
    intersection->distance = norm(intersection->collision_point - ray.origin);
  } else {
    auto path = text_renderer_->GetLocalPath();
    float text_half_height =
        text_renderer_->GetTextHeight() * (1 + state_.padding_ratio) * 0.5f;
    bool hit_path = false;

    if (path.ok() && !path->empty()) {
      Plane plane(float3{0., 0., 1.}, 0.);
      float3 hit_point_on_plane;
      if (PlaneIntersectsRay(plane, local_ray, &hit_point_on_plane) ==
          Result::kDoesNotIntersect) {
        return std::nullopt;
      }

      for (int i = 0; i < path->size() - 1; ++i) {
        if (Capsule2DContainPoint(LineSegment(path->at(i), path->at(i + 1)),
                                  text_half_height, hit_point_on_plane.xy) ==
            Result::kDoesIntersect) {
          intersection->collision_point =
              GetNode()->WorldFromLocalPoint(hit_point_on_plane);
          intersection->distance =
              norm(intersection->collision_point - ray.origin);
          hit_path = true;
          break;
        }
      }
    }
    if (!hit_path) {
      return std::nullopt;
    }
  }

  return RayHit(intersection->distance, kIdentityQuatf,
                intersection->collision_point, GetNode());
}

std::optional<DoubleRayHit> TextCollider::IntersectPrecise(
    const DoubleRay& ray) {
  if (!text_renderer_.IsValid()) {
    return std::nullopt;
  }
  if (text_renderer_->GetVertexDomain() == filament::VertexDomain::DEVICE) {
    return std::nullopt;
  }

  mat4 local_from_world = inverse(GetNode()->GetWorldTrsPrecise());
  DoubleRay local_ray = ray.GetTransformed(local_from_world);
  // TODO Consider adding a helper function to compare the 2D AABB
  // with the point, instead of making 3D AABB checking.
  std::optional<RayIntersection<double>> intersection =
      AABBIntersectsRay(GetLocalBounds(), local_ray);
  if (!intersection.has_value()) {
    return std::nullopt;
  }

  if (!text_renderer_->GetTextLayoutProvider().has_value()) {
    intersection->collision_point =
        GetNode()->WorldFromLocalPointPrecise(intersection->collision_point);
    intersection->distance = norm(intersection->collision_point - ray.origin);
  } else {
    auto path = text_renderer_->GetLocalPath();
    float text_half_height =
        text_renderer_->GetTextHeight() * (1 + state_.padding_ratio) * 0.5f;
    bool hit_path = false;

    if (path.ok() && !path->empty()) {
      DoublePlane plane(float3{0., 0., 1.}, 0.);
      double3 hit_point_on_plane;
      if (PlaneIntersectsRay(plane, local_ray, &hit_point_on_plane) ==
          Result::kDoesNotIntersect) {
        return std::nullopt;
      }

      for (int i = 0; i < path->size() - 1; ++i) {
        if (Capsule2DContainPoint(LineSegment(path->at(i), path->at(i + 1)),
                                  text_half_height, hit_point_on_plane.xy) ==
            Result::kDoesIntersect) {
          intersection->collision_point =
              GetNode()->WorldFromLocalPointPrecise(hit_point_on_plane);
          intersection->distance =
              norm(intersection->collision_point - ray.origin);
          hit_path = true;
          break;
        }
      }
    }
    if (!hit_path) {
      return std::nullopt;
    }
  }

  return DoubleRayHit(intersection->distance,
                      /*world_orientation=*/{1, 0, 0, 0},
                      intersection->collision_point, GetNode());
}

std::optional<RayHit> TextCollider::Intersect(float2 screen_pos) {
  if (!text_renderer_.IsValid()) {
    return std::nullopt;
  }
  if (text_renderer_->GetVertexDomain() != filament::VertexDomain::DEVICE) {
    return Intersect(
        GetView().GetCameraManager().GetCamera()->WorldRayFromPixelPoint(
            screen_pos));
  }

  absl::StatusOr<Rect> bounds = GetScreenBounds();
  if (!bounds.ok()) {
    return std::nullopt;
  }

  // Expand the bounds by the global_screen_padding in all directions to account
  // for the tap radius.
  float global_screen_padding = GetView()
                                    .GetComponentManager()
                                    .GetComponentSystem<TextCollider>()
                                    .GetScreenPadding();
  bounds->half_extent += global_screen_padding;

  if (!text_renderer_->GetTextLayoutProvider().has_value()) {
    if (RectContainsPoint(*bounds, screen_pos) != Result::kDoesIntersect) {
      return std::nullopt;
    }
  } else {
    auto screen_path = GetScaledScreenPath();
    float2 subpixel_ratio = GetView().GetHost()->GetSubpixelRatio();
    float text_half_height = text_renderer_->GetTextHeight() *
                             (1 + state_.padding_ratio) / subpixel_ratio.y *
                             0.5f;
    bool hit_path = false;

    if (screen_path.ok() && !screen_path->empty()) {
      for (int i = 0; i < screen_path->size() - 1; ++i) {
        if (Capsule2DContainPoint(
                LineSegment(imp::float3(screen_path->at(i), 0.0),
                            imp::float3(screen_path->at(i + 1), 0.0)),
                text_half_height + global_screen_padding,
                screen_pos) == Result::kDoesIntersect) {
          hit_path = true;
          break;
        }
      }
    }
    if (!hit_path) {
      return std::nullopt;
    }
  }

  return RayHit(0.0f, kIdentityQuatf, float3{screen_pos, 0}, GetNode());
}

std::optional<DoubleRayHit> TextCollider::IntersectPrecise(float2 screen_pos) {
  if (!text_renderer_.IsValid()) {
    return std::nullopt;
  }
  if (text_renderer_->GetVertexDomain() != filament::VertexDomain::DEVICE) {
    return IntersectPrecise(
        GetView().GetCameraManager().GetCamera()->WorldRayFromPixelPointPrecise(
            screen_pos));
  }

  absl::StatusOr<Rect> bounds = GetScreenBounds();
  if (!bounds.ok()) {
    return std::nullopt;
  }

  // Expand the bounds by the global_screen_padding in all directions to account
  // for the tap radius.
  float global_screen_padding = GetView()
                                    .GetComponentManager()
                                    .GetComponentSystem<TextCollider>()
                                    .GetScreenPadding();
  bounds->half_extent += global_screen_padding;

  if (!text_renderer_->GetTextLayoutProvider().has_value()) {
    if (RectContainsPoint(*bounds, screen_pos) != Result::kDoesIntersect) {
      return std::nullopt;
    }
  } else {
    auto screen_path = GetScaledScreenPath();
    float2 subpixel_ratio = GetView().GetHost()->GetSubpixelRatio();
    float text_half_height =
        text_renderer_->GetTextHeight() / subpixel_ratio.y * 0.5f;
    bool hit_path = false;

    if (screen_path.ok() && !screen_path->empty()) {
      for (int i = 0; i < screen_path->size() - 1; ++i) {
        if (Capsule2DContainPoint(
                LineSegment(imp::float3(screen_path->at(i), 0.0),
                            imp::float3(screen_path->at(i + 1), 0.0)),
                text_half_height + global_screen_padding,
                screen_pos) == Result::kDoesIntersect) {
          hit_path = true;
          break;
        }
      }
    }
    if (!hit_path) {
      return std::nullopt;
    }
  }

  return DoubleRayHit(0.0f, kIdentityQuat, double3{screen_pos, 0}, GetNode());
}

bool TextCollider::IsInScreenSpace() const {
  return text_renderer_->GetVertexDomain() == filament::VertexDomain::DEVICE;
}

absl::StatusOr<std::vector<float2>> TextCollider::GetScaledScreenPath() const {
  absl::StatusOr<std::vector<float3>> screen_path =
      text_renderer_->GetScreenPath();
  if (!screen_path.ok()) return screen_path.status();
  absl::StatusOr<float2> screen_position =
      text_renderer_->GetNodeScreenPosition();
  if (!screen_position.ok()) return screen_position.status();

  float2 subpixel_ratio = GetView().GetHost()->GetSubpixelRatio();
  float2 scaled_screen_position = *screen_position * subpixel_ratio;

  std::vector<float2> scaled_screen_path;
  scaled_screen_path.resize(screen_path->size());
  for (int i = 0; i < screen_path->size(); ++i) {
    // Scale the original screen path by the `padding_ratio` (applying
    // perspective scaling).
    scaled_screen_path[i] = (screen_path->at(i).xy - scaled_screen_position) *
                                (1 + state_.padding_ratio) +
                            scaled_screen_position;
  }

  return scaled_screen_path;
}

absl::StatusOr<Rect> TextCollider::GetScreenBounds() const {
  if (!text_renderer_.IsValid()) {
    return absl::UnavailableError("TextRenderer is not valid");
  }
  absl::StatusOr<Rect> bounds = text_renderer_->GetScreenLocalBounds();
  if (!bounds.ok()) {
    return bounds.status();
  }
  absl::StatusOr<float2> screen_position =
      text_renderer_->GetNodeScreenPosition();
  if (!screen_position.ok()) {
    return screen_position.status();
  }

  float scale = 1 + state_.padding_ratio;
  bounds->center *= scale;
  bounds->center += *screen_position;

  bounds->half_extent *= scale;
  return bounds;
}

Box TextCollider::GetLocalBounds() const {
  Box bounds = text_renderer_->GetLocalBounds();
  bounds.halfExtent *= (1 + state_.padding_ratio);
  return bounds;
}

Box TextCollider::GetWorldBounds() const {
  return TransformBounds(GetLocalBounds(), GetNode()->GetWorldTrs());
}

void TextCollider::Visualize(VisualizationStyle visualization_style) {
  if (!text_renderer_.IsValid()) {
    return;
  }
  debug_draw::Color color =
      debug_draw::DefaultColorFromVisualizationStyle(visualization_style);
  if (text_renderer_->GetVertexDomain() == filament::VertexDomain::DEVICE) {
    absl::StatusOr<std::vector<float3>> screen_path =
        text_renderer_->GetScreenPath();
    if (!screen_path.ok()) {
      absl::StatusOr<Rect> screen_bounds = GetScreenBounds();
      if (!screen_bounds.ok()) {
        IMP_LOG(imp::WARNING) << screen_bounds.status();
        return;
      }

      debug_draw::NormalizedScreen().RectLines(
          debug_draw::ScreenToClipSpace(GetView().GetSize(), *screen_bounds),
          color);

      return;
    }

    if (text_renderer_->IsPathChanged()) {
      absl::StatusOr<std::vector<float2>> scaled_screen_path =
          GetScaledScreenPath();
      if (!scaled_screen_path.ok()) {
        IMP_LOG(imp::WARNING) << scaled_screen_path.status();
        return;
      }

      float2 subpixel_ratio = GetView().GetHost()->GetSubpixelRatio();
      float text_height = text_renderer_->GetTextHeight() *
                          (1 + state_.padding_ratio) / subpixel_ratio.y;

      path_text_collider_outline_ =
          debug_draw::GenerateOutline2D(*scaled_screen_path, text_height);

      text_renderer_->ResetPathChanged();
    }

    for (int i = 0; i < path_text_collider_outline_.size() - 1; ++i) {
      debug_draw::NormalizedScreen().Line(
          imp::debug_draw::ScreenToClipSpace(GetView().GetSize(),
                                             path_text_collider_outline_.at(i)),
          imp::debug_draw::ScreenToClipSpace(
              GetView().GetSize(), path_text_collider_outline_.at(i + 1)),
          color);
    }

    return;
  }

  debug_draw::Local(GetNode().GetEntity()).BoxLines(GetLocalBounds(), color);
}

void TextCollider::System::BeforeFirstComponentAdded() {
  GetView()
      .GetCollisionManager()
      .AddCollisionSystem<TextCollider, CollisionSystem>(&GetView());
}

void TextCollider::System::AfterLastComponentRemoved() {
  GetView()
      .GetCollisionManager()
      .RemoveCollisionSystem<TextCollider, CollisionSystem>();
}

void TextCollider::System::SetScreenPadding(float screen_padding) {
  screen_padding_ = screen_padding;
}

float TextCollider::System::GetScreenPadding() const { return screen_padding_; }

}  // namespace imp
