// Copyright 2025 Google LLC
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

#include "extensions/sceneviewerxr/ux/plane.h"

#include <openxr/openxr.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "extensions/sceneviewerxr/ux/ramp.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/vertex_format.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/materials/svxr_plane_material.h"
#include "vr/android_xr/sceneviewerxr/assets/sceneviewerxr_assets.h"

namespace svxr {

namespace {

constexpr float kPlaneEpsilon = 1e-3f;
constexpr float kPulseHeightThreshold = 0.1f;
constexpr auto kDefaultFalloffColor = imp::float4(0.f);
constexpr auto kDefaultCutoffColor = imp::float4(0.f);
constexpr auto kFullGlowFalloffColor = imp::float4(1.f);
constexpr auto kFullGlowCutoffColor = imp::float4(0.f);
constexpr auto kDefaultPlaneControl = imp::float3(0.01f, 0.1f, 2.f);
constexpr auto kFullGlowPlaneControl = imp::float3(0.01f, 1.f, 2.f);
constexpr auto kFadeoutPlaneControl = imp::float3(0.5f, 2.f, 2.f);

constexpr auto kGlowDuration = absl::Milliseconds(500);
constexpr auto kGlowTimeout = absl::Milliseconds(1000);
constexpr auto kDefaultUvScale = imp::float2(16.f, 16.f);
constexpr char kDebugName[] = "svp";

using ::imp::AssetPtr;
using ::imp::float2;
using ::imp::float3;
using ::imp::Future;
using ::imp::ImageAsset;
using ::imp::OwnedTexturePtr;
using ::imp::TexturePtr;

constexpr float kClipEpsilon = 1e-7f;

typedef std::vector<imp::float2> Polygon;

// Calculates signed area. Positive = CCW, Negative = CW (assuming standard
// Cartesian coords)
float GetSignedArea(const Polygon& p) {
  float area = 0.0f;
  if (p.size() < 3) return 0.0f;

  for (size_t i = 0; i < p.size(); ++i) {
    imp::float2 p1 = p[i];
    imp::float2 p2 = p[(i + 1) % p.size()];
    area += (p1.x * p2.y - p2.x * p1.y);
  }
  return area * 0.5f;
}

// Calculates the absolute area of the polygon
float GetPolygonArea(const Polygon& p) { return std::abs(GetSignedArea(p)); }

// Modifies the polygon in-place to ensure it is Counter-Clockwise
void EnsureCCW(Polygon& p) {
  if (GetSignedArea(p) < 0) {
    std::reverse(p.begin(), p.end());
  }
}

bool IsInside(imp::float2 p, imp::float2 c1, imp::float2 c2) {
  // Cross product check.
  // For CCW polygons, positive result means "Left" (Inside).
  return (c2.x - c1.x) * (p.y - c1.y) - (c2.y - c1.y) * (p.x - c1.x) >=
         -kClipEpsilon;
}

imp::float2 Intersect(imp::float2 s, imp::float2 p, imp::float2 c1,
                      imp::float2 c2) {
  float dx_sp = p.x - s.x, dy_sp = p.y - s.y;
  float dx_c = c2.x - c1.x, dy_c = c2.y - c1.y;
  float denominator = dy_c * dx_sp - dx_c * dy_sp;

  if (std::abs(denominator) < kClipEpsilon) return s;

  float t = (dx_c * (s.y - c1.y) - dy_c * (s.x - c1.x)) / denominator;
  return {s.x + t * dx_sp, s.y + t * dy_sp};
}

// Updated to take Polygons by value so we can modify/normalize them locally
Polygon ClipPolygon(Polygon& subject_polygon, Polygon& clip_polygon) {
  // 1. Normalize both polygons to Counter-Clockwise
  EnsureCCW(subject_polygon);
  EnsureCCW(clip_polygon);

  Polygon output_list = subject_polygon;
  Polygon::size_type clip_count = clip_polygon.size();

  for (int i = 0; i < clip_count; i++) {
    Polygon input_list = output_list;
    output_list.clear();
    imp::float2 c1 = clip_polygon[i];
    imp::float2 c2 = clip_polygon[(i + 1) % clip_count];

    if (input_list.empty()) {
      break;
    }

    imp::float2 s = input_list.back();
    for (const auto& p : input_list) {
      bool s_is_inside = IsInside(s, c1, c2);
      bool p_is_inside = IsInside(p, c1, c2);
      if (s_is_inside && p_is_inside) {
        output_list.push_back(p);
      } else if (s_is_inside && !p_is_inside) {
        output_list.push_back(Intersect(s, p, c1, c2));
      } else if (!s_is_inside && p_is_inside) {
        output_list.push_back(Intersect(s, p, c1, c2));
        output_list.push_back(p);
      }
      s = p;
    }
  }
  return output_list;
}

#define CHECK_ENUM(a, b) \
  static_assert(static_cast<int>(a) == b, #a " must match " #b)

CHECK_ENUM(PlaneTrackingState::kTracking, XR_TRACKING_STATE_TRACKING_ANDROID);
CHECK_ENUM(PlaneTrackingState::kStopped, XR_TRACKING_STATE_STOPPED_ANDROID);
CHECK_ENUM(PlaneTrackingState::kPaused, XR_TRACKING_STATE_PAUSED_ANDROID);

CHECK_ENUM(PlaneType::kHorizontalDownwardFacing,
           XR_PLANE_TYPE_HORIZONTAL_DOWNWARD_FACING_ANDROID);
CHECK_ENUM(PlaneType::kHorizontalUpwardFacing,
           XR_PLANE_TYPE_HORIZONTAL_UPWARD_FACING_ANDROID);
CHECK_ENUM(PlaneType::kVertical, XR_PLANE_TYPE_VERTICAL_ANDROID);
CHECK_ENUM(PlaneType::kArbitrary, XR_PLANE_TYPE_ARBITRARY_ANDROID);

CHECK_ENUM(PlaneLabel::kUnknown, XR_PLANE_LABEL_UNKNOWN_ANDROID);
CHECK_ENUM(PlaneLabel::kWall, XR_PLANE_LABEL_WALL_ANDROID);
CHECK_ENUM(PlaneLabel::kFloor, XR_PLANE_LABEL_FLOOR_ANDROID);
CHECK_ENUM(PlaneLabel::kCeiling, XR_PLANE_LABEL_CEILING_ANDROID);
CHECK_ENUM(PlaneLabel::kTable, XR_PLANE_LABEL_TABLE_ANDROID);

#undef CHECK_ENUM

Future<OwnedTexturePtr> LoadImageTexture(imp::BaseView& view) {
  return view.GetAssetManager()
      .LoadImage(android_xr::kDotPattern64Png)
      .Then([&view](AssetPtr<ImageAsset> image) {
        return OwnedTexturePtr(view.GetTextureFactory().CreateTexture(*image));
      });
}

imp::Rect BuildRect(absl::Span<imp::float2> vertices) {
  imp::float2 v0 = vertices[0];
  imp::float2 min_bound = v0;
  imp::float2 max_bound = v0;
  for (size_t i = 1; i < vertices.size(); ++i) {
    min_bound = min(min_bound, vertices[i]);
    max_bound = max(max_bound, vertices[i]);
  }
  auto center = (min_bound + max_bound) * 0.5f;
  return imp::Rect(center, (max_bound - center) + imp::float2(kPlaneEpsilon));
}

std::unique_ptr<imp::MeshData> BuildMeshData(absl::Span<imp::float2> vertices,
                                             imp::float2 uv_offset) {
  using VertexAttribute = ::imp::VertexFormat::VertexAttribute;
  using AttributeType = ::imp::VertexFormat::AttributeType;
  size_t vertex_count = vertices.size();
  size_t triangle_count = vertex_count - 2;

  auto mesh_data = std::make_unique<imp::MeshData>(imp::MeshDescription{
      .vertex_format = {{VertexAttribute::POSITION, AttributeType::FLOAT3},
                        {VertexAttribute::TANGENTS, AttributeType::FLOAT4},
                        {VertexAttribute::UV0, AttributeType::FLOAT2}},
      .index_type = imp::MeshDescription::IndexType::USHORT,
      .vertex_count = vertex_count,
      .index_count = triangle_count * 3 * 2,
  });

  uint16_t i0 = 0;
  uint16_t i1 = 1;
  size_t indices_index = 0;

  auto tangents = imp::quatf{};
  imp::float4 tangents_vec =
      imp::float4(tangents.x, tangents.y, tangents.z, tangents.w);

  mesh_data->VertexAttributeAt<imp::float3>(0, VertexAttribute::POSITION) =
      imp::float3(vertices[0].x, 0.0f, vertices[0].y);
  mesh_data->VertexAttributeAt<imp::float4>(0, VertexAttribute::TANGENTS) =
      tangents_vec;
  mesh_data->VertexAttributeAt<imp::float2>(0, VertexAttribute::UV0) =
      vertices[0] + uv_offset;
  mesh_data->VertexAttributeAt<imp::float3>(1, VertexAttribute::POSITION) =
      imp::float3(vertices[1].x, 0.0f, vertices[1].y);
  mesh_data->VertexAttributeAt<imp::float4>(1, VertexAttribute::TANGENTS) =
      tangents_vec;
  mesh_data->VertexAttributeAt<imp::float2>(1, VertexAttribute::UV0) =
      vertices[1] + uv_offset;

  for (size_t vertex_index = 2; vertex_index < vertex_count; ++vertex_index) {
    mesh_data->VertexAttributeAt<imp::float3>(vertex_index,
                                              VertexAttribute::POSITION) =
        imp::float3(vertices[vertex_index].x, 0.0f, vertices[vertex_index].y);
    mesh_data->VertexAttributeAt<imp::float4>(
        vertex_index, VertexAttribute::TANGENTS) = tangents_vec;
    mesh_data->VertexAttributeAt<imp::float2>(vertex_index,
                                              VertexAttribute::UV0) =
        vertices[vertex_index] + uv_offset;

    uint16_t i2 = vertex_index;
    mesh_data->IndexAt<uint16_t>(indices_index++) = i0;
    mesh_data->IndexAt<uint16_t>(indices_index++) = i1;
    mesh_data->IndexAt<uint16_t>(indices_index++) = i2;
    // Add a backwards triangle to make the mesh double-sided.
    mesh_data->IndexAt<uint16_t>(indices_index++) = i0;
    mesh_data->IndexAt<uint16_t>(indices_index++) = i2;
    mesh_data->IndexAt<uint16_t>(indices_index++) = i1;
    i1 = i2;
  }
  return mesh_data;
}

}  // namespace

Plane::Plane() : machine_(plane_interaction_states::Initialized{}, this) {}

Plane::~Plane() = default;

imp::Future<absl::Status> Plane::Setup(PlaneTrackingState tracking_state,
                                       PlaneType type, PlaneLabel label,
                                       absl::Span<imp::float2> vertices) {
  auto& view = GetNode()->GetView();

  auto image_texture_future = LoadImageTexture(view);

  auto material_future = android_xr::SVXRPlaneMaterial::Create(view);

  return material_future.Merge(image_texture_future)
      .Then([this, tracking_state, type, label,
             vertices_copy =
                 std::vector<imp::float2>(vertices.begin(), vertices.end())](
                std::tuple<std::unique_ptr<android_xr::SVXRPlaneMaterial>,
                           OwnedTexturePtr>
                    result) mutable -> absl::Status {
        auto& [material, texture] = result;

        return Setup(tracking_state, type, label, absl::MakeSpan(vertices_copy),
                     std::move(material), std::move(texture));
      });
}

bool Plane::IsRelevant() const {
  if (tracking_state_ != PlaneTrackingState::kTracking) {
    return false;
  }

  if (type_ != PlaneType::kHorizontalUpwardFacing) {
    // The projected point is not on a plane that we care about.
    return false;
  }

  auto plane_node = GetNode();
  auto world_from_local = plane_node->GetWorldTrs();
  auto normal = (world_from_local * imp::float4(0.f, 1.f, 0.f, 0.f)).xyz;
  bool is_really_upward_facing = dot(normal, imp::kUp) > 0.9f;

  if (!is_really_upward_facing) {
    return false;
  }

  return true;
}

bool Plane::ShouldBeActive(imp::float3 target_position_world) const {
  auto plane_node = GetNode();

  if (!IsRelevant()) {
    return false;
  }

  auto target_position_local =
      plane_node->LocalFromWorldPoint(target_position_world);

  if (fabs(target_position_local.y) > kPulseHeightThreshold) {
    return false;
  }

  return true;
}

void Plane::Activate(imp::float3 target_position_world) {
  material_->SetHighlightPoint(target_position_world);

  machine_.UpdateWithAlternatives(
      [this](plane_interaction_states::Hidden& state)
          -> InteractionMachine::OptionalState {
        return plane_interaction_states::Active{
            .glow = Ramp<float>(0.f),
            .last_interaction_time = GetView().GetFrameTime().GetElapsedTime(),
        };
      },
      [this](plane_interaction_states::Active& state)
          -> InteractionMachine::OptionalState {
        if (state.glow.GetTarget() != 1.f) {
          state.glow.SetTarget(1.f, kGlowDuration);
        }
        state.last_interaction_time = GetView().GetFrameTime().GetElapsedTime();
        return {};
      },
      [](auto& state) -> InteractionMachine::OptionalState { return {}; });
}

absl::Status Plane::Setup(
    PlaneTrackingState tracking_state, PlaneType type, PlaneLabel label,
    absl::Span<imp::float2> vertices,
    std::unique_ptr<android_xr::SVXRPlaneMaterial> material,
    OwnedTexturePtr texture) {
  texture_ = std::move(texture);
  material_ = std::move(material);
  tracking_state_ = tracking_state;
  type_ = type;
  label_ = label;

  material_->SetDotPattern(texture_.Borrow());
  material_->SetPlaneControl(kDefaultPlaneControl);
  material_->SetFalloffColor(kDefaultFalloffColor);
  material_->SetCutoffColor(kDefaultCutoffColor);
  material_->SetUVScale(kDefaultUvScale);

  absl::c_copy(vertices, std::back_inserter(vertices_));
  rect_ = BuildRect(vertices);

  auto node = GetNode();
  renderer_ = node->AddComponent<imp::MeshRenderer>();
  imp::Box aabb = imp::Box(
      imp::float3(rect_.center.x, 0.f, rect_.center.y),
      imp::float3(rect_.half_extent.x, kPlaneEpsilon, rect_.half_extent.y));

  // The system may segment a large surface into multiple planes, so make the
  // UVs coherent across plane boundaries by applying an offset based on the
  // origin of the plane.
  auto world_from_node = node->GetWorldTrs();
  auto u_vector = (world_from_node * imp::float4(imp::kRight, 0.f)).xyz;
  auto v_vector = (world_from_node * imp::float4(imp::kBack, 0.f)).xyz;
  auto world_pos = (world_from_node * imp::float4(imp::kZero3, 1.f)).xyz;
  auto uv_offset =
      imp::float2(dot(u_vector, world_pos), dot(v_vector, world_pos));

  renderer_->SetMesh(GetView().GetMeshFactory().CreateByMovingMeshData(
      filament::backend::PrimitiveType::TRIANGLES,
      BuildMeshData(vertices, uv_offset), aabb,
      imp::MeshFactory::MeshDataStorageMode::kDiscardMeshData, kDebugName));

  renderer_->SetMaterial(material_->GetMaterial());

  CalculateArea();

  return absl::OkStatus();
}

void Plane::OnUpdate(const imp::FrameTime& delta_time) {
  machine_.UpdateWithAlternatives(
      [](plane_interaction_states::Initialized& state)
          -> plane_interaction_states::Machine::OptionalState {
        return plane_interaction_states::Hidden{};
      },
      [](plane_interaction_states::Hidden& state)
          -> plane_interaction_states::Machine::OptionalState { return {}; },
      [delta_time, this](plane_interaction_states::Active& state)
          -> plane_interaction_states::Machine::OptionalState {
        return UpdateActive(state, delta_time);
      });
}

float Plane::GetCollisionArea(const Plane& other) const {
  if (!renderer_ || !other.renderer_) {
    return 0.0f;
  }

  bool this_is_horizontal = GetType() == PlaneType::kHorizontalUpwardFacing ||
                            GetType() == PlaneType::kHorizontalDownwardFacing;
  bool other_is_horizontal =
      other.GetType() == PlaneType::kHorizontalUpwardFacing ||
      other.GetType() == PlaneType::kHorizontalDownwardFacing;

  if (!this_is_horizontal || !other_is_horizontal) {
    return 0.0f;
  }

  if (GetTrackingState() != PlaneTrackingState::kTracking ||
      other.GetTrackingState() != PlaneTrackingState::kTracking) {
    return 0.0f;
  }

  // TODO: Use the rect_ property for a quick check if planes
  // collide.

  // Do full polygon intersection on XZ plane.
  auto create_polygon = [](const Plane& plane) {
    Polygon polygon;
    auto world_trs = plane.GetNode()->GetWorldTrs();
    for (const auto& v : plane.vertices_) {
      imp::float3 wp = (world_trs * imp::float4(v.x, 0, v.y, 1.0)).xyz;
      polygon.push_back({wp.x, wp.z});
    }
    return polygon;
  };

  Polygon subject = create_polygon(*this);
  Polygon clip = create_polygon(other);

  Polygon intersection_polygon = ClipPolygon(subject, clip);

  return GetPolygonArea(intersection_polygon);
}

float Plane::GetArea() const { return area_; }

void Plane::Cleanup() {
  GetNode()->RemoveComponent<imp::MeshRenderer>();
  material_.reset();
  texture_ = {};
}

void Plane::OnStateChange(const InteractionMachine& machine,
                          const InteractionMachine::State& current_state,
                          const InteractionMachine::State& next_state) {
  if (std::holds_alternative<plane_interaction_states::Hidden>(next_state)) {
    // Disable the node when in Hidden so SysUI hands don't render over it.
    GetNode()->SetEnabled(false);
  } else if (std::holds_alternative<plane_interaction_states::Active>(
                 next_state)) {
    GetNode()->SetEnabled(true);
  }
}

Plane::InteractionMachine::OptionalState Plane::UpdateActive(
    plane_interaction_states::Active& state, const imp::FrameTime& delta_time) {
  absl::Duration non_interaction_time =
      delta_time.GetElapsedTime() - state.last_interaction_time;

  if (non_interaction_time > kGlowTimeout && state.glow.GetTarget() != 0.f) {
    state.glow.SetTarget(0.f, kGlowDuration);
  }

  if (!state.glow.IsAtTarget()) {
    state.glow.Step(delta_time.GetDeltaTime());
  }

  float glow = state.glow.Get();
  bool fading_out = state.glow.GetTarget() == 0.f;

  auto falloff_color = kDefaultFalloffColor +
                       (kFullGlowFalloffColor - kDefaultFalloffColor) * glow;

  auto cutoff_color =
      kDefaultCutoffColor + (kFullGlowCutoffColor - kDefaultCutoffColor) * glow;

  auto control = fading_out
                     ? (kFadeoutPlaneControl +
                        (kFullGlowPlaneControl - kFadeoutPlaneControl) * glow)
                     : (kDefaultPlaneControl +
                        (kFullGlowPlaneControl - kDefaultPlaneControl) * glow);

  material_->SetFalloffColor(falloff_color);
  material_->SetCutoffColor(cutoff_color);
  material_->SetPlaneControl(control);

  if (glow == 0.f) {
    return plane_interaction_states::Hidden{};
  }

  return {};
}

void Plane::CalculateArea() {
  if (vertices_.empty()) return;
  area_ = GetPolygonArea(vertices_);
}

}  // namespace svxr
