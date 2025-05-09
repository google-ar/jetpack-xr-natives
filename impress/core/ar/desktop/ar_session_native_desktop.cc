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

#include "core/ar/desktop/ar_session_native_desktop.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#include <memory>
#include <numeric>

#include "glog/logging.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/ar/ar_hdr_lighting.h"
#include "core/collision/collision_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/lighting/environment_light_factory.h"
#include "core/math/math.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "dear_imgui/imgui.h"
#include "filament/filament/include/filament/IndirectLight.h"
#include "filament/filament/include/filament/Stream.h"
#include "filament/libs/math/include/math/vec3.h"

namespace imp {
namespace ar {

namespace {
// Rotation of floor plane in radians.
constexpr float kAngle = ToRadians(30.0);
// The floor plane transform in world space.
const mat4f kWorldFloorTransform(float4{cos(kAngle), 0.0f, -sin(kAngle), 0.0},
                                 float4{0.0f, 1.0f, 0.0f, 0.0f},
                                 float4{sin(kAngle), 0.0f, cos(kAngle), 0.0f},
                                 float4{-0.58f, -0.98f, -1.9f, 1.0f});
// Placement of couch plane
const mat4f kCouchTransform(float4{cos(kAngle), 0.0f, -sin(kAngle), 0.0},
                            float4{0.0f, 1.0f, 0.0f, 0.0f},
                            float4{sin(kAngle), 0.0f, cos(kAngle), 0.0f},
                            float4{-1.5f, -0.25f, -0.8f, 1.0f});
// Placement of the vertical window plane.
const mat4f kWindowTransform(float4{cos(kAngle), 0.0f, -sin(kAngle), 0.0},
                             float4{0.0f, 1.0f, 0.0f, 0.0f},
                             float4{sin(kAngle), 0.0f, cos(kAngle), 0.0f},
                             float4{-2.5f, 0.0f, -4.2f, 1.0f});

const ar::HdrLighting::DirectionalLightInfo kDirectionalLightInfo{
    .color = {1.0, 2.0, 3.0},
    .intensity = 1.0,
    .direction = {1.5, 2.5, 3.5},
    .cast_shadow = false};

// The floor size in meters.
const float2 kFloorExtents(0.8f, 1.0f);
const float2 kCouchExtents(0.4f, 0.6f);
const float2 kWindowExtents(1.0f, 1.0f);
// Invalid id for initializing data.
constexpr ArTrackableId kInvalidId(-1);

// Constant data of a placeholder camera texture for automated rendering tests.
// Used to create a simple multi-colored 2x2 pixel texture.
static constexpr uint32_t kRedPixel = 255;
static constexpr uint32_t kGreenPixel = 65280;
static constexpr uint32_t kBluePixel = 16711680;
static constexpr uint32_t kPinkPixel = 16711935;

static constexpr int kPlaceholderCameraTextureSize = 2;
static constexpr int kNumPlaceholderCameraTexturePixels =
    kPlaceholderCameraTextureSize * kPlaceholderCameraTextureSize;
static constexpr std::array<uint32_t, kNumPlaceholderCameraTexturePixels>
    kPlaceholderCameraTexturePixels = {kRedPixel, kGreenPixel, kBluePixel,
                                       kPinkPixel};

// A default set of of floor vertices.
std::vector<float3> GetFloorVertices() {
  return std::vector<float3>{
      float3(-kFloorExtents.x, 0.0f, -kFloorExtents.y),
      float3(-kFloorExtents.x, 0.0f, kFloorExtents.y),
      float3(kFloorExtents.x, 0.0f, kFloorExtents.y),
      float3(kFloorExtents.x, 0.0f, -kFloorExtents.y),
  };
}

std::vector<float3> GetUpperVertices() {
  return std::vector<float3>{
      float3(-kCouchExtents.x, 0.0f, -kCouchExtents.y),
      float3(-kCouchExtents.x, 0.0f, kCouchExtents.y),
      float3(kCouchExtents.x, 0.0f, kCouchExtents.y),
      float3(kCouchExtents.x, 0.0f, -kCouchExtents.y),
  };
}

// Vertical window surface.
std::vector<float3> GetWindowVertices() {
  return std::vector<float3>{
      float3(-kWindowExtents.x, -kWindowExtents.y, 0.0f),
      float3(-kWindowExtents.x, kWindowExtents.y, 0.0f),
      float3(kWindowExtents.x, kWindowExtents.y, 0.0f),
      float3(kWindowExtents.x, -kWindowExtents.y, 0.0f),
  };
}

// Simulates trackable Id generation.
ArTrackableId GenerateTrackableId() {
  static int64_t id = 0;
  return ArTrackableId(id++);
}
}  // namespace

Future<absl::Status> ArSessionNative::CanCreateForPlatform(
    const BaseView* view, const ArSessionConfig& config) {
  if (config.tracking_mode == ArSessionConfig::TrackingMode::kFace) {
    return Future<absl::Status>(absl::UnavailableError(
        "AR face tracking support not available on desktop"));
  }
  return Future<absl::Status>(absl::OkStatus());
}

Future<std::unique_ptr<ArSessionNative>> ArSessionNative::CreateForPlatform(
    BaseView* view, const ArSessionConfig& config) {
  return ArSessionNativeDesktop::CreateSession(view->GetHost()->GetEngine(),
                                               view);
}

Future<std::unique_ptr<ArSessionNative>> ArSessionNativeDesktop::CreateSession(
    filament::Engine* engine, BaseView* view) {
  auto* session = new ArSessionNativeDesktop(engine, view);
  return Future<std::unique_ptr<ArSessionNative>>(
      absl::WrapUnique<ArSessionNative>(session));
}

std::unique_ptr<HdrLighting> ArSessionNativeDesktop::GetHdrLighting() {
  auto hdr_lighting = std::make_unique<HdrLighting>();
  hdr_lighting->environment_light =
      view_->GetEnvironmentLightFactory().WrapIndirectLight(
          filament::IndirectLight::Builder().build(*engine_));

  hdr_lighting->directional_light_info.emplace(kDirectionalLightInfo);

  return hdr_lighting;
}

ArSessionNativeDesktop::ArSessionNativeDesktop(filament::Engine* engine,
                                               BaseView* view)
    : engine_(engine), view_(view) {}

void ArSessionNativeDesktop::GetCameraTextureUVs(
    const absl::Span<const float2> normalized_screen_coordinates,
    std::vector<float2>* uvs_out) const {
  // The desktop session assumes only 3 vertices for display geometry.
  const int kExpectedCoordinateCount = 3;
  

  // Create UV's for triangle.
  // (broken link)/
  uvs_out->at(0) = {0.0f, 0.0f};
  uvs_out->at(1) = {0.0f, 2.0f};
  uvs_out->at(2) = {2.0f, 0.0f};
}

std::array<float3, 2u> ArSessionNativeDesktop::GetUvFromNdcTransform() const {
  // Returns an affine identity matrix.
  return {float3{1, 0, 0}, float3{0, 1, 0}};
}

void ArSessionNativeDesktop::SetDisplayGeometry(
    window::WindowRotation /*orientation*/, int width, int height, float near,
    float far) {
  width_ = width;
  height_ = height;
  near_ = near;
  far_ = far;
}

void ArSessionNativeDesktop::Pause() {
  if (desktop_camera_controller_) {
    desktop_camera_controller_->Pause();
  }
  // Simulates planes being lost.
  for (auto& trackable : ar_trackables_) {
    if (trackable->GetType() == TrackableType::kPlane) {
      trackable->tracking_state_ = TrackingState::kStopped;
    }
  }
  UpdateAnchors();

  keyboard_event_connection_.Disconnect();
}

void ArSessionNativeDesktop::Resume() {
  EnableVirtualCameraControlOnView();

  if (desktop_camera_controller_) {
    desktop_camera_controller_->Resume();
  }

  ConfigureKeyboardInputListener();
}

TexturePtr CreateCameraTexture(BaseView* view, uint2 texture_dimensions) {
  TexturePtr texture = view->GetTextureFactory().CreateTexture(
      kPlaceholderCameraTextureSize, kPlaceholderCameraTextureSize,
      filament::Texture::InternalFormat::RGBA8);

  filament::Texture::PixelBufferDescriptor buffer(
      kPlaceholderCameraTexturePixels.data(),
      sizeof(uint32_t) * kNumPlaceholderCameraTexturePixels,
      filament::Texture::Format::RGBA, filament::Texture::Type::UBYTE);

  filament::Engine* engine = BaseView::GetSharedEngine();
  texture->GetTexture()->setImage(*engine, 0, std::move(buffer));

  return texture;
}

absl::optional<ArFrame> ArSessionNativeDesktop::Update(
    absl::Time last_submitted_timestamp) {
  UpdateUI();
  if (desktop_camera_controller_) {
    desktop_camera_controller_->Update();
    projection_matrix_ = desktop_camera_controller_->GetProjectionMatrix();
    view_matrix_ = desktop_camera_controller_->GetViewMatrix();
  }
  if (!camera_texture_) {
    camera_texture_ = CreateCameraTexture(view_, {width_, height_});
  }

  TrackableTuple updated_trackables;
  std::get<std::vector<ArPoint>>(updated_trackables) = GetUpdatedPoints();
  std::get<std::vector<ArPlane>>(updated_trackables) = GetUpdatedPlanes();

  // Updates anchors and applies changes to the frame.
  UpdateAnchors();
  std::get<std::vector<ArAnchor>>(updated_trackables) = GetAnchors();

  return ArFrame(absl::Now(), camera_texture_.get(), 0, projection_matrix_,
                 view_matrix_, std::move(updated_trackables));
}

uint4 ArSessionNativeDesktop::GetDebugSessionId() { return uint4(1, 2, 3, 4); }

std::vector<ArHitResult> ArSessionNativeDesktop::HitTest(
    float2 screen_pos, absl::optional<float> guessed_distance,
    TrackableTuple* out_generated_trackables) {
  // Instant hit test always generates a new point.
  if (!find_instant_placement_point_) {
    return {};
  }

  // Construct a ray from the screen_pos;
  Ray ray =
      view_->GetCameraManager().GetCamera()->WorldRayFromPixelPoint(screen_pos);

  // Simply takes a point at the guessed distance along the ray.
  float3 world_collision_point =
      ray.origin +
      ray.direction * (guessed_distance ? guessed_distance.value() : 2);

  mat4f transform = kWorldFloorTransform;
  transform[3].xyz = world_collision_point;

  find_instant_placement_point_ = false;
  auto point =
      std::make_unique<ArPoint>(GenerateTrackableId(), TrackingState::kTracking,
                                ArPoint::TrackingMethod::kRealDepth, transform);

  std::vector<ArHitResult> hit_results;
  hit_results.emplace_back(
      point->GetTransform().toQuaternion(), world_collision_point,
      length(world_collision_point - ray.origin), point->GetId());

  std::get<std::vector<ArPoint>>(*out_generated_trackables).push_back(*point);
  // Adds the point to the main trackables list.
  ar_trackables_.push_back(std::move(point));
  return hit_results;
}

std::vector<ArHitResult> ArSessionNativeDesktop::HitTestRay(
    const Ray& ray, TrackableTuple* out_generated_trackables) {
  // Generates a plane in local space.
  const mat4f kIdentity;

  std::vector<ArHitResult> results;
  for (auto& trackable : ar_trackables_) {
    if (trackable->GetType() != TrackableType::kPlane ||
        trackable->GetTrackingState() != TrackingState::kTracking) {
      continue;
    }
    auto* ar_plane = static_cast<ArPlane*>(trackable.get());

    // Transforms the ray into plane space.
    Ray local_ray = ray.GetTransformed(inverse(ar_plane->GetTransform()));

    // Creates min/max bounds from plane extens
    float3 extents{ar_plane->GetExtents().x, 0.0f, ar_plane->GetExtents().y};
    float3 local_min = kIdentity[3].xyz - extents;
    float3 local_max = kIdentity[3].xyz + extents;

    // Calculate collision plane in local space, local plane is assumed to be
    // at the origin.
    float3 local_normal = kIdentity[1].xyz;
    float distance = dot(local_normal, kIdentity[3].xyz);
    Plane plane(local_normal, distance);

    float3 local_collision_point(0.0f, 0.0f, 0.0f);
    if (collision::Result::kDoesNotIntersect ==
        collision::PlaneIntersectsRay(plane, local_min, local_max, local_ray,
                                      &local_collision_point)) {
      continue;
    }

    // Converts collision point back into world space.
    float3 world_collision_point =
        (ar_plane->GetTransform() * local_collision_point).xyz;

    results.emplace_back(
        ar_plane->GetTransform().toQuaternion(), world_collision_point,
        length(world_collision_point - ray.origin), ar_plane->GetId());
  }

  return results;
}

absl::StatusOr<ArAnchor> ArSessionNativeDesktop::CreateAnchor(
    float3 position, quatf rotation, absl::optional<ArTrackableId> id) {
  mat4f anchor_transform(rotation);
  anchor_transform[3].xyz = position;

  if (!id.has_value()) {
    ar_anchors_.emplace_back(std::make_unique<ArAnchor>(
        GenerateTrackableId(), TrackingState::kTracking, anchor_transform));
    // Anchor is not tied to a plane.
    anchor_to_plane_.insert(
        std::pair<imp::ar::ArTrackableId, imp::ar::AnchorInfo>(
            ar_anchors_.back()->GetId(), AnchorInfo(kInvalidId)));
  } else {
    auto found = absl::c_find_if(
        ar_trackables_, [&id](const std::unique_ptr<ArTrackable>& trackable) {
          return trackable->GetId() == id;
        });
    if (found == ar_trackables_.end()) {
      return absl::InternalError(
          "The given associated ArTrackableId was not found!");
    } else {
      ar_anchors_.emplace_back(std::make_unique<ArAnchor>(
          GenerateTrackableId(), TrackingState::kTracking, anchor_transform));
      // Ties the anchor to the plane.
      anchor_to_plane_.insert(std::pair<imp::ar::ArTrackableId,
                                        imp::ar::AnchorInfo>(
          ar_anchors_.back()->GetId(),
          // Stores plane-to-anchor transformation and anchor-to-plane lookup.
          AnchorInfo(inverse(found->get()->GetTransform()) * anchor_transform,
                     found->get()->GetId())));
    }
  }

  return *ar_anchors_.back();
}

void ArSessionNativeDesktop::DestroyAnchor(ArAnchor anchor) {
  for (auto iter = ar_anchors_.begin(); iter != ar_anchors_.end(); ++iter) {
    if ((*iter)->GetId() == anchor.GetId()) {
      anchor_to_plane_.erase((*iter)->GetId());
      ar_anchors_.erase(iter);
      return;
    }
  }
}

std::vector<ArPoint> ArSessionNativeDesktop::GetPoints() {
  std::vector<ArPoint> result;
  for (auto& trackable : ar_trackables_) {
    if (trackable->GetType() == TrackableType::kPoint) {
      result.push_back(*static_cast<ArPoint*>(trackable.get()));
    }
  }
  return result;
}

std::vector<ArPlane> ArSessionNativeDesktop::GetPlanes() {
  std::vector<ArPlane> result;
  for (auto& trackable : ar_trackables_) {
    if (trackable->GetType() == TrackableType::kPlane) {
      result.push_back(*static_cast<ArPlane*>(trackable.get()));
    }
  }
  return result;
}

std::vector<ArAnchor> ArSessionNativeDesktop::GetAnchors() {
  std::vector<ArAnchor> results;
  results.reserve(ar_anchors_.size());
  for (auto& anchor : ar_anchors_) {
    results.push_back(*anchor);
  }
  return results;
}

std::vector<ArPoint> ArSessionNativeDesktop::GetUpdatedPoints() {
  return GetPoints();
}

std::vector<ArPlane> ArSessionNativeDesktop::GetUpdatedPlanes() {
  return GetPlanes();
}

void ArSessionNativeDesktop::EnableVirtualCameraControlOnView() {
  assert(Executor::CurrentExecutor() == Executor::ForegroundExecutor());
  if (!desktop_camera_controller_) {
    desktop_camera_controller_ =
        std::make_unique<DesktopCameraController>(view_);
    desktop_camera_controller_->Initialize(true);
    desktop_camera_controller_->Resume();
  }
}

void ArSessionNativeDesktop::UpdateAnchors() {
  // Updates the anchors.
  for (auto pair : anchor_to_plane_) {
    ArTrackableId& anchor_id = pair.first;
    AnchorInfo& anchor_info = pair.second;
    // Checks if the anchor is attached to a plane and calculates world
    // transform from plane relative transformation.
    if (anchor_info.attach_id != kInvalidId) {
      for (auto& trackable : ar_trackables_) {
        if (trackable->GetId() == anchor_info.attach_id) {
          for (auto& anchor : ar_anchors_) {
            if (anchor->GetId() == anchor_id) {
              anchor->transform_ =
                  trackable->GetTransform() * anchor_info.world_transform;
              // Updates tracking state to match the trackable that this anchor
              // depends on.
              anchor->tracking_state_ = trackable->tracking_state_;
              break;
            }
          }
          break;
        }
      }
    }
  }
}

std::vector<float3> ArSessionNativeDesktop::GetSphericalHarmonicsLighting() {
  const std::vector<float3> kSphericalHarmonics{
      float3{1, 1, 1}, float3{1, 1, 1}, float3{1, 1, 1},
      float3{1, 1, 1}, float3{1, 1, 1}, float3{1, 1, 1},
      float3{1, 1, 1}, float3{1, 1, 1}, float3{1, 1, 1},
  };
  return kSphericalHarmonics;
}

void ArSessionNativeDesktop::ConfigureKeyboardInputListener() {
  // Connect to desktop input event.
  keyboard_event_connection_ = view_->GetDispatcher().Connect(
      [this](const DesktopCombinedInputEvent& input) mutable {
        if (input.keyboard_events.empty()) return;
        const imp::KeyboardEvent& keyboard_event = input.keyboard_events.back();

        if (keyboard_event.type == KeyboardEventType::kOnUp) {
          switch (keyboard_event.key.code) {
            case VirtualKeyCode::VK_p: {
              // Adds virtual plane when 'P' key is pressed.
              bool all_planes_lost = true;
              for (auto& plane : ar_trackables_) {
                if (plane->GetType() == TrackableType::kPlane &&
                    plane->GetTrackingState() == TrackingState::kTracking) {
                  all_planes_lost = false;
                  break;
                }
              }
              if (all_planes_lost) {
                // Re-adds a plane in the tracking state.
                ar_trackables_.push_back(std::make_unique<ArPlane>(
                    GenerateTrackableId(), TrackingState::kTracking,
                    kWorldFloorTransform, kFloorExtents,
                    ArPlane::PlaneType::kHorizontalUpFacing,
                    GetFloorVertices()));
                ar_trackables_.push_back(std::make_unique<ArPlane>(
                    GenerateTrackableId(), TrackingState::kTracking,
                    kCouchTransform, kCouchExtents,
                    ArPlane::PlaneType::kHorizontalUpFacing,
                    GetUpperVertices()));
                ar_trackables_.push_back(std::make_unique<ArPlane>(
                    GenerateTrackableId(), TrackingState::kTracking,
                    kWindowTransform, kWindowExtents,
                    ArPlane::PlaneType::kVertical, GetWindowVertices()));
              }
            } break;
            case VirtualKeyCode::VK_i: {
              find_instant_placement_point_ = true;
            } break;
            // Sets all planes to a stopped state.
            case VirtualKeyCode::VK_l: {
              // Simulates a plane being lost.
              for (auto& plane : ar_trackables_) {
                if (plane->GetType() == TrackableType::kPlane) {
                  plane->tracking_state_ = TrackingState::kStopped;
                }
              }
            } break;
            default:
              break;
          }
        }
      },
      view_);
}

void ArSessionNativeDesktop::UpdateUI() {
  view_->GetHost()->QueueImGuiCommandBlock([]() {
    bool show_overlay = true;
    ImGuiIO& io = ImGui::GetIO();
    // Draws ar mode controls.
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, io.DisplaySize.y));
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always, ImVec2(0, 0));
    ImGui::SetNextWindowBgAlpha(0.0f);
    if (ImGui::Begin("Usage Info", &show_overlay,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                         ImGuiWindowFlags_NoResize |
                         ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoMouseInputs |
                         ImGuiWindowFlags_NoFocusOnAppearing)) {
      ImGui::Text("Input Controls");
      ImGui::Text("LeftMouse: Move Camera");
      ImGui::Text("Detect plane: 'P'");
      ImGui::Text("Lose plane: 'L'");
      ImGui::Text("Discover Instant Point: 'I'");
      ImGui::Text("Ctrl+LeftMouse: 'Drag'");
      ImGui::Text("Ctrl+Shift+Wheel: 'Scale'");
      ImGui::Text("Double-Tap: 'T'");
      ImGui::Text("Move: 'WASD'");
    }
    ImGui::End();
  });
}

bool ArSessionNativeDesktop::IsReadyToRender() const {
  if (desktop_camera_controller_) {
    return desktop_camera_controller_->IsReadyToRender();
  }
  return true;
}

}  // namespace ar
}  // namespace imp
