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

#include "core/view/framework/camera/camera_component.h"

#include <optional>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Frustum.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/plane.h"
#include "core/collision/ray.h"
#include "core/config.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/view/base_view.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/camera/camera_state.proto.imp.h"
#include "core/view/view_events.h"

namespace imp {

namespace {

using ProjectionType = CameraState::ProjectionType;

// Represents half the height of a viewbox that is 10x10x30.
// This viewbox represents the bounds of the orthographic projection.
constexpr float kDefaultOrthographicScale = 5;
// Magic constants courtesy of sceneform/rendering/Renderer.java:72
constexpr float kVerticalFOV = 45.0f;
constexpr float kClipNear = 0.01f;
constexpr float kClipFar = 30.0f;
constexpr float kCameraAperture = 4.0f;
constexpr float kCameraShutterSpeed = 1.0f / 30.0f;
constexpr float kCameraIso = 320;
}  // namespace

void CameraComponent::UpdateProjection(float2 scale) {
  float fov = state_.fov.value_or(kVerticalFOV);

  switch (GetProjectionType()) {
    case (ProjectionType::PERSPECTIVE):
      if (update_projection_override_.has_value()) {
        fov = (*update_projection_override_)(fov, GetAspectRatio(scale));
      }
      camera_->setProjection(fov, GetAspectRatio(scale), GetNearClip(),
                             GetFarClip());
      break;
    case (ProjectionType::ORTHOGRAPHIC):
      camera_->setCustomProjection(GetOrthographicProjection(), GetNearClip(),
                                   GetFarClip());
      break;
    case (ProjectionType::CUSTOM):
      camera_->setCustomProjection(GetCustomProjection(), GetNearClip(),
                                   GetFarClip());
      break;
  }
}

void CameraComponent::SetCameraProjectionUpdateOverride(
    Invocable<float(float, float)> update_projection_override) {
  update_projection_override_ = std::move(update_projection_override);
}

void CameraComponent::SetCameraTransformOverride(
    std::optional<mat4f> override) {
  if (!is_owned_) {
    IMP_LOG(imp::FATAL) << "Attempt to set transform override on external camera.";
  }

  filament::TransformManager& transform_manager =
      GetView().GetHost()->GetEngine()->getTransformManager();

  if (camera_->getEntity() == GetEntity()) {
    // If the camera is currently sharing the entity of the Impress camera node,
    // create a new entity just for the camera so that it can have a different
    // transform from the node.

    // Store the projection matrix so it can be restored.
    mat4 projection_matrix = GetProjectionMatrixPrecise();
    float near = GetNearClip();
    float far = GetFarClip();
    // Destroy the old camera component as it will no longer be used.
    GetView().GetHost()->GetEngine()->destroyCameraComponent(camera_entity_);
    // Create a new, separate entity for the camera.
    camera_entity_ =
        GetView().GetHost()->GetEngine()->getEntityManager().create();
    transform_manager.create(camera_entity_);

    bool is_active_camera =
        &GetView().GetHost()->GetView()->getCamera() == camera_;

    // Create a new camera.
    camera_ = GetView().GetHost()->GetEngine()->createCamera(camera_entity_);
    camera_->setExposure(kCameraAperture, kCameraShutterSpeed, kCameraIso);
    camera_->setCustomProjection(projection_matrix, near, far);
    if (is_active_camera) {
      // Update the View's camera.
      GetView().GetHost()->GetView()->setCamera(camera_);
    }
  }

  filament::TransformManager::Instance camera_transform =
      transform_manager.getInstance(camera_entity_);
  // If override is not nullopt, set the transform of the camera entity and
  // unparent. If it is nullopt, clear transform and parent.
  if (override) {
    transform_manager.setParent(camera_transform,
                                filament::TransformManager::Instance());
    GetNode()->SetLocalTrs(*override);
  } else {
    GetNode()->SetLocalTrs(kIdentityMat4f);
    transform_manager.setParent(camera_transform,
                                transform_manager.getInstance(GetEntity()));
  }
}

void CameraComponent::Setup() {
  is_owned_ = true;
  camera_entity_ = GetEntity();
  camera_ = GetView().GetHost()->GetEngine()->createCamera(camera_entity_);
  camera_->setExposure(kCameraAperture, kCameraShutterSpeed, kCameraIso);

  if (state_.start_as_current_camera.value_or(false)) {
    GetView().GetCameraManager().SetCamera(GetHandle(this));
  }

  UpdateProjection();

  GetView().GetDispatcher().Connect(
      [this](const ViewSizeChangedEvent& view_size_changed_event) mutable {
        if (IsAspectRatioLocked()) return;

        const float2 screen_size = this->GetView().GetSize();
        const float aspect_ratio_new = screen_size.x / screen_size.y;

        const float aspect_ratio_current = this->CalculateAspectRatioFromMatrix(
            this->GetProjectionMatrixPrecise());
        if (imp::AlmostEqual(aspect_ratio_new, aspect_ratio_current)) return;

        this->UpdateProjection(aspect_ratio_new);
      },
      this);
}

void CameraComponent::Setup(filament::Camera* camera) {
  is_owned_ = false;
  camera_ = camera;

  state_.near = camera->getNear();
  state_.far = camera->getCullingFar();

  imp::mat4 projection_matrix = camera->getProjectionMatrix();
  state_.projection_type = CalculateProjectionTypeFromMatrix(projection_matrix);

  if (GetProjectionType() == ProjectionType::PERSPECTIVE) {
    state_.fov =
        camera->getFieldOfViewInDegrees(filament::Camera::Fov::VERTICAL);
  } else if (GetProjectionType() == ProjectionType::ORTHOGRAPHIC) {
    absl::StatusOr<float> calculated_scale_result =
        CalculateOrthographicScaleFromMatrix(projection_matrix);
    if (calculated_scale_result.ok()) {
      state_.orthographic_scale = *calculated_scale_result;
    } else {
      // The failed absl::Status means the matrix was not a valid
      // orthographic matrix. Set the projection type to custom.
      state_.projection_type = ProjectionType::CUSTOM;
    }
  }
  if (GetProjectionType() == ProjectionType::CUSTOM) {
    state_.custom_projection_matrix = projection_matrix;
  }
}

void CameraComponent::Cleanup() {
  if (is_owned_) {
    GetView().GetHost()->GetEngine()->destroyCameraComponent(camera_entity_);
    // If the camera has a separate entity, destroy that too.
    if (GetEntity() != camera_entity_) {
      GetView().GetHost()->GetEngine()->getEntityManager().destroy(
          camera_entity_);
    }
    camera_ = nullptr;
    is_owned_ = false;
  }
}

void CameraComponent::OnIsfStateChanged() {
  if (GetNearClip() >= GetFarClip()) {
    IMP_LOG(imp::WARNING) << "CameraComponent on Node " << GetNode()->GetName()
                 << " has invalid near/far clip. Near: " << GetNearClip()
                 << ", Far: " << GetFarClip()
                 << ". Near clip should always be smaller than far clip";
  }
  if (GetAspectRatio() <= 0.0f) {
    IMP_LOG(imp::WARNING) << "CameraComponent on Node " << GetNode()->GetName()
                 << " has invalid aspect ratio: " << GetAspectRatio();
  }
  UpdateProjection();
  // TODO Check state_.start_as_current_camera and possibly disable
  // other cameras
}

filament::Camera* CameraComponent::GetCamera() const { return camera_; }

void CameraComponent::HandleTransitionScale(float2 scale) {
  UpdateProjection(1.0f / scale);
}

void CameraComponent::SetProjection(float vertical_fov_in_degrees,
                                    float near_clip, float far_clip) {
  state_.projection_type = ProjectionType::PERSPECTIVE;
  state_.fov = vertical_fov_in_degrees;
  state_.near = near_clip;
  state_.far = far_clip;
  UpdateProjection();
}

ProjectionType CameraComponent::GetProjectionType() const {
  return state_.projection_type;
}

void CameraComponent::SetPerspectiveProjection(float vertical_fov_in_degrees,
                                               float near_clip,
                                               float far_clip) {
  SetProjection(vertical_fov_in_degrees, near_clip, far_clip);
}

void CameraComponent::SetOrthographicProjection(float scale) {
  
  state_.projection_type = CameraState::ORTHOGRAPHIC;
  state_.orthographic_scale = scale;
  UpdateProjection();
}

void CameraComponent::SetCustomProjection(const imp::mat4f& projection_matrix) {
  SetProjectionMatrix(projection_matrix);
}

void CameraComponent::SetCustomProjection(const imp::mat4& projection_matrix) {
  SetProjectionMatrix(projection_matrix);
}

void CameraComponent::SetVerticalFovInDegrees(float vertical_fov_in_degrees) {
  state_.fov = vertical_fov_in_degrees;
  UpdateProjection();
}

float CameraComponent::GetVerticalFovInDegrees() const {
  return camera_->getFieldOfViewInDegrees(filament::Camera::Fov::VERTICAL);
}

float CameraComponent::GetHorizontalFovInDegrees() const {
  return camera_->getFieldOfViewInDegrees(filament::Camera::Fov::HORIZONTAL);
}

float CameraComponent::GetOrthographicScale() const {
  return state_.orthographic_scale.value_or(kDefaultOrthographicScale);
}

void CameraComponent::SetNearClip(float near_clip) {
  
  state_.near = near_clip;
  UpdateProjection();
}

float CameraComponent::GetNearClip() const {
  return state_.near.value_or(kClipNear);
}

void CameraComponent::SetFarClip(float far_clip) {
  
  state_.far = far_clip;
  UpdateProjection();
}

float CameraComponent::GetFarClip() const {
  return state_.far.value_or(kClipFar);
}

void CameraComponent::SetNearAndFarClip(float near_clip, float far_clip) {
  
  state_.near = near_clip;
  state_.far = far_clip;
  UpdateProjection();
}

void CameraComponent::LockAspectRatio(float aspect_ratio) {
  
  state_.locked_aspect_ratio = aspect_ratio;

  // Update the camera to use the newly assigned aspect ratio.
  UpdateProjection();
}

void CameraComponent::LockAspectRatio() {
  state_.locked_aspect_ratio = GetAspectRatio();
}

void CameraComponent::UnlockAspectRatio() {
  state_.locked_aspect_ratio.reset();

  // Update the camera to use the viewport's aspect ratio.
  UpdateProjection();
}

void CameraComponent::SetProjectionMatrix(const imp::mat4& projection_matrix) {
  state_.projection_type = ProjectionType::CUSTOM;
  state_.custom_projection_matrix = projection_matrix;
  UpdateProjection();
}

void CameraComponent::SetProjectionMatrix(const imp::mat4f& projection_matrix) {
  SetProjectionMatrix(static_cast<mat4>(projection_matrix));
}

mat4f CameraComponent::GetProjectionMatrix() const {
  return mat4f(GetProjectionMatrixPrecise());
}

mat4 CameraComponent::GetProjectionMatrixPrecise() const {
  return camera_->getProjectionMatrix();
}

#if IMP_RUNTIME(DEV)
bool CameraComponent::IsActiveCameraOnStart() const {
  return state_.start_as_current_camera.value_or(false);
}

void CameraComponent::SetAsActiveCameraOnStart(bool active_camera_on_start) {
  state_.start_as_current_camera = active_camera_on_start;
}
#endif

mat4 CameraComponent::WorldFromClip() const {
  return camera_->getModelMatrix() *
         filament::Camera::inverseProjection(GetProjectionMatrixPrecise());
}

mat4 CameraComponent::ClipFromWorld() const {
  return GetProjectionMatrixPrecise() * camera_->getViewMatrix();
}

Ray CameraComponent::WorldRayFromClipPoint(const float2& clip_point) const {
  return Ray(WorldRayFromClipPointPrecise(clip_point));
}

DoubleRay CameraComponent::WorldRayFromClipPointPrecise(
    const double2& clip_point) const {
  mat4 world_from_clip = WorldFromClip();
  // In order to support orthogonal projections, the origin of the ray is
  // computed by using the `clip_point` on the near plane (z = -1).
  double4 origin = world_from_clip * double4(clip_point, -1, 1);
  origin /= origin.w;
  double4 world_point = world_from_clip * double4(clip_point, 0, 1);
  world_point /= world_point.w;
  double3 direction = normalize(world_point.xyz - origin.xyz);
  DoubleRay ray(origin.xyz, direction);
  // Extend the ray origin back to where the camera is such that for perspective
  // projections the ray originates from the camera.
  double3 camera_plane_point;
  if (collision::PlaneIntersectsRay(
          DoublePlane(camera_->getForwardVector(), /*distance=*/0),
          DoubleRay(ray.origin - camera_->getPosition(), -ray.direction),
          &camera_plane_point) == collision::Result::kDoesIntersect) {
    ray.origin = camera_plane_point + camera_->getPosition();
  }
  return ray;
}

Ray CameraComponent::WorldRayFromUVPoint(const float2& uv) const {
  return Ray(WorldRayFromUVPointPrecise(uv));
}

DoubleRay CameraComponent::WorldRayFromUVPointPrecise(const float2& uv) const {
  return WorldRayFromClipPointPrecise(ClipFromUVPoint(uv));
}

Ray CameraComponent::WorldRayFromPixelPoint(const float2& pixel_point) const {
  return Ray(WorldRayFromPixelPointPrecise(pixel_point));
}

DoubleRay CameraComponent::WorldRayFromPixelPointPrecise(
    const float2& pixel_point) const {
  return WorldRayFromClipPointPrecise(ClipFromPixelPoint(pixel_point));
}

float3 CameraComponent::WorldFromClipPoint(const float3& clip_point) const {
  return WorldFromClipPointPrecise(clip_point);
}

double3 CameraComponent::WorldFromClipPointPrecise(
    const double3& clip_point) const {
  double4 world_point = WorldFromClip() * clip_point;
  return world_point.xyz / world_point.w;
}

std::optional<float3> CameraComponent::ClipFromWorldPoint(
    const float3& world_point) const {
  return ClipFromWorldPointPrecise(world_point);
}

std::optional<double3> CameraComponent::ClipFromWorldPointPrecise(
    const double3& world_point) const {
  // View is inverse of model.
  double4 clip_point = ClipFromWorld() * world_point;
  if (clip_point.w <= 0) {  // Point is behind the camera.
    return {};
  }
  return {clip_point.xyz / clip_point.w};
}

std::optional<float2> CameraComponent::UVFromWorldPoint(
    const float3& world_point) const {
  return UVFromWorldPointPrecise(world_point);
}

std::optional<float2> CameraComponent::UVFromWorldPointPrecise(
    const double3& world_point) const {
  if (auto clip = ClipFromWorldPointPrecise(world_point)) {
    return UVFromClipPoint(*clip);
  }
  return {};
}

std::optional<float2> CameraComponent::PixelFromWorldPoint(
    const float3& world_point) const {
  return PixelFromWorldPointPrecise(world_point);
}

std::optional<float2> CameraComponent::PixelFromWorldPointPrecise(
    const double3& world_point) const {
  if (auto uv = UVFromWorldPointPrecise(world_point)) {
    return PixelFromUVPoint(*uv);
  }
  return {};
}

float2 CameraComponent::ClipFromPixelPoint(const float2& pixel_point) const {
  return ClipFromUVPoint(UVFromPixelPoint(pixel_point));
}

float2 CameraComponent::PixelFromClipPoint(const float3& clip_point) const {
  return PixelFromUVPoint(UVFromClipPoint(clip_point));
}

float2 CameraComponent::UVFromPixelPoint(const float2& pixel_point) const {
  uint2 size = GetView().GetSize();
  return float2(pixel_point.x / size.x, pixel_point.y / size.y);
}

float2 CameraComponent::PixelFromUVPoint(const float2& uv_point) const {
  uint2 size = GetView().GetSize();
  return float2(uv_point.x * size.x, uv_point.y * size.y);
}

bool CameraComponent::IntersectsFrustum(const Box& world_bounds) const {
  return camera_->getFrustum().intersects(world_bounds);
}

float2 CameraComponent::ClipFromUVPoint(const float2& uv) {
  // Convert to [-1,1].  Also flip y, so that +y is up.
  return float2(2.0f * (uv.x - 0.5f), -2.0f * (uv.y - 0.5f));
}

float2 CameraComponent::UVFromClipPoint(const float3& clip_point) {
  // Convert from [-1,1] to [0,1].
  // Also flip y, so that +y is down (0,0 is top left pixel)
  return float2(0.5f + (clip_point.x * 0.5f), 0.5f - (clip_point.y * 0.5f));
}

bool CameraComponent::IsAspectRatioLocked() const {
  return state_.locked_aspect_ratio.has_value();
}

float CameraComponent::GetAspectRatio(float2 scale) const {
  if (IsAspectRatioLocked()) {
    return state_.locked_aspect_ratio.value();
  } else {
    float2 size = GetView().GetSize() * scale;
    float width = size.x;
    float height = size.y;
    
    
    return width / height;
  }
}

imp::mat4 CameraComponent::GetOrthographicProjection() const {
  float scale = GetOrthographicScale();
  float2 scaled_aspect_ratio = {GetAspectRatio() * scale, scale};
  float2 halved_dimension = scaled_aspect_ratio * .5f;
  return imp::mat4::ortho(-halved_dimension.x, halved_dimension.x,
                          -halved_dimension.y, halved_dimension.y,
                          GetNearClip(), GetFarClip());
}

imp::mat4 CameraComponent::GetCustomProjection() const {
  if (state_.custom_projection_matrix.has_value()) {
    return state_.custom_projection_matrix.value();
  }
  IMP_LOG(imp::WARNING) << "No custom matrix was given to the camera. "
                  "Returning the camera's original projection matrix.";
  return camera_->getProjectionMatrix();
}

ProjectionType CameraComponent::CalculateProjectionTypeFromMatrix(
    imp::mat4 projection_matrix) const {
  if (imp::AlmostEqual(projection_matrix[2][3], 0.0) &&
      imp::AlmostEqual(projection_matrix[3][3], 1.0) &&
      projection_matrix[1][1] > 0) {
    return ProjectionType::ORTHOGRAPHIC;
  } else if (imp::AlmostEqual(projection_matrix[2][3], -1.0)) {
    return ProjectionType::PERSPECTIVE;
  } else {
    return ProjectionType::CUSTOM;
  }
}

absl::StatusOr<float> CameraComponent::CalculateOrthographicScaleFromMatrix(
    imp::mat4 projection_matrix) const {
  // A standard orthographic matrix looks like this:
  //
  //     2                        r + l
  //   -----      0        0    - -----
  //   r - l                      r - l
  //
  //              2               t + b
  //     0      -----      0    - -----
  //            t - b             t - b
  //
  //                       2      f + n
  //     0        0    - -----  - -----
  //                     f - n    f - n
  //
  //
  //     0        0        0        1
  //
  //
  // We want to get the scale from the height of the viewing volume.
  // We can get this from the difference between top (t) and bottom (b)
  // The value at [1][1], 2 / t - b, can give us this number.
  float inverted_scale = projection_matrix[1][1];
  if (inverted_scale > 0) {
    return 2.0f / inverted_scale;
  }
  // Catch cases where the value at [1][1] was not valid.
  // Note: this should pretty much never happen; in the case that it does,
  // something is probably wrong with CalculateProjectionTypeFromMatrix.
  return absl::InvalidArgumentError("Given matrix is not orthographic.");
}

float CameraComponent::CalculateAspectRatioFromMatrix(
    imp::mat4 projection_matrix) const {
  return projection_matrix[0][0] / projection_matrix[1][1];
}

}  // namespace imp
