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

#ifndef THIRD_PARTY_IMPRESS_CORE_CAMERA_CAMERA_COMPONENT_H_
#define THIRD_PARTY_IMPRESS_CORE_CAMERA_CAMERA_COMPONENT_H_

#include <optional>

#include "filament/filament/include/filament/Camera.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/camera/camera_state.proto.imp.h"
#include "core/collision/ray.h"
#include "core/config.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"

namespace imp {

class CameraComponent : public Component {
 public:
  // Minimum and default value for the near clipping plane.
  static constexpr float kClipNearMinimum = 0.01f;
  // Closest the far clipping plane can be to the near clipping plane.
  static constexpr float kClipNearToFarTolerance = 1.0f;

  void Setup();
  void Cleanup();
  void OnIsfStateChanged();

  CameraState::ProjectionType GetProjectionType() const;

  // Sets the camera's projection type to Perspective, and then uses
  // the values passed in to setup the view.
  void SetPerspectiveProjection(float vertical_fov_in_degrees, float near_clip,
                                float far_clip);

  // Sets the camera's projection type to Orthographic, and then uses
  // the aspect ratio along with the given scale to setup the view.
  // The scale is traditionally half the height of a viewbox,
  // the viewbox being a bounding box in the scene.
  // Whatever is within the viewbox is what the orthographic
  // camera will render.
  void SetOrthographicProjection(float scale);

  // Sets the camera's projection type to Custom, and then sets
  // the camera's view via the given projection matrix.
  void SetCustomProjection(const imp::mat4f& projection_matrix);

  // Sets the camera's projection type to Custom, and then sets
  // the camera's view via the given projection matrix.
  void SetCustomProjection(const imp::mat4& projection_matrix);

  // Sets the projection matrix directly with a float 4x4. This overrides
  // all other fields related to the projection matrix, and automatic projection
  // updates are disabled. To resume default behavior, call SetProjection() with
  // the intended fov and clip values.
  // It will automatically set the projection type to Custom.
  void SetProjectionMatrix(const imp::mat4f& projection_matrix);

  // Sets the projection matrix directly with a double 4x4. This overrides
  // all other fields related to the projection matrix, and automatic projection
  // updates are disabled. To resume default behavior, call SetProjection() with
  // the intended fov and clip values.
  // It will automatically set the projection type to Custom.
  void SetProjectionMatrix(const imp::mat4& projection_matrix);

  // Sets the projection matrix using all of the properties.
  // It will automatically set the projection type to Perspective.
  void SetProjection(float vertical_fov_in_degrees, float near_clip,
                     float far_clip);

  // Returns the projection matrix.
  mat4f GetProjectionMatrix() const;
  // Returns the projection matrix in double precision.
  mat4 GetProjectionMatrixPrecise() const;

#if IMP_RUNTIME(DEV)
  bool IsActiveCameraOnStart() const;
  void SetAsActiveCameraOnStart(bool active_camera_on_start);
#endif

  // The following setters all automatically recalculate the projection matrix
  // using the existing other values.
  void SetVerticalFovInDegrees(float vertical_fov_in_degrees);
  float GetVerticalFovInDegrees() const;
  float GetHorizontalFovInDegrees() const;
  float GetOrthographicScale() const;
  void SetNearClip(float near_clip);
  float GetNearClip() const;
  void SetFarClip(float far_clip);
  float GetFarClip() const;
  void SetNearAndFarClip(float near_clip, float far_clip);

  // Locks the camera's aspect ratio, and if a valid value is passed in,
  // set the aspect ratio to the given value. Aspect ratio must be > 0.
  void LockAspectRatio(float aspect_ratio);
  // Locks the camera's aspect ratio to the last aspect ratio
  // the camera was using.
  void LockAspectRatio();
  // Unlocks the camera, allowing its aspect ratio to dynamically
  // match the dimensions of the viewport.
  void UnlockAspectRatio();

  // Overrides the transform of the camera instead of using the transform of the
  // Node to which this component is attached.
  // If nullopt is passed, any override is unset.
  void SetCameraTransformOverride(absl::optional<mat4f> override);

  // Overrides the default call to update the projection whenever one is needed.
  // Once set, anything that invokes an update to the CameraComponent's
  // projection will trigger this overriding function; Hence, take note to not
  // make any mutable calls to CameraComponent within the function, or it will
  // trigger an infinite loop.
  // The function is called with the current vertical FOV and the aspect ratio
  // and should return an updated vertical FOV to set the new projection. The
  // default projection update call uses the default vertical FOV or the value
  // defined in CameraComponentState to set the camera projection.
  void SetCameraProjectionUpdateOverride(
      Invocable<float(float, float)> update_projection_override);

  // Project a point from clip coordinate space into a ray in world space.
  // |clip_point| should have values in the range ([-1,1], [-1,1]).
  //
  // Internally, this calls WorldRayFromClipPointPrecise() and truncate the
  // double result to float before returning.
  Ray WorldRayFromClipPoint(const float2& clip_point) const;

  // Project a point from clip coordinate space into a ray in world space.
  // |clip_point| should have values in the range ([-1,1], [-1,1]).
  DoubleRay WorldRayFromClipPointPrecise(const double2& clip_point) const;

  // Project a point from camera texture space into a ray in world space.
  // |uv_point| should have values in the range [0,1].
  //
  // Internally, this calls WorldRayFromUVPointPrecise() and truncate the
  // double result to float before returning.
  Ray WorldRayFromUVPoint(const float2& uv_point) const;
  // Project a point from camera texture space into a ray in world space.
  // |uv_point| should have values in the range [0,1].
  DoubleRay WorldRayFromUVPointPrecise(const float2& uv_point) const;

  // Project a point from pixel space into a ray in world space. |pixel_point|
  // should have values in the viewport.
  //
  // Internally, this calls WorldRayFromPixelPointPrecise() and truncate the
  // double result to float before returning.
  Ray WorldRayFromPixelPoint(const float2& pixel_point) const;
  // Project a point from pixel space into a ray in world space. |pixel_point|
  // should have values in the viewport.
  DoubleRay WorldRayFromPixelPointPrecise(const float2& pixel_point) const;

  // Convert a point in clip space to world space. |clip_point|'s values should
  // be in the range ([-1,1], [-1,1], [0,1]) for a result in the view frustrum.
  //
  // Internally, this calls WorldFromClipPointPrecise() and truncate the double
  // result to float before returning.
  float3 WorldFromClipPoint(const float3& clip_point) const;
  // Convert a point in clip space to world space. |clip_point|'s values should
  // be in the range ([-1,1], [-1,1], [0,1]) for a result in the view frustrum.
  double3 WorldFromClipPointPrecise(const double3& clip_point) const;

  // Convert a point in world space to clip space.  If |world_point| is in the
  // view frustrum, the result will be in the range ([-1,1], [-1,1], [0,1]).
  // Points in front of the near plane / behind the camera return a nullopt.
  //
  // Internally, this calls ClipFromWorldPointPrecise() and truncate the double
  // result to float before returning.
  std::optional<float3> ClipFromWorldPoint(const float3& world_point) const;
  // Convert a point in world space to clip space.  If |world_point| is in the
  // view frustrum, the result will be in the range ([-1,1], [-1,1], [0,1]).
  // Points in front of the near plane / behind the camera return a nullopt.
  std::optional<double3> ClipFromWorldPointPrecise(
      const double3& world_point) const;

  // Convert a point in world space to camera texture space.  If |world_point|
  // is in the view frustrum, the result's values will be in the range [0,1].
  // Points in front of the near plane / behind the camera return a nullopt.
  //
  // Internally, this calls UVFromWorldPointPrecise() and truncate the double
  // result to float before returning.
  std::optional<float2> UVFromWorldPoint(const float3& world_point) const;
  // Convert a point in world space to camera texture space.  If |world_point|
  // is in the view frustrum, the result's values will be in the range [0,1].
  // Points in front of the near plane / behind the camera return a nullopt.
  std::optional<float2> UVFromWorldPointPrecise(
      const double3& world_point) const;

  // Convert a point in world space to pixel space.  If |world_point| is in
  // front of the near plane / behind the camera, returns a nullopt.
  //
  // Internally, this calls PixelFromWorldPointPrecise() and truncate the double
  // result to float before returning.
  std::optional<float2> PixelFromWorldPoint(const float3& world_point) const;
  // Convert a point in world space to pixel space.  If |world_point| is in
  // front of the near plane / behind the camera, returns a nullopt.
  std::optional<float2> PixelFromWorldPointPrecise(
      const double3& world_point) const;

  // Convert a point in pixel space to clip space.  If |pixel_point| is inside
  // the viewport, the result will be in the range ([-1,1], [-1,1]).
  float2 ClipFromPixelPoint(const float2& pixel_point) const;

  // Convert a point in clip space to pixel space.  If |clip_point| is in the
  // range ([-1,1], [-1,1], [0, 1]), the result will be in the viewport.
  float2 PixelFromClipPoint(const float3& clip_point) const;

  // Convert a point in pixel space to camera texture space.  If |pixel_point|
  // is inside the viewport, the result will be in the range [0, 1].
  float2 UVFromPixelPoint(const float2& pixel_point) const;

  // Convert a point in camera texture space to pixel space.  If |uv_point| is
  // in the range [0, 1], the result will be inside the viewport.
  float2 PixelFromUVPoint(const float2& uv_point) const;

  // Returns true if the given box is intersected in the camera's view frustum.
  bool IntersectsFrustum(const Box& world_bounds) const;

  // Convert a point in camera texture space to clip space.  If |uv_point| is in
  // the range [0, 1], the result will be in the range ([-1,1], [-1,1]). This
  // will flip the Y value, since texture space has y == 0 as the top, and clip
  // space has y == 1 as the top.
  static float2 ClipFromUVPoint(const float2& uv_point);

  // Convert a point in clip space to camera texture space.  If |clip_point| is
  // in the range ([-1,1], [-1,1], 0), the result will be in the range  [0, 1].
  // This will flip the Y value, since texture space has y == 0 as the top, and
  // clip space has y == 1 as the top.
  static float2 UVFromClipPoint(const float3& clip_point);

  // Setup with an existing camera instance. This should only be used if
  // integrating with another runtime that owns its own filament::Camera*.
  void Setup(filament::Camera* camera);

  // Returns the underlying filament camera instance.
  // Do not use this API unless you understand the underlying details of
  // filament.
  // TODO Wrap enough API so this isn't needed for common usages.
  filament::Camera* GetCamera() const;

  // Notifies the camera of updates to transient transition parameters, which
  // are caused by e.g. device rotation on iPad.  See
  // `ViewTransitionParametersChangedEvent` in view_events.h for an explanation
  // of these fields.
  void HandleTransitionScale(float2 scale);

  // Returns the view projection matrix.
  mat4 ClipFromWorld() const;
  // Returns the inverse view projection matrix.
  mat4 WorldFromClip() const;

  bool IsNearClipValid() const;
  bool IsFarClipValid() const;

 private:
  // Helper method to recompute the current aspect ratio
  void UpdateProjection(float2 scale = kOne2);

  void SanitizeClipPlanes();

  filament::Camera* camera_ = nullptr;
  utils::Entity camera_entity_;
  bool is_owned_ = false;
  std::optional<Invocable<float(float, float)>> update_projection_override_;

  // If true, the aspect ratio is set to a fixed value and locked.
  bool IsAspectRatioLocked() const;
  float GetAspectRatio(float2 scale = kOne2) const;

  imp::mat4 GetOrthographicProjection() const;

  imp::mat4 GetCustomProjection() const;

  CameraState::ProjectionType CalculateProjectionTypeFromMatrix(
      imp::mat4 projection_matrix) const;

  absl::StatusOr<float> CalculateOrthographicScaleFromMatrix(
      imp::mat4 projection_matrix) const;

  float CalculateAspectRatioFromMatrix(imp::mat4 projection_matrix) const;

  CameraState state_;

 public:
  using IsfInfo = IsfInfo<&CameraComponent::state_>;
  static constexpr bool kRunInEditMode = true;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CAMERA_CAMERA_COMPONENT_H_
