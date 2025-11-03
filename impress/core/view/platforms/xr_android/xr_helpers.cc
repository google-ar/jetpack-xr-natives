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

#include "core/view/platforms/xr_android/xr_helpers.h"

#include <cmath>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/TransformManager.h"
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/monitor/duration_measurement_data.h"
#include "core/monitor/monitor_summary.h"
#include "core/view/platforms/xr_android/openxr_includes.h"

namespace imp {

absl::Status ToStatus(XrInstance instance, XrResult result) {
  // In OpenXR, all successful completion codes are non-negative values,
  // including zero.
  // https://registry.khronos.org/OpenXR/specs/1.0-khr/html/xrspec.html#return-codes
  if (result >= XR_SUCCESS) {
    return absl::OkStatus();
  }

  std::string msg;
  if (instance != XR_NULL_HANDLE) {
    char buffer[XR_MAX_RESULT_STRING_SIZE] = {0};
    if (XR_UNQUALIFIED_SUCCESS(xrResultToString(instance, result, buffer))) {
      msg = buffer;
      return absl::InternalError(msg);
    }
  }

  return absl::UnknownError(
      absl::StrFormat("OpenXR UNKNOWN_FAILURE XrResult=%d %s", result, msg));
}

Transform<float> ToTransform(XrPosef pose) {
  Transform<float> transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;

  transform.rotation.x = pose.orientation.x;
  transform.rotation.y = pose.orientation.y;
  transform.rotation.z = pose.orientation.z;
  transform.rotation.w = pose.orientation.w;

  return transform;
}

mat4 GetLocalTransform(const mat4& model_matrix, const XrPosef& xr_pose) {
  return inverse(model_matrix) * ToTransform(xr_pose).AsMat4();
}

mat4 GetLocalTranslation(const mat4& model_matrix, const XrPosef& xr_pose) {
  return mat4::translation(
      (inverse(model_matrix) * ToVector3(xr_pose.position)).xyz);
}

Transform<float> GetEyeCenterTransform(const XrView& left_eye_view,
                                       const XrView& right_eye_view) {
  // TODO: Figure out why each eye orientation values might not be
  // almost equal and either uncomment the commented code below or even adapt
  // the fustrum calculation implementation in this file according to the
  // findings.
  // Assert that the eyes are parallel.
  // 

  // Find the midpoint between the eyes.
  float3 left_position = ToVector3(left_eye_view.pose.position);
  float3 right_position = ToVector3(right_eye_view.pose.position);
  float3 eye_center_position = (left_position + right_position) / 2.0f;

  // Use the orientation of the left eye.
  Transform<float> eye_center_transform = ToTransform(left_eye_view.pose);
  eye_center_transform.translation = eye_center_position;
  return eye_center_transform;
}

bool AreLeftAndRightFovAnglesMirrored(const XrFovf& left_eye_fov,
                                      const XrFovf& right_eye_fov) {
  return AlmostEqual(left_eye_fov.angleDown, right_eye_fov.angleDown) &&
         AlmostEqual(left_eye_fov.angleUp, right_eye_fov.angleUp) &&
         AlmostEqual(left_eye_fov.angleLeft, -right_eye_fov.angleRight) &&
         AlmostEqual(left_eye_fov.angleRight, -right_eye_fov.angleLeft);
}

bool AreXrQuaternionfsEqual(const XrQuaternionf& quaternion_1,
                            const XrQuaternionf& quaternion_2) {
  return AlmostEqual(quaternion_1.x, quaternion_2.x) &&
         AlmostEqual(quaternion_1.y, quaternion_2.y) &&
         AlmostEqual(quaternion_1.z, quaternion_2.z);
}

mat4 GetEncompassingProjectionMatrix(const XrView& left_eye_view,
                                     const XrView& right_eye_view,
                                     const float near_plane,
                                     const float far_plane) {
  // Assert that the eyes are symmetrical.
  
  // TODO: Figure out why each eye orientation values might not be
  // almost equal and either uncomment the commented code below or even adapt
  // the fustrum calculation implementation in this file according to the
  // findings.
  // Assert gaze is parallel.
  // 

  /*
   * For a symmetrical stereoscopic configuration, starting from head space and
   * matching the FOV angles of each eye, a shift backwards will result in a
   * frustum that encompasses both eye frustums.
   *
   *    Top down view:
   *
   *    \   /   \   /
   *     \ /     \ /
   *      L       R    _
   *       \     /     |
   *        \   /      | D
   *         \ /       |
   *          C        -
   *
   * L and R are the left and right eye positions.
   * C is the origin/point of the frustum that encompasses both left and right
   * eyes.
   * D is the distance that needs to be shifted backwards from head space.
   *
   * Solving the triangle, the formula for D is:
   * D = (half of distance between eyes) / tan(half of horizontal fov)
   */

  float distance_between_eyes =
      distance(ToVector3(left_eye_view.pose.position),
               ToVector3(right_eye_view.pose.position));
  // Note angleRight/angleUp are positive, angleLeft/angleDown are negative.
  float shift_backwards_distance =
      (distance_between_eyes / 2.0f) / tan(left_eye_view.fov.angleRight);

  // Use the left FOV of the left eye, the right FOV of the right eye, and the
  // up/bottom angles of either.
  XrFovf encompassing_frustum_fov = {
      .angleLeft = left_eye_view.fov.angleLeft,
      .angleRight = right_eye_view.fov.angleRight,
      .angleUp = left_eye_view.fov.angleUp,
      .angleDown = right_eye_view.fov.angleDown};

  // We add shift_backwards_distance to the near and far planes because we will
  // be shifting the frustum backwards.
  mat4 head_space_projection_matrix = GetProjectionMatrix(
      encompassing_frustum_fov, near_plane + shift_backwards_distance,
      far_plane + shift_backwards_distance);

  // Shift the frustum backwards by multiplying a forward translation.
  return head_space_projection_matrix *
         mat4::translation(float3{0, 0, -shift_backwards_distance});
}

mat4 GetProjectionMatrix(XrFovf fov, float nearZ, float farZ) {
  mat4 result;
  const float tanLeft = tan(fov.angleLeft);
  const float tanRight = tan(fov.angleRight);

  const float tanDown = tan(fov.angleDown);
  const float tanUp = tan(fov.angleUp);
  const float tanAngleWidth = tanRight - tanLeft;

  // Set to tanDown - tanUp for a clip space with positive Y down
  // (Vulkan). Set to tanUp - tanDown for a clip space with positive Y
  // up (OpenGL / D3D / Metal).
  const float tanAngleHeight = tanUp - tanDown;
  // Set to nearZ for a [-1,1] Z clip space (OpenGL / OpenGL ES).
  // Set to zero for a [0,1] Z clip space (Vulkan / D3D / Metal).
  const float offsetZ = nearZ;

  if (farZ <= nearZ) {
    // place the far plane at infinity
    result[0][0] = 2.0f / tanAngleWidth;
    result[1][0] = 0.0f;
    result[2][0] = (tanRight + tanLeft) / tanAngleWidth;
    result[3][0] = 0.0f;

    result[0][1] = 0.0f;
    result[1][1] = 2.0f / tanAngleHeight;
    result[2][1] = (tanUp + tanDown) / tanAngleHeight;
    result[3][1] = 0.0f;

    result[0][2] = 0.0f;
    result[1][2] = 0.0f;
    result[2][2] = -1.0f;
    result[3][2] = -(nearZ + offsetZ);

    result[0][3] = 0.0f;
    result[1][3] = 0.0f;
    result[2][3] = -1.0f;
    result[3][3] = 0.0f;
  } else {
    result[0][0] = 2.0f / tanAngleWidth;
    result[1][0] = 0.0f;
    result[2][0] = (tanRight + tanLeft) / tanAngleWidth;
    result[3][0] = 0.0f;

    result[0][1] = 0.0f;
    result[1][1] = 2.0f / tanAngleHeight;
    result[2][1] = (tanUp + tanDown) / tanAngleHeight;
    result[3][1] = 0.0f;

    result[0][2] = 0.0f;
    result[1][2] = 0.0f;
    result[2][2] = -(farZ + offsetZ) / (farZ - nearZ);
    result[3][2] = -(farZ * (nearZ + offsetZ)) / (farZ - nearZ);

    result[0][3] = 0.0f;
    result[1][3] = 0.0f;
    result[2][3] = -1.0f;
    result[3][3] = 0.0f;
  }
  return result;
}

void SetupXrTimingSummary(
    MonitorSummary& summary,
    std::unordered_map<std::string_view, MonitorSummary::CustomMetricHandle>&
        metricsStore) {
  summary.Configure(kUpdatesPerCollection, kEstimatedFps,
                    /*seconds_in_first_average =*/kFirstWindowSizeInSeconds,
                    /*seconds_in_second_average =*/kSecondWindowSizeInSeconds,
                    /*seconds_in_third_average =*/kThirdWindowSizeInSeconds);
  metricsStore[kXrAvgSampleAgePrefix] =
      summary.AddMetric<SampleAgeMetric>(kXrAvgSampleAgePrefix);
  metricsStore[kXrPercentageOfFramesWithSysUIDisplayEnabledPrefix] =
      summary.AddMetric<PeriodInMsMetric>(
          kXrPercentageOfFramesWithSysUIDisplayEnabledPrefix,
          kXrDisplayEnabledStatistics, kXrDisplayEnabledStatistics);
  metricsStore[kXrMsPerFrameSubmittedPrefix] =
      summary.AddMetric<PeriodInMsMetric>(kXrMsPerFrameSubmittedPrefix,
                                          kXrBetweenFrameTiming,
                                          kXrBetweenFrameTiming);
  metricsStore[kXrMsPerFrameScheduledPrefix] =
      summary.AddMetric<PeriodInMsMetric>(kXrMsPerFrameScheduledPrefix,
                                          kXrScheduledFrameTiming,
                                          kXrScheduledFrameTiming);
  metricsStore[kXrMsPerFrameCpu1Prefix] = summary.AddMetric<IntervalInMsMetric>(
      kXrMsPerFrameCpu1Prefix, kXrWaitFrameTiming, kXrBetweenFrameTiming);
  metricsStore[kXrMsPerFrameCpu2Prefix] = summary.AddMetric<PeriodInMsMetric>(
      kXrMsPerFrameCpu2Prefix, kXrBeginFrameToEndFrame,
      kXrScheduledFrameTiming);
  metricsStore[kXrPercentCpu1Prefix] =
      summary.AddMetric<PercentageOfIntervalMetric>(
          kXrPercentCpu1Prefix, kXrWaitFrameTiming, kXrBetweenFrameTiming,
          kXrBetweenFrameTiming);
  metricsStore[kXrPercentCpu2Prefix] =
      summary.AddMetric<PercentageOfDurationMetric>(
          kXrPercentCpu2Prefix, kXrBeginFrameToEndFrame, kXrBetweenFrameTiming);
}

void SetCustomEyeProjectionOnCamera(filament::Camera* camera,
                                    std::vector<XrView>& latest_views) {
  // Set the projection matrix of each eye.
  mat4 projection_matrix_array[4];
  for (int i = 0; i < latest_views.size(); i++) {
    projection_matrix_array[i] = GetProjectionMatrix(
        latest_views[i].fov, camera->getNear(), camera->getCullingFar());
  }

  // Set the projection culling matrix, which is a projection matrix
  // representing the frustum that encompasses the FOV of both eyes.
  // Whether or not we are using quad views, the first and second XrViews
  // represent the left eye outer view and the right eye outer view,
  // respectively.
  mat4 projection_culling_matrix = GetEncompassingProjectionMatrix(
      latest_views[0], latest_views[1], camera->getNear(),
      camera->getCullingFar());
  camera->setCustomEyeProjection(projection_matrix_array, latest_views.size(),
                                 projection_culling_matrix, camera->getNear(),
                                 camera->getCullingFar());
}

void SetEyeModelMatrixOnCamera(filament::Engine* engine,
                               filament::Camera* camera,
                               std::vector<XrView>& latest_views) {
  // Camera::getModelMatrix returns the world transform, so we have to get it
  // from the transform manager.
  auto& transform_manager = engine->getTransformManager();
  mat4 camera_model_matrix = transform_manager.getTransformAccurate(
      transform_manager.getInstance(camera->getEntity()));
  for (int i = 0; i < latest_views.size(); ++i) {
    camera->setEyeModelMatrix(
        i, GetLocalTransform(camera_model_matrix, latest_views[i].pose));
  }
}

std::string DumpXrFrameTiming(Monitor& monitor, MonitorSummary& summary) {
  return absl::StrCat(" XrFrameTiming: ", summary);
}

float3 ToVector3(XrVector3f xr_vector_3f) {
  return float3{xr_vector_3f.x, xr_vector_3f.y, xr_vector_3f.z};
}
}  // namespace imp
