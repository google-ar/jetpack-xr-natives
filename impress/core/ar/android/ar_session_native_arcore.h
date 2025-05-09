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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_AR_SESSION_NATIVE_ARCORE_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_AR_SESSION_NATIVE_ARCORE_H_

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "core/ar/android/android_camera_texture.h"
#include "core/ar/android/ar_core_ptrs.h"
#include "core/ar/android/deeplight_controller.h"
#include "core/ar/android/depth_texture_controller.h"
#include "core/ar/ar_magical_surface_point.h"
#include "core/ar/ar_plane.h"
#include "core/ar/ar_point.h"
#include "core/ar/ar_point_cloud.h"
#if IMP_PLATFORM(ANDROID)
#include "core/ar/ar_prior_map.h"
#endif
#include "core/ar/ar_session_config.h"
#include "core/ar/ar_session_native.h"
#include "core/ar/ar_trackable.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/render/texture_factory.h"
#include "filament/filament/include/filament/Engine.h"

namespace imp {
namespace ar {

// An ArSessionNative that uses ARCore to provide tracking data.
// Call static CreateSession(context, engine) to create a new session.
class ArSessionNativeArCore : public ArSessionNative {
 public:
  // Create a new session using the given JNI context, filament engine, and
  // custom settings.
  // TODO Remove filament engine references - must remain safe to
  // call from background thread
  static Future<std::unique_ptr<ArSessionNative>> CreateSession(
      const Context& context, filament::Engine* engine,
      imp::TextureFactory* texture_factory,
      imp::EnvironmentLightFactory* env_light_factory,
      const ArSessionConfig& config, std::string settings = "");
  // Create a new session with an existing ArSession for tests.
  // TODO Remove filament engine references - must remain safe to
  // call from background thread
  static std::unique_ptr<ArSessionNative> CreateSession(
      filament::Engine* engine, imp::TextureFactory* texture_factory,
      imp::EnvironmentLightFactory* env_light_factory,
      const ArSessionConfig& config, UniqueArSession ar_session);

  // Sets the method by which hit tests for object placement are performed.
  void SetPlacementMode(ArSessionConfig::PlacementMode mode) override;

  void GetCameraTextureUVs(
      const absl::Span<const float2> normalized_screen_coordinates,
      std::vector<float2>* uvs_out) const override;
  std::array<float3, 2u> GetUvFromNdcTransform() const override;
  bool GetDisplayGeometryChanged() const override;
  void SetDisplayGeometry(window::WindowRotation orientation, int width,
                          int height, float near, float far) override;

  void Pause() override;
  void Resume() override;
  absl::optional<ArFrame> Update(absl::Time last_submitted_timestamp) override;
  uint4 GetDebugSessionId() override;

  std::vector<ArHitResult> HitTest(
      float2 screen_pos, absl::optional<float> guessed_distance,
      TrackableTuple* out_generated_trackables) override;
  std::vector<ArHitResult> HitTestRay(
      const Ray& ray, TrackableTuple* out_generated_trackables) override;

  // Checks ARCores camera tracking state.
  TrackingState GetCameraTrackingState() const override;

  absl::StatusOr<ArAnchor> CreateAnchor(
      float3 position, quatf rotation,
      absl::optional<ArTrackableId> id) override;

  void DestroyAnchor(ArAnchor anchor) override;

  void SetDepthTextureCreatedHandler(
      std::function<void(DepthData const&)> handler) override;

  float GetDepthRegionConfidence(int32_t min_x, int32_t max_x, int32_t width,
                                 int32_t height) override;

  bool IsDepthSupported() const override;

  bool IsHdrLightingEnabled() const override;

  std::unique_ptr<HdrLighting> GetHdrLighting() override;

  std::vector<float3> GetSphericalHarmonicsLighting() override;
  absl::Status GetLatestModelMatrix(mat4f* matrix_out,
                                    absl::Time* timestamp_out) const override;

  float4 GetAmbientLighting() override;

  TrackingFailureReason GetTrackingFailureReason() const override;

  ArPointCloud GetPointCloud() const override;

  absl::Status StartRecording(absl::string_view dataset_uri) override;

  absl::Status StopRecording() override;

 private:
  friend class SessionHelper;
  // Construct this using an existing ARCore ArSession object.
  ArSessionNativeArCore(filament::Engine* engine,
                        imp::TextureFactory* texture_factory,
                        imp::EnvironmentLightFactory* env_light_factory,
                        const ArSessionConfig& config, UniqueArSession session);
  absl::Status InternalInitialization();
  void CreateCameraTextures();
  uint2 GetCameraTextureDimensions() const;
  ArAnchor ConvertAnchor(ArAnchor_* native_anchor) const;
  std::vector<ArAnchor> ConvertAnchors(
      const ArAnchorListHelper& anchor_list) const;
  std::vector<ArPoint> GetUpdatedPoints() const;
  std::vector<ArMagicalSurfacePoint> GetUpdatedMagicalSurfacePoints() const;
  std::vector<ArPlane> GetUpdatedPlanes() const;
  std::vector<ArAnchor> GetUpdatedAnchors() const;
#ifdef IMP_PRIOR_MAP
  std::vector<ArPriorMap> GetUpdatedPriorMaps() const;
#endif
  absl::optional<ArHitResult> ConvertArHitResult(
      const ArHitResultPtr& hit_result,
      TrackableTuple* out_generated_trackables) const;
  std::vector<ArHitResult> ConvertArHitResults(
      const ArHitResultListPtr& hit_result_list,
      TrackableTuple* out_generated_trackables) const;

  filament::Engine* engine_;
  imp::TextureFactory* texture_factory_;
  imp::EnvironmentLightFactory* env_light_factory_;
  ArSessionConfig ar_session_config_;
  UniqueArSession ar_session_;
  UniqueArConfig ar_config_;
  UniqueArRecordingConfig ar_recording_config_;
  UniqueArFrame ar_frame_;
  UniqueArCamera ar_camera_;
  UniqueArPose ar_pose_;
  int2 camera_dimensions_;
  std::unique_ptr<AndroidCameraTexture> camera_texture_ = nullptr;
  float width_;
  float height_;
  float near_ = 0.0;
  float far_ = 0.0;
  absl::optional<DepthTextureController> depth_texture_controller_;
  DeeplightController deeplight_controller_;
  ArSessionConfig::PlacementMode placement_mode_;
};

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_AR_SESSION_NATIVE_ARCORE_H_
