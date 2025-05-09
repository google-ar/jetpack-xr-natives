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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_DESKTOP_AR_SESSION_NATIVE_DESKTOP_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_DESKTOP_AR_SESSION_NATIVE_DESKTOP_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "core/ar/ar_hdr_lighting.h"
#include "core/ar/ar_plane.h"
#include "core/ar/ar_session_native.h"
#include "core/ar/ar_trackable.h"
#include "core/ar/desktop/desktop_camera_controller.h"
#include "core/common/robin_map.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndirectLight.h"
#include "filament/filament/include/filament/Texture.h"

namespace imp {

class BaseView;

namespace ar {

struct AnchorInfo {
  AnchorInfo() {}
  explicit AnchorInfo(ArTrackableId attach_id) : attach_id(attach_id) {}
  AnchorInfo(const mat4f& world_transform, ArTrackableId attach_id)
      : world_transform(world_transform), attach_id(attach_id) {}
  mat4f world_transform;
  ArTrackableId attach_id;
};

// An ArSessionNative that runs on desktop, the desktop session mocks AR data
// such as planes, providing them to the cross-platorm AR system.
class ArSessionNativeDesktop : public ArSessionNative {
 public:
  // Create a new session using the given JNI context, filament engine, and
  // custom settings.
  static Future<std::unique_ptr<ArSessionNative>> CreateSession(
      filament::Engine* engine, BaseView* view);

  bool GetDisplayGeometryChanged() const override { return false; }

  void GetCameraTextureUVs(
      const absl::Span<const float2> normalized_screen_coordinates,
      std::vector<float2>* uvs_out) const override;
  std::array<float3, 2u> GetUvFromNdcTransform() const override;
  void SetDisplayGeometry(window::WindowRotation orientation, int width,
                          int height, float near, float far) override;
  void Pause() override;
  void Resume() override;
  bool IsHdrLightingEnabled() const override { return false; }

  std::unique_ptr<HdrLighting> GetHdrLighting() override;

  absl::optional<ArFrame> Update(absl::Time last_submitted_timestamp) override;
  uint4 GetDebugSessionId() override;

  std::vector<ArHitResult> HitTest(
      float2 screen_pos, absl::optional<float> guessed_distance,
      TrackableTuple* out_generated_trackables) override;
  std::vector<ArHitResult> HitTestRay(
      const Ray& ray, TrackableTuple* out_generated_trackables) override;

  TrackingState GetCameraTrackingState() const override {
    return TrackingState::kTracking;
  }
  absl::StatusOr<ArAnchor> CreateAnchor(
      float3 position, quatf rotation,
      absl::optional<ArTrackableId> id) override;
  void DestroyAnchor(ArAnchor anchor) override;
  std::vector<float3> GetSphericalHarmonicsLighting() override;
  // Enables desktop camera controls on the view camera.
  void EnableVirtualCameraControlOnView();
  // Disables desktop camera controls on the view camera.
  // Blocks desktop AR mode on the backround model being ready.
  bool IsReadyToRender() const override;

 private:
  ArSessionNativeDesktop(filament::Engine* engine, BaseView* view);

  std::vector<ArPoint> GetPoints();
  std::vector<ArPlane> GetPlanes();
  std::vector<ArAnchor> GetAnchors();
  std::vector<ArPoint> GetUpdatedPoints();
  // Gets planes that have changed state this frame. Desktop AR conjures a plane
  // to satisify its dependent AR systems.
  std::vector<ArPlane> GetUpdatedPlanes();

  // Updates anchors according to plane positions (simulates ARCore behavior).
  void UpdateAnchors();

  // Sets up keyboard handling for toggling AR features.
  void ConfigureKeyboardInputListener();

  // Draw desktop usage UI.
  void UpdateUI();

  std::vector<std::unique_ptr<ArTrackable>> ar_trackables_;
  std::vector<std::unique_ptr<ArAnchor>> ar_anchors_;
  RobinMap<ArTrackableId, AnchorInfo> anchor_to_plane_;
  mat4 projection_matrix_;
  mat4f view_matrix_;
  TexturePtr camera_texture_;
  filament::Engine* engine_;
  std::unique_ptr<DesktopCameraController> desktop_camera_controller_;
  Dispatcher::Connection keyboard_event_connection_;
  BaseView* view_;
  int width_;
  int height_;
  float near_;
  float far_;
  bool find_instant_placement_point_ = false;
};

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_DESKTOP_AR_SESSION_NATIVE_DESKTOP_H_
