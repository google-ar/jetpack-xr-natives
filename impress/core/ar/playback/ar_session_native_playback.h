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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_PLAYBACK_AR_SESSION_NATIVE_PLAYBACK_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_PLAYBACK_AR_SESSION_NATIVE_PLAYBACK_H_

#include <optional>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "core/ar/ar_magical_surface_point.h"
#include "core/ar/ar_plane.h"
#include "core/ar/ar_point.h"
#include "core/ar/ar_session_config.h"
#include "core/ar/ar_session_native.h"
#include "core/ar/ar_trackable.h"
#include "core/ar/playback/ar_playback_data.h"
#include "core/video/video_controller.h"
#include "core/view/base_view.h"

namespace imp::ar {

// An ArSessionNative that plays back a video with recorded AR data.
// Call static CreateSession(view, config) to create a new session.
class ArSessionNativePlayback : public ArSessionNative {
 public:
  explicit ArSessionNativePlayback(BaseView* view);

  void GetCameraTextureUVs(
      const absl::Span<const float2> normalized_screen_coordinates,
      std::vector<float2>* uvs_out) const override;
  std::array<float3, 2u> GetUvFromNdcTransform() const override;
  bool GetDisplayGeometryChanged() const override;
  // Note that the return values for methods dependent on these values (e.g.
  // GetCameraTextureUVs) will change immediately, not on the next update.
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
  TrackingState GetCameraTrackingState() const override;

  absl::StatusOr<ArAnchor> CreateAnchor(
      float3 position, quatf rotation,
      absl::optional<ArTrackableId> id) override;

  void DestroyAnchor(ArAnchor anchor) override;

  void SetPlaybackScene(std::unique_ptr<ArPlaybackData> playback_data) override;

 private:
  friend class SessionHelper;
  struct PlaneInfo {
    ArPlane plane;
    absl::flat_hash_set<ArTrackableId> anchors;
  };

  struct AnchorInfo {
    ArAnchor anchor;
    absl::optional<ArTrackableId> plane;
  };

  struct VideoTimestampInfo {
    int64_t presentation_timestamp;
    absl::Duration playback_time;
    absl::Duration video_duration;
  };

  // Returns a transform from screen coordinates to video texture coordinates
  // where (0, 0) is the center of each coordinate system.
  mat2f GetCenteredCoordinateTransform() const;

  // Returns true if the video player and metadata are ready to be used.
  bool UpdatePlayerState();

  ArFrame CreateFrame(int frame_index, absl::Time frame_time,
                      std::vector<ArPlane> planes,
                      std::vector<ArAnchor> anchors, imp::Texture* texture);
  absl::StatusOr<VideoTimestampInfo> GetVideoTimestampInfo();

  BaseView* view_;
  std::unique_ptr<ArPlaybackData> playback_data_;
  absl::optional<ComponentHandle<VideoController>> video_controller_;
  std::unique_ptr<video::VideoSource> video_source_player_;
  absl::optional<mat4f> camera_pose_;
  absl::Time base_time_;

  int last_frame_;
  int total_loops_;
  bool paused_;

  window::WindowRotation orientation_;
  float width_;
  float height_;
  float video_width_;
  float video_height_;
  float near_;
  float far_;
  bool display_geometry_changed_;
  bool pending_display_geometry_changed_;

  absl::flat_hash_map<ArTrackableId, PlaneInfo> planes_;
  absl::flat_hash_map<ArTrackableId, AnchorInfo> anchors_;
  std::vector<ArPoint> points_;
};

}  // namespace imp::ar

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_VIDEO_AR_SESSION_NATIVE_VIDEO_H_
