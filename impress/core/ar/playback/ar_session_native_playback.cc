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

#include "core/ar/playback/ar_session_native_playback.h"

#include <functional>

#include "glog/logging.h"
#include "core/common/log.h"
#include "absl/strings/str_replace.h"
#include "third_party/arcore/proto/video/ar_derived_data.pb.h"
#include "core/ar/ar_magical_surface_point.h"
#include "core/collision/collision_helpers.h"
#include "core/video/video_controller.h"
#include "imp.h"

namespace imp::ar {
namespace {

constexpr float2 kNormalizedVideoCenter = {0.5f, 0.5f};

mat4f GetPose(const ::ar::video::Pose& pose) {
  return mat4f(
      mat3f(filament::math::quatf(pose.quaternion(3), pose.quaternion(0),
                                  pose.quaternion(1), pose.quaternion(2))),
      float3(pose.translation(0), pose.translation(1), pose.translation(2)));
}

ArPlane ConvertPlane(const ::ar::video::ARPlane& plane) {
  mat4f model_mat4 = GetPose(plane.center_pose());

  TrackingState tracking_state = TrackingState::kTracking;

  ArPlane::PlaneType plane_type;
  switch (plane.plane_type()) {
    case ::ar::video::ARPlane::AR_PLANE_HORIZONTAL_UPWARD_FACING:
      plane_type = ArPlane::PlaneType::kHorizontalUpFacing;
      break;
    case ::ar::video::ARPlane::AR_PLANE_HORIZONTAL_DOWNWARD_FACING:
      plane_type = ArPlane::PlaneType::kHorizontalDownFacing;
      break;
    case ::ar::video::ARPlane::AR_PLANE_VERTICAL:
      plane_type = ArPlane::PlaneType::kVertical;
      break;
    default:
      break;
  }

  std::vector<float3> vertices;
  vertices.reserve(plane.polygon_point_size() / 2);
  for (int i = 0; i < plane.polygon_point_size(); i += 2) {
    vertices.emplace_back(plane.polygon_point(i), 0.0f,
                          plane.polygon_point(i + 1));
  }

  float2 extents(plane.extent_x() / 2, plane.extent_z() / 2);

  return ArPlane(ArTrackableId(plane.id()), tracking_state, model_mat4, extents,
                 plane_type, std::move(vertices));
}

ArTrackableId GenerateTrackableId() {
  static int64_t next_id = 0;
  // Use 1 for lower bits so that we cannot conflict with recorded IDs.
  return ArTrackableId(next_id++, 1);
}

}  // namespace

ArSessionNativePlayback::ArSessionNativePlayback(BaseView* view)
    : view_(view),
      last_frame_(-1),
      total_loops_(0),
      paused_(false),
      orientation_(window::WindowRotation::kRotation0),
      width_(0),
      height_(0),
      video_width_(540),
      video_height_(720),
      near_(0),
      far_(0),
      display_geometry_changed_(false),
      pending_display_geometry_changed_(false) {}

void ArSessionNativePlayback::SetPlaybackScene(
    std::unique_ptr<::imp::ar::ArPlaybackData> playback_data) {
  playback_data_ = std::move(playback_data);
}

mat2f ArSessionNativePlayback::GetCenteredCoordinateTransform() const {
  float view_ratio = static_cast<float>(height_) / width_;
  float video_ratio = video_height_ / video_width_;

  bool different_view_video_orientations =
      (view_ratio > 1) != (video_ratio > 1);
  int quarter_circles = static_cast<int>(orientation_);
  // Add additional rotations if the video aspect ratio does not match the
  // display.
  if (different_view_video_orientations) {
    view_ratio = 1 / view_ratio;
  }
  // If the number of quarter circle rotations is odd, then the aspect ratio
  // will be flipped. If the view orientation is different from the video
  // orientation, then we want to make the number of quarter rotations odd so
  // that the orientations match. If the view and video orientations match, then
  // we want to make the number of quarter rotations even.
  if (different_view_video_orientations != ((quarter_circles % 2) == 1)) {
    // TODO: Keep track of the camera rotation during recording so
    // that we know the default amount that we should rotate the display
    quarter_circles--;
  }

  // If the video ratio is smaller (i.e. shorter) than the view ratio, then we
  // want to scale along the x axis so that view points sample from horizontally
  // more centered points in the video so that the sides of the video are
  // cropped out. If the video ratio is larger, then we want to crop vertically
  // along the y axis.
  mat2f coordinate_scale =
      mat2f::scaling(float2(std::min(1.0f, video_ratio / view_ratio),
                            std::min(1.0f, view_ratio / video_ratio)));

  // We want to rotate coordinates so that the video orientation matches the
  // view orientation.
  mat2f coordinate_rotation = mat2f::rotate((quarter_circles)*M_PI / 2);
  return coordinate_scale * coordinate_rotation;
}

void ArSessionNativePlayback::GetCameraTextureUVs(
    const absl::Span<const float2> normalized_screen_coordinates,
    std::vector<float2>* uvs_out) const {
  

  mat2f coordinate_transform = GetCenteredCoordinateTransform();
  for (int i = 0; i < normalized_screen_coordinates.size(); ++i) {
    // The normalized screen and texture coordinates range from 0 to 1, so we
    // offset by (0.5, 0.5) to rotate and scale around the center of the image.
    float2 coordinates = normalized_screen_coordinates[i];
    coordinates -= kNormalizedVideoCenter;
    coordinates = coordinate_transform * coordinates;
    coordinates += kNormalizedVideoCenter;
    uvs_out->at(i) = coordinates;
  }
}

std::array<float3, 2u> ArSessionNativePlayback::GetUvFromNdcTransform() const {
  // NDC range from -1 to 1, but UV coordinates range from 0 to 1, so we halve
  // the coordinates to get the range we want. They are also oriented
  // differently, so we flip the y coordinate.
  constexpr float2 kNdcToUvRange = {0.5f, -0.5f};
  mat2f coordinate_transform =
      GetCenteredCoordinateTransform() * mat2f(kNdcToUvRange);
  // We offset by the video center so that coordinates range from 0 to 1 instead
  // of -0.5 to 0.5.
  return {float3{coordinate_transform[0][0], coordinate_transform[1][0],
                 kNormalizedVideoCenter.x},
          float3{coordinate_transform[0][1], coordinate_transform[1][1],
                 kNormalizedVideoCenter.y}};
}

bool ArSessionNativePlayback::GetDisplayGeometryChanged() const {
  return display_geometry_changed_;
}

void ArSessionNativePlayback::SetDisplayGeometry(
    window::WindowRotation orientation, int width, int height, float near,
    float far) {
  if ((orientation_ == orientation) && (width_ == width) &&
      (height_ == height) && imp::AlmostEqual(near_, near) &&
      imp::AlmostEqual(far_, far)) {
    return;
  }
  orientation_ = orientation;
  width_ = width;
  height_ = height;
  near_ = near;
  far_ = far;
  pending_display_geometry_changed_ = true;
}

void ArSessionNativePlayback::Pause() {
  paused_ = true;
  UpdatePlayerState();
}

void ArSessionNativePlayback::Resume() {
  paused_ = false;
  UpdatePlayerState();
}

bool ArSessionNativePlayback::UpdatePlayerState() {
  if (!playback_data_) {
    return false;
  }

  if (!video_controller_.has_value()) {
    if (!playback_data_->video_controller_future.Ready()) {
      return false;
    }
    auto& result = playback_data_->video_controller_future.Get();
    if (result.status().ok()) {
      video_controller_ = *playback_data_->video_controller_future.Move();
      if (!(*video_controller_)->SetLoopCount(-1).ok()) {
        IMP_LOG(imp::WARNING)
            << "Failed to set playback to loop. Still trying to continue";
      }
      if (auto video_size = video_controller_.value()->GetVideoSize();
          video_size.ok() && video_size->y > 0 && video_size->x > 0) {
        video_height_ = static_cast<float>(video_size->y);
        video_width_ = static_cast<float>(video_size->x);
      }

    } else {
      IMP_LOG(imp::FATAL) << result.status();
    }
  }

  VideoController& video_player = **video_controller_;

  absl::StatusOr<VideoController::State> state = video_player.GetVideoState();
  if (!state.ok()) {
    return false;
  }

  if ((*state != VideoController::State::kPlaying) && !paused_) {
    if (!video_player.Play().ok()) {
      return false;
    }
  } else if ((*state == VideoController::State::kPlaying) && paused_) {
    if (!video_player.Pause().ok()) {
      return false;
    }
  }

  return true;
}

ArFrame ArSessionNativePlayback::CreateFrame(int frame_index,
                                             absl::Time frame_time,
                                             std::vector<ArPlane> planes,
                                             std::vector<ArAnchor> anchors,
                                             imp::Texture* texture) {
  const ::ar::video::ARFrame& frame =
      playback_data_->playback_scene.scene_data().frames(frame_index);
  mat2f coordinate_transform = GetCenteredCoordinateTransform();
  mat3f display_rotation = mat3f::rotation(
      -atan2(coordinate_transform[0].y, coordinate_transform[0].x),
      float3(0, 0, 1));

  float3 video_size =
      abs(display_rotation * float3(frame.camera().camera_intrinsics().width(),
                                    frame.camera().camera_intrinsics().height(),
                                    1));

  float3 scale =
      near_ / abs(display_rotation *
                  float3(frame.camera().camera_intrinsics().fx(),
                         frame.camera().camera_intrinsics().fy(), 1));
  float3 offset =
      scale * abs(display_rotation *
                  float3((frame.camera().camera_intrinsics().cx() -
                          (frame.camera().camera_intrinsics().width() / 2.0f)),
                         (frame.camera().camera_intrinsics().cy() -
                          (frame.camera().camera_intrinsics().height() / 2.0f)),
                         1));

  float view_ratio = static_cast<float>(height_) / width_;
  float video_ratio = static_cast<float>(video_size[1]) / video_size[0];
  float view_height = video_size.y * std::min(1.0f, view_ratio / video_ratio);
  float view_width = video_size.x * std::min(1.0f, video_ratio / view_ratio);
  mat4 projection =
      mat4::frustum(scale.x * -view_width / 2.0f - offset.x,
                    scale.y * view_width / 2.0f - offset.x,
                    // Color camera's coordinates has y pointing downwards so we
                    // negate this term.
                    scale.y * -view_height / 2.0f + offset.y,
                    scale.y * view_height / 2.0f + offset.y, near_, far_);
  mat4f camera_pose = GetPose(frame.camera().pose()) * mat4f(display_rotation);
  // Use current time instead of video time in order to keep monotonically
  // increasing timestamps when the video loops.
  return ArFrame(frame_time, texture, 0, projection, camera_pose,
                 {{},
                  std::move(planes),
                  {},
                  std::move(anchors),
                  {}
#ifdef IMP_PRIOR_MAP
                  ,
                  {}
#endif
                 });
}

absl::StatusOr<ArSessionNativePlayback::VideoTimestampInfo>
ArSessionNativePlayback::GetVideoTimestampInfo() {
  VideoTimestampInfo timestamp_info;
  MP_ASSIGN_OR_RETURN(timestamp_info.playback_time,
                   (*video_controller_)->GetPlaybackTime());
  MP_ASSIGN_OR_RETURN(timestamp_info.video_duration,
                   (*video_controller_)->GetDuration());
  // Convert nanoseconds to 90kHz MP4 presentation time clock.
  timestamp_info.presentation_timestamp =
      absl::ToInt64Nanoseconds(timestamp_info.playback_time) * 90'000 /
      1'000'000'000;
  return timestamp_info;
}

absl::optional<ArFrame> ArSessionNativePlayback::Update(
    absl::Time last_submitted_timestamp) {
  if (!UpdatePlayerState()) {
    base_time_ = absl::Now();
    return ArFrame(base_time_, nullptr, 0, mat4(), mat4f(),
                   {{},
                    {},
                    {},
                    {},
                    {}
#ifdef IMP_PRIOR_MAP
                    ,
                    {}
#endif
                   });
  }

  absl::StatusOr<VideoTimestampInfo> timestamp = GetVideoTimestampInfo();
  if (!timestamp.ok()) {
    return absl::nullopt;
  }

  absl::flat_hash_set<ArTrackableId> modified_planes;
  int i;
  for (i = 0; i < playback_data_->playback_scene.scene_data().frames_size();
       i++) {
    const ::ar::video::ARFrame& frame =
        playback_data_->playback_scene.scene_data().frames(i);
    if (last_frame_ < i) {
      for (const ::ar::video::ARPlane& plane : frame.planes()) {
        ArPlane ar_plane = ConvertPlane(plane);
        modified_planes.insert(ar_plane.GetId());
        planes_[ar_plane.GetId()].plane = std::move(ar_plane);
      }
    }

    if (playback_data_->playback_scene.scene_data()
            .frames(i)
            .presentation_timestamp() >= timestamp->presentation_timestamp) {
      if (last_frame_ < i) {
        // We currently do not seek through videos, so if the last frame goes
        // backwards, then we have completed a video loop.
        total_loops_++;
      } else if (last_frame_ == i && !paused_) {
        return absl::nullopt;
      }
      last_frame_ = i;
      break;
    }
  }

  std::vector<ArPlane> planes;
  std::vector<ArAnchor> anchors;
  for (ArTrackableId id : modified_planes) {
    PlaneInfo& plane_info = planes_[id];
    planes.push_back(plane_info.plane);
    for (ArTrackableId anchor_id : plane_info.anchors) {
      AnchorInfo& anchor_info = anchors_[anchor_id];
      // TODO: Improve point tracking.
      // We currently just track the y coordinate of the plane, but do not
      // handle any side to side movement.
      anchor_info.anchor.GetTransform()[3].y =
          plane_info.plane.GetTransform()[3].y;
      anchors.push_back(anchor_info.anchor);
    }
  }

  if (i >= playback_data_->playback_scene.scene_data().frames_size()) {
    return absl::nullopt;
  }

  display_geometry_changed_ = pending_display_geometry_changed_;
  pending_display_geometry_changed_ = false;
  absl::Time frame_time = base_time_ +
                          total_loops_ * timestamp->video_duration +
                          timestamp->playback_time;
  ArFrame frame =
      CreateFrame(i, frame_time, std::move(planes), std::move(anchors),
                  (*video_controller_)->GetVideoTexture());
  camera_pose_ = frame.model_matrix();
  return frame;
}

uint4 ArSessionNativePlayback::GetDebugSessionId() {
  // TODO: Keep track of a session ID during recording
  return kDefaultDebugSessionId;
}

std::vector<ArHitResult> ArSessionNativePlayback::HitTest(
    float2 screen_pos, absl::optional<float> guessed_distance,
    TrackableTuple* out_generated_trackables) {
  // Construct a ray from the screen_pos;
  Ray ray =
      view_->GetCameraManager().GetCamera()->WorldRayFromPixelPoint(screen_pos);
  return HitTestRay(ray, out_generated_trackables);
}

std::vector<ArHitResult> ArSessionNativePlayback::HitTestRay(
    const Ray& ray, TrackableTuple* out_generated_trackables) {
  std::vector<ArHitResult> hits;
  for (const auto& [plane_id, plane_info] : planes_) {
    const ArPlane& plane = plane_info.plane;

    // Get distance from plane to origin.
    auto trs = plane.GetTransform();
    float3 up = normalize(trs[1].xyz);
    float distance = dot(up, trs[3].xyz);
    Plane collision_plane(up, distance);

    float3 collision_point;
    if (collision::Result::kDoesNotIntersect ==
        collision::PlaneIntersectsRay(collision_plane, ray, &collision_point)) {
      continue;
    }

    hits.emplace_back(plane.GetTransform().toQuaternion(), collision_point,
                      length(collision_point - ray.origin), plane.GetId());
  }

  std::sort(hits.begin(), hits.end(),
            [](const ArHitResult& a, const ArHitResult& b) {
              return a.GetDistance() < b.GetDistance();
            });
  return hits;
}

TrackingState ArSessionNativePlayback::GetCameraTrackingState() const {
  if (video_controller_.has_value()) {
    return TrackingState::kTracking;
  }
  return TrackingState::kPaused;
}

absl::StatusOr<ArAnchor> ArSessionNativePlayback::CreateAnchor(
    float3 position, quatf rotation, absl::optional<ArTrackableId> id) {
  ArAnchor anchor = ArAnchor(GenerateTrackableId(), TrackingState::kTracking,
                             mat4f(mat3f(rotation), position));
  anchors_[anchor.GetId()] = {anchor, id};
  if (id.has_value() && planes_.contains(*id)) {
    planes_[*id].anchors.insert(anchor.GetId());
  }
  return anchor;
}

void ArSessionNativePlayback::DestroyAnchor(ArAnchor anchor) {
  auto anchor_iter = anchors_.find(anchor.GetId());
  if (anchor_iter == anchors_.end()) {
    return;
  }

  if (anchor_iter->second.plane.has_value()) {
    planes_.at(*anchor_iter->second.plane).anchors.erase(anchor.GetId());
  }
  anchors_.erase(anchor.GetId());
}

}  // namespace imp::ar
