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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_SESSION_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_SESSION_H_

#include <functional>
#include <optional>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "core/ar/ar_frame.h"
#include "core/ar/ar_plane.h"
#include "core/ar/ar_session_native.h"
#include "core/ar/ar_trackable.h"
#include "core/ar/ar_trackable_handle.h"
#include "core/ar/ar_trackable_helpers.h"
#include "core/ar/ar_trackables_manager.h"
#include "core/ar/playback/ar_playback_data.h"
#include "core/async/future.h"
#include "core/view/base_view.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

namespace ar {

class DebugHelper {
 public:
  virtual ~DebugHelper() = default;
  template <typename T>
  T* CastTo() {
    return static_cast<T*>(this);
  }
  virtual void Pause() {}
  virtual void Resume() {}
};

// These two function types are used internally to allow unit tests to override
// session creation to create mocks instead of real platform sessions.
class ArSession;
using CanCreateArSessionFn = std::function<bool(const BaseView* view)>;
using CreateArSessionFn = std::function<Future<std::unique_ptr<ArSession>>(
    BaseView* view, const ArSessionConfig& config)>;

// Manages the lifetime and operation of an AR tracking session.
class ArSession {
 public:
  static Future<absl::Status> CanCreate(const BaseView* view,
                                        const ArSessionConfig& config);

  static Future<std::unique_ptr<ArSession>> Create(
      BaseView* view, const ArSessionConfig& config);
  virtual ~ArSession() {}

  // Pauses the AR tracking. Subsequent calls to Update() will do nothing.
  virtual void Pause();

  // Resumes the session if it was previously paused.
  virtual void Resume();

  // Updates the tracking state of the session.
  // Will result in a new camera texture / transform and updated trackables.
  // Returns true if the state of the session was updated, false otherwise.
  virtual bool Update();

  // Render debug geometry.
  void RenderDev();

  // Updates the submitted timestamp.
  void OnPostRender();

  // Provides a unique identifier associated with the session.
  virtual std::string GetDebugSessionId() const;

  // Gets the UVs to use for the camera texture given screen coordinates [-1,
  // 1]. Note that the canonical use of this uses an expanded triangle to
  // avoid having to use a quad, so technically the coords can go beyond that
  // range.
  virtual void GetCameraTextureUVs(
      const absl::Span<const float2> normalized_screen_coordinates,
      std::vector<float2>* uvs_out) const;

  virtual std::array<float3, 2u> GetUvFromNdcTransform() const {
    return session_native_->GetUvFromNdcTransform();
  }

  using DepthData = ArSessionNative::DepthData;

  // Sets a callback to be invoked each time depth data is updated to be used
  // for rendering Ar occlusions. Typically, this is invoked every frame if
  // occlusions are enabled on a supported device.
  virtual void SetDepthTextureCreatedHandler(
      std::function<void(DepthData const&)> handler) {
    session_native_->SetDepthTextureCreatedHandler(std::move(handler));
  }

  virtual float GetDepthRegionConfidence(int32_t min_x, int32_t max_x,
                                         int32_t width, int32_t height) const {
    return session_native_->GetDepthRegionConfidence(min_x, max_x, width,
                                                     height);
  }

  // Returns true if the display geometry changed this frame.
  virtual bool GetDisplayGeometryChanged() const;

  // Sets the display orientation, width, height, near, and far values to use.
  // This will affect the projection matrix and UVs for the next frame.
  virtual void SetDisplayGeometry(window::WindowRotation orientation, int width,
                                  int height, float near, float far);

  // Sends the parameters from a `ViewTransitionParametersChangedEvent` event to
  // the AR session (which is at a lower layer than the view/dispatcher).  See
  // the comments for `ViewTransitionParametersChangedEvent` in view_events.h
  // for more context about their meaning.
  virtual void SetTransitionParameters(float2 view_scale,
                                       float counter_rotation);

  // Does a hit-test against all trackables using a screen position.
  // If an optional guessed distance is given, this creates a point along the
  // ray at that distance.
  // Note that, in general, tracking information improves over time and
  // anchoring the point returned from this method allows the point's depth to
  // improve from guessed value to real value as information improves.
  // Non-anchored points may stop tracking on the next frame.
  // Returns a sorted list of hit results, where the hit closest to the camera
  // is first in the list.
  // If guessed_distance is not nullopt, then it will only return points as that
  // is the "guessed distance / instant hit" mode for use before planes are
  // detected. The current placement mode setting also affects this, for example
  // if it is set to magical point mode, it will only return magical point hits.
  virtual std::vector<ArHitResult> HitTest(
      const float2 screen, absl::optional<float> guessed_distance);
  // Does a hit-test against all trackables using a 3D ray.
  // Returns a sorted list of hit results, where the hit closest to the camera
  // is first in the list.
  // If guessed_distance is not nullopt, then it will only return points as that
  // is the "guessed distance / instant hit" mode for use before planes are
  // detected. The current placement mode setting also affects this, for example
  // if it is set to magical point mode, it will only return magical point hits.
  virtual std::vector<ArHitResult> HitTestRay(
      const Ray& ray, absl::optional<float> guessed_distance);

  virtual const imp::Texture* camera_texture() const {
    return frame_ ? frame_->camera_texture() : nullptr;
  }
  virtual const std::unique_ptr<ArFrame::YUV420Image> move_camera_image() {
    return frame_ ? frame_->move_camera_image() : nullptr;
  }
  virtual const mat4f& model_matrix() const {
    return frame_ ? frame_->model_matrix() : kDefaultCameraModelMatrix;
  }
  virtual const absl::optional<mat4f>& camera_node_matrix() const {
    return frame_ ? frame_->camera_node_matrix() : kDefaultCameraNodeMatrix;
  }
  // This function returns the most up to date camera pose with its associated
  // timestamp. Unlike model matrix this pose is not sync'ed to the latest
  // available frame which is usually delayed by 40+ms.
  virtual absl::Status GetLatestModelMatrix(mat4f* matrix_out,
                                            absl::Time* timestamp_out) const {
    return session_native_->GetLatestModelMatrix(matrix_out, timestamp_out);
  }
  virtual const mat4& projection_matrix() const {
    return frame_ ? frame_->projection_matrix() : kDefaultProjectionMatrix;
  }

  // Returns a handle to the camera texture. In GL, this value should be
  // static_cast to a GLuint. On iOS, it is a CVPixelBufferRef.
  virtual const absl::optional<intptr_t> camera_texture_id() const {
    return frame_ ? absl::make_optional(frame_->camera_texture_id())
                  : absl::nullopt;
  }

  // Checks if the session has valid frame data. Empty frame data means the
  // session may not have started or is currently unreliable.
  virtual bool HasFrameData() const { return frame_.has_value(); }

  // Gets the time stamp of the current frame.
  virtual absl::optional<absl::Time> Timestamp() const {
    return frame_ ? absl::make_optional(frame_->timestamp()) : absl::nullopt;
  }

  // Gets all trackables in the session.
  // Note: these objects are only valid for the current frame - only the ID is
  // long-lived.
  template <typename T>
  const std::vector<ArTrackableHandle<T>> GetTrackables() {
    std::vector<ArTrackableHandle<T>> result;
    auto& map = std::get<TrackableMap<T>>(trackable_manager_.GetTuple());
    for (auto& pair : map) {
      result.push_back(ArTrackableHandle<T>(&trackable_manager_, pair.first));
    }
    return result;
  }

  // Visits all trackables in the session.
  // Note: these objects are only valid for the current frame - only the ID is
  // long-lived.
  template <typename T, typename Visitor>
  void VisitTrackables(Visitor&& visitor) {
    auto& map = std::get<TrackableMap<T>>(trackable_manager_.GetTuple());
    for (auto& pair : map) {
      visitor(pair.first, pair.second);
    }
  }

  // Gets a list of trackables that have changed this frame.
  // Note: these objects are only valid for the current frame - only the ID is
  // long-lived.
  template <typename T>
  std::vector<ArTrackableHandle<T>> GetUpdatedTrackables() {
    if (!frame_) {
      return {};
    }

    auto& updated_list = frame_->GetUpdatedTrackables<T>();
    std::vector<ArTrackableHandle<T>> result;
    for (auto trackable : updated_list) {
      result.push_back(
          ArTrackableHandle<T>(&trackable_manager_, trackable.GetId()));
    }
    return result;
  }

  template <typename T>
  const ArTrackableHandle<T> GetTrackableHandle(ArTrackableId id) {
    return ArTrackableHandle<T>(&trackable_manager_, id);
  }

  // Gets the tracking state of the AR Camera. If the state is anything other
  // than TRACKING the pose should not be considered useful. Changing the
  // active camera configuration may cause the tracking state on certain
  // devices to become permanently PAUSED. For consistent behavior across all
  // supported devices, release any previously created anchors and trackables
  // when setting a new camera config.
  virtual TrackingState GetCameraTrackingState() const {
    return session_native_->GetCameraTrackingState();
  }

  // Create an AR anchor, which notifies the AR tracking system that this
  // location is important (i.e. virtual content will be placed there and
  // should remain anchored to the real world). The optional ArTrackableId
  // lets the system know that your anchor should be associated with the given
  // real world trackable object (ex: an ArPlane) and its transform will more
  // closely track that of that object (ex: the height will always match the
  // ArPlane height).
  virtual absl::StatusOr<ArTrackableHandle<ArAnchor>> CreateAnchor(
      float3 position, quatf rotation, absl::optional<ArTrackableId> id);

  // Create an AR anchor, which notifies the AR tracking system that this
  // location is important (i.e. virtual content will be placed there and
  // should remain anchored to the real world). This version creates a "geo"
  // anchor based on lat/long/altitude with a scene/gl rotation.
  virtual absl::StatusOr<ArTrackableHandle<ArAnchor>> CreateAnchor(
      double latitude_degrees, double longitude_degrees,
      double wgs84_relative_altitude_meters, quatf rotation);

  // Creates an environment probe anchor, the environment probe anchor provides
  // lighting information when HDR lighting is enabled. Applies to iOS only,
  // for details see:
  // https://developer.apple.com/documentation/arkit/arenvironmentprobeanchor?language=objc
  virtual ArTrackableHandle<ArAnchor> CreateEnvironmentProbeAnchor(
      float3 position, quatf rotation);

  virtual void DestroyAnchor(ArTrackableHandle<ArAnchor> anchor);

  // Checks whether HDR lighting is enabled and should be used by getting
  // lighting from GetHdrLighting.
  virtual bool IsHdrLightingEnabled() const {
    return session_native_->IsHdrLightingEnabled();
  }

  virtual std::unique_ptr<HdrLighting> GetHdrLighting() {
    return session_native_->GetHdrLighting();
  }

  // Gets Spherical harmonics for the current scene according AR light
  // estimate. Returns an empty vector if the data is not available.
  virtual std::vector<float3> GetSphericalHarmonicsLighting() {
    return session_native_->GetSphericalHarmonicsLighting();
  }

  // Gets the latest ambient light estimation data for the current scene
  // according to the latest AR light estimate.  Returns nullptr if new scene
  // lighting is available.
  virtual float4 GetAmbientLighting() {
    return session_native_->GetAmbientLighting();
  }

  // Gets whether depth has been enabled on the AR session.
  virtual bool IsDepthSupported() const {
    return session_native_->IsDepthSupported();
  }

  // Sets the method by which hit tests for object placement are performed.
  virtual void SetPlacementMode(ArSessionConfig::PlacementMode mode) {
    session_native_->SetPlacementMode(mode);
  }

  // Gets the current tracking failure reason of the AR Camera.
  virtual TrackingFailureReason GetTrackingFailureReason() const {
    return session_native_->GetTrackingFailureReason();
  }

  // Sets the playback data to use for playback.
  void SetPlaybackScene(std::unique_ptr<ArPlaybackData> playback_data) {
    return session_native_->SetPlaybackScene(std::move(playback_data));
  }

  // Gets a set of estimated 3D points attached to real-world geometry
  // from the current frame.
  // Returns an empty vector on all platforms but Android.
  // TODO: implement for remaining platforms.
  virtual ArPointCloud GetPointCloud() const {
    return session_native_->GetPointCloud();
  }

  // Enables debug visualization such as drawing planes.
  void EnableDebugVisualization(BaseView* view);

  // Disables visualizations.
  void DisableDebugVisualization();

  // Returns true if a platform-level session interruption is in effect.
  bool IsSessionInterrupted() const;

  // Returns true when the session is ready to render.
  bool IsReadyToRender() const;

  // Starts recording the ar session to the specified uri file.
  virtual absl::Status StartRecording(absl::string_view dataset_uri) {
    return session_native_->StartRecording(dataset_uri);
  }

  // Stops the recording of the ar session.
  virtual absl::Status StopRecording() {
    return session_native_->StopRecording();
  }

 protected:
  // Can be called to override the session for testing.
  static void SetCreateArSessionOverride(
      CanCreateArSessionFn can_create_session_fn,
      CreateArSessionFn create_session_fn);

  explicit ArSession(std::unique_ptr<ArSessionNative> session_native);

  ArTrackablesManager* GetTrackableManager() { return &trackable_manager_; }

 private:
  // Gets a reference to the static CanCreateArSessionNative override function.
  static CanCreateArSessionFn& GetCanCreateArSessionOverrideFn();

  // Gets a reference to the static CreateArSessionNative override function.
  static CreateArSessionFn& GetCreateArSessionOverrideFn();

  void UpdateTrackableManager(const TrackableTuple& updates);

  ArTrackablesManager trackable_manager_;
  std::unique_ptr<ArSessionNative> session_native_;
  absl::optional<ArFrame> frame_;
  std::unique_ptr<DebugHelper> debug_helper_;
  absl::Time last_submitted_timestamp_;
  static constexpr mat4 kDefaultProjectionMatrix = mat4();
  static constexpr mat4f kDefaultCameraModelMatrix = mat4f();
  static constexpr absl::optional<mat4f> kDefaultCameraNodeMatrix =
      absl::nullopt;
};

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_SESSION_H_
