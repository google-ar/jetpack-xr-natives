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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_AR_AR_SCENE_CONTROLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_AR_AR_SCENE_CONTROLLER_H_

#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "absl/hash/hash.h"
#include "absl/types/optional.h"
#include "core/ar/ar_session.h"
#include "core/ar/ar_trackable.h"
#include "core/ar/ar_trackable_handle.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/node.h"
#include "core/ncsb/system.h"
#include "core/view/ar/ar_anchor_component.h"
#include "core/view/ar/ar_camera_renderer.h"
#include "core/view/ar/collision/collider_ar_plane.h"
#include "core/view/base_view.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/window/window_rotation.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

// A notification sent when a platform-level AR interruption begins or ends.
struct ArInterruptionEvent : public Event {
  // If true, the camera feed will not deliver new frames for platform security
  // reasons, e.g. due to multiple foregrounded apps.  If false, the
  // interruption is over and the camera feed will resume delivering new frames.
  bool is_now_interrupted;
};

// An event that is sent when the tracking state changes to tracking.
struct TrackingAcquiredEvent : public Event {};

// Provides high level access to the underlying AR platform. The
// ArSceneController initializes and updates an AR Session to populate a scene
// with AR objects, which provides interaction with AR level objects throughout
// the Imp platform.
// TODO: Add missing unit/scuba tests for this class.
class ArSceneController : public System {
 public:
  // Ensures that e.g. ARCore is installed and sufficiently recent.  This check
  // is done on a background thread to avoid strict mode violations.
  static Future<absl::Status> ArAvailability(
      const BaseView* view, const ar::ArSessionConfig& config = {});

  explicit ArSceneController(BaseView* view, std::string dataset_path = "",
                             bool enable_ar_plane_collision = true);
  ~ArSceneController() override;

  // Initializes the AR session with the currently active camera.
  imp::Future<std::weak_ptr<imp::ar::ArSession>> Setup(
      const ar::ArSessionConfig& config = ar::ArSessionConfig());
  // Initializes the AR session with the camera passed in.
  imp::Future<std::weak_ptr<imp::ar::ArSession>> Setup(
      ComponentHandle<CameraComponent> camera,
      const ar::ArSessionConfig& config = ar::ArSessionConfig());

  // Ticks the AR session, must be called to generate the next AR session frame.
  // Returns true if a new frame was generated, or if the ar session is not
  // running. The ar session is not running if it's either paused or has not yet
  // finished being setup. Otherwise, false is returned.
  //
  // The return value is used to skip rendering when we have no new ArFrame.
  //
  // Update is called automatically. This method is only public to support a C9
  // case where they call Update to force generation of the camera texture
  // early.
  // TODO: Find better solution for managing C9 camera texture.
  bool Update();

  // Dev-mode display of the active tracking state.
  void RenderDev();

  // Pauses the AR session if it is not already paused
  void Pause();

  // Resumes the AR session if it is not already resumed
  void Resume();

  // If and only if true, the camera image will be rendered to the screen.
  void SetVisible(bool visible);

  std::string GetArSessionId();

  // Gets a list of all trackables.
  template <typename T>
  std::vector<ar::ArTrackableHandle<T>> GetTrackables() const {
    if (!SessionReady()) {
      return {};
    }
    return ar_session_->GetTrackables<T>();
  }
  // Gets the AR camera tracking state.
  ar::TrackingState GetCameraTrackingState() {
    if (!SessionReady()) {
      return ar::TrackingState::kStopped;
    }
    return ar_session_->GetCameraTrackingState();
  }

  // Gets a pointer to the AR session.
  // Warning: will std::abort() if called before the session is ready.
  std::weak_ptr<ar::ArSession> GetSession() const;

  // Returns true if the session has been initialized.
  bool SessionReady() const;

  // Set the method by which hit tests for object placement are performed.
  void SetPlacementMode(imp::ar::ArSessionConfig::PlacementMode mode);

  // Performs a raycast against all tracked planes as of their last update.
  // When only_closest is true, the result will have one or zero eleents.
  // TODO This can be deprecated once ArSession::HitTest no longer
  // leaks points.
  std::vector<RayHit> ArPlaneRaycast(const Ray& world_ray,
                                     bool only_closest = true);

  // If true, the render pass will be skipped if the AR session yields a
  // duplicate camera frame.
  void SetSkipDuplicateFrames(bool skip_duplicate_frames) {
    skip_duplicate_frames_ = skip_duplicate_frames;
  }

  // Returns true when the session is ready and can render.
  bool IsReadyToRender() const;

  // Calls the given function the next time tracking is acquired.
  template <typename Fn, typename Owner>
  void CallWhenTracking(Fn fn, Owner owner) {
    if (GetCameraTrackingState() == ar::TrackingState::kTracking) {
      fn();
    } else {
      GetView().GetDispatcher().Connect(
          [fn = std::move(fn)](
              const TrackingAcquiredEvent& tracking_acquired_event) {
            fn();
            tracking_acquired_event.Disconnect();
          },
          owner);
    }
  }

 private:
  friend struct UpdateForEachType;
  // Apply updates from the ar session.
  void UpdateArTrackables();

  // Update texture stream from the devices camera.
  void UpdateCameraStream();

  // Tell the scene controller that a frame finished rendering.
  void OnPostRender();

  // Tell the scene controller (and any listeners) that a platform-level AR
  // interruption is beginning or ending.
  void SetIsInterrupted(bool next_interrupted);

  // Helper method creates the AR Session.
  std::shared_ptr<ar::ArSession> CreateArSession(
      const ar::ArSessionConfig& config);
  // Completes scene controller setup.
  void SetupInternal(ComponentHandle<CameraComponent> camera,
                     std::shared_ptr<ar::ArSession> session);

  // Helper method used when enable_ar_collision_ is false.
  NodeHandle GetNodeFromTrackable(ar::ArTrackableId id);

  ComponentHandle<CameraComponent> camera_;
  std::shared_ptr<ar::ArSession> ar_session_;
  imp::Future<std::weak_ptr<ar::ArSession>> ar_session_future_;
  NodeHandle camera_renderer_node_;
  absl::optional<Future<ComponentHandle<ArCameraRenderer>>>
      camera_renderer_future_;
  ar::TrackingState previous_tracking_state_;

  Dispatcher::ScopedConnection pre_frame_update_connection_;
  Dispatcher::ScopedConnection post_frame_update_connection_;
  Dispatcher::ScopedConnection size_changed_connection_;
  Dispatcher::ScopedConnection rotation_changed_connection_;
  Dispatcher::ScopedConnection transition_parameters_changed_connection_;
  bool should_display_geometry_change_;
  bool running_;
  bool skip_duplicate_frames_;
  bool is_interrupted_;
  bool enable_ar_plane_collision_;
  RobinMap<ar::ArTrackableHandle<ar::ArPlane>, NodeHandle> local_trackable_map_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_AR_AR_SCENE_CONTROLLER_H_
