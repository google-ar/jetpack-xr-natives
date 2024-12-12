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

#ifndef JETPACK_XR_NATIVES_OPENXR_OPENXR_MANAGER_H_
#define JETPACK_XR_NATIVES_OPENXR_OPENXR_MANAGER_H_
#include <jni.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <cstdint>
#include <ctime>
#include <memory>
#include <thread>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/numeric/int128.h"
#include "absl/synchronization/mutex.h"
#include "openxr/openxr_manager_clock.h"

namespace androidx::xr::openxr {

// A class for managing an OpenXR session. To start the session call Init. Only
// one session can be active at a time.
// This class is thread compatible.
class OpenXrManager {
 public:
  // Plane Constraints used when searching for a plane.
  struct PlaneConstraints {
    // The minimum width of the plane. The selected plane must have a width
    // greater than or equal to min_width.
    float min_width;
    // The minimum height of the plane. The selected plane must have a height
    // greater than or equal to min_height.
    float min_height;
    //  The type of plane. The selected plane type must be equal to the type
    //  provided unless the provided type is XR_PLANE_TYPE_ARBITRARY_ANDROID in
    //  which case any type is acceptable.
    XrPlaneTypeANDROID type;
    // The label of the plane. The selected plane label must be equal to the
    // label provided unless the provided label is
    // XR_PLANE_LABEL_UNKNOWN_ANDROID in which case any label is acceptable.
    XrPlaneLabelANDROID label;
  };

  // Amount of time between calls xrPollEvent on the polling loop.
  const int32_t kNanosPerSecond = 1000000000;
  const timespec kPollingInterval = {0, kNanosPerSecond / 60};

  explicit OpenXrManager(OpenXrManagerClockInterface* clock) : clock_(clock) {}

  // A static function to get a singleton for the OpenXR Manager.
  static OpenXrManagerClockInterface* GetOpenXrManagerClock();

  // A static function to get a singleton for the OpenXR Manager.
  static OpenXrManager& GetOpenXrManager();

  // A static function to get a singleton for the OpenXR Manager with the
  // provided clock.
  static OpenXrManager& GetOpenXrManager(OpenXrManagerClockInterface* clock);

  // Initializes the OpenXrManager. This is broken down into loading OpenXR,
  // creating an OpenXR instance, and creating a session from that instance,
  // then starting the polling loop. The instance must be associated with one
  // activity. This will return true if the instance is already initialized and
  // will resume polling if paused. The default_reference_space is the reference
  // space that will be used while querying OpenXR.
  // TODO: b/328505653 -  Support multiple activities in the OpenXR manager.
  bool Init(JNIEnv* env, jobject activity,
            XrReferenceSpaceType default_reference_space =
                XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Destroys the Session and Instance held by the OpenXrManager. Once destroyed
  // it can be reinitialized. If stop_polling_thread is true this will wait for
  // the polling thread to join. stop_polling_thread must be false when DeInit
  // is triggered from the polling thread. This can be called externally or
  // triggered by an event on the polling thread.
  void DeInit(bool stop_polling_thread = true) ABSL_LOCKS_EXCLUDED(mutex_);

  // Stops polling of OpenXR events by joining the polling thread and ends the
  // session. The polling thread must be recreated with Init().
  bool PauseSession() ABSL_LOCKS_EXCLUDED(mutex_);

  // Returns a vector containing tracked planes from the trackable tracker.
  std::vector<XrTrackableANDROID> GetPlanes();

  // Gets the OpenXR data associated with the plane  for the trackable_id at the
  // specified time in the specified reference space. If the time is a negative
  // number, the current time will be used. This function is thread safe.
  // Returns true if successful and populates the plane_data. Returns false if
  // there was an error getting the plane data.
  bool GetPlaneState(XrTrackableANDROID plane_id,
                     XrReferenceSpaceType reference_space, XrTime time,
                     XrTrackablePlaneANDROID& out_plane);

  // Chooses a plane that fits the constraints from the vector of planes
  // provided.  If no suitable plane is found, this will return false and
  // out_trackable and out_plane will not be updated.
  bool ChoosePlane(const PlaneConstraints& plane_constraints,
                   XrTrackableANDROID* out_trackable,
                   XrTrackablePlaneANDROID* out_plane);

  // Creates an anchor at the pose provided in the default reference space.
  // Returns true if the XrSpace for the anchor was created successfully.
  // Returns false if there was an error creating the anchor space.
  bool CreateAnchor(XrTime time, const XrPosef& pose,
                    XrSpace* out_anchor_space);

  // Creates an anchor at a point relative to the center point of the provided
  // trackable and plane. If the plane is null this will load the plane from
  // the trackable. This is thread safe. Returns true if successful and
  // populates the out_anchor_space. Returns false if there was an error
  // creating the anchor.
  bool CreateAnchorForPlane(XrTrackableANDROID trackable,
                            XrTrackablePlaneANDROID* plane, XrTime time,
                            const XrPosef& relative_pose,
                            XrSpace* out_anchor_space);

  // Returns the OpenXR location data associated with an anchor space at a
  // specified time. May return a location that is invalid or untracked, as
  // indicated by the flag bits. Returns true if successful and populates the
  // out_anchor_space_location. Returns false if there was an error getting the
  // anchor space location.
  //
  // out_anchor_location must be allocated by the caller and must specify the
  // type as XR_TYPE_SPACE_LOCATION for the call to be successful.
  bool GetAnchorLocationData(XrSpace anchor_space, XrTime time,
                             XrSpaceLocation* out_anchor_location);

  // Exports the provided anchor. Returns true if successful and populates the
  // out_anchor_token. Returns false if there was an error exporting the anchor.
  // This is expected to be called from the jni thread through
  // CreateSemanticAnchor and possibly directly in the future.
  bool ExportAnchor(XrSpace anchor_space, AIBinder** out_anchor_token);

  // Creates and exports an anchor on the center pose of a plane that fulfills
  // the provided constraints. Returns true if successful and populates the
  // out_anchor_token. Returns false if no suitable plane was found or there was
  // an error creating or exporting the anchor. This is a public function that
  // is expected to be called from the jni thread.
  bool CreateSemanticAnchor(const PlaneConstraints& plane_constraints,
                            AIBinder** out_anchor_token,
                            XrSpace* out_anchor_space);

  // Destroys the XrSpace for the provided anchor and removes if from
  // anchor_map_. Returns false if there was an error destroying the anchor or
  // there was no anchor associated with the anchor ID. This should be called
  // from the jni thread.
  bool DestroyAnchor(XrSpace anchor_space);

  // Gets the head pose in the default reference space at the provided time.
  // Returns true if successful and populates the out_pose. Returns false if
  // there was an error getting the head pose. This is expected to be called
  // from the jni thread.
  bool GetHeadPose(XrTime time, XrPosef* out_pose);

  // Gets the left and right views in the default reference space at the
  // provided time. Returns true if successful and populates the out_views.
  // Returns false if there was an error. This function only works if there are
  // exactly 2 views and out_views must have size=2. This is expected to be
  // called from the jni thread.
  bool GetStereoViews(XrTime time, std::vector<XrView>* out_views);

  // Returns the current time in XrTime. This is used to get a time to get
  // trackables from the trackable tracker. It can be called from any thread.
  // It is recommended that calls within a single frame use a single XrTime for
  // that frame.  This is to ensure that all things queried in that frame are
  // predicted / evaluated for the same XrTime and are therefore in sync with
  // each other.
  XrTime GetXrTimeNow() const;

  // Returns the XrTime for a given time in nanoseconds. It can be called from
  // any thread. It is recommended that calls within a single frame use a single
  // XrTime for that frame.  This is to ensure that all things queried in that
  // frame are predicted / evaluated for the same XrTime and are therefore in
  // sync with each other.
  XrTime GetXrTimeFromNanoseconds(int64_t time_ns) const;

  // Returns the XrTime for a given timespec. It can be called from any thread.
  // It is recommended that calls within a single frame use a single XrTime for
  // that frame.  This is to ensure that all things queried in that frame are
  // predicted / evaluated for the same XrTime and are therefore in sync with
  // each other.
  XrTime GetXrTimeFromTimespec(const timespec& timespec_time) const;

  // Persists the anchor defined by `anchor_space` and returns whether the query
  // is sent successful. If it's, out_anchor_uuid contains the UUID that user
  // could save to recreate the anchor. The call is thread-safe.
  bool PersistAnchor(XrSpace anchor_space, XrUuidEXT* out_anchor_uuid)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Gets the persistent state by "anchor_uuid" and returns whether
  // the operation is successful. If it is, out_persist_state contains
  // the persistent state of the anchor. This call is thread-safe.
  bool GetAnchorPersistState(const XrUuidEXT& anchor_uuid,
                             XrAnchorPersistStateANDROID* out_persist_state)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Unpersists the anchor and returns whether the operation is successful. The
  // call is thread-safe.
  bool UnpersistAnchor(const XrUuidEXT& anchor_uuid)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Locates an anchor persisted in the previous sessions using the
  // `anchor_uuid`. The anchor space will be created if it is not already
  // created. If the anchor is located successfully, out_anchor_space contains
  // the anchor space. The call is thread-safe.
  bool LocatePersistedAnchorSpace(const XrUuidEXT& anchor_uuid,
                                  XrSpace* out_anchor_space)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Gets the UUIDs of all persisted anchors. The call is thread-safe.
  std::vector<XrUuidEXT> GetPersistedAnchorUuids() ABSL_LOCKS_EXCLUDED(mutex_);

  bool HitTest(XrRaycastInfoANDROID* raycast_info,
               XrRaycastHitResultsANDROID* out_hit_results);

  // Gets the smooth depth image from the depth swapchain. This is a public
  // function that is expected to be called from the jni thread.
  bool GetDepthImage(XrTime time, const float** out_smooth_depth_image,
                     int* out_image_width, int* out_image_height)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Waits for the polling thread to finish.
  void JoinPollingThread();

  // Returns the current XrSession.
  XrSession GetXrSession();

  // Returns the current XrInstance.
  XrInstance GetXrInstance();

 private:
  // Enum values representing whether OpenXR instance and session have started.
  // This is separate from the XrSessionState which handles states of the
  // session once it has been initialized.
  enum class OpenXrState : uint8_t {
    kUninitialized,
    kInitializing,
    kResumed,
    kPaused,
    kUninitializing
  };

  OpenXrManager(const OpenXrManager&) = delete;
  OpenXrManager& operator=(const OpenXrManager&) = delete;

  inline static OpenXrManager* manager_;

  // Calls DeInit with the initialization_mutex_ already held.
  void DeInitWithLockHeld(bool stop_polling_thread = true)
      ABSL_SHARED_LOCKS_REQUIRED(initialization_mutex_);

  // Retrieves the XR system ID.
  bool GetXrSystem() ABSL_LOCKS_EXCLUDED(mutex_);

  // Loads the OpenXR runtime.
  bool LoadOpenXr() ABSL_LOCKS_EXCLUDED(mutex_);

  // Creates an OpenXR instance. The OpenXR runtime must first be loaded by
  // calling LoadOpenXr.
  bool CreateInstance() ABSL_LOCKS_EXCLUDED(mutex_);

  // Creates an OpenXR session. An instance must first be created by calling
  // CreateInstance.
  bool CreateSession() ABSL_LOCKS_EXCLUDED(mutex_);

  // Loads in the OpenXR extension functions needed for functionality. This
  // should be called from Init by the main thread.
  bool InitExtensionFunctions() ABSL_LOCKS_EXCLUDED(mutex_);

  // Creates a reference space at origin of the STAGE reference space type. This
  // should be called from Init and is used as a reference point when finding
  // planes.
  bool CreateStageReferenceSpace();

  // Creates a reference space at origin of the UNBOUNDED reference space type.
  // This should be called from Init and is used as a reference point when
  // finding planes.
  bool CreateUnboundedReferenceSpace();

  // Creates a view space at origin of the VIEW reference space type. This
  // should be called from Init and is used to get the head pose.
  bool CreateViewReferenceSpace();

  // Returns a space representing identity in the provided reference space type.
  XrSpace GetSpaceInReferenceSpace(XrReferenceSpaceType space_type);

  // Returns a space representing identity in the default reference space type.
  XrSpace GetSpaceInDefaultReferenceSpace();

  // Handles the session changed event. This will update the session state based
  // on changes form the runtime. It should only be called by the polling thread
  // in response to a SESSION_STATE_CHANGED event. Currently this handles
  // the following session state changes:
  //   READY - Calls xrBeginSession to start the session
  //   STOPPING - Calls xrEndSession to set the session state to IDLE
  //   EXITING - Calls xrDestroySession to end the session, and DeInit to clear
  //             out the instance data from the manager.
  //   SYNCHRONIZED - Calls MaybeCreateTrackableTracker to start looking for
  //                  trackables.
  void HandleSessionChangedEvent(
      const XrEventDataSessionStateChanged& changed_event);

  // Starts a polling thread. This will be used to check for OpenXR for events.
  void StartPollingThread() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // A loop that calls into OpenXR to get events. This should only be called
  // from the polling thread.
  void PollingLoop() ABSL_LOCKS_EXCLUDED(mutex_);

  // Polls OpenXR for events.
  void PollOpenXR() ABSL_LOCKS_EXCLUDED(mutex_);

  // Checks if polling has stopped.
  bool ShouldPoll() const ABSL_LOCKS_EXCLUDED(mutex_);

  // Creates a trackable tracker. This is used to search for and keep track of
  // the current trackables. This will be called when the session becomes in
  // focus or whenever planes are queried if it does not already exist. It
  // should be kept alive as long as we expect to use trackables (which for now
  // is the entire session). Only one trackable tracker is required for the
  // duration of the session.
  bool MaybeCreateTrackableTracker() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Creates persistence_handle_ if it is not already created. Returns true if
  // the handle is created or has already been created before. Returns false if
  // there was an error creating the handle. This should be called by any
  // functions that expect valid persistence_handle_.
  bool CreatePersistenceHandleIfNecessary()
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Creates depth_swapchain_handle_ if it is not already created. Returns true
  // if the handle is created or has already been created before. Returns false
  // if there was an error creating the handle. This should be called by any
  // functions that expect valid depth_swapchain_handle_. This is a function
  // that is expected to be called from the jni thread before GetDepthImage is
  // called.
  bool CreateDepthSwapchainIfNecessary() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  XrInstance instance_ ABSL_GUARDED_BY(mutex_) = XR_NULL_HANDLE;
  XrSystemId system_id_ ABSL_GUARDED_BY(mutex_) = XR_NULL_SYSTEM_ID;
  XrSession session_ ABSL_GUARDED_BY(mutex_) = XR_NULL_HANDLE;
  XrReferenceSpaceType default_reference_space_ =
      XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID;
  XrSpace stage_space_ = XR_NULL_HANDLE;
  XrSpace view_space_ = XR_NULL_HANDLE;
  XrSpace unbounded_space_ = XR_NULL_HANDLE;
  XrTrackableTrackerANDROID planes_trackable_tracker_ ABSL_GUARDED_BY(mutex_) =
      XR_NULL_HANDLE;
  XrDeviceAnchorPersistenceANDROID persistence_handle_ ABSL_GUARDED_BY(mutex_) =
      XR_NULL_HANDLE;
  // absl::uint128 is a substitute for XrUuidExt.
  absl::flat_hash_map<absl::uint128, XrSpace> persist_anchor_uuid_to_space_map_
      ABSL_GUARDED_BY(mutex_);
  XrDepthSwapchainANDROID depth_swapchain_handle_ ABSL_GUARDED_BY(mutex_);
  std::vector<XrDepthSwapchainImageANDROID> depth_images_
      ABSL_GUARDED_BY(mutex_);
  XrDepthCameraResolutionANDROID supported_depth_resolution_
      ABSL_GUARDED_BY(mutex_);

  std::vector<XrTrackableANDROID> all_plane_trackables_;
  OpenXrState open_xr_state_ ABSL_GUARDED_BY(mutex_) =
      OpenXrState::kUninitialized;

  OpenXrManagerClockInterface* clock_;

  // This is created after the session is initialized. Internally it
  // periodically calls xrPollEvent, which will result in calls to
  // HandleSessionChangedEvent.
  std::unique_ptr<std::thread> polling_thread_ = nullptr;

  // This flag controls whether or not the polling thread should still run.
  bool stop_polling_ ABSL_GUARDED_BY(mutex_) = false;

  // Mutex to guard variables that are accessible by the polling thread. It must
  // be called after the initialization_mutex_.
  mutable absl::Mutex mutex_ ABSL_ACQUIRED_AFTER(initialization_mutex_);

  // Mutex to guard openXR initialization. This is enables a thread to
  // wait for an initialization started by another thread.It will then try to
  // connect to that session. It prevents an OpenXR session from being
  // initialized while OpenXR the OpenXR session is being deinitialized. Note
  // that our DeInit can be called from the polling thread as well as an app
  // thread. We use a shared lock while Deinitializing to enable multiple
  // threads to deinitialize OpenXR at the same time. We still lock the state
  // variables with mutex_.
  mutable absl::Mutex initialization_mutex_;

  JNIEnv* java_env_ = nullptr;
  JavaVM* app_vm_ = nullptr;
  jobject activity_ = {};

  // Loaded OpenXR functions.
  PFN_xrVoidFunction convert_time_;
  PFN_xrCreateTrackableTrackerANDROID create_trackable_tracker_;
  PFN_xrGetAllTrackablesANDROID get_all_trackables_;
  PFN_xrGetTrackablePlaneANDROID get_trackable_plane_;
  PFN_xrDestroyTrackableTrackerANDROID destroy_trackable_tracker_;
  PFN_xrCreateAnchorSpaceANDROID create_anchor_space_;
  PFN_xrShareAnchorANDROID share_anchor_;

  PFN_xrCreateDeviceAnchorPersistenceANDROID create_device_anchor_persistence_;
  PFN_xrDestroyDeviceAnchorPersistenceANDROID
      destroy_device_anchor_persistence_;
  PFN_xrPersistAnchorANDROID persist_anchor_;
  PFN_xrEnumeratePersistedAnchorsANDROID enumerate_persisted_anchors_;
  PFN_xrGetAnchorPersistStateANDROID get_anchor_persist_state_;
  PFN_xrUnpersistAnchorANDROID unpersist_anchor_;
  PFN_xrCreatePersistedAnchorSpaceANDROID create_persisted_anchor_space_;

  PFN_xrRaycastANDROID raycast_;

  PFN_xrCreateDepthSwapchainANDROID create_depth_swapchain_;
  PFN_xrDestroyDepthSwapchainANDROID destroy_depth_swapchain_;
  PFN_xrEnumerateDepthSwapchainImagesANDROID enumerate_depth_swapchain_images_;
  PFN_xrEnumerateDepthResolutionsANDROID enumerate_depth_resolutions_;
  PFN_xrAcquireDepthSwapchainImagesANDROID acquire_depth_swapchain_images_;
};
}  // namespace androidx::xr::openxr
#endif  // JETPACK_XR_NATIVES_OPENXR_OPENXR_MANAGER_H_
