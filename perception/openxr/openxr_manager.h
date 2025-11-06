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

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <memory>
#include <optional>
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

  // Enum returned by functions that call xrCreateAnchorSpaceANDROID which
  // provides details on the result returned by OpenXR.
  enum class CreateAnchorResult : int32_t {
    kSuccess = 0,               // XR_SUCCESS
    kErrorRuntimeFailure = -2,  // XR_ERROR_RUNTIME_FAILURE
    kErrorLimitReached = -10,   // XR_ERROR_LIMIT_REACHED
  };

  // Enum representing the configuration state for the plane trackers.
  enum class PlaneTrackingMode : uint8_t {
    kDisabled = 0x00,
    kHorizontalAndVertical = 0x01
  };

  // Enum representing the configuration state for the hand trackers.
  enum class HandTrackingMode : uint8_t { kDisabled = 0x00, kBoth = 0x01 };

  // Enum representing the configuration state for hand tracking.
  enum class HeadTrackingMode : uint8_t { kDisabled = 0x00, kLastKnown = 0x01 };

  // Enum representing the configuration state for depth estimation.
  enum class DepthEstimationMode : uint8_t {
    kDisabled = 0x00,
    kRawOnly = 0x01,
    kSmoothOnly = 0x02,
    kSmoothAndRaw = 0x03
  };

  // Enum representing the configuration state for anchor persistence.
  enum class AnchorPersistenceMode : uint8_t {
    kDisabled = 0x00,
    kLocal = 0x01
  };

  // Enum representing the configuration state for face tracking.
  enum class FaceTrackingMode : uint8_t { kDisabled = 0x00, kUser = 0x01 };

  // Enum representing the calibration state for face tracking.
  enum class FaceTrackingCalibrationState : uint8_t {
    kUnknown = 0x00,
    kServiceNotReady = 0x01,
    kNotCalibrated = 0x02,
    kCalibrated = 0x03,
  };

  enum class DepthImageBufferOrder : uint8_t {
    kLeftEyeImage = 0x00,
    kRightEyeImage = 0x01,
    kLeftEyeConfidenceImage = 0x02,
    kRightEyeConfidenceImage = 0x03
  };

  enum class ObjectTrackingMode : uint8_t { kDisabled = 0x00, kEnabled = 0x01 };

  enum class GeospatialMode : uint8_t {
    kDisabled = 0x00,
    kEnabled = 0x01,
  };

  enum class EarthState : int32_t {
    kRunning = 1,
    kStopped = 0,
    kErrorInternal = -1,
    kErrorNotAuthorized = -2,
    kErrorResourcesExhausted = -3,
    kErrorApkVersionTooOld = -4,
    kErrorAppPreempted = -5
  };

  enum class GeospatialPoseResult : int32_t {
    kSuccess = 0,
    kErrorInvalidArgument = -1,
    kErrorIllegalState = -2,
    kErrorNotTracking = -3,
  };

  enum class EyeTrackingMode : uint8_t {
    kDisabled = 0x00,
    kCoarse = 0x01,
    kFine = 0x02,
    kCoarseAndFine = 0x03
  };

  // Struct that contains the configuration settings that can be set at runtime
  // by passing to ConfigureSession().
  struct ConfigSettings {
    PlaneTrackingMode plane_tracking_mode = PlaneTrackingMode::kDisabled;
    HandTrackingMode hand_tracking_mode = HandTrackingMode::kDisabled;
    HeadTrackingMode head_tracking_mode = HeadTrackingMode::kDisabled;
    DepthEstimationMode depth_estimation_mode = DepthEstimationMode::kDisabled;
    AnchorPersistenceMode anchor_persistence_mode =
        AnchorPersistenceMode::kDisabled;
    FaceTrackingMode face_tracking_mode = FaceTrackingMode::kDisabled;
    ObjectTrackingMode object_tracking_mode = ObjectTrackingMode::kDisabled;
    EyeTrackingMode eye_tracking_mode = EyeTrackingMode::kDisabled;
    std::vector<XrObjectLabelANDROID> object_tracking_labels = {};
    GeospatialMode geospatial_mode = GeospatialMode::kDisabled;
  };

  // Struct that contains a depth image buffer and its size.
  struct DepthImageBuffer {
    void const* buffer;
    int buffer_size;
  };

  // Amount of time between calls xrPollEvent on the polling loop.
  const int32_t kNanosPerSecond = 1000000000;
  const timespec kPollingInterval = {0, kNanosPerSecond / 60};

  const int kFloatPerPosition = 3;
  const int kFloatPerQuaternion = 4;
  const int kFloatPerPose = kFloatPerQuaternion + kFloatPerPosition;
  const int kFloatPerFov = 4;
  const int kFaceTrackerStartupWaitMs = 50;
  const int kFaceTrackerStartupCheckMaxAttempts = 20;

  const size_t kHandJointsBufferSize =
      sizeof(int) + XR_HAND_JOINT_COUNT_EXT * kFloatPerPose * sizeof(float);

  explicit OpenXrManager(OpenXrManagerClockInterface* clock) : clock_(clock) {}

  // A static function to get a singleton for the OpenXR Manager.
  static OpenXrManagerClockInterface* GetOpenXrManagerClock();

  // A static function to get a singleton for the OpenXR Manager.
  static OpenXrManager& GetOpenXrManager();

  // A static function to get a singleton for the OpenXR Manager with the
  // provided clock.
  static OpenXrManager& GetOpenXrManager(OpenXrManagerClockInterface* clock);

  // Initializes the OpenXrManager. This is broken down into loading OpenXR,
  // creating an OpenXR instance, and creating a session from that instance. The
  // instance must be associated with one activity. This will return true if the
  // initialization was successful, or if the instance is already initialized.
  // If start_polling_loop is set to true, this function will also resume the
  // session polling loop regardless of previous state. start_polling_thread =
  // false is ignored if the polling thread is running; clients must call
  // PauseSession or DeInit to stop the polling thread after this function has
  // been called with start_polling_thread = true. The default_reference_space
  // is the reference space that will be used while querying OpenXR.
  // TODO: (broken link) -  Support multiple activities in the OpenXR manager.
  bool Init(JNIEnv* env, jobject activity,
            XrReferenceSpaceType default_reference_space =
                XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID,
            bool start_polling_thread = true) ABSL_LOCKS_EXCLUDED(mutex_);

  // Destroys the Session and Instance held by the OpenXrManager. Once destroyed
  // it can be reinitialized. If stop_polling_thread is true this will wait for
  // the polling thread to join. stop_polling_thread must be false when DeInit
  // is triggered from the polling thread. This can be called externally or
  // triggered by an event on the polling thread.
  void DeInit(bool stop_polling_thread = true) ABSL_LOCKS_EXCLUDED(mutex_);

  // Stops polling of OpenXR events by joining the polling thread and ends the
  // session. The polling thread must be recreated with Init().
  bool PauseSession() ABSL_LOCKS_EXCLUDED(mutex_);

  // Returns a vector containing tracked objects from the trackable tracker.
  std::vector<XrTrackableANDROID> GetTrackableObjects(XrTime time);

  // Gets the OpenXR data associated with the trackable object for the trackable
  // id at the specified time in the specified reference space. If the time is a
  // negative number, the current time will be used.
  // This function is thread safe.
  // Returns true if successful and populates the object_data. Returns false if
  // there was an error getting the object data.
  bool GetTrackableObjectState(
      XrTrackableANDROID object_id, XrReferenceSpaceType reference_space,
      XrTime time, XrTrackableObjectANDROID& out_object);

  // Returns a vector containing tracked planes from the trackable tracker.
  std::vector<XrTrackableANDROID> GetPlanes();

  // Gets the OpenXR data associated with the plane  for the trackable_id at the
  // specified time in the specified reference space. If the time is a negative
  // number, the current time will be used. This function is thread safe.
  // Returns true if successful and populates the plane_data. Returns false if
  // there was an error getting the plane data.
  bool GetPlaneState(XrTrackableANDROID plane_id,
                     XrReferenceSpaceType reference_space, XrTime time,
                     XrTrackablePlaneANDROID& out_plane,
                     std::vector<XrVector2f>& out_vertices);

  // Chooses a plane that fits the constraints from the vector of planes
  // provided.  If no suitable plane is found, this will return false and
  // out_trackable and out_plane will not be updated.
  bool ChoosePlane(const PlaneConstraints& plane_constraints,
                   XrTrackableANDROID* out_trackable,
                   XrTrackablePlaneANDROID* out_plane);

  // Creates an anchor at the pose provided in the default reference space.
  // Returns a CreateAnchorResult enum corresponding to whether the anchor was
  // loaded successfully and populated in out_anchor_space, or if the function
  // encountered an error.
  CreateAnchorResult CreateAnchor(XrTime time, const XrPosef& pose,
                                  XrSpace* out_anchor_space);

  // Creates an anchor at a point relative to the center point of the provided
  // trackable and plane. If the plane is null this will load the plane from
  // the trackable. This is thread safe. Returns a CreateAnchorResult enum
  // corresponding to whether the anchor was loaded successfully and populated
  // in out_anchor_space, or if the function encountered an error.
  CreateAnchorResult CreateAnchorForPlane(XrTrackableANDROID trackable,
                                          XrTrackablePlaneANDROID* plane,
                                          XrTime time,
                                          const XrPosef& relative_pose,
                                          XrSpace* out_anchor_space);

  // Creates a trackable object anchor at a point relative to the center point
  // of the provided trackable and object. If the object is null this will load
  // the object from the trackable. This is thread safe. Returns a
  // CreateAnchorResult enum corresponding to whether the anchor was loaded
  // successfully and populated in out_anchor_space, or if the function
  // encountered an error.
  CreateAnchorResult CreateAnchorForObject(XrTrackableANDROID trackable,
                                           XrTrackableObjectANDROID* object,
                                           XrTime time,
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

  // Gets the left and right views at the provided time in the default
  // reference space when head tracking is enabled and the values in the VIEW
  // reference space when head tracking is disabled. Returns true if successful
  // and populates the out_views. Returns false if there was an error.
  // This function only works if there are exactly 2 views and out_views must
  // have size=2. This is expected to be called from the jni thread.
  bool GetStereoViews(XrTime time, bool is_head_tracking_enabled,
                      std::vector<XrView>* out_views);

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
  // created. The call is thread-safe. Returns a CreateAnchorResult enum
  // corresponding to whether the anchor was loaded successfully and populated
  // in out_anchor_space, or if the function encountered an error.
  CreateAnchorResult LocatePersistedAnchorSpace(const XrUuidEXT& anchor_uuid,
                                                XrSpace* out_anchor_space)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Gets the UUIDs of all persisted anchors. The call is thread-safe.
  std::vector<XrUuidEXT> GetPersistedAnchorUuids() ABSL_LOCKS_EXCLUDED(mutex_);

  // Performs a raycast using the provided raycast_info and returns whether the
  // operation is successful. If it is, out_hit_results contains the hit results
  // of the raycast.
  bool HitTest(XrRaycastInfoANDROID* raycast_info,
               XrRaycastHitResultsANDROID* out_hit_results);

  // Locates the hand joints and wraps the data in a std::byte*. It also
  // manages the memory of the previous buffer and current buffer.
  std::byte* GetHandDataBuffer(bool is_left_hand, XrTime time);

  // Gets the face tracking state.
  XrResult GetFaceState(XrTime time, XrFaceStateANDROID* outFaceState,
                       std::vector<float>& out_blend_shape_values,
                       std::vector<float>& out_confidence_values);

  // Checks if the face tracker is calibrated.
  bool IsFaceTrackerCalibrated();

  // Gets eye tracking info.
  XrResult GetEyesInfo(XrTime time, XrEyesANDROID* out_eyes,
                       bool is_fine_tracking_mode);

  // Gets the smooth depth image from the depth swapchain. This is a public
  // function that is expected to be called from the jni thread.
  bool GetDepthImage(XrTime time, const float** out_smooth_depth_image,
                     int* out_image_width, int* out_image_height)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Gets the all depth images and confidence images from the depth swapchain
  // This is a public function that is expected to be called from the jni
  // thread.
  bool GetAllDepthImages(XrTime time,
                         std::vector<DepthImageBuffer>& out_image_buffers)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Gets the width of the depth image. This is a public function that is
  // expected to be called from the jni thread.
  int GetDepthImageWidth() ABSL_LOCKS_EXCLUDED(mutex_);

  // Gets the height of the depth image. This is a public function that is
  // expected to be called from the jni thread.
  int GetDepthImageHeight() ABSL_LOCKS_EXCLUDED(mutex_);

  // Gets the earth state. This is a public function that is expected to be
  // called from the jni thread.
  EarthState GetEarthState() ABSL_LOCKS_EXCLUDED(mutex_);

  // Locates a geospatial pose from a local pose. Returns a GeospatialPoseResult
  // enum corresponding to whether the operation was successful, or what type of
  // error occurred.
  GeospatialPoseResult LocateGeospatialPose(
      XrTime time, const XrPosef& pose,
      XrGeospatialPoseResultANDROIDX1* out_geospatial_pose_result)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Waits for the polling thread to finish.
  void JoinPollingThread();

  // Returns the current XrSession.
  XrSession GetXrSession();

  // Returns the current XrInstance.
  XrInstance GetXrInstance();

  // Makes changes to runtime resources based on the provided configuration
  // settings, including enabling/disabling trackers. This function fails fast
  // and attempts to revert all changes in the event of a failure, returning the
  // XrResult from the failed operation. If all changes were successful, returns
  // XR_SUCCESS.
  XrResult ConfigureSession(const ConfigSettings& new_config_settings)
      ABSL_LOCKS_EXCLUDED(mutex_);

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
  static const int CACHE_SIZE = 100;

  // Calls DeInit with the initialization_mutex_ already held.
  void DeInitWithLockHeld(bool stop_polling_thread = true)
      ABSL_SHARED_LOCKS_REQUIRED(initialization_mutex_);

  // Retrieves the XR system ID.
  bool GetXrSystem() ABSL_LOCKS_EXCLUDED(mutex_);

  // Gets the extensions to be loaded from the required and optional extensions.
  bool GetEnabledExtensions(std::vector<const char*>& enabled_exts);

  // Loads the OpenXR runtime.
  bool LoadOpenXr(jobject activity) ABSL_LOCKS_EXCLUDED(mutex_);

  // Creates an OpenXR instance. The OpenXR runtime must first be loaded by
  // calling LoadOpenXr.
  bool CreateInstance(jobject activity) ABSL_LOCKS_EXCLUDED(mutex_);

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

  // Creates a view space at origin of the VIEW reference space.
  bool CreateViewReferenceSpace()
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(initialization_mutex_);

  // Locates the hand joints and fills in the hand_joints.
  bool LocateHandJoints(bool is_left_hand, XrTime time,
                        XrHandJointLocationsEXT* hand_joints);

  // Allocates a buffer and cleans up the previous buffer.
  std::byte* PrepareHandDataBuffer(bool is_left_hand);

  // Fills in the hand data buffer with the hand joints.
  void FillInHandDataBuffer(std::byte* buffer,
                            XrHandJointLocationsEXT hand_joints);


  // Fills in the vector3 into the float buffer.
  void FillVector3IntoFloatBuffer(float* floatBuffer, XrVector3f vector);

  // Fills in the quaternion into the float buffer.
  void FillQuaternionIntoFloatBuffer(
      float* floatBuffer, XrQuaternionf quaternion);

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
  //                  trackables and MaybeCreateHandTrackers to start tracking
  //                  hands.
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

  // Sets all configuration functions for the passed config_settings_, returning
  // an early XrResult in case of a failure, or XR_SUCCESS if all configurations
  // succeeded.
  XrResult ConfigureFeatures(const ConfigSettings& new_config_settings)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Sets all configuration functions for the current config_settings_ to revert
  // any in-flight changes.
  void AbortConfigureSession() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Initializes or destroys the plane tracker depending on the mode.
  XrResult ConfigurePlaneTracking(PlaneTrackingMode mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Initializes or destroys the object tracker depending on the mode.
  XrResult ConfigureObjectTracking(
      const ObjectTrackingMode& object_tracking_mode,
      const std::vector<XrObjectLabelANDROID>& object_tracking_labels)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Initializes or destroys the hand trackers depending on the mode.
  XrResult ConfigureHandTracking(HandTrackingMode mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Initializes or destroys the view space depending on the mode.
  XrResult ConfigureHeadTracking(HeadTrackingMode mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Initializes or destroys the depth estimation handlers depending on the
  // mode.
  XrResult ConfigureDepthEstimation(DepthEstimationMode mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Initializes or destroys the anchor persistence handle depending on the
  // mode.
  XrResult ConfigureAnchorPersistence(AnchorPersistenceMode mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Initializes or destroys the face tracker depending on the mode.
  XrResult ConfigureFaceTracking(FaceTrackingMode mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Initializes or stops the earth tracker depending on the mode.
  XrResult ConfigureEarthTracking(GeospatialMode mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Initializes or destroys the eye tracking depending on the mode.
  XrResult ConfigureEyeTracking(EyeTrackingMode mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Creates a planes tracker. This is used to search for and keep track of
  // the plane trackables. This will be called when the session becomes in
  // focus or whenever planes are queried if it does not already exist. It
  // should be kept alive as long as we expect to use trackables (which for now
  // is the entire session). Only one trackable tracker is required for the
  // duration of the session.
  XrResult MaybeCreatePlanesTracker() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Creates the left and right hand trackers if they are not already created.
  XrResult MaybeCreateHandTrackers() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Creates the face tracker if it is not already created.
  XrResult MaybeCreateFaceTracker() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Creates the object tracker if it is not already created.
  XrResult MaybeCreateObjectTracker() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Creates the earth tracker if it is not already created.
  XrResult MaybeCreateEarthTracker() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Creates the eye tracker if it is not already created.
  XrResult MaybeCreateEyeTracker() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Creates persistence_handle_ if it is not already created. Returns
  // XR_SUCCESS if the handle is created or has already been created before.
  // Returns a negative XR_RESULT if there was an error creating the handle.
  // This should be called by any functions that expect valid
  // persistence_handle_.
  XrResult CreatePersistenceHandleIfNecessary()
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Creates depth_swapchain_handle_ if it is not already created. Returns
  // XR_SUCCESS if the handle is created or has already been created before.
  // Returns a negative XR_RESULT if there was an error creating the handle.
  // This should be called by any functions that expect valid
  // depth_swapchain_handle_. This is a function that is expected to be called
  // from the jni thread before GetDepthImage is called.
  XrResult CreateDepthSwapchainIfNecessary(DepthEstimationMode mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Populates the DepthImageBuffer returned by the OpenXR manager. This
  // function is used by GetAllDepthImages.
  void PopulateDepthImageBuffer(
      std::vector<DepthImageBuffer>& out_image_buffers,
      const float* image_buffers, const uint8_t* confidence_image_buffers)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

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
  XrTrackableTrackerANDROID object_trackable_tracker_ ABSL_GUARDED_BY(mutex_) =
      XR_NULL_HANDLE;
  XrHandTrackerEXT left_hand_tracker_ ABSL_GUARDED_BY(mutex_) = XR_NULL_HANDLE;
  XrHandTrackerEXT right_hand_tracker_ ABSL_GUARDED_BY(mutex_) = XR_NULL_HANDLE;
  XrHandJointLocationEXT
      left_hand_joint_locations_[XR_HAND_JOINT_COUNT_EXT] ABSL_GUARDED_BY(
          mutex_);
  XrHandJointLocationEXT
      right_hand_joint_locations_[XR_HAND_JOINT_COUNT_EXT] ABSL_GUARDED_BY(
          mutex_);
  XrFaceTrackerANDROID face_tracker_ ABSL_GUARDED_BY(mutex_) = XR_NULL_HANDLE;
  XrEarthTrackerANDROIDX1 earth_tracker_ ABSL_GUARDED_BY(mutex_) =
      XR_NULL_HANDLE;
  std::optional<XrEventDataEarthTrackerStateChangedANDROIDX1>
      last_earth_tracker_state_update_ ABSL_GUARDED_BY(mutex_) = std::nullopt;
  XrEyeTrackerANDROID eye_tracker_ ABSL_GUARDED_BY(mutex_) = XR_NULL_HANDLE;
  FaceTrackingCalibrationState face_tracker_calibration_state_
      ABSL_GUARDED_BY(mutex_) = FaceTrackingCalibrationState::kUnknown;
  std::byte* left_hand_joint_poses_buffer_[CACHE_SIZE] ABSL_GUARDED_BY(
      mutex_) = {nullptr};
  std::byte* right_hand_joint_poses_buffer_[CACHE_SIZE] ABSL_GUARDED_BY(
      mutex_) = {nullptr};
  int left_hand_joint_buffer_index_ ABSL_GUARDED_BY(mutex_) = 0;
  int right_hand_joint_buffer_index_ ABSL_GUARDED_BY(mutex_) = 0;
  XrDeviceAnchorPersistenceANDROID persistence_handle_ ABSL_GUARDED_BY(mutex_) =
      XR_NULL_HANDLE;
  XrDepthSwapchainANDROID depth_swapchain_handle_ ABSL_GUARDED_BY(mutex_) =
      XR_NULL_HANDLE;
  std::vector<XrDepthSwapchainImageANDROID> depth_images_
      ABSL_GUARDED_BY(mutex_);
  XrDepthCameraResolutionANDROID supported_depth_resolution_
      ABSL_GUARDED_BY(mutex_);
  // The depth image buffer count is dependant on the configuration.
  int depth_image_buffer_count_ ABSL_GUARDED_BY(mutex_) = 0;
  int depth_image_width_ ABSL_GUARDED_BY(mutex_) = 0;
  int depth_image_height_ ABSL_GUARDED_BY(mutex_) = 0;
  // depth data buffer sizes and element counts are dependent on the depth image
  // resolution.
  size_t depth_data_image_buffer_size_ ABSL_GUARDED_BY(mutex_) = 0;
  size_t depth_data_confidence_image_buffer_size_ ABSL_GUARDED_BY(mutex_) = 0;
  size_t depth_data_image_num_elements_ ABSL_GUARDED_BY(mutex_) = 0;

  std::vector<XrTrackableANDROID> all_plane_trackables_;
  OpenXrState open_xr_state_ ABSL_GUARDED_BY(mutex_) =
      OpenXrState::kUninitialized;

  XrTrackableObjectConfigurationANDROID object_tracking_config_
      ABSL_GUARDED_BY(mutex_);

  OpenXrManagerClockInterface* clock_;

  // This is created after the session is initialized. Internally it
  // periodically calls xrPollEvent, which will result in calls to
  // HandleSessionChangedEvent.
  std::unique_ptr<std::thread> polling_thread_ = nullptr;

  // This flag controls whether or not the polling thread should still run.
  bool stop_polling_ ABSL_GUARDED_BY(mutex_) = false;

  // An object that contains the current state of the runtime configuration.
  ConfigSettings config_settings_ ABSL_GUARDED_BY(mutex_);

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

  // Loaded OpenXR functions.
  PFN_xrVoidFunction convert_time_;
  PFN_xrCreateTrackableTrackerANDROID create_trackable_tracker_;
  PFN_xrGetAllTrackablesANDROID get_all_trackables_;
  PFN_xrGetTrackablePlaneANDROID get_trackable_plane_;
  PFN_xrGetTrackableObjectANDROID get_trackable_object_;
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

  PFN_xrCreateHandTrackerEXT create_hand_tracker_;
  PFN_xrDestroyHandTrackerEXT destroy_hand_tracker_;
  PFN_xrLocateHandJointsEXT locate_hand_joints_;

  PFN_xrCreateFaceTrackerANDROID create_face_tracker_;
  PFN_xrDestroyFaceTrackerANDROID destroy_face_tracker_;
  PFN_xrGetFaceCalibrationStateANDROID get_face_calibration_state_;
  PFN_xrGetFaceStateANDROID get_face_state_;

  PFN_xrCreateDepthSwapchainANDROID create_depth_swapchain_;
  PFN_xrDestroyDepthSwapchainANDROID destroy_depth_swapchain_;
  PFN_xrEnumerateDepthSwapchainImagesANDROID enumerate_depth_swapchain_images_;
  PFN_xrEnumerateDepthResolutionsANDROID enumerate_depth_resolutions_;
  PFN_xrAcquireDepthSwapchainImagesANDROID acquire_depth_swapchain_images_;

  PFN_xrCreateEarthTrackerANDROIDX1 create_earth_tracker_;
  PFN_xrDestroyEarthTrackerANDROIDX1 destroy_earth_tracker_;
  PFN_xrLocateGeospatialPoseANDROIDX1 locate_geospatial_pose_;

  PFN_xrCreateEyeTrackerANDROID create_eye_tracker_;
  PFN_xrDestroyEyeTrackerANDROID destroy_eye_tracker_;
  PFN_xrGetFineTrackingEyesInfoANDROID get_fine_tracking_eyes_info_;
  PFN_xrGetCoarseTrackingEyesInfoANDROID get_coarse_tracking_eyes_info_;
};
}  // namespace androidx::xr::openxr
#endif  // JETPACK_XR_NATIVES_OPENXR_OPENXR_MANAGER_H_
