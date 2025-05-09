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

#include "core/ar/android/ar_session_native_arcore.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#include <functional>
#include <memory>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "third_party/arcore/ar/core/c_api/arcore_c_api.h"
#include "third_party/arcore/ar/core/c_api/pose.h"
#include "core/ar/android/ar_core_ptrs.h"
#include "core/ar/android/deeplight_controller.h"
#include "core/ar/ar_magical_surface_point.h"
#include "core/lighting/environment_light_factory.h"
#if IMP_PLATFORM(ANDROID)
#include "core/ar/ar_prior_map.h"
#endif
#include "core/ar/ar_trackable.h"
#include "core/common/filament_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/common/trace.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "filament/filament/include/filament/Stream.h"
#include "filament/filament/include/filament/Texture.h"

// Emulates ArCores public to private Api remapping.
// TODO Remove these macros when ArCore 1.17 release.
#if ARCORE_FEATURE_ENABLED(depth_api_private)
#undef ArSession_isDepthModeSupported
#define ArSession_isDepthModeSupported ArSession_isDepthModeSupported_private

#undef ArFrame_acquireDepthImage
#define ArFrame_acquireDepthImage ArFrame_acquireDepthImage_private

#undef ArFrame_getDepthRegionConfidence
#define ArFrame_getDepthRegionConfidence \
  ArFrame_getDepthRegionConfidence_private

#undef ArConfig_setDepthMode
#define ArConfig_setDepthMode ArConfig_setDepthMode_private

#undef ArConfig_getDepthMode
#define ArConfig_getDepthMode ArConfig_getDepthMode_private

#undef ArConfig_setMagicalSurfaceHitTestMode
#define ArConfig_setMagicalSurfaceHitTestMode \
  ArConfig_setMagicalSurfaceHitTestMode_private

#undef ArMagicalSurfacePoint_getOrientationMode
#define ArMagicalSurfacePoint_getOrientationMode \
  ArMagicalSurfacePoint_getOrientationMode_private
#endif

// TODO Remove these after 1.19 release.
#if ARCORE_FEATURE_ENABLED(instant_placement_config_private)
#undef ArConfig_setInstantPlacementMode
#define ArConfig_setInstantPlacementMode \
  ArConfig_setInstantPlacementMode_private

#undef ArConfig_getInstantPlacementMode
#define ArConfig_getInstantPlacementMode \
  ArConfig_getInstantPlacementMode_private
#endif

namespace imp {
namespace ar {

// Helper methods
namespace {
UniqueArPose CreateUniqueArPose(
    const UniqueArSession& ar_session,
    absl::optional<const std::array<float, 7>> pose_raw) {
  IMP_TRACE();
  ArPose_* ar_pose;
  ArPose_create(ar_session.get(),
                pose_raw.has_value() ? pose_raw->data() : nullptr, &ar_pose);
  if (!ar_pose) {
    IMP_LOG(imp::FATAL) << "Failed to create ArPose!";
  }
  return UniqueArPose(ar_pose);
}

// Helper method for extracting a transform from a pose.
mat4f GetTransformFromPose(const UniqueArSession& ar_session, ArPose_* pose) {
  IMP_TRACE();
  mat4f transform;
  ArPose_getMatrix(ar_session.get(), pose, transform[0].v);
  return transform;
}

// Helper method for extracting specialized pose data from trackable types.
mat4f GetPose(const UniqueArSession& ar_session, const UniqueArPose& ar_pose,
              ArTrackable_* trackable) {
  IMP_TRACE();
  ArTrackableType trackable_type;
  ArTrackable_getType(ar_session.get(), trackable, &trackable_type);
  switch (trackable_type) {
    case AR_TRACKABLE_PLANE: {
      ArPlane_* plane = ArAsPlane(trackable);
      ArPlane_getCenterPose(ar_session.get(), plane, ar_pose.get());
      return GetTransformFromPose(ar_session, ar_pose.get());
    }
    case AR_TRACKABLE_POINT: {
      ArPoint_* point = ArAsPoint(trackable);
      ArPoint_getPose(ar_session.get(), point, ar_pose.get());
      return GetTransformFromPose(ar_session, ar_pose.get());
    }
#ifdef IMP_PRIOR_MAP
    case AR_TRACKABLE_PRIOR_MAP_TRACKABLE: {
      return mat4f();
    }
#endif
    default: {
      IMP_LOG(imp::FATAL) << "Requesting pose of unsupported trackable type.";
      return mat4f();
    }
  }
}

// Converts an Imp trackable type into an ARCore trackable type.
template <typename T>
constexpr ArTrackableType ToARCoreTrackableType() {
  if constexpr (IsPlane<T>()) {
    return AR_TRACKABLE_PLANE;
  } else if constexpr (IsPoint<T>()) {
    return AR_TRACKABLE_POINT;
  } else if constexpr (IsMagicalSurfacePoint<T>()) {
    return AR_TRACKABLE_MAGICAL_SURFACE_POINT;
  } else if constexpr (IsAnchor<T>()) {
    return AR_TRACKABLE_BASE_TRACKABLE;
#ifdef IMP_PRIOR_MAP
  } else if constexpr (IsPriorMap<T>()) {
    return AR_TRACKABLE_PRIOR_MAP_TRACKABLE;
#endif
  } else {
    return AR_TRACKABLE_NOT_VALID;
  }
}

// Converts an ARCore tracking state into an Imp tracking state.
constexpr TrackingState ToImpTrackingState(ArTrackingState state) {
  switch (state) {
    case ArTrackingState::AR_TRACKING_STATE_TRACKING:
      return TrackingState::kTracking;
    case ArTrackingState::AR_TRACKING_STATE_PAUSED:
      return TrackingState::kPaused;
    case ArTrackingState::AR_TRACKING_STATE_STOPPED:
      return TrackingState::kStopped;
    default:
      return TrackingState::kStopped;
  }
}

constexpr ArPoint::TrackingMethod ToImpTrackingMethod(
    ArPointTrackingMethod method) {
  switch (method) {
    case ArPointTrackingMethod::AR_POINT_TRACKING_METHOD_DISTANCE_GUESS:
      return ArPoint::TrackingMethod::kDistanceGuess;
    case ArPointTrackingMethod::AR_POINT_TRACKING_METHOD_REAL_DEPTH:
      return ArPoint::TrackingMethod::kRealDepth;
    case ArPointTrackingMethod::
        AR_POINT_TRACKING_METHOD_DISTANCE_GUESS_AND_REAL_DEPTH:
      return ArPoint::TrackingMethod::kDistanceGuess_kRealDepth;
    default:
      return ArPoint::TrackingMethod::kUnavailable;
  }
}

constexpr ArPlane::PlaneType ToImpPlaneType(ArPlaneType type) {
  switch (type) {
    default:
    case ArPlaneType::AR_PLANE_HORIZONTAL_UPWARD_FACING:
      return ArPlane::PlaneType::kHorizontalUpFacing;
    case ArPlaneType::AR_PLANE_HORIZONTAL_DOWNWARD_FACING:
      return ArPlane::PlaneType::kHorizontalDownFacing;
    case ArPlaneType::AR_PLANE_VERTICAL:
      return ArPlane::PlaneType::kVertical;
  }
}

constexpr const char* GetTrackingStateName(TrackingState state) {
  switch (state) {
    case TrackingState::kTracking:
      return "Tracking";
    case TrackingState::kPaused:
      return "Paused";
    case TrackingState::kStopped:
      return "Stopped";
    default:
      return "Unknown Tracking State.";
  }
}

constexpr const char* GetTrackingMethodName(ArPoint::TrackingMethod method) {
  switch (method) {
    case ArPoint::TrackingMethod::kDistanceGuess:
      return "DistanceGuess";
    case ArPoint::TrackingMethod::kRealDepth:
      return "RealDepth";
    case ArPoint::TrackingMethod::kDistanceGuess_kRealDepth:
      return "DistanceGuess & RealDepth";
    default:
      return "Unknown Tracking Method";
  }
}

constexpr const char* GetTrackableTypeName(ArTrackableType type) {
  switch (type) {
    case AR_TRACKABLE_BASE_TRACKABLE:
      return "Base";
    case AR_TRACKABLE_PLANE:
      return "Plane";
    case AR_TRACKABLE_POINT:
      return "Point";
#ifdef IMP_PRIOR_MAP
    case AR_TRACKABLE_PRIOR_MAP_TRACKABLE:
      return "PriorMap";
#endif
    default:
      return "Unknown Trackable Type";
  }
}

constexpr TrackingFailureReason ToImpTrackingFailureReason(
    ArTrackingFailureReason failure_reason) {
  switch (failure_reason) {
    case AR_TRACKING_FAILURE_REASON_NONE:
      return imp::ar::TrackingFailureReason::kNone;
    case AR_TRACKING_FAILURE_REASON_BAD_STATE:
      return imp::ar::TrackingFailureReason::kBadState;
    case AR_TRACKING_FAILURE_REASON_INSUFFICIENT_LIGHT:
      return imp::ar::TrackingFailureReason::kLowLight;
    case AR_TRACKING_FAILURE_REASON_EXCESSIVE_MOTION:
      return imp::ar::TrackingFailureReason::kExcessiveMotion;
    case AR_TRACKING_FAILURE_REASON_INSUFFICIENT_FEATURES:
      return imp::ar::TrackingFailureReason::kInsufficientFeatures;
    case AR_TRACKING_FAILURE_REASON_CAMERA_UNAVAILABLE:
      return imp::ar::TrackingFailureReason::kCameraUnavailable;
    default:
      return imp::ar::TrackingFailureReason::kUnknown;
  }
}

ArCameraConfig* ObtainBestCameraConfig(ArSession* ar_session,
                                       int ideal_texture_height,
                                       int ideal_fps) {
  // Retrieve supported camera configs.
  ArCameraConfigList* all_camera_configs = nullptr;
  int32_t num_configs = 0;
  ArCameraConfigList_create(ar_session, &all_camera_configs);
  // Create filter first to get both 30 and 60 fps.
  ArCameraConfigFilter* camera_config_filter = nullptr;
  ArCameraConfigFilter_create(ar_session, &camera_config_filter);
  ArCameraConfigFilter_setTargetFps(
      ar_session, camera_config_filter,
      AR_CAMERA_CONFIG_TARGET_FPS_30 | AR_CAMERA_CONFIG_TARGET_FPS_60);
  ArSession_getSupportedCameraConfigsWithFilter(
      ar_session, camera_config_filter, all_camera_configs);
  ArCameraConfigFilter_destroy(camera_config_filter);
  ArCameraConfigList_getSize(ar_session, all_camera_configs, &num_configs);

  absl::optional<int> best_index;
  absl::optional<float> best_score;

  // Negative to correct for sign, bias away from higher resolutions.
  constexpr float kHeightPenalty = -2;
  // We care less about Image res than texture res.
  constexpr float kImageHeightScale = 0.25f;
  // Texture res is the primary axis we're optimizing for
  constexpr float kTextureHeightScale = 1.0f;
  // Make fps divergence significantly costlier than resolution divergence.
  constexpr float kFpsScale = 200;
  // Negative to correct for sign, bias away from higher fps.
  constexpr float kFpsPenalty = -2;

  IMP_LOG(imp::INFO) << "Checking " << num_configs << " ARCore camera configs";
  for (int i = 0; i < num_configs; ++i) {
    ArCameraConfig* camera_config = nullptr;

    ArCameraConfig_create(ar_session, &camera_config);
    ArCameraConfigList_getItem(ar_session, all_camera_configs, i,
                               camera_config);
    int2 image_size;
    ArCameraConfig_getImageDimensions(ar_session, camera_config, &image_size.x,
                                      &image_size.y);
    int2 texture_size;
    ArCameraConfig_getTextureDimensions(ar_session, camera_config,
                                        &texture_size.x, &texture_size.y);

    int32_t min_fps, max_fps;
    ArCameraConfig_getFpsRange(ar_session, camera_config, &min_fps, &max_fps);
    ArCameraConfigFacingDirection facing;
    ArCameraConfig_getFacingDirection(ar_session, camera_config, &facing);
    uint32_t depth_sensor_usage;
    ArCameraConfig_getDepthSensorUsage(ar_session, camera_config,
                                       &depth_sensor_usage);
    char* camera_id = nullptr;
    ArCameraConfig_getCameraId(ar_session, camera_config, &camera_id);

    int delta_image_height = image_size.y - ideal_texture_height;
    int delta_texture_height = texture_size.y - ideal_texture_height;
    int delta_fps = max_fps - ideal_fps;
    float score = delta_image_height * kImageHeightScale *
                      (delta_image_height >= 0 ? 1 : kHeightPenalty) +
                  delta_texture_height * kTextureHeightScale *
                      (delta_texture_height >= 0 ? 1 : kHeightPenalty) +
                  delta_fps * kFpsScale * (delta_fps >= 0 ? 1 : kFpsPenalty);
    IMP_LOG(imp::INFO) << "ARCore camera config " << i << "/" << num_configs
              << "  (score " << score << "): id '" << camera_id
              << "', image w/h=" << image_size.x << "x" << image_size.y
              << ", tex w/h=" << texture_size.x << "x" << texture_size.y
              << ", fps range=" << min_fps << " - " << max_fps << " facing "
              << facing << " depth usage " << depth_sensor_usage;

    if (!best_score || *best_score > score) {
      best_score = score;
      best_index = i;
    }
    ArString_release(camera_id);
    ArCameraConfig_destroy(camera_config);
  }

  ArCameraConfig* result_camera_config = nullptr;
  if (best_index) {
    IMP_LOG(imp::INFO) << "Selected ARCore camera config " << *best_index;
    ArCameraConfig_create(ar_session, &result_camera_config);
    ArCameraConfigList_getItem(ar_session, all_camera_configs, *best_index,
                               result_camera_config);
  } else {
    IMP_LOG(imp::ERROR) << "Failed to select a camera config";
  }

  // Cleanup the list obtained as it is safe to destroy the list as camera
  // config instances were explicitly created and copied. Refer to the
  // previous comment.
  ArCameraConfigList_destroy(all_camera_configs);
  return result_camera_config;
}

#ifdef IMP_PRIOR_MAP
Transform<float> GetSessionToADFFromPriorMap(ArSession* ar_session,
                                             ArTrackable_* trackable) {
  ArPriorMapTrackable_* prior_map_trackable =
      reinterpret_cast<ArPriorMapTrackable_*>(trackable);
  ArPose_* pose;
  ArPose_create(ar_session, nullptr, &pose);

  ArPriorMapTrackable_getGlWorldToMapPose(ar_session, prior_map_trackable,
                                          pose);
  quatf quaternion = {pose->quaternion[0], pose->quaternion[1],
                      pose->quaternion[2], pose->quaternion[3]};
  float3 translation = {pose->translation[0], pose->translation[1],
                        pose->translation[2]};
  ArPose_destroy(pose);
  return Transform<float>(translation, quaternion, float3(1.f));
}
#endif

}  // namespace

class SessionHelper {
  template <typename T>
  using IfArPoint = std::enable_if_t<IsPoint<T>(), int>;
  template <typename T>
  using IfArPlane = std::enable_if_t<IsPlane<T>(), int>;
  template <typename T>
  using IfArMagicalSurfacePoint =
      std::enable_if_t<IsMagicalSurfacePoint<T>(), int>;

#ifdef IMP_PRIOR_MAP
  template <typename T>
  using IfArPriorMap = std::enable_if_t<IsPriorMap<T>(), int>;
#endif

 public:
  explicit SessionHelper(const ArSessionNativeArCore* imp_session)
      : imp_session_(imp_session) {}
  const ArSessionNativeArCore* operator->() { return imp_session_; }

  template <typename Trackable>
  static std::vector<Trackable> GetUpdatedTrackables(
      const ArSessionNativeArCore* imp_session) {
    IMP_TRACE();
    SessionHelper helper(imp_session);
    ArTrackableListHelper trackable_list(helper->ar_session_.get());
    ArFrame_getUpdatedTrackables(
        helper->ar_session_.get(), helper->ar_frame_.get(),
        ToARCoreTrackableType<Trackable>(), trackable_list.get());

    std::vector<Trackable> results;
    trackable_list.ForEach(
        [&results, &helper](UniqueArTrackable trackable_ptr) {
          Trackable imp_trackable =
              helper.CreateImpTrackable<Trackable>(std::move(trackable_ptr));
          results.push_back(imp_trackable);
        });

    return results;
  }

  template <typename Trackable>
  static std::vector<Trackable> GetTrackables(
      ArSessionNativeArCore* imp_session) {
    IMP_TRACE();
    SessionHelper helper(imp_session);
    // Creates an ArCore list of the trackable type.
    ArTrackableListHelper trackable_list(helper->ar_session_.get());
    ArSession_getAllTrackables(helper->ar_session_.get(),
                               ToARCoreTrackableType<Trackable>(),
                               trackable_list.get());

    // Converts the list to an Imp trackable type.
    std::vector<Trackable> results;
    trackable_list.ForEach(
        [&helper, &results](UniqueArTrackable trackable_ptr) {
          Trackable imp_trackable =
              helper.CreateImpTrackable<Trackable>(std::move(trackable_ptr));
          results.push_back(imp_trackable);
        });
    return results;
  }

  // A Creator method that handles the Point trackable type.
  template <typename T, IfArPoint<T> = 0>
  ArPoint CreateImpTrackable(UniqueArTrackable trackable,
                             absl::optional<ArPose_*> pose = absl::nullopt) {
    IMP_TRACE();
    // Getting a pose from point when using instant (kDistanceGuess) mode,
    // according to ArCore documentation  will get a point found with
    // REAL_DEPTH if available otherwise a DISTANCE_GUESS based point is
    // returned.
    mat4f transform;
    if (pose) {
      transform = GetTransformFromPose(imp_session_->ar_session_, pose.value());
    } else {
      transform = GetPose(imp_session_->ar_session_, imp_session_->ar_pose_,
                          trackable.get());
    }
    // Gets the current tracking state.
    ArTrackingState tracking_state = ArTrackingState::AR_TRACKING_STATE_STOPPED;
    ArTrackable_getTrackingState(imp_session_->ar_session_.get(),
                                 trackable.get(), &tracking_state);

    // Gets the tracking method the point is using.
    ArPointTrackingMethod tracking_method =
        ArPointTrackingMethod::AR_POINT_TRACKING_METHOD_DISTANCE_GUESS;
    ArPoint_getTrackingMethod(imp_session_->ar_session_.get(),
                              ArAsPoint(trackable.get()), &tracking_method);

    // Constructs and returns an ArPoint.
    return ArPoint(ArTrackableId(reinterpret_cast<int64_t>(trackable.get())),
                   ToImpTrackingState(tracking_state),
                   ToImpTrackingMethod(tracking_method), transform,
                   std::move(trackable));
  }

  // This creator is only used for GetUpdatedTrackables to update magical
  // surface trackable map.
  template <typename T, IfArMagicalSurfacePoint<T> = 0>
  ArMagicalSurfacePoint CreateImpTrackable(
      UniqueArTrackable&& trackable,
      absl::optional<ArPose_*> pose = absl::nullopt) {
    IMP_TRACE();
    mat4f transform;
    // If no pose is provided, we have no way of updating the pose of a
    // MagicalSurfacePoint since ARCore does not provide an accessor.
    // This is fine, since these points only stick around if an anchor is
    // created on the frame of the hit-test that spawned the points and that
    // anchor will still have the correct pose.
    if (pose) {
      transform = GetTransformFromPose(imp_session_->ar_session_, pose.value());
    }

    // Gets the current tracking state.
    ArTrackingState tracking_state;
    ArTrackable_getTrackingState(imp_session_->ar_session_.get(),
                                 trackable.get(), &tracking_state);

    // Constructs and returns an ArMagicalSurfacePoint. ArMagicalSurfacePoint_
    // does not have pos info, and thus only tracking state will be fetched to
    // update the magical surface trackable map to discard the trackables with
    // stopped state.
    return ArMagicalSurfacePoint(
        ArTrackableId(reinterpret_cast<int64_t>(trackable.get())),
        ToImpTrackingState(tracking_state), transform, std::move(trackable));
  }

  // A creator method that handles the Plane trackable type.
  template <typename T, IfArPlane<T> = 0>
  ArPlane CreateImpTrackable(UniqueArTrackable trackable,
                             absl::optional<ArPose_*> pose = absl::nullopt) {
    IMP_TRACE();
    mat4f transform;
    if (pose) {
      transform = GetTransformFromPose(imp_session_->ar_session_, pose.value());
    } else {
      transform = GetPose(imp_session_->ar_session_, imp_session_->ar_pose_,
                          trackable.get());
    }

    float2 extents;
    ArPlane_getExtentX(imp_session_->ar_session_.get(),
                       ArAsPlane(trackable.get()), &extents.x);
    ArPlane_getExtentZ(imp_session_->ar_session_.get(),
                       ArAsPlane(trackable.get()), &extents.y);

    // Halving the extents to improve ergonomics, better support the most common
    // usage pattern: max = center + extents, min = center - extents.
    extents = extents * 0.5f;

    // Gathers mesh vertices of the surface polygon.
    int32_t polygon_size = 0;
    ArPlane_getPolygonSize(imp_session_->ar_session_.get(),
                           ArAsPlane(trackable.get()), &polygon_size);
    thread_local std::vector<float2> elements;
    elements.resize(polygon_size / 2);
    ArPlane_getPolygon(imp_session_->ar_session_.get(),
                       ArAsPlane(trackable.get()), elements[0].v);
    ArTrackingState tracking_state;
    ArTrackable_getTrackingState(imp_session_->ar_session_.get(),
                                 trackable.get(), &tracking_state);

    ArPlaneType plane_type;
    ArPlane_getType(imp_session_->ar_session_.get(), ArAsPlane(trackable.get()),
                    &plane_type);

    thread_local std::vector<float3> vertices;
    vertices.reserve(polygon_size / 2);

    for (const float2& vertex : elements) {
      vertices.emplace_back(vertex.x, 0.0f, vertex.y);
    }

    ArPlane_* ar_subsumed_by = nullptr;
    ArPlane_acquireSubsumedBy(imp_session_->ar_session_.get(),
                              ArAsPlane(trackable.get()), &ar_subsumed_by);
    UniqueArPlane subsumed_by(ar_subsumed_by);
    if (subsumed_by) {
      // Stops tracking the plane if it's subsumed.
      tracking_state = ArTrackingState::AR_TRACKING_STATE_STOPPED;
    }

    // Constructs and returns an ArPlane.
    return ArPlane(ArTrackableId(reinterpret_cast<int64_t>(trackable.get())),
                   ToImpTrackingState(tracking_state), transform, extents,
                   ToImpPlaneType(plane_type), std::move(vertices));
  }

#ifdef IMP_PRIOR_MAP
  // A Creator method that handles the PriorMap trackable type.
  template <typename T, IfArPriorMap<T> = 0>
  ArPriorMap CreateImpTrackable(UniqueArTrackable&& trackable,
                                absl::optional<ArPose_*> pose = absl::nullopt) {
    // Gets the current tracking state.
    ArTrackingState tracking_state;
    ArTrackable_getTrackingState(imp_session_->ar_session_.get(),
                                 trackable.get(), &tracking_state);
    Transform<float> transform;

#if 0
    // BUG((broken link)): Missing symbol ArPriorMapTrackable_getGlWorldToMapPose.
    // If GetSessionToADFFromPriorMap is called the app will crash.
    // Call to GetSessionToADFFromPriorMap removed to work around (broken link).
    if (tracking_state == ArTrackingState::AR_TRACKING_STATE_TRACKING) {
      transform = GetSessionToADFFromPriorMap(imp_session_->ar_session_.get(),
                                              trackable.get());
    } else {
      // A fallback value when transformation from session space to ADF space is
      // not available.
      const imp::quatf kSessionToADF = {0.7071068f, 0.7071068f, 0.0f, 0.0f};
      transform = Transform<float>(imp::kZero3, kSessionToADF, imp::kOne3);
    }
#else
    const imp::quatf kSessionToADF = {0.7071068f, 0.7071068f, 0.0f, 0.0f};
    transform = Transform<float>(imp::kZero3, kSessionToADF, imp::kOne3);
#endif
    // Constructs and returns an ArPriorMap.
    return ArPriorMap(ArTrackableId(reinterpret_cast<int64_t>(trackable.get())),
                      ToImpTrackingState(tracking_state), transform);
  }
#endif

 private:
  const ArSessionNativeArCore* imp_session_;
};

static absl::optional<Future<absl::Status>>& GetAvailabilityFuture(
    const BaseView* view) {
  // ArCoreApk_checkAvailability takes a long time and causes strict mode
  // violations when it is called on the main thread.
  static auto* instance = new absl::optional<Future<absl::Status>>();
  if (!instance->has_value()) {
    instance->emplace(Future<absl::Status>::Schedule(
        [context = view->GetContext()]() -> absl::Status {
          ArAvailability availability;
          ArCoreApk_checkAvailability(
              context.GetJniEnv(), context.GetActivityContext(), &availability);
          auto result = absl::OkStatus();
          if (availability != AR_AVAILABILITY_SUPPORTED_INSTALLED) {
            result.Update(absl::InternalError("ARCore not available"));
            result.SetPayload(kPlatformArFailureUrl,
                              absl::Cord(absl::StrFormat(
                                  "%d", static_cast<int32_t>(availability))));
          }
          return result;
        },
        Executor::Type::kBackground));
  }
  return *instance;
}

Future<absl::Status> ArSessionNative::CanCreateForPlatform(
    const BaseView* view, const ArSessionConfig& config) {
  if (config.tracking_mode == ArSessionConfig::TrackingMode::kFace) {
    return Future<absl::Status>(
        absl::UnimplementedError("AR face tracking support not implemented"));
  }
  return GetAvailabilityFuture(view).value();
}

Future<std::unique_ptr<ArSessionNative>> ArSessionNative::CreateForPlatform(
    BaseView* view, const ArSessionConfig& config) {
  std::string settings;
  if (!config.dataset_path.empty()) {
    settings = absl::StrFormat("dataset_path,%s", config.dataset_path.c_str());
  }
  return ArSessionNativeArCore::CreateSession(
      view->GetContext(), view->GetHost()->GetEngine(),
      &view->GetTextureFactory(), &view->GetEnvironmentLightFactory(), config,
      settings);
}

Future<std::unique_ptr<ArSessionNative>> ArSessionNativeArCore::CreateSession(
    const Context& context, filament::Engine* engine,
    imp::TextureFactory* texture_factory,
    imp::EnvironmentLightFactory* env_light_factory,
    const ArSessionConfig& config, std::string settings) {
  IMP_TRACE();
  return Future<UniqueArSession>::Schedule(
             [context, settings, config]() -> absl::StatusOr<UniqueArSession> {
               IMP_TRACE_BLOCK("Schedule");
               // First stage of creating ArSession on background thread.
               // Create the actual ARCore session here, which is slow because
               // of booting up the camera.
               ArSession_* ar_session;
               if (ArStatus status =
                       settings.empty()
                           ? ArSession_create(context.GetJniEnv(),
                                              context.GetActivityContext(),
                                              &ar_session)
                           : ArSession_createWithSettings(
                                 context.GetJniEnv(),
                                 context.GetActivityContext(), settings.c_str(),
                                 &ar_session);
                   status != AR_SUCCESS) {
                 // TODO: handle session creation failure
                 // gracefully.
                 return absl::InternalError(absl::StrFormat(
                     "Failed to create ArSession! (got AR error code %d)",
                     static_cast<int>(status)));
               }
               return UniqueArSession(ar_session, ar::ArSessionDeleter());
             },
             Executor::Type::kBackground)
      .Then([engine, texture_factory, env_light_factory,
             config](UniqueArSession ar_session)
                -> absl::StatusOr<std::unique_ptr<ArSessionNative>> {
        IMP_TRACE_BLOCK("Then");
        // Second stage of creating ArSession on foreground thread.
        // We can access the View and Filament here to do things
        // like create textures. This is not the slow part of session
        // creation, so fine to do on the foreground.
        // MP_RETURN_IF_ERROR(ar_session_or);
        return CreateSession(engine, texture_factory, env_light_factory, config,
                             std::move(ar_session));
      });
}

std::unique_ptr<ArSessionNative> ArSessionNativeArCore::CreateSession(
    filament::Engine* engine, imp::TextureFactory* texture_factory,
    imp::EnvironmentLightFactory* env_light_factory,
    const ArSessionConfig& config, UniqueArSession arcore_session) {
  auto ar_session_native = absl::WrapUnique(
      new ArSessionNativeArCore(engine, texture_factory, env_light_factory,
                                config, std::move(arcore_session)));
  absl::Status status = ar_session_native->InternalInitialization();
  if (!status.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to initialize the ARCore native session: ("
               << status.code() << ") " << status;
    ar_session_native.reset(nullptr);
  }
  return ar_session_native;
}

void ArSessionNativeArCore::SetPlacementMode(
    ArSessionConfig::PlacementMode mode) {
  ArConfig_setInstantPlacementMode(
      ar_session_.get(), ar_config_.get(),
      mode == ArSessionConfig::PlacementMode::kInstantPlacementMode
          ? AR_INSTANT_PLACEMENT_MODE_ENABLED
          : AR_INSTANT_PLACEMENT_MODE_DISABLED);
  ArConfig_setMagicalSurfaceHitTestMode(
      ar_session_.get(), ar_config_.get(),
      mode == ArSessionConfig::PlacementMode::kMagicalSurfaceMode
          ? AR_MAGICAL_SURFACE_HIT_TEST_MODE_ENABLED
          : AR_MAGICAL_SURFACE_HIT_TEST_MODE_DISABLED);
  
  placement_mode_ = mode;
}

ArSessionNativeArCore::ArSessionNativeArCore(
    filament::Engine* engine, imp::TextureFactory* texture_factory,
    imp::EnvironmentLightFactory* env_light_factory,
    const ArSessionConfig& config, UniqueArSession ar_session)
    : engine_(engine),
      texture_factory_(texture_factory),
      env_light_factory_(env_light_factory),
      ar_session_config_(config),
      ar_session_(std::move(ar_session)),
      deeplight_controller_(engine, env_light_factory),
      placement_mode_(ArSessionConfig::PlacementMode::kPlanePlacementMode) {}

absl::Status ArSessionNativeArCore::InternalInitialization() {
  ArConfig_* ar_config;
  ArConfig_create(ar_session_.get(), &ar_config);
  if (!ar_config) {
    return absl::InternalError("Failed to create ArConfig!");
  }
  ar_config_ = UniqueArConfig(ar_config);

  ArConfig_setUpdateMode(ar_session_.get(), ar_config_.get(),
                         AR_UPDATE_MODE_LATEST_CAMERA_IMAGE);

  ArConfig_setPlaneFindingMode(ar_session_.get(), ar_config_.get(),
                               AR_PLANE_FINDING_MODE_HORIZONTAL_AND_VERTICAL);

  // Queries ARCore for depth support, and configures it if available.
  int32_t is_depth_supported = 0;
  if (ar_session_config_.depth_mode == ArSessionConfig::DepthMode::kAutomatic) {
    ArSession_isDepthModeSupported(ar_session_.get(), AR_DEPTH_MODE_AUTOMATIC,
                                   &is_depth_supported);
    if (is_depth_supported) {
      ArConfig_setDepthMode(ar_session_.get(), ar_config_.get(),
                            AR_DEPTH_MODE_AUTOMATIC);
    }
  } else if (ar_session_config_.depth_mode ==
             ArSessionConfig::DepthMode::kDisabled) {
    ArConfig_setDepthMode(ar_session_.get(), ar_config_.get(),
                          AR_DEPTH_MODE_DISABLED);
  }

  ArLightEstimationMode light_estimation_mode;
  switch (ar_session_config_.lighting_mode) {
    default:
    case ArSessionConfig::LightingMode::kHdr:
      light_estimation_mode = AR_LIGHT_ESTIMATION_MODE_ENVIRONMENTAL_HDR;
      break;
    case ArSessionConfig::LightingMode::kAmbient:
      light_estimation_mode = AR_LIGHT_ESTIMATION_MODE_AMBIENT_INTENSITY;
      break;
    case ArSessionConfig::LightingMode::kLightingDisabled:
      light_estimation_mode = AR_LIGHT_ESTIMATION_MODE_DISABLED;
      break;
  }

  ArConfig_setLightEstimationMode(ar_session_.get(), ar_config_.get(),
                                  light_estimation_mode);

  ArStatus status =
      ArSession_checkSupported(ar_session_.get(), ar_config_.get());
  if (status != AR_SUCCESS) {
    return absl::InternalError(
        absl::StrFormat("ArConfig not supported: %i", status));
  }

#if ARCORE_FEATURE_ENABLED(set_frame_delay)
  if (ar_session_config_.frame_delay_override.has_value()) {
    ArConfig_setFrameDelayOverride(
        ar_session_.get(), ar_config_.get(),
        ar_session_config_.frame_delay_override.value());
  }
#endif  // ARCORE_FEATURE_ENABLED(set_frame_delay)

  // TODO: this is weird - we should just roll this into the
  // ArSession_configure call below as that is now happening twice.
  SetPlacementMode(ar_session_config_.placement_mode);

  status = ArSession_configure(ar_session_.get(), ar_config_.get());
  if (status != AR_SUCCESS) {
    return absl::InternalError(
        absl::StrFormat("Failed to create ArCamera: %i", status));
  }

  ArFrame_* ar_frame;
  ArFrame_create(ar_session_.get(), &ar_frame);
  if (!ar_frame) {
    return absl::InternalError("Error creating ArFrame!");
  }
  ar_frame_ = UniqueArFrame(ar_frame);

  ArCameraConfig* camera_config = ObtainBestCameraConfig(
      ar_session_.get(), ar_session_config_.desired_camera_texture_height,
      ar_session_config_.desired_framerate ==
              ArSessionConfig::DesiredFramerate::k60hz
          ? 60
          : 30);

  if (ArSession_setCameraConfig(ar_session_.get(), camera_config) !=
      AR_SUCCESS) {
    return absl::InternalError("Failed to set camera config");
  }
  ArCameraConfig_getTextureDimensions(ar_session_.get(), camera_config,
                                      &camera_dimensions_.x,
                                      &camera_dimensions_.y);
  ArCameraConfig_destroy(camera_config);

  // Before the session is resumed set the playback uri if one has been passed
  // in the session configuration.
  if (!ar_session_config_.playback_dataset_uri.empty()) {
    status = ArSession_setPlaybackDatasetUri(
        ar_session_.get(), ar_session_config_.playback_dataset_uri.c_str());
    if (status != AR_SUCCESS) {
      return absl::InternalError(
          absl::StrFormat("Failed to set the specified playback uri "
                          "= %s with error %d",
                          ar_session_config_.playback_dataset_uri, status));
    }
  }

  status = ArSession_resume(ar_session_.get());
  if (status != AR_SUCCESS) {
    return absl::InternalError(
        absl::StrFormat("Failed to resume ArSession: %i", status));
  }

  ArCamera_* ar_camera;
  ArFrame_acquireCamera(ar_session_.get(), ar_frame_.get(), &ar_camera);
  if (!ar_camera) {
    return absl::InternalError("Failed to acquire ArCamera!");
  }
  ar_camera_ = UniqueArCamera(ar_camera);

  ar_pose_ = CreateUniqueArPose(ar_session_, absl::nullopt);

  if (is_depth_supported) {
    if (!engine_) {
      return absl::InternalError("filament engine not initialized");
    }
    depth_texture_controller_.emplace(engine_, texture_factory_);
  }

  return absl::OkStatus();
}

absl::Status ArSessionNativeArCore::StartRecording(
    absl::string_view dataset_uri) {
  ArRecordingConfig* ar_recording_config = nullptr;
  ArRecordingConfig_create(ar_session_.get(), &ar_recording_config);
  if (!ar_recording_config) {
    return absl::InternalError("Failed to create ArRecordingConfig!");
  }
  ar_recording_config_ = UniqueArRecordingConfig(ar_recording_config);

  ArRecordingConfig_setMp4DatasetUri(ar_session_.get(), ar_recording_config,
                                     std::string(dataset_uri).c_str());
  ArRecordingConfig_setAutoStopOnPause(ar_session_.get(), ar_recording_config,
                                       true);
  ArStatus status =
      ArSession_startRecording(ar_session_.get(), ar_recording_config);
  if (status != AR_SUCCESS) {
    return absl::InternalError(
        absl::StrFormat("Failed to start the recording: %i", status));
  }
  return absl::OkStatus();
}

absl::Status ArSessionNativeArCore::StopRecording() {
  ArStatus status = ArSession_stopRecording(ar_session_.get());
  if (status != AR_SUCCESS) {
    return absl::InternalError(
        absl::StrFormat("Failed to stop the recording: %i", status));
  }
  return absl::OkStatus();
}

void ArSessionNativeArCore::CreateCameraTextures() {
  assert(Executor::CurrentExecutor() == Executor::ForegroundExecutor());

  uint2 texture_dimensions = GetCameraTextureDimensions();

  std::function<void(std::vector<GLuint>)> set_texture_id_callback =
      [this](std::vector<GLuint> texture_ids) {
        if (texture_ids.size() == 1) {
          ArSession_setCameraTextureName(ar_session_.get(), texture_ids[0]);
        } else {
          ArSession_setCameraTextureNames(ar_session_.get(), texture_ids.size(),
                                          texture_ids.data());
        }
      };

  camera_texture_ = std::make_unique<AndroidCameraTexture>(
      engine_, texture_factory_, texture_dimensions,
      ar_session_config_.num_camera_textures, set_texture_id_callback);
}

uint2 ArSessionNativeArCore::GetCameraTextureDimensions() const {
  return camera_dimensions_;
}

void ArSessionNativeArCore::GetCameraTextureUVs(
    const absl::Span<const float2> normalized_screen_coordinates,
    std::vector<float2>* uvs_out) const {
  
  ArFrame_transformCoordinates2d(
      ar_session_.get(), ar_frame_.get(), AR_COORDINATES_2D_VIEW_NORMALIZED,
      normalized_screen_coordinates.size(),
      reinterpret_cast<const float*>(normalized_screen_coordinates.data()),
      AR_COORDINATES_2D_TEXTURE_NORMALIZED,
      reinterpret_cast<float*>(uvs_out->data()));
}

std::array<float3, 2u> ArSessionNativeArCore::GetUvFromNdcTransform() const {
  auto static constexpr kNdcBasis = std::array<float2, 3u>{
      float2{0, 0},
      float2{1, 0},
      float2{0, 1},
  };

  // Transforms NDC basis points to UV space.
  auto uvs = std::array<float2, 3u>{};
  ArFrame_transformCoordinates2d(
      ar_session_.get(), ar_frame_.get(),
      // Input NDC coordinates.
      AR_COORDINATES_2D_OPENGL_NORMALIZED_DEVICE_COORDINATES, kNdcBasis.size(),
      reinterpret_cast<float const*>(kNdcBasis.data()),
      // Output UVs.
      AR_COORDINATES_2D_TEXTURE_NORMALIZED,
      reinterpret_cast<float*>(uvs.data()));

  // Computes the scale and offset from NDC to UV space.
  auto uv_from_ndc_transform = std::array<float3, 2u>{};
  auto ndc_origin = uvs[0];
  uv_from_ndc_transform[0] = {
      uvs[1].x - ndc_origin.x,
      uvs[2].x - ndc_origin.x,
      ndc_origin.x,
  };
  uv_from_ndc_transform[1] = {
      uvs[1].y - ndc_origin.y,
      uvs[2].y - ndc_origin.y,
      ndc_origin.y,
  };

  return uv_from_ndc_transform;
}

bool ArSessionNativeArCore::GetDisplayGeometryChanged() const {
  int32_t geometry_changed;
  ArFrame_getDisplayGeometryChanged(ar_session_.get(), ar_frame_.get(),
                                    &geometry_changed);
  return geometry_changed;
}

void ArSessionNativeArCore::SetDisplayGeometry(
    window::WindowRotation orientation, int width, int height, float near,
    float far) {
  ArSession_setDisplayGeometry(ar_session_.get(), static_cast<int>(orientation),
                               width, height);
  width_ = width;
  height_ = height;
  near_ = near;
  far_ = far;
}

void ArSessionNativeArCore::Pause() {
  // We call GetHdrLighting to clear out any pending deeplight frames so they
  // won't be fetched later on when they may no longer be valid.
  GetHdrLighting();
  ArStatus status = ArSession_pause(ar_session_.get());
  if (status != AR_SUCCESS) {
    IMP_LOG(imp::FATAL) << "Failed to pause ArSession: " << status;
  }
}

void ArSessionNativeArCore::Resume() {
  ArStatus status = ArSession_resume(ar_session_.get());
  if (status != AR_SUCCESS) {
    IMP_LOG(imp::FATAL) << "Failed to resume ArSession: " << status;
  }
}

absl::Status ArSessionNativeArCore::GetLatestModelMatrix(
    mat4f* matrix_out, absl::Time* timestamp_out) const {
  IMP_TRACE();
  // Crash in case of bad/invalid input.
  
  
#if ARCORE_FEATURE_ENABLED(get_latest_pose)
  int64_t latest_timestamp_ns;
  if (ArCamera_getLatestDisplayOrientedPose(ar_session_.get(), ar_camera_.get(),
                                            &latest_timestamp_ns,
                                            ar_pose_.get()) == AR_SUCCESS) {
    *timestamp_out = absl::FromUnixNanos(latest_timestamp_ns);
    *matrix_out = GetTransformFromPose(ar_session_, ar_pose_.get());
    return absl::OkStatus();
  } else {
    return absl::UnavailableError("Latest pose is not available.");
  }
#else
  return ArSessionNative::GetLatestModelMatrix(matrix_out, timestamp_out);
#endif
}

absl::optional<ArFrame> ArSessionNativeArCore::Update(
    absl::Time last_submitted_timestamp) {
  IMP_TRACE();
  ArSession* session = ar_session_.get();
  if (!camera_texture_) {
    CreateCameraTextures();
  }

  ArStatus status = ArSession_update(session, ar_frame_.get());
  if (status != AR_SUCCESS) {
    IMP_LOG(imp::ERROR) << "Failed to update ArSession: " << status;
  }
  int64_t timestamp_nanos;
  ArFrame_getTimestamp(session, ar_frame_.get(), &timestamp_nanos);
  absl::Time timestamp = absl::FromUnixNanos(timestamp_nanos);
  if (timestamp <= last_submitted_timestamp) {
    // Only update the camera matrices if we have a new camera image.
    return {};
  }

  GLuint camera_texture_id = 0;
  ArFrame_getCameraTextureName(session, ar_frame_.get(), &camera_texture_id);

  // For some reason, this is always zero the first frame. Don't change the
  // texture id in that case.
  if (camera_texture_id != 0) {
    camera_texture_->SetTextureId(camera_texture_id);
  }

  if (depth_texture_controller_) {
    depth_texture_controller_->Update(session, ar_frame_.get());
  }

  deeplight_controller_.Update(session, ar_config_.get(), ar_frame_.get());

  float projection_matrix[16]{};
  mat4 projection_mat4;
  ArCamera_getProjectionMatrix(session, ar_camera_.get(), near_, far_,
                               projection_matrix);
  imp::FillMat4<double>(projection_matrix, &projection_mat4);

  ArCamera_getDisplayOrientedPose(session, ar_camera_.get(), ar_pose_.get());
  mat4f model_mat4 = GetTransformFromPose(ar_session_, ar_pose_.get());

  TrackableTuple updated_trackables;
  // TODO Update additional trackable types based config
  // structure.
  std::get<std::vector<ArPoint>>(updated_trackables) = GetUpdatedPoints();
  std::get<std::vector<ArMagicalSurfacePoint>>(updated_trackables) =
      GetUpdatedMagicalSurfacePoints();
  std::get<std::vector<ArPlane>>(updated_trackables) = GetUpdatedPlanes();
  std::get<std::vector<ArAnchor>>(updated_trackables) = GetUpdatedAnchors();
#ifdef IMP_PRIOR_MAP
  std::get<std::vector<ArPriorMap>>(updated_trackables) = GetUpdatedPriorMaps();
#endif
  ArImage_* output_image = nullptr;
#if IMP_RUNTIME(DEV)
  ArStatus image_status =
      ArFrame_acquireCameraImage(session, ar_frame_.get(), &output_image);
#else
  ArStatus image_status = AR_ERROR_NOT_YET_AVAILABLE;
#endif
  if (image_status == AR_SUCCESS && output_image != nullptr) {
    ArImageFormat format;
    ArImage_getFormat(session, output_image, &format);
    auto yuv_image = std::make_unique<ArFrame::YUV420Image>();
    // getPlaneData and its ptrs are valid as long as ArImage is valid.
    // ArImage is valid until we clean it up.
    // See
    // https://developer.android.com/reference/android/graphics/ImageFormat#YUV_420_888
    // for documentation.
    // Y = 0, U = 1, V = 2
    if (format == AR_IMAGE_FORMAT_YUV_420_888) {
      ArImage_getHeight(session, output_image, &yuv_image->height);
      ArImage_getWidth(session, output_image, &yuv_image->width);
      ArImage_getPlaneData(session, output_image, 0, &yuv_image->y_ptr,
                           &yuv_image->y_buffer_size);
      ArImage_getPlaneData(session, output_image, 1, &yuv_image->u_ptr,
                           &yuv_image->u_buffer_size);
      ArImage_getPlaneData(session, output_image, 2, &yuv_image->v_ptr,
                           &yuv_image->v_buffer_size);
      ArImage_getPlanePixelStride(session, output_image, 1,
                                  &yuv_image->uv_stride);
      yuv_image->on_delete = std::make_unique<std::function<void()>>(
          [output_image]() { ArImage_release(output_image); });

      return ArFrame(timestamp, camera_texture_->GetTexture(),
                     camera_texture_->GetTextureId(), projection_mat4,
                     model_mat4, std::move(updated_trackables),
                     std::move(yuv_image));
    }
  }
  if (output_image != nullptr) {
    ArImage_release(output_image);
  }
  return ArFrame(timestamp, camera_texture_->GetTexture(),
                 camera_texture_->GetTextureId(), projection_mat4, model_mat4,
                 std::move(updated_trackables));
}

uint4 ArSessionNativeArCore::GetDebugSessionId() {
  auto result = kDefaultDebugSessionId;
  ArSession_getDebugSessionId(ar_session_.get(),
                              reinterpret_cast<uint8_t*>(&result));
  return result;
}

absl::optional<ArHitResult> ArSessionNativeArCore::ConvertArHitResult(
    const ArHitResultPtr& hit_result,
    TrackableTuple* out_generated_trackables) const {
  IMP_TRACE();
  ArTrackable_* ar_trackable = nullptr;
  ArHitResult_acquireTrackable(ar_session_.get(), hit_result.get(),
                               &ar_trackable);
  UniqueArTrackable trackable(ar_trackable);

  // Don't consider hits on trackables that are not being tracked.
  ArTrackingState tracking_state = AR_TRACKING_STATE_PAUSED;
  ArTrackable_getTrackingState(ar_session_.get(), trackable.get(),
                               &tracking_state);
  if (tracking_state != AR_TRACKING_STATE_TRACKING) {
    return absl::nullopt;
  }

  ArTrackableId trackable_id(reinterpret_cast<int64_t>(trackable.get()));

  ArTrackableType ar_trackable_type = AR_TRACKABLE_NOT_VALID;
  ArTrackable_getType(ar_session_.get(), trackable.get(), &ar_trackable_type);

  if (ar_trackable_type == AR_TRACKABLE_POINT) {
    ArPoint point =
        SessionHelper(this).CreateImpTrackable<ArPoint>(std::move(trackable));
    std::get<std::vector<ArPoint>>(*out_generated_trackables).push_back(point);
  } else if (ar_trackable_type == AR_TRACKABLE_MAGICAL_SURFACE_POINT) {
    ArMagicalSurfacePoint_* ar_magical_surface_point =
        reinterpret_cast<ArMagicalSurfacePoint_*>(ar_trackable);
    ArMagicalSurfacePointOrientationMode orientation_mode;
    ArMagicalSurfacePoint_getOrientationMode(
        ar_session_.get(), ar_magical_surface_point, &orientation_mode);

    if (orientation_mode !=
        AR_MAGICAL_SURFACE_POINT_ORIENTATION_ESTIMATED_SURFACE_NORMAL) {
      return absl::nullopt;
    }

    // Get transform.
    ArHitResult_getHitPose(ar_session_.get(), hit_result.get(), ar_pose_.get());
    auto magical_surface_point =
        SessionHelper(this).CreateImpTrackable<ArMagicalSurfacePoint>(
            std::move(trackable), ar_pose_.get());
    // This point may have been created by this hit test call, so we need
    // to add it to the generated points in order for it to be in session
    // trackables immediately in case the consumer of these hit results tries
    // to use the trackable immediately, i.e. to create an anchor.
    std::get<std::vector<ArMagicalSurfacePoint>>(*out_generated_trackables)
        .push_back(magical_surface_point);
  }
  ArHitResult_getHitPose(ar_session_.get(), hit_result.get(), ar_pose_.get());
  mat4f transform = GetTransformFromPose(ar_session_, ar_pose_.get());
  float3 hit_position;
  quatf hit_rotation;
  float3 hit_scale;
  Decompose(transform, &hit_position, &hit_rotation, &hit_scale);

  // Creates the hit result output.
  float distance = std::numeric_limits<float>::max();
  ArHitResult_getDistance(ar_session_.get(), hit_result.get(), &distance);
  return ArHitResult(hit_rotation, hit_position, distance, trackable_id);
}

std::vector<ArHitResult> ArSessionNativeArCore::ConvertArHitResults(
    const ArHitResultListPtr& hit_result_list,
    TrackableTuple* out_generated_trackables) const {
  int size = 0;
  ArHitResultList_getSize(ar_session_.get(), hit_result_list.get(), &size);

  std::vector<ArHitResult> results;
  for (int i = 0; i < size; ++i) {
    ArHitResultPtr hit_result(ar_session_.get());
    ArHitResultList_getItem(ar_session_.get(), hit_result_list.get(), i,
                            hit_result.get());
    if (!hit_result) {
      continue;
    }

    if (auto converted_result =
            ConvertArHitResult(hit_result, out_generated_trackables)) {
      results.push_back(converted_result.value());
    }
  }
  return results;
}

std::vector<ArHitResult> ArSessionNativeArCore::HitTest(
    float2 screen_pos, absl::optional<float> guessed_distance,
    TrackableTuple* out_generated_trackables) {
  IMP_TRACE();
  ArHitResultListPtr hit_result_list(ar_session_.get());
  // If the current placement mode is kMagicalSurfaceMode, ignore the guessed
  // distance because InstantHitTest does not support magical surface points.
  if (guessed_distance &&
      placement_mode_ != ArSessionConfig::PlacementMode::kMagicalSurfaceMode) {
    ArFrame_instantHitTest(ar_session_.get(), ar_frame_.get(), screen_pos.x,
                           screen_pos.y, guessed_distance.value(),
                           hit_result_list.get());
  } else {
    ArFrame_hitTest(ar_session_.get(), ar_frame_.get(), screen_pos.x,
                    screen_pos.y, hit_result_list.get());
  }

  return ConvertArHitResults(hit_result_list, out_generated_trackables);
}

std::vector<ArHitResult> ArSessionNativeArCore::HitTestRay(
    const Ray& ray, TrackableTuple* out_generated_trackables) {
  ArHitResultListPtr hit_result_list(ar_session_.get());
  ArFrame_hitTestRay(ar_session_.get(), ar_frame_.get(), ray.origin.v,
                     ray.direction.v, hit_result_list.get());
  return ConvertArHitResults(hit_result_list, out_generated_trackables);
}

TrackingState ArSessionNativeArCore::GetCameraTrackingState() const {
  ArTrackingState tracking_state = AR_TRACKING_STATE_PAUSED;
  ArCamera_getTrackingState(ar_session_.get(), ar_camera_.get(),
                            &tracking_state);
  return ToImpTrackingState(tracking_state);
}

std::vector<ArPoint> ArSessionNativeArCore::GetUpdatedPoints() const {
  return SessionHelper::GetUpdatedTrackables<ArPoint>(this);
}

std::vector<ArMagicalSurfacePoint>
ArSessionNativeArCore::GetUpdatedMagicalSurfacePoints() const {
  return SessionHelper::GetUpdatedTrackables<ArMagicalSurfacePoint>(this);
}

std::vector<ArPlane> ArSessionNativeArCore::GetUpdatedPlanes() const {
  return SessionHelper::GetUpdatedTrackables<ArPlane>(this);
}

#ifdef IMP_PRIOR_MAP
std::vector<ArPriorMap> ArSessionNativeArCore::GetUpdatedPriorMaps() const {
  return SessionHelper::GetUpdatedTrackables<ArPriorMap>(this);
}
#endif

absl::StatusOr<ArAnchor> ArSessionNativeArCore::CreateAnchor(
    float3 position, quatf rotation, absl::optional<ArTrackableId> id) {
  IMP_TRACE();
  std::array<float, 7> pose_raw = {rotation.x, rotation.y, rotation.z,
                                   rotation.w, position.x, position.y,
                                   position.z};

  auto pose = CreateUniqueArPose(ar_session_, pose_raw);

  ArAnchor_* anchor = nullptr;
  if (!id) {
    ArSession_acquireNewAnchor(ar_session_.get(), pose.get(), &anchor);
  } else {
    ArTrackable_* trackable = reinterpret_cast<ArTrackable_*>(id->GetLow());

    ArTrackingState tracking_state;
    ArTrackable_getTrackingState(ar_session_.get(), trackable, &tracking_state);

    // Compares tracking state to try and preempt an ARCore error by verifying
    // we can attach to the id.
    if (tracking_state != AR_TRACKING_STATE_TRACKING) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Cannot anchor to ArTrackableId (%ld) with tracking status (%s)",
          id->GetLow(),
          GetTrackingStateName(ToImpTrackingState(tracking_state))));
    }

    // Does the anchor attachment.
    if (ArStatus status = ArTrackable_acquireNewAnchor(
            ar_session_.get(), trackable, pose.get(), &anchor);
        status != AR_SUCCESS) {
      return absl::InternalError(absl::StrFormat(
          "Anchor failed to attach to ArTrackableId (%ld), trackable "
          "status (%d), with tracking status (%s).",
          id->GetLow(), status,
          GetTrackingStateName(ToImpTrackingState(tracking_state))));
    }
  }

  // Note: we explicitly do not want to make a UniqueArAnchor here or release
  // the anchor because we will do that when the client deletes it (see
  // DestroyAnchor(anchor) below).
  return ConvertAnchor(anchor);
}

void ArSessionNativeArCore::DestroyAnchor(ArAnchor anchor) {
  IMP_TRACE();
  ArAnchor_* native_anchor =
      reinterpret_cast<ArAnchor_*>(anchor.GetId().GetLow());
  ArAnchor_detach(ar_session_.get(), native_anchor);
  ArAnchor_release(native_anchor);
}

void ArSessionNativeArCore::SetDepthTextureCreatedHandler(
    std::function<void(DepthData const&)> handler) {
  if (depth_texture_controller_) {
    depth_texture_controller_->SetTextureCreatedHandler(
        [handler](imp::Texture const& texture) {
          DepthData data;
          data.depth_texture = &texture;
          handler(data);
        });
  } else {
    IMP_LOG(imp::WARNING)
        << "attempting to set depth texture created handler when depth is not "
           "initialized";
  }
}

float ArSessionNativeArCore::GetDepthRegionConfidence(int32_t min_x,
                                                      int32_t max_x,
                                                      int32_t width,
                                                      int32_t height) {
  IMP_TRACE();
  auto confidence = 0.0f;
  ArFrame_getDepthRegionConfidence(ar_session_.get(), ar_frame_.get(), min_x,
                                   max_x, width, height, &confidence);
  return confidence;
}

bool ArSessionNativeArCore::IsDepthSupported() const {
  return depth_texture_controller_.has_value();
}

bool ArSessionNativeArCore::IsHdrLightingEnabled() const {
  return deeplight_controller_.IsHdrLightingEnabled();
}
std::vector<float3> ArSessionNativeArCore::GetSphericalHarmonicsLighting() {
  return deeplight_controller_.GetSphericalHarmonicsLighting();
}

std::unique_ptr<HdrLighting> ArSessionNativeArCore::GetHdrLighting() {
  return deeplight_controller_.GetHdrLighting(ar_session_.get());
}

float4 ArSessionNativeArCore::GetAmbientLighting() {
  return deeplight_controller_.GetAmbientLighting(ar_session_.get());
}

ArAnchor ArSessionNativeArCore::ConvertAnchor(ArAnchor_* native_anchor) const {
  ArTrackingState tracking_state;
  ArAnchor_getTrackingState(ar_session_.get(), native_anchor, &tracking_state);
  ArAnchor_getPose(ar_session_.get(), native_anchor, ar_pose_.get());

  mat4f transform = GetTransformFromPose(ar_session_, ar_pose_.get());
  return ArAnchor(ArTrackableId(reinterpret_cast<int64_t>(native_anchor)),
                  ToImpTrackingState(tracking_state), transform);
}

std::vector<ArAnchor> ArSessionNativeArCore::ConvertAnchors(
    const ArAnchorListHelper& anchor_list) const {
  IMP_TRACE();
  std::vector<ArAnchor> anchors;
  anchor_list.ForEach([this, &anchors](UniqueArAnchor anchor_ptr) {
    anchors.push_back(ConvertAnchor(anchor_ptr.get()));
  });
  return anchors;
}

std::vector<ArAnchor> ArSessionNativeArCore::GetUpdatedAnchors() const {
  IMP_TRACE();
  ArAnchorListHelper anchor_list(ar_session_.get());
  ArFrame_getUpdatedAnchors(ar_session_.get(), ar_frame_.get(),
                            anchor_list.get());
  return ConvertAnchors(anchor_list);
}

TrackingFailureReason ArSessionNativeArCore::GetTrackingFailureReason() const {
  ArTrackingFailureReason out_tracking_failure_reason;
  ArCamera_getTrackingFailureReason(ar_session_.get(), ar_camera_.get(),
                                    &out_tracking_failure_reason);
  return ToImpTrackingFailureReason(out_tracking_failure_reason);
}

ArPointCloud ArSessionNativeArCore::GetPointCloud() const {
  const ArSession* session = ar_session_.get();
  ::ArPointCloud* ar_core_point_cloud;

  ArFrame_acquirePointCloud(session, ar_frame_.get(), &ar_core_point_cloud);
  int32_t num_points = 0;
  ArPointCloud_getNumberOfPoints(session, ar_core_point_cloud, &num_points);
  ArPointCloud point_cloud;
  point_cloud.reserve(num_points);

  // Obtain pointer to data.
  const float* point_cloud_data_raw;
  ArPointCloud_getData(session, ar_core_point_cloud, &point_cloud_data_raw);

  // Retrieve point IDs.
  const int32_t* point_ids_raw;
  ArPointCloud_getPointIds(session, ar_core_point_cloud, &point_ids_raw);

  for (int i = 0; i < num_points; ++i) {
    const int j = i * 4;
    const ArPointCloudPoint point = {
        .position = float3(point_cloud_data_raw[j], point_cloud_data_raw[j + 1],
                           point_cloud_data_raw[j + 2]),
        .id = static_cast<uint64_t>(point_ids_raw[i]),
        .confidence = point_cloud_data_raw[j + 3],
    };

    point_cloud.emplace_back(point);
  }

  ArPointCloud_release(ar_core_point_cloud);

  return point_cloud;
}

}  // namespace ar
}  // namespace imp
