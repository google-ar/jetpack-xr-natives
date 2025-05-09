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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_SESSION_NATIVE_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_SESSION_NATIVE_H_

#include <functional>
#include <optional>
#include <string_view>
#include <vector>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/ar/ar_frame.h"
#include "core/ar/ar_hdr_lighting.h"
#include "core/ar/ar_hit_result.h"
#include "core/ar/ar_point_cloud.h"
#include "core/ar/ar_session_config.h"
#include "core/ar/ar_trackable.h"
#include "core/ar/playback/ar_playback_data.h"
#include "core/async/future.h"
#include "core/collision/ray.h"
#include "core/window/window_rotation.h"

namespace imp {
namespace ar {

class ArFrame;

constexpr absl::string_view kPlatformArFailureUrl = "url.payload.ar_failure";
// If there is no session ID available from the platform, return this.
constexpr int4 kDefaultDebugSessionId = {1, 2, 3, 4};

// These two function types are used internally to allow unit tests to override
// session creation to create mocks instead of real platform sessions.
using CanCreateArSessionNativeFn =
    std::function<Future<absl::Status>(const BaseView* view)>;
using CreateArSessionNativeFn =
    std::function<Future<std::unique_ptr<ArSessionNative>>(
        BaseView* view, const ArSessionConfig& config)>;

// A pure virtual base session for any AR tracking system implementation.
// Different platforms must implement this to support AR (ex: Android, iOS).
// TODO: Rename this to ArSessionSpecificPlatform to avoid
// confusion with ARCore's internal ArSessionNative class.
class ArSessionNative {
 public:
  // Returns true if an AR session can be created on this platform.
  // Note: this has to be in the .h to avoid a backwards reference linker error.
  static Future<absl::Status> CanCreate(const BaseView* view,
                                        const ArSessionConfig& config) {
    CanCreateArSessionNativeFn& can_create_override_fn =
        GetCanCreateArSessionNativeOverrideFn();
    if (can_create_override_fn) {
      return can_create_override_fn(view);
    }
    return CanCreateForPlatform(view, config);
  }

  // Returns an ArSessionNative appropriate for the current platform.
  // Called by platform integration code to instantiate the AR session.
  // This makes it possible for a developer to write a cross-platform impress
  // app. without needing to write platform specific boilerplate to instantiate
  // the correct AR session on each platform.
  // Note: this has to be in the .h to avoid a backwards reference linker error.
  static Future<std::unique_ptr<ArSessionNative>> Create(
      BaseView* view, const ArSessionConfig& config) {
    if (auto f = CanCreate(view, config); !f.Ready() || !f.Get().ok()) {
      IMP_LOG(imp::FATAL) << "Can't create ArSessionNative! You must call and ensure "
                    "CanCreate() before Create()!";
    }

    CreateArSessionNativeFn& create_override_fn =
        GetCreateArSessionNativeOverrideFn();
    if (create_override_fn) {
      return create_override_fn(view, config);
    }
    return CreateForPlatform(view, config);
  }

  // Can be called to override what session is created. This allows for more
  // flexible control of the type of session used on different platforms or for
  // testing.
  static void SetCreateArSessionNativeOverride(
      CanCreateArSessionNativeFn can_create_session_fn,
      CreateArSessionNativeFn create_session_fn);

  virtual ~ArSessionNative() {}
  // This must be called after constructing and can fail.
  virtual void Initialize() {}
  // Pauses the tracking system.
  virtual void Pause() = 0;
  // Resumes tracking on a previously-paused session.
  virtual void Resume() = 0;
  // Advances the tracking session to get a new, updated frame.
  virtual absl::optional<ArFrame> Update(absl::Time submitted_timestamp) = 0;
  // Provides a unique identifier associated with the session.
  virtual uint4 GetDebugSessionId() { return kDefaultDebugSessionId; }

  // Returns true if SetDisplayGeometry was called this frame
  virtual bool GetDisplayGeometryChanged() const = 0;

  // Is called when display geometry for the app changes so the camera will
  // match.
  virtual void SetDisplayGeometry(window::WindowRotation orientation, int width,
                                  int height, float near, float far) = 0;
  // Notifies the native session of updates to transient transition parameters,
  // which are caused by e.g. device rotation on iPad.  See
  // `ViewTransitionParametersChangedEvent` in view_events.h for an explanation
  // of these fields.
  virtual void SetTransitionParameters(float2 view_scale,
                                       float counter_rotation) {}
  // Gets the uvs to use for the camera texture from normalized screen space.
  virtual void GetCameraTextureUVs(
      const absl::Span<const float2> normalized_screen_coordinates,
      std::vector<float2>* uvs_out) const = 0;
  // Gets a 2x3 affine matrix that transforms NDC coordinates to camera UV
  // coordinates.
  virtual std::array<float3, 2u> GetUvFromNdcTransform() const = 0;

  // Creates an anchored point whose position will be accurately tracked.
  // Call DestroyAnchor(anchor) when done with this anchor.
  // Optional ArTrackableId is for an associated object, like a plane, to which
  // the anchor will remain relatively fixed.
  virtual absl::StatusOr<ArAnchor> CreateAnchor(
      float3 position, quatf rotation, absl::optional<ArTrackableId> id) = 0;

  virtual absl::StatusOr<ArAnchor> CreateAnchor(
      double latitude_degrees, double longitude_degrees,
      double wgs84_relative_altitude_meters, quatf rotation) {
    return absl::UnimplementedError(
        "Geo-based anchors are not supported on this platform.");
  }

  virtual absl::StatusOr<ArAnchor> CreateEnvironmentProbeAnchor(
      float3 position, quatf rotation) {
    return absl::Status(
        absl::StatusCode::kUnimplemented,
        "CreateEnvironmentProbeAnchor not supported on this platform.");
  }

  // Destroys a previously-created anchor.
  virtual void DestroyAnchor(ArAnchor anchor) = 0;
  // Returns the latest available pose matrix with its associated timestamp.
  virtual absl::Status GetLatestModelMatrix(mat4f* matrix_out,
                                            absl::Time* timestamp_out) const {
    return absl::UnimplementedError("");
  }

  virtual std::vector<ArHitResult> HitTest(
      float2 screen_pos, absl::optional<float> guessed_distance,
      TrackableTuple* out_generated_trackables) = 0;
  virtual std::vector<ArHitResult> HitTestRay(
      const Ray& ray, TrackableTuple* out_generated_trackables) = 0;

  // Gets the tracking state of the AR Camera. If the state is anything other
  // than TRACKING the pose should not be considered useful. Changing the active
  // camera configuration may cause the tracking state on certain devices to
  // become permanently PAUSED. For consistent behavior across all supported
  // devices, release any previously created anchors and trackables when setting
  // a new camera config.
  virtual TrackingState GetCameraTrackingState() const = 0;

  struct DepthData {
    // Texture containing depth information used for rendering Ar occlusions.
    //
    // On Android, this contains the texture resulting from calling
    // ArFrame_acquireDepthImage.
    //
    // On iOS, this contains a texture where the x channel is ARKit's dilated
    // depth texture, and the y channel is ARKit's matte texture.
    const imp::Texture* depth_texture = nullptr;
  };

  // The depth texture from ARCore is not guaranteed to be immediately
  // available. This interface uses a callback so that a client does not need to
  // repeatedly poll for the filament texture being managed. If depth is not
  // supported or configured, it will not be called.
  virtual void SetDepthTextureCreatedHandler(
      std::function<void(DepthData const&)>) {}

  virtual float GetDepthRegionConfidence(int32_t min_x, int32_t max_x,
                                         int32_t width, int32_t height) {
    return 0.0f;
  }

  // Gets whether depth has been enabled on the AR session.
  virtual bool IsDepthSupported() const { return false; }

  // Checks whether HDR lighting is enabled and should be used by getting
  // lighting from GetHdrLighting.
  virtual bool IsHdrLightingEnabled() const { return false; }

  // Gets the latest lighting data for the current scene according to the
  // latest AR light estimate.  Returns nullptr if new scene lighting is
  // available.
  virtual std::unique_ptr<HdrLighting> GetHdrLighting() { return nullptr; }

  virtual std::vector<float3> GetSphericalHarmonicsLighting() { return {}; }
  // Gets the latest ambient light estimation data for the current scene
  // according to the latest AR light estimate.  Returns nullptr if new scene
  // lighting is available.
  virtual float4 GetAmbientLighting() { return float4(1.0f); }

  // Sets the method by which hit tests for object placement are performed.
  virtual void SetPlacementMode(ArSessionConfig::PlacementMode mode) {}

  // Gets the current tracking failure reason of the AR Camera.
  virtual TrackingFailureReason GetTrackingFailureReason() const {
    return TrackingFailureReason::kUnknown;
  }

  // Sets the synthetic AR sessions playback scene data.
  virtual void SetPlaybackScene(std::unique_ptr<ArPlaybackData> playback_data) {
    IMP_LOG(imp::FATAL) << "SetPlaybackScene not implemented on this platform.";
  }

  // Gets a set of estimated 3D points attached to real-world geometry
  // from the current frame.
  virtual ArPointCloud GetPointCloud() const { return {}; }

  // Returns true when the platform has temporarily revoked camera access (e.g.
  // when multiple foreground apps are active on iPad).
  virtual bool IsSessionInterrupted() const { return false; }

  // Returns true when the session is ready to render.
  virtual bool IsReadyToRender() const { return true; }

  // Starts recording the ar session to the specified uri file.
  virtual absl::Status StartRecording(absl::string_view dataset_uri) {
    return absl::UnimplementedError("Recording is not supported.");
  }

  // Stops the recording of the ar session.
  virtual absl::Status StopRecording() {
    return absl::UnimplementedError("Recording is not supported.");
  }

 private:
  // Gets a reference to the static CanCreateArSessionNative override function.
  static CanCreateArSessionNativeFn& GetCanCreateArSessionNativeOverrideFn();

  // Gets a reference to the static CreateArSessionNative override function.
  static CreateArSessionNativeFn& GetCreateArSessionNativeOverrideFn();

  // Each platform needs to implement this function.
  static void PreInitializeForPlatform(const BaseView* view);

  // Each platform needs to implement this function.
  static Future<absl::Status> CanCreateForPlatform(
      const BaseView* view, const ArSessionConfig& config);

  // Each platform needs to implement this function.
  static Future<std::unique_ptr<ArSessionNative>> CreateForPlatform(
      BaseView* view, const ArSessionConfig& config);
};

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_SESSION_NATIVE_H_
