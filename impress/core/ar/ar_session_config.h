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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_SESSION_CONFIG_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_SESSION_CONFIG_H_

#include <string>

#include "absl/types/optional.h"

namespace imp {

class BaseView;

namespace ar {

struct ArSessionConfig {
  // Mode for the tracking configuration of the ArSession.
  enum TrackingMode {
    // Tracks the position of a device in relation to objects in the environment
    kWorld = 0,
    // Detects and tracks a face with the front-facing selfie camera.
    kFace,
  } tracking_mode = kWorld;

  enum LightingMode {
    // Corresponds to ARCore's Deeplight feature
    kHdr = 0,
    // Corresponds to ARCore's Ambient Light Estimation feature
    kAmbient,
    // Disabled.
    kLightingDisabled
  } lighting_mode = kHdr;

  enum DepthMode {
    // No depth information will be provied.
    kDisabled,
    // Provides depth information on supported devices based on hardware and
    // software sources. Adds significant computational load.
    // Note: On iOS, this mode will provide depth information depending on
    // device and version. It will return depth information for People
    // Occlusions only for devices > iPhone XS/XR.
    // TODO: Enable kAutomatic depth mode on iOS for iPhone 12+
    // with new ARKit depth API.
    kAutomatic,
  } depth_mode = kAutomatic;

  enum PlacementMode {
    // Perform hit tests with AR planes (disabling instant placement modes).
    kPlanePlacementMode,
    // Perform instant hit tests using augmented regions. See
    // (broken link).
    kInstantPlacementMode,
    // Perform instant hit tests on vertical surfaces. See
    // (broken link).
    kMagicalSurfaceMode,
  } placement_mode = kInstantPlacementMode;

  // Sets the light probe creation scheme, applies to iOS only.
  enum class EnvironmentTexturingMode {
    kAutomatic,
    kManual,
  } environment_texturing_mode = EnvironmentTexturingMode::kAutomatic;

  // The type of AR session to create.
  enum class SessionType {
    kNative,
    kPlayback,
  } session_type = SessionType::kNative;

  // android/arcore only
  // TODO: Improve the Impress AR playback capabilities.
  std::string dataset_path;
  // A lseek compatible uri of an ARCore recorded mp4 video to use as playback.
  // (broken link) for more
  // info.
  std::string playback_dataset_uri;

  // The number of textures to use for multi-texture rendering (see
  // (broken link)), if greater than 1.
  //
  // On Android, this will tell ARCore to use a ring buffer of textures that are
  // swapped between in a round-robin fashion for rendering the camera when the
  // number of textures is >1. If the number of textures is 1, then filament
  // will perform extra texture copies to synchronize the camera texture between
  // the user thread and the render thread. On all other platforms, this
  // currently does nothing.
  int num_camera_textures = 4;

  // If present, override the default ARCore frame delay; see
  // (broken link). Android only.
  absl::optional<int> frame_delay_override;

  // Desired texture height to score camera configs from ARCore (android-only).
  int desired_camera_texture_height = 1080;
  // Desired frame rate to score camera configs from ARCore (android-only).
  enum class DesiredFramerate {
    k30hz,
    k60hz,
  } desired_framerate = DesiredFramerate::k60hz;
};

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_SESSION_CONFIG_H_
