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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_HELPERS_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/math/mat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/monitor/monitor.h"
#include "core/monitor/monitor_summary.h"
#include "core/view/base_view.h"
#include "core/view/platforms/xr_android/openxr_includes.h"

namespace imp {
// The number of frames between collection of samples for the MonitorSummary.
// Assuming 75 FPS (midpoint between our suported framerates [60, 90]), and
// updating at one third of the frame rate.
constexpr int kUpdatesPerCollection = 25;
// The estimated FPS of the device. Used to estimate the size of the sample
// windows.
constexpr int kEstimatedFps = 75;
// The number of seconds covered by samples in the first MovingAverage window.
constexpr float kFirstWindowSizeInSeconds = 1.5f;
// The number of seconds covered by samples in the second MovingAverage window.
constexpr float kSecondWindowSizeInSeconds = 5.f;
// The number of seconds covered by samples in the third MovingAverage window.
constexpr float kThirdWindowSizeInSeconds = 30.f;

// prefix for the average sample age metric in the MonitorSummary.
constexpr absl::string_view kXrAvgSampleAgePrefix = "  average sample age: ";
// prefix for the percentage of frames with SysUI display enabled in the
// MonitorSummary.
constexpr absl::string_view kXrPercentageOfFramesWithSysUIDisplayEnabledPrefix =
    "  percent of frames with SysUI display enabled: ";
// prefix for the average ms per frame submitted in the MonitorSummary.
constexpr absl::string_view kXrMsPerFrameSubmittedPrefix =
    "  ms per frame submitted: ";
// prefix for the average ms per frame scheduled in the MonitorSummary.
constexpr absl::string_view kXrMsPerFrameScheduledPrefix =
    "  ms per frame scheduled (requires headtracking): ";
// prefix for the average ms per frame on the CPU in the MonitorSummary.
constexpr absl::string_view kXrMsPerFrameCpu1Prefix =
    "  ms per frame cpu1/impress (elapsed-wait): ";
// prefix for the average ms per frame on the CPU in the MonitorSummary.
constexpr absl::string_view kXrMsPerFrameCpu2Prefix =
    "  ms per frame cpu2/filament (begin frame to end frame): ";
// prefix for the average percentage of CPU1 spent to produce the frame in
// the MonitorSummary.
constexpr absl::string_view kXrPercentCpu1Prefix =
    "  percent cpu1/impress utilization: ";
// prefix for the average percentage of CPU2 spent to produce the frame
// in the MonitorSummary.
constexpr absl::string_view kXrPercentCpu2Prefix =
    "  percent cpu2/filament utilization: ";
// Time spent in xrWaitFrame.  All calls to wait frame will be counted
// including frames that are skipped.
constexpr absl::string_view kXrWaitFrameTiming = "xr_wait_frame_timing";
// Time between successful calls to XrSessionHost::AdvanceFrame.  Incremented
// after a frame is fully rendered.
constexpr absl::string_view kXrBetweenFrameTiming = "xr_between_frame_timing";
// Time measured on the Filament thread between BeginFrame and EndFrame calls
// This approximates CPU time spent on the rendering thread.
constexpr absl::string_view kXrBeginFrameToEndFrame =
    "xr_begin_frame_to_end_frame";

// Time from the XrFrameState.predictedDisplayPeriod.  Early exit conditions
// such as frame pacing or tracking lost will not be counted in the predicted
// frames.
constexpr absl::string_view kXrScheduledFrameTiming =
    "xr_scheduled_frame_timing";

// Value representing the display state.
constexpr absl::string_view kXrDisplayEnabledStatistics = "xr_display_enabled";

namespace XrHelpers {
// kDisplayDisabled is a power saved mode which will not render or composite
// the frame.
enum class DisplayState { kDisplayEnabled, kDisplayDisabled };
}  // namespace XrHelpers

namespace output {

// Set this to true to enable debug logging for Xr code.
constexpr bool kEnableXrLog = false;

// Used for debug logging in Xr related code.
//
// This is used so that we can check in logs that are disabled at compile time
// by default but can be enabled for debugging.
template <class... Args>
void Xr(const absl::FormatSpec<Args...>& format, Args&&... args) {
  if constexpr (kEnableXrLog) {
    IMP_LOG(imp::ERROR) << absl::StrFormat(format, std::forward<Args>(args)...);
  }
}

}  // namespace output

// Converts the result of Xr function calls into an absl::Status.
//
// Most Xr calls return an XrResult and use out parameters. This is useful for
// converting them into google style using absl::Status and absl::StatusOr.
absl::Status ToStatus(XrInstance instance, XrResult result);

// Converts an XrPosef into an Impress transform.
//
// Useful for assigning a pose coming from OpenXR to an Impress node.
Transform<float> ToTransform(XrPosef pose);

// Returns a translation matrix representing the relative position of the XrPose
// in model/local space.
mat4 GetLocalTranslation(const mat4& model_matrix, const XrPosef& xr_pose);

// Returns a transform representing the eye center.
// The eyes are assumed to be parallel.
Transform<float> GetEyeCenterTransform(const XrView& left_eye_view,
                                       const XrView& right_eye_view);

bool AreLeftAndRightFovAnglesMirrored(const XrFovf& left_eye_fov,
                                      const XrFovf& right_eye_fov);

bool AreXrQuaternionfsEqual(const XrQuaternionf& quaternion_1,
                            const XrQuaternionf& quaternion_2);

// Returns a projection matrix representing the frustum that encompasses the
// frustums of both the left and right eyes.
// This calculation assumes symmetry between eyes and will otherwise fatal.
// Eyes should be side-by-side and facing perpendicularly with identical FOV
// - see diagram.
mat4 GetEncompassingProjectionMatrix(const XrView& left_eye_view,
                                     const XrView& right_eye_view,
                                     float near_plane, float far_plane);

// Creates a projection matrix according to the OpenXR spec.
//
// This is based on the logic from XrMatrix4x4f_CreateProjectionFov.
mat4 GetProjectionMatrix(XrFovf fov, float nearZ, float farZ);

// Setup the MonitorSummary with specific reports for Xr.
void SetupXrTimingSummary(
    MonitorSummary& summary,
    std::unordered_map<std::string_view, MonitorSummary::CustomMetricHandle>&
        metricsStore);

void SetCustomEyeProjectionOnCamera(filament::Camera* camera,
                                    std::vector<XrView>& latest_views);

// Set the position of each eye relative to the local transform of the camera.
void SetEyeModelMatrixOnCamera(filament::Engine* engine,
                               filament::Camera* camera,
                               std::vector<XrView>& latest_views);

// Extract Xr specific frame timing and reset counters.
std::string DumpXrFrameTiming(Monitor& monitor, MonitorSummary& summary);

// Converts an XrVector3f into an Impress vector.
float3 ToVector3(XrVector3f xr_vector_3f);
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_HELPERS_H_
