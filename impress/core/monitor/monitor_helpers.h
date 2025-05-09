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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_MEASUREMENT_NAMES_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_MEASUREMENT_NAMES_H_

#include "absl/strings/string_view.h"

namespace imp {

// Measures the time between successive calls to View::OnHostPostRender
// Calls to OnHostPostRender indicate that the impress frame loop render made
// it all the way through to end frame calls without being skipped. This is an
// observation from the impress main thread or cpu calls, it does not directly
// measure how much work the GPU is doing or GPU frame completion. With that
// caveat, this is a good measurement to calculate FPS.
inline constexpr absl::string_view kFramePresented = "frame_presented_timing";
// Tracks the delta_time passed into View::Advance().
// Note: This tracks calls into the Impress frame loop, not actual FPS.
// Impress may skip rendering frames for various reasons, and frame skips will
// not be reflected here.
inline constexpr absl::string_view kViewFrameTime = "view_frame_timing";
// Measures the time spent in View::Advance().
inline constexpr absl::string_view kViewAdvance = "view_advance_timing";
inline constexpr absl::string_view kArSessionUpdate = "ar_update_timing";
// Measures the time between calls to Filament::Engine::beginFrame and endFrame.
inline constexpr absl::string_view kFilamentFrameTiming =
    "filament_frame_timing";
// Measures the time spent in View::AdvanceForegroundExecutor().
inline constexpr absl::string_view kForegroundExecutorTiming =
    "foreground_executor_timing";
// Measures the time spent in View::AdvanceBackgroundExecutor().
inline constexpr absl::string_view kBackgroundExecutorTiming =
    "background_executor_timing";
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_MEASUREMENT_NAMES_H_
