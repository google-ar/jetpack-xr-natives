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

#include "core/monitor/performance_report_writer.h"

#include <type_traits>

#include "logs/proto/vr/arcore/impress/impress_log.pb.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/monitor/duration_measurement_data.h"
#include "core/monitor/duration_report_writer.h"
#include "core/monitor/impress_log_helpers.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor.h"
#include "core/monitor/monitor_helpers.h"
#include "mediapipe/framework/port/status_macros.h"
#include "mediapipe/framework/deps/clock.h"

namespace imp {

using PerformanceReport = ::logs::proto::vr::arcore::impress::PerformanceReport;
using DurationReport = ::logs::proto::vr::arcore::impress::DurationReport;

absl::StatusOr<PerformanceReport> CreatePerformanceReport(Monitor& monitor) {
  PerformanceReport log;

  MP_ASSIGN_OR_RETURN(
      *log.mutable_start_time(),
      EncodeTimestampProto<logs::proto::vr::arcore::impress::Timestamp>(
          monitor.GetStartTime()));
  MP_ASSIGN_OR_RETURN(
      *log.mutable_total_duration(),
      EncodeDurationProto<logs::proto::vr::arcore::impress::Duration>(
          monitor.GetClock()->TimeNow() - monitor.GetStartTime()));

  auto set_duration_report =
      [&monitor, &log](absl::string_view name,
                       DurationReport* (PerformanceReport::*generator_func)())
      -> absl::Status {
    MeasurementData::MeasurementId id = monitor.GetMeasurementId(name);
    auto* data =
        static_cast<DurationMeasurementData*>(monitor.GetMeasurementData(id));
    if (data &&
        (data->GetSampleCount() > 0 || data->GetCancelledSampleCount() > 0)) {
      MP_RETURN_IF_ERROR(EncodeDurationReportProto<DurationReport>(
          data, (log.*generator_func)()));
    }
    return absl::OkStatus();
  };

  MP_RETURN_IF_ERROR(set_duration_report(
      kViewFrameTime, &PerformanceReport::mutable_view_frame_timing));
  MP_RETURN_IF_ERROR(set_duration_report(
      kViewAdvance, &PerformanceReport::mutable_view_advance_timing));
  MP_RETURN_IF_ERROR(set_duration_report(
      kArSessionUpdate, &PerformanceReport::mutable_ar_session_update_timing));
  MP_RETURN_IF_ERROR(set_duration_report(
      kFilamentFrameTiming, &PerformanceReport::mutable_filament_frame_timing));
  MP_RETURN_IF_ERROR(set_duration_report(
      kForegroundExecutorTiming,
      &PerformanceReport::mutable_foreground_executor_timing));

  return log;
}

}  // namespace imp
