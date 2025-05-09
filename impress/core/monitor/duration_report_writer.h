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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_DURATION_REPORT_WRITER_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_DURATION_REPORT_WRITER_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/monitor/duration_measurement_data.h"
#include "core/monitor/impress_log_helpers.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

// Converts DurationMeasurementData to any proto with the matching fields.
template <typename T>
absl::Status EncodeDurationReportProto(
    const imp::DurationMeasurementData* performance_metric_src,
    T* performance_metric_dest) {
  if (!performance_metric_src) {
    return absl::OkStatus();
  }

  if (performance_metric_src->GetSampleCount() > 0) {
    performance_metric_dest->set_count(
        performance_metric_src->GetSampleCount());

    MP_RETURN_IF_ERROR(
        EncodeDurationProto(performance_metric_src->GetTotalDuration(),
                            performance_metric_dest->mutable_total_time()));
    if (performance_metric_src->GetEnablePercentiles()) {
      MP_RETURN_IF_ERROR(EncodeDurationProto(
          performance_metric_src->GetLongestSampleDuration(),
          performance_metric_dest->mutable_percentile100_time()));
      MP_RETURN_IF_ERROR(EncodeDurationProto(
          performance_metric_src->GetShortestSampleDuration(),
          performance_metric_dest->mutable_percentile0_time()));
    }
    if (performance_metric_src->GetEnableAverages()) {
      MP_RETURN_IF_ERROR(EncodeDurationProto(
          performance_metric_src->GetMovingAverageSampleDuration(),
          performance_metric_dest->mutable_moving_avg_time()));
    }
  }

  if (performance_metric_src->GetCancelledSampleCount() > 0) {
    performance_metric_dest->set_cancelled_count(
        performance_metric_src->GetCancelledSampleCount());
  }

  if (performance_metric_src->GetHistogram().has_value()) {
    const imp::SimpleHistogram& histogram_src =
        performance_metric_src->GetHistogram().value();

    typedef typename std::remove_const<typename std::remove_reference<
        decltype(performance_metric_dest->histogram())>::type>::type
        HistogramProtoType;

    HistogramProtoType* histogram_dest =
        performance_metric_dest->mutable_histogram();
    // map_of_ms_to_count may be a google::protobuf::Map or
    // ::google::protobuf_opensource::Map
    auto* map_of_ms_to_count = histogram_dest->mutable_ms_vs_count();
    for (const auto& src_bucket : histogram_src.GetBuckets()) {
      if (src_bucket.count) {
        map_of_ms_to_count->insert(
            {src_bucket.bucket_lower_bound, src_bucket.count});
      }
    }

    if (histogram_src.GetCountBelowMin() > 0) {
      histogram_dest->set_count_below_range(histogram_src.GetCountBelowMin());
    }
  }
  return absl::OkStatus();
}
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_DURATION_REPORT_WRITER_H_
