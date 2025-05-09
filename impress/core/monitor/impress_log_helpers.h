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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_IMPRESS_LOG_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_IMPRESS_LOG_HELPERS_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "core/monitor/simple_histogram.h"

namespace imp {

// ValidateDuration is based on protoutil's util_time::Validate, but works on
// our local protobuf as well.  Validation requirements documented at:
// (broken link)
absl::Status ValidateDuration(int64_t sec, int64_t ns);

// ValidateTimestamp is based on protoutil's util_time::Validate, but works on
// our local protobuf as well. Validation requirements documented at:
// (broken link)
absl::Status ValidateTimestamp(int64_t sec, int64_t ns);

// Converts absl::Duration to any matching proto with seconds and nanos
template <typename T>
absl::StatusOr<T> EncodeDurationProto(absl::Duration d) {
  T duration_proto;

  // s and n may both be negative, per the Duration proto spec.
  const int64_t s = absl::IDivDuration(d, absl::Seconds(1), &d);
  const int64_t n = absl::IDivDuration(d, absl::Nanoseconds(1), &d);
  duration_proto.set_seconds(s);
  duration_proto.set_nanos(n);
  absl::Status status =
      ValidateDuration(duration_proto.seconds(), duration_proto.nanos());
  if (!status.ok()) return status;

  return duration_proto;
}

// Converts absl::Duration to any matching proto with seconds and nanos
template <typename T>
absl::Status EncodeDurationProto(absl::Duration src, T* dest) {
  absl::StatusOr<T> result = EncodeDurationProto<T>(src);
  if (!result.ok()) {
    return result.status();
  }
  *(dest) = *std::move(result);
  return absl::OkStatus();
}

// Converts absl::Time to any matching proto with seconds and nanos
template <typename T>
absl::StatusOr<T> EncodeTimestampProto(absl::Time t) {
  T timestamp_proto;
  const int64_t s = absl::ToUnixSeconds(t);
  timestamp_proto.set_seconds(s);
  timestamp_proto.set_nanos((t - absl::FromUnixSeconds(s)) /
                            absl::Nanoseconds(1));
  absl::Status status =
      ValidateTimestamp(timestamp_proto.seconds(), timestamp_proto.nanos());
  if (!status.ok()) return status;
  return timestamp_proto;
}

// Converts SimpleHistogram to any proto with the matching fields.
template <typename T>
absl::StatusOr<T> EncodeHistogramProto(const SimpleHistogram& hist) {
  T histogram_proto;

  // encoded_buckets may be a google::protobuf::Map or ::google::protobuf_opensource::Map
  auto* encoded_buckets = histogram_proto.mutable_ms_vs_count();
  for (const auto& src_bucket : hist.GetBuckets()) {
    if (src_bucket.count) {
      encoded_buckets->insert(
          {src_bucket.bucket_lower_bound, src_bucket.count});
    }
  }

  if (hist.GetCountBelowMin() > 0) {
    histogram_proto.set_count_below_range(hist.GetCountBelowMin());
  }

  return histogram_proto;
}
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_IMPRESS_LOG_HELPERS_H_
