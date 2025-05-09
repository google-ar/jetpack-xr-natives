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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_SIMPLE_HISTOGRAM_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_SIMPLE_HISTOGRAM_H_

#include <stdint.h>

#include <optional>
#include <vector>

#include "absl/types/span.h"

namespace imp {

// Places values into a vector of buckets for histogram reporting.
// Buckets use discrete values and are spaced at fixed width.
// The final bucket will contain all the values above the histogram.
class SimpleHistogram {
 public:
  struct Bucket {
    int64_t bucket_lower_bound;
    int64_t bucket_upper_bound;
    int64_t count;
  };

  SimpleHistogram(int64_t lower_bound, int64_t bucket_width,
                  int64_t bucket_count);

  // Increments the count on the matching bucket if value is in the valid range,
  // or increments the above/below valid range count.
  void Add(int64_t value);

  // Empties all buckets.
  void Reset();

  // Returns the lower bound which is the lowest discrete number inside the
  // valid range.
  int64_t GetLowerBound() const { return lower_bound_; }

  // Returns the width of each bucket.
  int64_t GetBucketWidth() const { return bucket_width_; }

  // Returns the count of values added below the valid range.
  int64_t GetCountBelowMin() const { return below_min_; }

  // Direct access to the underlying buckets.
  absl::Span<const Bucket> GetBuckets() const {
    return absl::Span<const Bucket>(buckets_.data(), buckets_.size());
  }

  // Returns the index of the bucket for the value, or -1 if the value is out of
  // range.
  int32_t IndexForValue(int64_t value);

  // Returns the count of values in the given range.
  // If upper bound or lower bound is not set, the range is unbounded in that
  // direction.
  int64_t CountInRange(std::optional<int64_t> lower_bound,
                       std::optional<int64_t> upper_bound) const;

 private:
  int64_t lower_bound_;
  int64_t bucket_width_;
  int64_t bucket_count_;
  int64_t below_min_;
  std::vector<Bucket> buckets_;
};

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_SIMPLE_HISTOGRAM_H_
