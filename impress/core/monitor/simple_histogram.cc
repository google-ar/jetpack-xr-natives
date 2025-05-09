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

#include "core/monitor/simple_histogram.h"

#include <stdint.h>

#include <algorithm>
#include <limits>
#include <optional>
#include <vector>

namespace imp {
namespace {
// returns true if lhs - rhs could overflow.
bool SubtractOverflows(int64_t lhs, int64_t rhs) {
  return ((rhs > 0 && lhs < std::numeric_limits<int64_t>::lowest() + rhs) ||
          (rhs < 0 && lhs > std::numeric_limits<int64_t>::max() + rhs));
}
}  // namespace

SimpleHistogram::SimpleHistogram(int64_t lower_bound, int64_t bucket_width,
                                 int64_t bucket_count)
    : lower_bound_(lower_bound),
      bucket_width_(bucket_width),
      bucket_count_(bucket_count),
      below_min_(0) {
  Reset();
}

void SimpleHistogram::Add(int64_t value) {
  int32_t index = IndexForValue(value);
  if (index != -1) {
    buckets_[index].count++;
  } else {
    below_min_++;
  }
}

void SimpleHistogram::Reset() {
  below_min_ = 0;
  buckets_.clear();
  int64_t bucket_lower_bound = lower_bound_;
  int64_t bucket_upper_bound = lower_bound_ + bucket_width_;
  for (int64_t i = 0; i < bucket_count_; ++i) {
    buckets_.emplace_back(Bucket{.bucket_lower_bound = bucket_lower_bound,
                                 .bucket_upper_bound = bucket_upper_bound,
                                 .count = 0});
    bucket_lower_bound = bucket_upper_bound;
    bucket_upper_bound = bucket_lower_bound + bucket_width_;
  }
}

int32_t SimpleHistogram::IndexForValue(int64_t value) {
  if (value < lower_bound_ || SubtractOverflows(value, lower_bound_)) {
    return -1;
  }

  int64_t bucket_index = (value - lower_bound_) / bucket_width_;

  // high values are added to the last bucket.
  bucket_index =
      std::min(static_cast<int64_t>(buckets_.size()) - 1, bucket_index);

  if (bucket_index < 0) {
    return -1;
  }

  return bucket_index;
}

int64_t SimpleHistogram::CountInRange(
    std::optional<int64_t> lower_bound,
    std::optional<int64_t> upper_bound) const {
  if (buckets_.empty()) return 0;
  uint64_t count = 0;

  for (const auto& bucket : buckets_) {
    if (lower_bound.has_value() &&
        lower_bound.value() >= bucket.bucket_upper_bound) {
      continue;
    }
    if (upper_bound.has_value() &&
        upper_bound.value() < bucket.bucket_lower_bound) {
      continue;
    }
    count += bucket.count;
  }
  return count;
}
}  // namespace imp
