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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_WINDOWED_AVERAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_WINDOWED_AVERAGE_H_

#include "absl/strings/str_format.h"
#include "core/math/almost_equal.h"
#include "core/math/moving_average.h"
#include "core/math/vec.h"

namespace imp {

// Maintains three weighted exponential moving average similar to load
// reported
// in linux for use in reporting.
class WindowedAverage {
 public:
  WindowedAverage(double initial_value, double first_window_weight,
                  double second_window_weight, double third_window_weight)
      : first_(initial_value, first_window_weight),
        second_(initial_value, second_window_weight),
        third_(initial_value, third_window_weight) {}

  inline void AddSample(double value) {
    first_.AddSample(value);
    second_.AddSample(value);
    third_.AddSample(value);
  }

  // This class can be printed using the absl string library.
  template <typename Sink>
  friend void AbslStringify(Sink& sink, const WindowedAverage& window) {
    absl::Format(&sink, "%4.2f / %4.2f / %4.2f", window.first_.GetAverage(),
                 window.second_.GetAverage(), window.third_.GetAverage());
  }

  // Returns true if averages are not stable over time.
  bool IsChanging() const {
    return !RoughlyEqual(first_.GetAverage(), second_.GetAverage()) ||
           !RoughlyEqual(second_.GetAverage(), third_.GetAverage());
  }

  float3 GetAverages() const {
    return float3(first_.GetAverage(), second_.GetAverage(),
                  third_.GetAverage());
  }

 private:
  MovingAverage first_;
  MovingAverage second_;
  MovingAverage third_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_WINDOWED_AVERAGE_H_
