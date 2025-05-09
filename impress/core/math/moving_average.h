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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_MOVING_AVERAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_MOVING_AVERAGE_H_
namespace imp {

/**
 * Calculates an exponentially weighted moving average for a series of data.
 */
class MovingAverage {
 public:
  static inline constexpr double kDefaultWeight = 0.9f;

  /**
   * Construct an object to track the exponentially weighted moving average for
   * a series of data. The weight is set to a default of 0.9, which is good for
   * data with lots of samples when the average should be resistant to spikes
   * (i.e. frame rate).
   *
   * The weight is a ratio between 0 and 1 that represents how much of the
   * previous average is kept compared to the new sample. With a weight of 0.9,
   * 90% of the previous average is kept and 10% of the new sample is added to
   * the average.
   *
   */
  explicit MovingAverage(double initial_sample)
      : MovingAverage(initial_sample, kDefaultWeight) {}

  /**
   * Construct an object to track the exponentially weighted moving average for
   * a series of data.
   *
   * The weight is a ratio between 0 and 1 that represents how much of the
   * previous average is kept compared to the new sample. With a weight of 0.9,
   * 90% of the previous average is kept and 10% of the new sample is added to
   * the average.
   */
  MovingAverage(double initial_sample, double weight) {
    average_ = initial_sample;
    weight_ = weight;
  }

  /** Add a sample and calculate a new average. */
  inline void AddSample(double sample) {
    average_ = weight_ * average_ + (1.0 - weight_) * sample;
  }

  /** Returns the current average for all samples. */
  inline double GetAverage() const { return average_; }

 private:
  double average_;
  double weight_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_MOVING_AVERAGE_H_
