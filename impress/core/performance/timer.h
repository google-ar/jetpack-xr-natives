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
#ifndef THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_TIMER_H_
#define THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_TIMER_H_

#include <cstdint>

#include "absl/strings/string_view.h"

namespace imp {
// Timer is a helper class for profiling a block of code. It adds a sample to
// the profiler when it is constructed and records its end time in that sample
// when it is destroyed.
// Adding a Timer to a scope will record a sample over that scope.
class Timer {
 public:
  explicit Timer(absl::string_view name);
  ~Timer();
  void Stop();

 private:
  const absl::string_view name_;
  bool stopped_;
  uint64_t id_;
};
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_TIMER_H_
