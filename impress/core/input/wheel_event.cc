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

#include "core/input/wheel_event.h"

#include "absl/time/time.h"
#include "core/math/vec.h"

namespace imp {

WheelEvent::WheelEvent(float2 delta, float2 point, absl::Duration elapsed_time)
    : delta_(delta), point_(point), elapsed_time_(elapsed_time) {}
}  // namespace imp
