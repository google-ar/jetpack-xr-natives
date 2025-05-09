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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_ANCHOR_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_ANCHOR_H_

#include "core/ar/ar_trackable.h"
#include "core/math/mat.h"

namespace imp {
namespace ar {

class ArAnchor : public ArTrackable {
  constexpr static TrackableType kType = TrackableType::kAnchor;

 public:
  ArAnchor()
      : ArTrackable(ArTrackableId(0), kType, TrackingState::kStopped, mat4f()) {
  }
  ArAnchor(ArTrackableId id, TrackingState tracking_state, mat4f transform)
      : ArTrackable(id, kType, tracking_state, transform) {}
};

// Helper to special case trackable types at compile time.
template <typename T>
constexpr bool IsAnchor() {
  return std::is_same_v<T, ArAnchor>;
}

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_ANCHOR_H_
