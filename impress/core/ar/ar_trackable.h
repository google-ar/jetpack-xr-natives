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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_H_

#include <functional>
#include <optional>

#include "absl/types/optional.h"
#include "core/ar/ar_trackable_id.h"
#include "core/math/mat.h"

namespace imp {
namespace ar {

// The cross-platform supported trackable types.
enum class TrackableType : uint32_t {
  kInvalid,
  kBase,
  kPoint,
  kPlane,
  kAnchor,
  kMagicalSurfacePoint,
  kPriorMap,
  kFace,
};

// The cross-platform supported tracking states.
enum class TrackingState : uint32_t {
  kTracking,
  kPaused,
  kStopped,
};

// The cross-platform supported tracking failure reasons of the AR Camera.
enum class TrackingFailureReason : uint32_t {
  kNone,
  kBadState,
  kLowLight,
  kExcessiveMotion,
  kInsufficientFeatures,
  kCameraUnavailable,
  kUnknown,
};

// A virtual representation of a real-world object or surface.
// Trackables can uniquely be identified by their id. All other information can
// change from frame to frame.
class ArTrackable {
 public:
  constexpr static TrackableType kType = TrackableType::kBase;

  ArTrackable() {}
  explicit ArTrackable(
      ArTrackableId id, TrackableType kType, TrackingState tracking_state,
      const mat4f& transform,
      absl::optional<std::shared_ptr<void>> underlying_resource = {})
      : transform_(transform),
        id_(id),
        trackable_type_(kType),
        tracking_state_(tracking_state),
        underlying_resource_(underlying_resource) {}
  virtual ~ArTrackable() {}

  ArTrackableId GetId() const { return id_; }
  mat4f& GetTransform() { return transform_; }
  const mat4f& GetTransform() const { return transform_; }
  // Gets the current state of this trackable in the underlying Ar Platorm.
  TrackingState GetTrackingState() const { return tracking_state_; }

  friend bool operator==(const ArTrackable& x, const ArTrackable& y) {
    return x.GetId() == y.GetId();
  }
  TrackableType GetType() { return trackable_type_; }

 protected:
  friend class ArSessionNativeDesktop;
  mat4f transform_;
  ArTrackableId id_;
  TrackableType trackable_type_;
  TrackingState tracking_state_;
  // Optional pointer keeps the underlying platform resource alive for the
  // lifetime of the trackable.
  absl::optional<std::shared_ptr<void>> underlying_resource_;
};

// Helper to special case base trackable types at compile time.
template <typename T>
constexpr bool IsBase() {
  return std::is_same_v<T, ArTrackable>;
}
}  // namespace ar
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_H_
