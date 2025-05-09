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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_AR_BASE_AR_COMPONENT_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_AR_BASE_AR_COMPONENT_H_

#include <optional>

#include "core/common/log.h"
#include "core/ar/ar_trackable.h"
#include "core/ar/ar_trackable_handle.h"
#include "core/collision/collider_mask_helpers.h"
#include "core/collision/collision_flags.h"
#include "core/common/enum_flags.h"
#include "core/common/filament_helpers.h"
#include "core/ncsb/component.h"

namespace imp {
// Provides base class functionality to Ar types. Implements commonly
// used APIs that are the same for all trackables.
template <typename Trackable>
class BaseArComponent : public Component,
                        public ColliderMaskHelpers<BaseArComponent<Trackable>> {
 public:
  using TrackableType = Trackable;
  explicit BaseArComponent(CollisionMask mask = CollisionMask::kNone)
      : collision_flags_(mask) {}
  void Setup(ar::ArTrackableHandle<Trackable> trackable) {
    trackable_ = trackable;
  }

  void UpdateLocation() {
    float3 position;
    quatf rotation;
    float3 scale;
    Decompose(trackable_->GetTransform(), &position, &rotation, &scale);
    GetNode()->SetWorldPosition(position);
    GetNode()->SetWorldRotation(rotation);
    // Do NOT set the scale, as there should be no scale component anyway and
    // this would prevent anchored content from being scaled by the app.
  }

  ar::TrackingState GetTrackingState() {
    if (trackable_) {
      return trackable_->GetTrackingState();
    }
    return ar::TrackingState::kStopped;
  }

  ar::ArTrackableId GetTrackableId() const {
    if (trackable_) {
      return trackable_->GetId();
    }
    IMP_LOG(imp::FATAL) << "Trackable is invalid, is the ArSession running?";
    return ar::ArTrackableId::InvalidId();
  }
  ar::ArTrackableHandle<Trackable> GetTrackable() { return trackable_; }

 protected:
  friend class ColliderMaskHelpers<BaseArComponent<Trackable>>;
  ar::ArTrackableHandle<Trackable> trackable_;
  Flags<CollisionMask> collision_flags_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_AR_BASE_AR_COMPONENT_H_
