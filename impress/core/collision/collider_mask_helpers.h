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

#ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLIDER_MASK_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLIDER_MASK_HELPERS_H_
#include "core/collision/collision_flags.h"
#include "core/common/crtp_helper.h"
#include "core/common/enum_flags.h"
#include "core/common/platform_helpers.h"

namespace imp {
// Provides collider mask helper functionality to all collider types.
// A collision mask is tested against collision flags which are provided during
// a hit test.
template <typename T>
class ColliderMaskHelpers : public CrtpHelper<T> {
 public:
  void SetMask(Flags<CollisionMask> mask) {
    this->GetUnderlying().collision_flags_ = mask;
  }
  void SetMask(CollisionMask mask) { SetMask(ToFlags(mask)); }
  Flags<CollisionMask> GetMask() const {
    return this->GetUnderlying().CollisionFlags();
  }
  bool TestCollisionFlags(Flags<CollisionMask> mask) const {
    return static_cast<bool>(this->GetUnderlying().collision_flags_ & mask);
  }

 private:
  friend T;
  ColliderMaskHelpers() {}
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLIDER_MASK_HELPERS_H_
