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

#include "core/view/framework/gestures/gesture_pointer_utils.h"

namespace imp {
GesturePointerUtils::GesturePointerUtils() : retained_pointer_ids_() {}

GesturePointerUtils::ScopedPointerRetainer GesturePointerUtils::RetainPointer(
    Pointer::Id id) {
  assert(!IsPointerRetained(id));
  retained_pointer_ids_.insert(id);
  return GesturePointerUtils::ScopedPointerRetainer(this, id);
}

bool GesturePointerUtils::IsPointerRetained(Pointer::Id id) const {
  return retained_pointer_ids_.find(id) != retained_pointer_ids_.end();
}

void GesturePointerUtils::ReleasePointer(Pointer::Id id) {
  retained_pointer_ids_.erase(id);
}

GesturePointerUtils::ScopedPointerRetainer::ScopedPointerRetainer()
    : utils_(nullptr), id_(kNullPointerId) {}

GesturePointerUtils::ScopedPointerRetainer::ScopedPointerRetainer(
    GesturePointerUtils* utils, Pointer::Id id)
    : utils_(utils), id_(id) {
  assert(utils_->IsPointerRetained(id_));
}

GesturePointerUtils::ScopedPointerRetainer::~ScopedPointerRetainer() {
  if (id_ != kNullPointerId) {
    ReleasePointer();
  }
}



void GesturePointerUtils::ScopedPointerRetainer::ReleasePointer() {
  if (utils_ && id_ != kNullPointerId && utils_->IsPointerRetained(id_)) {
    utils_->ReleasePointer(id_);
  }
  id_ = kNullPointerId;
  utils_ = nullptr;
}

}  // namespace imp
