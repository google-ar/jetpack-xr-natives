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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_GESTURE_POINTER_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_GESTURE_POINTER_UTILS_H_

#include <set>

#include "core/input/input_manager.h"
#include "core/input/pointer_event.h"
#include "core/view/utils/macros.h"

namespace imp {
/**
 * Utility class to help track pointers (by retaining and releasing pointer IDs)
 * so that they can only be used in one gesture at a time.
 */
class GesturePointerUtils {
 public:
  // ScopedPointerRetainer is returned by GesturePointerUtils::RetainPointer.
  // Call ReleasePointer on this object to explicitly release the pointer ID.
  // The pointer ID will automatically be released when this object goes out
  // of scope.
  class IMP_WARN_UNUSED_RESULT ScopedPointerRetainer {
   public:
    ScopedPointerRetainer();
    ScopedPointerRetainer(GesturePointerUtils* utils_, Pointer::Id id_);
    
    ~ScopedPointerRetainer();

    // Explicitly releases the pointer ID.
    void ReleasePointer();

   private:
    static constexpr Pointer::Id kNullPointerId = UINT32_MAX;
    GesturePointerUtils* utils_;
    Pointer::Id id_;

    
  };

  GesturePointerUtils();
  // Retain pointer IDs so that the pointers are only used in one gesture at a
  // time.
  ScopedPointerRetainer RetainPointer(Pointer::Id id);
  bool IsPointerRetained(Pointer::Id id) const;

 private:
  std::set<Pointer::Id> retained_pointer_ids_;
  void ReleasePointer(Pointer::Id id);
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_GESTURE_POINTER_UTILS_H_
