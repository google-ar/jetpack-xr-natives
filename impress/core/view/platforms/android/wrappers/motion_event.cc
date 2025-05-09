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

#include "core/view/platforms/android/wrappers/motion_event.h"

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "core/common/platform_helpers.h"
#include "core/input/pointer_event.h"

namespace imp::android {

namespace {
absl::StatusOr<MotionEvent::Action> ConvertPointerEventTypeToAction(
    PointerEventType pointer_event_type) {
  switch (pointer_event_type) {
    case imp::PointerEventType::kDown:
      return MotionEvent::Action::kDown;
    case imp::PointerEventType::kUp:
      return MotionEvent::Action::kUp;
    case imp::PointerEventType::kMove:
      return MotionEvent::Action::kMove;
    default:
      return absl::InvalidArgumentError("Unsupported PointerEventType");
  }
}
}  // namespace

MotionEvent::MotionEvent(JNIEnv* env, float2 surface_coordinates, Action action)
    : JavaWrapper(env, "android/view/MotionEvent") {
  int action_value = static_cast<int>(action);

  obtain_ =
      GetStaticMethodHandle("obtain", "(JJIFFI)Landroid/view/MotionEvent;");

  SetSelf(Env()->NewGlobalRef(
      CallStaticObjectMethod(obtain_, 0, 0, action_value, surface_coordinates.x,
                             surface_coordinates.y, 0)));
}

MotionEvent::MotionEvent(JNIEnv* env, float2 surface_coordinates,
                         PointerEventType pointer_event_type)
    : JavaWrapper(env, "android/view/MotionEvent") {
  auto action = ConvertPointerEventTypeToAction(pointer_event_type);
  if (!action.ok()) {
    IMP_LOG(imp::FATAL) << action.status();
  }

  int action_value = static_cast<int>(*action);

  obtain_ =
      GetStaticMethodHandle("obtain", "(JJIFFI)Landroid/view/MotionEvent;");

  SetSelf(Env()->NewGlobalRef(
      CallStaticObjectMethod(obtain_, 0, 0, action_value, surface_coordinates.x,
                             surface_coordinates.y, 0)));
}

}  // namespace imp::android
