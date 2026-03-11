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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_MOTION_EVENT_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_MOTION_EVENT_H_

#include "core/common/jni_helpers.h"
#include "core/input/pointer_event.h"
#include "core/math/vec.h"

namespace imp::android {

// JNI wrapper for the Android MotionEvent class.
class MotionEvent : public JavaWrapper {
 public:
  // See https://developer.android.com/reference/android/view/MotionEvent.
  // It's important to keep those values like this since the values of Action
  // and ToolType will be statically converted into int when passing to Java
  // side as values for actions.
  enum class Action : int {
    kUp = 1,
    kDown = 0,
    kMove = 2,
    kScroll = 8,
    kHoverEnter = 9,
    kHoverExit = 10,
    kHoverMove = 7,
  };

  enum class ToolType : int {
    kUnknown = 0,
    kFinger = 1,
    kStylus = 2,
    kMouse = 3,
    kEraser = 4,
  };

  // Constructs a MotionEvent by calling MotionEvent.obtain.
  MotionEvent(JNIEnv* env, float2 surface_coordinates, Action action);

  // Constructs a MotionEvent by calling MotionEvent.obtain.
  // This also converts PointerEventType to MotionEvent.Action
  MotionEvent(JNIEnv* env, float2 surface_coordinates,
              PointerEventType pointer_event_type);

  // Constructs a MotionEvent with scroll deltas.
  MotionEvent(JNIEnv* env, float2 surface_coordinates, Action action,
              ToolType tool_type, float2 scroll_delta);

 private:
  // https://developer.android.com/reference/android/view/MotionEvent#obtain(long,%20long,%20int,%20float,%20float,%20int)
  JniHandle obtain_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_MOTION_EVENT_H_
