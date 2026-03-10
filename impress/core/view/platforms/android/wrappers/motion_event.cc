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

#include <jni.h>

#include <utility>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "core/common/jni_helpers.h"
#include "core/input/pointer_event.h"
#include "core/math/math.h"

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

  JniUniquePtr<jobject> motion_event =
      CallStaticObjectMethod(obtain_, 0, 0, action_value, surface_coordinates.x,
                             surface_coordinates.y, 0);
  SetSelf(LocalToGlobalRef(std::move(motion_event)));
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

  JniUniquePtr<jobject> motion_event =
      CallStaticObjectMethod(obtain_, 0, 0, action_value, surface_coordinates.x,
                             surface_coordinates.y, 0);
  SetSelf(LocalToGlobalRef(std::move(motion_event)));
}

MotionEvent::MotionEvent(JNIEnv* env, float2 surface_coordinates, Action action,
                         ToolType tool_type, float2 delta)
    : JavaWrapper(env, "android/view/MotionEvent") {
  // Create a PointerProperties with the id and toolType set.
  JniUniquePtr<jclass> pointer_properties_class = WrapJni(
      env, Env()->FindClass("android/view/MotionEvent$PointerProperties"));
  jmethodID pointer_properties_constructor =
      Env()->GetMethodID(pointer_properties_class.get(), "<init>", "()V");
  JniUniquePtr<jobject> pointer_properties =
      WrapJni(Env(), Env()->NewObject(pointer_properties_class.get(),
                                      pointer_properties_constructor));
  int tool_type_value = static_cast<int>(tool_type);
  jfieldID pointer_properties_pointer_id_field =
      Env()->GetFieldID(pointer_properties_class.get(), "id", "I");
  Env()->SetIntField(pointer_properties.get(),
                     pointer_properties_pointer_id_field, 0);
  jfieldID pointer_properties_tool_type_field =
      Env()->GetFieldID(pointer_properties_class.get(), "toolType", "I");
  Env()->SetIntField(pointer_properties.get(),
                     pointer_properties_tool_type_field, tool_type_value);

  // Create PointerProperties array with the single element created above.
  JniUniquePtr<jobjectArray> pointer_properties_array = WrapJni(
      env, Env()->NewObjectArray(1, pointer_properties_class.get(), nullptr));
  Env()->SetObjectArrayElement(pointer_properties_array.get(), 0,
                               pointer_properties.get());

  // Create a PointerCoords object with the hit surface coordinates and the
  // scroll amounts in their axes.
  JniUniquePtr<jclass> pointer_coords_class =
      WrapJni(env, Env()->FindClass("android/view/MotionEvent$PointerCoords"));
  jmethodID pointer_coords_constructor =
      Env()->GetMethodID(pointer_coords_class.get(), "<init>", "()V");
  JniUniquePtr<jobject> pointer_coords = WrapJni(
      Env(),
      Env()->NewObject(pointer_coords_class.get(), pointer_coords_constructor));
  jfieldID pointer_coords_x_field =
      Env()->GetFieldID(pointer_coords_class.get(), "x", "F");
  Env()->SetFloatField(pointer_coords.get(), pointer_coords_x_field,
                       surface_coordinates.x);
  jfieldID pointer_coords_y_field =
      Env()->GetFieldID(pointer_coords_class.get(), "y", "F");
  Env()->SetFloatField(pointer_coords.get(), pointer_coords_y_field,
                       surface_coordinates.y);
  jint axis_vscroll =
      GetStaticIntField(GetStaticFieldHandle("AXIS_VSCROLL", "I"));
  jint axis_hscroll =
      GetStaticIntField(GetStaticFieldHandle("AXIS_HSCROLL", "I"));
  jmethodID pointer_coords_set_axis_value_method =
      Env()->GetMethodID(pointer_coords_class.get(), "setAxisValue", "(IF)V");
  Env()->CallVoidMethod(pointer_coords.get(),
                        pointer_coords_set_axis_value_method, axis_vscroll,
                        -delta.y);
  Env()->CallVoidMethod(pointer_coords.get(),
                        pointer_coords_set_axis_value_method, axis_hscroll,
                        delta.x);
  // Create PointerCoords array with single element created above.
  JniUniquePtr<jobjectArray> pointer_coords_array = WrapJni(
      env, Env()->NewObjectArray(1, pointer_coords_class.get(), nullptr));
  Env()->SetObjectArrayElement(pointer_coords_array.get(), 0,
                               pointer_coords.get());

  int action_value = static_cast<int>(action);
  obtain_ = GetStaticMethodHandle(
      "obtain",
      "(JJII[Landroid/view/MotionEvent$PointerProperties;[Landroid/view/"
      "MotionEvent$PointerCoords;IIFFIIII)Landroid/view/MotionEvent;");

  JniUniquePtr<jobject> motion_event = CallStaticObjectMethod(
      obtain_, 0, 0, action_value, 1, pointer_properties_array.get(),
      pointer_coords_array.get(), 0, 0, 0.f, 0.f, 0, 0, 0, 0);
  SetSelf(LocalToGlobalRef(std::move(motion_event)));
}

}  // namespace imp::android
