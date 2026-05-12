// Copyright 2025 Google LLC
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

#include <jni.h>

#include <memory>

#include "core/common/log.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/jni_helpers.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "split_engine/input/split_engine_input_event.h"
#include "split_engine/input/split_engine_input_event_hit_info.h"

#define JNI_METHOD_ACTIVITY(return_type, method_name) \
  IMP_JNI return_type JNICALL                         \
      Java_com_google_androidxr_splitengine_SplitEngineInputEvent_##method_name  // NOLINT

namespace {

template <class T>
using SplitEngineInputEventAllowlist =
    ::imp::JniAllowlist<T, android_xr::SplitEngineInputEvent>;

template <class T>
constexpr auto ToJava = &SplitEngineInputEventAllowlist<T>::ToJava;

template <class T>
constexpr auto FromJava = &SplitEngineInputEventAllowlist<T>::FromJava;

imp::NodeHandle ToNodeHandle(int node_id) {
  return imp::NodeHandle(utils::Entity::import(node_id));
}

android_xr::SplitEngineInputEvent::DispatchFlag GetInputEventDispatchFlag(
    int dispatch_flag) {
  // TODO: Confirm if multiple flags can be set, if so we need
  // a more complicated parser.
  switch (dispatch_flag) {
    using enum android_xr::SplitEngineInputEvent::DispatchFlag;
    // LINT.IfChange(dispatch_flag)
    case static_cast<int>(NONE):
    case static_cast<int>(CAPTURED_POINTER):
    case static_cast<int>(TWO_D):
      return static_cast<android_xr::SplitEngineInputEvent::DispatchFlag>(
          dispatch_flag);
    // LINT.ThenChange(//depot/google3/third_party/split_engine/input/split_engine_input_event.h:dispatch_flag)
    default:
      IMP_LOG(imp::WARNING) << "[split_engine_input_event_jni] Unknown dispatch flag: "
                   << dispatch_flag;
      return NONE;
  }
}

android_xr::SplitEngineInputEvent::DeviceType GetInputEventDeviceType(
    int device_type) {
  switch (device_type) {
    using enum android_xr::SplitEngineInputEvent::DeviceType;
    // LINT.IfChange(device_type)
    case static_cast<int>(UNKNOWN):
    case static_cast<int>(HEAD):
    case static_cast<int>(CONTROLLER):
    case static_cast<int>(HANDS):
    case static_cast<int>(MOUSE):
    case static_cast<int>(GAZE_AND_GESTURE):
    case static_cast<int>(DIRECT_TOUCH):
      return static_cast<android_xr::SplitEngineInputEvent::DeviceType>(
          device_type);
    // LINT.ThenChange(//depot/google3/third_party/split_engine/input/split_engine_input_event.h:device_type)
    default:
      IMP_LOG(imp::WARNING) << "[split_engine_input_event_jni] Unknown device_type: "
                   << device_type;
      return UNKNOWN;
  }
}

android_xr::SplitEngineInputEvent::PointerType GetInputEventPointerType(
    int pointer_type) {
  switch (pointer_type) {
    using enum android_xr::SplitEngineInputEvent::PointerType;
    // LINT.IfChange(pointer_type)
    case static_cast<int>(DEFAULT):
    case static_cast<int>(LEFT):
    case static_cast<int>(RIGHT):
    case static_cast<int>(HEAD):
    case static_cast<int>(EYE):
      return static_cast<android_xr::SplitEngineInputEvent::PointerType>(
          pointer_type);
    // LINT.ThenChange(//depot/google3/third_party/split_engine/input/split_engine_input_event.h:pointer_type)
    default:
      IMP_LOG(imp::WARNING) << "[split_engine_input_event_jni] Unknown pointer_type: "
                   << pointer_type;
      return DEFAULT;
  }
}

android_xr::SplitEngineInputEvent::Action GetInputEventAction(int action) {
  switch (action) {
    using enum android_xr::SplitEngineInputEvent::Action;
    // LINT.IfChange(action)
    case static_cast<int>(ACTION_DOWN):
    case static_cast<int>(ACTION_UP):
    case static_cast<int>(ACTION_MOVE):
    case static_cast<int>(ACTION_CANCEL):
    case static_cast<int>(ACTION_HOVER_MOVE):
    case static_cast<int>(ACTION_HOVER_ENTER):
    case static_cast<int>(ACTION_HOVER_EXIT):
      return static_cast<android_xr::SplitEngineInputEvent::Action>(action);
    // LINT.ThenChange(//depot/google3/third_party/split_engine/input/split_engine_input_event.h:action)
    default:
      IMP_LOG(imp::WARNING) << "[split_engine_input_event_jni] Unknown action: "
                   << action;
      return ACTION_CANCEL;
  }
}

}  // namespace

extern "C" {

JNI_METHOD_ACTIVITY(jlong, nCreateSplitEngineInputEvent)
(JNIEnv* env, jclass /*clazz*/) {
  auto input_event = new android_xr::SplitEngineInputEvent();
  return ToJava<android_xr::SplitEngineInputEvent>(input_event);
}

JNI_METHOD_ACTIVITY(void, nDestroySplitEngineInputEvent)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  delete event;
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetDispatchFlags)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jint dispatch_flags) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  event->dispatch_flag = GetInputEventDispatchFlag(dispatch_flags);
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetDeviceType)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jint device_type) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  event->device_type = GetInputEventDeviceType(device_type);
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetAction)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jint action) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  event->action = GetInputEventAction(action);
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetPointerType)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jint pointer_type) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  event->pointer_type = GetInputEventPointerType(pointer_type);
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetTimestampMs)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jlong timestamp_ms) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  event->timestamp_ms = timestamp_ms;
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetOrigin)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jfloat x, jfloat y,
 jfloat z) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  event->origin.x = x;
  event->origin.y = y;
  event->origin.z = z;
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetDirection)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jfloat x, jfloat y,
 jfloat z) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  event->direction.x = x;
  event->direction.y = y;
  event->direction.z = z;
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetButtonState)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jint button_state) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  event->button_state = button_state;
}

// Call this before calling any of the [nSplitEngineInputEventSetHitNode*]
// methods.  Does nothing if the hit node already exists.
//
// There is no corresponding destroy, memory will be freed when
// [nDestroySplitEngineInputEvent] is called.
JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventCreateHitNode)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  if (event->hit_node) {
    return;
  }
  event->hit_node =
      std::make_unique<android_xr::SplitEngineInputEventHitInfo>();
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetHitNodeNodeId)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jint node_id) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  if (!event->hit_node) {
    return;
  }
  event->hit_node->target = ToNodeHandle(node_id);
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetHitNodeHitPositionIsValid)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jboolean is_valid) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  if (!event->hit_node) {
    return;
  }
  event->hit_node->hit_position_is_valid = is_valid;
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetHitNodeHitPosition)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jfloat x, jfloat y,
 jfloat z) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  if (!event->hit_node) {
    return;
  }
  // Hit positions are sent from Spaceflinger relative to the task space.
  // This makes them relative to the subspace they will be handled by.
  imp::float4 hit_position(x, y, z, 1.0f);
  event->hit_node->hit_position =
      (event->hit_node->transform * hit_position).xyz;
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetHitNodeTransform)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jfloat m00, jfloat m01,
 jfloat m02, jfloat m03, jfloat m10, jfloat m11, jfloat m12, jfloat m13,
 jfloat m20, jfloat m21, jfloat m22, jfloat m23, jfloat m30, jfloat m31,
 jfloat m32, jfloat m33) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  if (!event->hit_node) {
    return;
  }
  event->hit_node->transform[0][0] = m00;
  event->hit_node->transform[0][1] = m01;
  event->hit_node->transform[0][2] = m02;
  event->hit_node->transform[0][3] = m03;
  event->hit_node->transform[1][0] = m10;
  event->hit_node->transform[1][1] = m11;
  event->hit_node->transform[1][2] = m12;
  event->hit_node->transform[1][3] = m13;
  event->hit_node->transform[2][0] = m20;
  event->hit_node->transform[2][1] = m21;
  event->hit_node->transform[2][2] = m22;
  event->hit_node->transform[2][3] = m23;
  event->hit_node->transform[3][0] = m30;
  event->hit_node->transform[3][1] = m31;
  event->hit_node->transform[3][2] = m32;
  event->hit_node->transform[3][3] = m33;
}

// Call this before calling any of the
// [nSplitEngineInputEventSetSecondaryHitNode*] methods.  Does nothing if the
// secondary hit node already exists.
//
// There is no corresponding destroy, memory will be freed when
// [nDestroySplitEngineInputEvent] is called.
JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventCreateSecondaryHitNode)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  if (event->secondary_hit_node) {
    return;
  }
  event->secondary_hit_node =
      std::make_unique<android_xr::SplitEngineInputEventHitInfo>();
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetSecondaryHitNodeNodeId)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jint node_id) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  if (!event->secondary_hit_node) {
    return;
  }
  event->secondary_hit_node->target = ToNodeHandle(node_id);
}

JNI_METHOD_ACTIVITY(void,
                    nSplitEngineInputEventSetSecondaryHitNodeHitPositionIsValid)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jboolean is_valid) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  if (!event->secondary_hit_node) {
    return;
  }
  event->secondary_hit_node->hit_position_is_valid = is_valid;
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetSecondaryHitNodeHitPosition)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jfloat x, jfloat y,
 jfloat z) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  if (!event->secondary_hit_node) {
    return;
  }
  event->secondary_hit_node->hit_position.x = x;
  event->secondary_hit_node->hit_position.y = y;
  event->secondary_hit_node->hit_position.z = z;
}

JNI_METHOD_ACTIVITY(void, nSplitEngineInputEventSetSecondaryHitNodeTransform)
(JNIEnv* env, jclass /*clazz*/, jlong event_handle, jfloat m00, jfloat m01,
 jfloat m02, jfloat m03, jfloat m10, jfloat m11, jfloat m12, jfloat m13,
 jfloat m20, jfloat m21, jfloat m22, jfloat m23, jfloat m30, jfloat m31,
 jfloat m32, jfloat m33) {
  if (!event_handle) {
    return;
  }
  auto event = FromJava<android_xr::SplitEngineInputEvent>(event_handle);
  if (!event->secondary_hit_node) {
    return;
  }
  event->secondary_hit_node->transform[0][0] = m00;
  event->secondary_hit_node->transform[0][1] = m01;
  event->secondary_hit_node->transform[0][2] = m02;
  event->secondary_hit_node->transform[0][3] = m03;
  event->secondary_hit_node->transform[1][0] = m10;
  event->secondary_hit_node->transform[1][1] = m11;
  event->secondary_hit_node->transform[1][2] = m12;
  event->secondary_hit_node->transform[1][3] = m13;
  event->secondary_hit_node->transform[2][0] = m20;
  event->secondary_hit_node->transform[2][1] = m21;
  event->secondary_hit_node->transform[2][2] = m22;
  event->secondary_hit_node->transform[2][3] = m23;
  event->secondary_hit_node->transform[3][0] = m30;
  event->secondary_hit_node->transform[3][1] = m31;
  event->secondary_hit_node->transform[3][2] = m32;
  event->secondary_hit_node->transform[3][3] = m33;
}

}  // extern "C"
