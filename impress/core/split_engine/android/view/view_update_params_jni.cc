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

#include "core/common/jni_helpers.h"
#include "core/split_engine/android/view/view_update_params.h"

#define JNI_METHOD_ACTIVITY(return_type, method_name) \
  IMP_JNI return_type JNICALL                         \
      Java_com_google_ar_imp_view_splitengine_ImpSplitEngine_##method_name  // NOLINT

namespace {

template <class T>
using ViewUpdateParamsAllowlist =
    ::imp::JniAllowlist<T, android_xr::ViewProjection,
                        android_xr::ViewUpdateParams>;

template <class T>
constexpr auto ToJava = &ViewUpdateParamsAllowlist<T>::ToJava;

template <class T>
constexpr auto FromJava = &ViewUpdateParamsAllowlist<T>::FromJava;

// Sets the values in `out`.
void SetViewProjection(jfloat angle_left, jfloat angle_right, jfloat angle_up,
                       jfloat angle_down, jfloat translate_x,
                       jfloat translate_y, jfloat translate_z,
                       jfloat quaternion_x, jfloat quaternion_y,
                       jfloat quaternion_z, jfloat quaternion_w,
                       android_xr::ViewProjection& out) {
  out.fov.angle_left = angle_left;
  out.fov.angle_right = angle_right;
  out.fov.angle_up = angle_up;
  out.fov.angle_down = angle_down;
  out.pose.translation.x = translate_x;
  out.pose.translation.y = translate_y;
  out.pose.translation.z = translate_z;
  out.pose.rotation.x = quaternion_x;
  out.pose.rotation.y = quaternion_y;
  out.pose.rotation.z = quaternion_z;
  out.pose.rotation.w = quaternion_w;
}

}  // namespace

extern "C" {

JNI_METHOD_ACTIVITY(jlong, nCreateViewUpdateParams)
(JNIEnv* env, jclass /*clazz*/) {
  auto params = new android_xr::ViewUpdateParams();
  return ToJava<android_xr::ViewUpdateParams>(params);
}

JNI_METHOD_ACTIVITY(void, nSetViewUpdateParams)
(JNIEnv* env, jclass /*clazz*/, jlong params_handle, jlong left_eye_handle,
 jlong right_eye_handle) {
  if (!params_handle) {
    return;
  }
  auto params = FromJava<android_xr::ViewUpdateParams>(params_handle);
  params->left_eye = FromJava<android_xr::ViewProjection>(left_eye_handle);
  params->right_eye = FromJava<android_xr::ViewProjection>(right_eye_handle);
}

JNI_METHOD_ACTIVITY(void, nDestroyViewUpdateParams)
(JNIEnv* env, jclass /*clazz*/, jlong params_handle) {
  if (!params_handle) {
    return;
  }
  auto params = FromJava<android_xr::ViewUpdateParams>(params_handle);
  delete params;
}

JNI_METHOD_ACTIVITY(jlong, nCreateViewProjection)
(JNIEnv* env, jclass /*clazz*/) {
  auto projection = new android_xr::ViewProjection();
  return ToJava<android_xr::ViewProjection>(projection);
}

// The `angle_*` params set fov.
// The `translate_*` params set the pose's translation.
// The `quaternion_*` params set the pose's rotation as a quaternion.
JNI_METHOD_ACTIVITY(void, nSetViewProjection)
(JNIEnv* env, jclass /*clazz*/, jlong projection_handle, jfloat angle_left,
 jfloat angle_right, jfloat angle_up, jfloat angle_down, jfloat translate_x,
 jfloat translate_y, jfloat translate_z, jfloat quaternion_x,
 jfloat quaternion_y, jfloat quaternion_z, jfloat quaternion_w) {
  if (!projection_handle) {
    return;
  }
  auto projection = FromJava<android_xr::ViewProjection>(projection_handle);
  SetViewProjection(angle_left, angle_right, angle_up, angle_down, translate_x,
                    translate_y, translate_z, quaternion_x, quaternion_y,
                    quaternion_z, quaternion_w, *projection);
}

JNI_METHOD_ACTIVITY(void, nDestroyViewProjection)
(JNIEnv* env, jclass /*clazz*/, jlong projection_handle) {
  if (!projection_handle) {
    return;
  }
  auto projection = FromJava<android_xr::ViewProjection>(projection_handle);
  delete projection;
}

}  // extern "C"
