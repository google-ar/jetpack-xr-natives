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

#include <cstdint>

#include "absl/status/status.h"
#if defined(__ANDROID__)
#include <android/native_window_jni.h>

#include "core/common/log.h"
#endif  // defined(__ANDROID__)

#include "absl/time/time.h"
#include "core/common/jni_helpers.h"
#include "core/common/trace.h"
#include "core/input/pointer_event.h"
#include "core/math/vec.h"
#include "core/view/framework/view.h"
#include "core/view/view_host.h"

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_view_input_InputManager_##method_name

namespace {

using ::imp::Device;
using ::imp::float2;
using ::imp::JniAllowlist;
using ::imp::Pointer;
using ::imp::ViewHost;

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, ViewHost>::FromJava(n);
}

}  // namespace

extern "C" {
// LINT.IfChange(api)

JNI_METHOD(void, nProcessPointerEvent)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jint action,
 jintArray id, jfloatArray x, jfloatArray y, jlong timestamp) {
  IMP_TRACE();
  auto* view_host = FromJava<ViewHost>(view_host_handle);
  const Device& device = view_host->GetView()->GetDevice();
  if (!device.IsPhysicalPixelRatioAvailable()) {
    IMP_LOG(imp::WARNING)
        << "Cannot process pointer input until physical pixels ratio is "
           "available.";
    return;
  }

  const int len = env->GetArrayLength(id);
  assert(len == env->GetArrayLength(x) && len == env->GetArrayLength(y));

  jint* idList = env->GetIntArrayElements(id, nullptr);
  jfloat* xList = env->GetFloatArrayElements(x, nullptr);
  jfloat* yList = env->GetFloatArrayElements(y, nullptr);

  std::vector<Pointer::Id> ids;
  ids.reserve(len);
  std::vector<float2> points;
  points.reserve(len);
  for (int i = 0; i < len; ++i) {
    ids.push_back(static_cast<Pointer::Id>(idList[i]));

    // Convert the raw physical pixels coming from the android motion event
    // into Impress virtual pixels. On android, this is essentially converting
    // from real pixels to dp.
    points.push_back(device.PhysicalPixelsToPixels(float2(xList[i], yList[i])));
  }

  env->ReleaseIntArrayElements(id, idList, 0);
  env->ReleaseFloatArrayElements(x, xList, 0);
  env->ReleaseFloatArrayElements(y, yList, 0);

  absl::Status status =
      view_host->GetView()->GetInputManager().ProcessPointerInput(
          action, ids, points, absl::Milliseconds((int64_t)timestamp));

  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to process pointer input: " << status;
  }
}

// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/view/input/\
//         InputManager.java:api
// )
}  // extern "C"
