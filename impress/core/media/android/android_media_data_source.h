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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_DATA_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_DATA_SOURCE_H_

#include <cstring>

#include "core/common/jni_helpers.h"
#include "core/media/media_asset.h"

namespace imp::media {
// LINT.IfChange()
class AndroidMediaDataSource : public JavaWrapper {
 public:
  AndroidMediaDataSource(const Context& context, const MediaAsset* media_asset)
      : JavaWrapper(context.GetJniEnv(),
                    "com/google/ar/imp/core/media/ImpMediaDataSource", "()V") {
    set_data_ = GetMethodHandle("setData", "([B)V");
    auto env = context.GetJniEnv();

    // Media data is copied entirely into the Java MediaDataSource for
    // performance reasons - reading data through the JNI layer is prohibitively
    // slow and greatly affects audio performance.
    jbyteArray retArray = env->NewByteArray(media_asset->GetSize());
    void* temp = (env->GetPrimitiveArrayCritical(static_cast<jarray>(retArray),
                                                 nullptr));
    memcpy(temp, media_asset->GetData(), media_asset->GetSize());
    env->ReleasePrimitiveArrayCritical(retArray, temp, 0);
    CallVoidMethod(set_data_, retArray);
  }

 private:
  imp::JniHandle set_data_;
};
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/ImpMediaDataSource.java
// )

}  // namespace imp::media

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_DATA_SOURCE_H_
