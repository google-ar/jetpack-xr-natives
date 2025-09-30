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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_JNI_PROTO_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_JNI_PROTO_HELPERS_H_

#include <jni.h>

#include "absl/strings/string_view.h"
#include "core/proto/proto_reader.h"

namespace imp {
namespace android {

template <typename Proto>
bool ByteArrayToProto(JNIEnv* env, jbyteArray input, Proto* proto) {
  const int size = env->GetArrayLength(input);
  if (size <= 0) {
    return false;
  }

  void* ptr = env->GetPrimitiveArrayCritical(input, nullptr);
  if (!ptr) {
    return false;
  }

  proto::ProtoReader reader(
      absl::string_view(reinterpret_cast<char*>(ptr), size));
  reader.ParseMsg<Proto>(proto);
  env->ReleasePrimitiveArrayCritical(input, ptr, 0);
  return true;
}

}  // namespace android
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_JNI_PROTO_HELPERS_H_
