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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_INPUT_STREAM_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_INPUT_STREAM_H_

#include <jni.h>

#include <cassert>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "core/async/future_interrupter.h"
#include "core/common/jni_helpers.h"
#include "core/resources/download_progress_info.h"

namespace imp {

// Java Wrapper for java.io.InputStream
class InputStream : public JavaWrapper {
 public:
  InputStream(JNIEnv* env, jobject java_input_stream)
      : JavaWrapper(env, java_input_stream),
        chunk_byte_array_(
            WrapJni(env, env->NewGlobalRef(env->NewByteArray(kBufferSize)))) {
    read_ = GetMethodHandle("read", "([BII)I");
    assert(read_);
    close_ = GetMethodHandle("close", "()V");
    assert(close_);
    available_ = GetMethodHandle("available", "()I");
    assert(available_);
  }

  ~InputStream() override { CallVoidMethod(close_); }

  // Reads the next chunk using the passed-in function to receive the chunk data
  // and returns the number of bytes read, or -1 if the stream is complete.
  // If there was an error, returns -1 and doesn't clear JNI exception state.
  int ReadChunk(JNIEnv* env,
                std::function<void(jbyteArray, size_t)> receive_chunk);

  // Makes blocking calls to read the contents of a java.io.InputStream into
  // a Cord and return it.
  // Note: If content_length is non-zero, then an entry for
  // string_uri will be added to download_progress_info and updated with
  // progress on each successful chunk read. (this is for native code only,
  // otherwise those two parameters should take trivial value)
  absl::StatusOr<absl::Cord> BlockingReadFromJavaInputStream(
      JNIEnv* env, std::string string_uri, size_t content_length,
      std::shared_ptr<resources::DownloadProgressInfo> download_progress_info,
      std::vector<FutureInterrupter> interrupters);

 private:
  jbyteArray GetByteArray() {
    return static_cast<jbyteArray>(chunk_byte_array_.get());
  }
  const size_t kBufferSize = 16 * 1024;

  JniHandle read_;
  JniHandle close_;
  JniHandle available_;
  JniUniquePtr<jobject> chunk_byte_array_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_INPUT_STREAM_H_
