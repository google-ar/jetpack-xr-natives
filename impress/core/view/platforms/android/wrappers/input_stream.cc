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

#include "core/view/platforms/android/wrappers/input_stream.h"

#include <jni.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/cord_buffer.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "core/async/future_interrupter.h"
#include "core/common/jni_helpers.h"
#include "core/monitor/profiling_clock.h"
#include "core/resources/download_progress_info.h"

namespace imp {
namespace {
constexpr absl::Duration kProgressUpdateIntervalLimit = absl::Milliseconds(16);
constexpr int kMaxZeroLengthReads = 5;
}  // namespace

int InputStream::ReadChunk(
    JNIEnv* env, std::function<void(jbyteArray, size_t)> receive_chunk) {
  int amount_to_read = CallIntMethod(available_);
  if (amount_to_read <= 0 || amount_to_read > kBufferSize) {
    // Clamp the available bytes to read to the size of our read buffer.
    // If available() returned 0 (either because this stream doesn't implement
    // it, or because we're at EOF) then default to the full buffer size.
    amount_to_read = kBufferSize;
  }
  // IMP_LOG(imp::ERROR) << "Impress: amount_to_read: " << amount_to_read;
  jint size = CallIntMethod(read_, chunk_byte_array_.get(), 0, amount_to_read);
  if (env->ExceptionCheck()) {
    // TODO: Handle read errors.
    return -1;
  }
  if (size > 0) {
    receive_chunk(chunk_byte_array_.get(), size);
  }
  // IMP_LOG(imp::ERROR) << "Impress: return size" << size;
  return size;
}

absl::StatusOr<absl::Cord> InputStream::BlockingReadFromJavaInputStream(
    JNIEnv* env, const std::string& string_uri, size_t content_length,
    std::shared_ptr<resources::DownloadProgressInfo> download_progress_info,
    std::vector<FutureInterrupter> interrupters) {
  for (const FutureInterrupter& interrupter : interrupters) {
    if (interrupter.IsInterrupted()) {
      return absl::CancelledError("Interrupted after creating input stream.");
    }
  }

  if (download_progress_info && content_length > 0) {
    absl::MutexLock lock(download_progress_info->download_progress_map_mutex);
    download_progress_info->download_progress_map.emplace(
        string_uri, resources::EntryProgressInfo{.downloaded_size = 0,
                                                 .total_size = content_length});
  }
  absl::Time last_progress_update_time =
      ProfilingClock::GetMonotonicClockTime();

  absl::Cord cord;
  size_t current_length = 0;
  int zero_length_read_count = 0;
  int chunk_size = 0;
  while (chunk_size >= 0) {
    chunk_size = ReadChunk(env, [&cord, env](jbyteArray chunk, size_t size) {
      int offset = 0;
      int remaining_size = size;
      while (remaining_size > 0) {
        absl::CordBuffer buffer =
            absl::CordBuffer::CreateWithDefaultLimit(remaining_size);
        absl::Span<char> data = buffer.available_up_to(remaining_size);
        env->GetByteArrayRegion(chunk, offset, data.size(),
                                reinterpret_cast<jbyte*>(data.data()));
        buffer.IncreaseLengthBy(data.size());
        offset += data.size();
        remaining_size -= data.size();
        cord.Append(std::move(buffer));
      }
    });
    if (chunk_size > 0) {
      current_length += chunk_size;
    }

    absl::Time current_time = ProfilingClock::GetMonotonicClockTime();
    if (download_progress_info && content_length > 0 &&
        (current_time - last_progress_update_time) >=
            kProgressUpdateIntervalLimit) {
      absl::MutexLock lock(download_progress_info->download_progress_map_mutex);
      download_progress_info->download_progress_map[string_uri]
          .downloaded_size = current_length;
      last_progress_update_time = current_time;
    }

    if (JavaExceptionPrintClear(env)) {
      // TODO: Handle read errors.
      return absl::InternalError("Failed to read from InputStream");
    }

    if (chunk_size == 0) {
      // read() provided zero bytes, but there's no pending exception and we're
      // not at EOF either. Weird. Increment a counter and try again; if we get
      // too many of these "didn't fail but didn't make progress" reads, exit
      // with an error to avoid spinning in an infinite loop.
      if (++zero_length_read_count >= kMaxZeroLengthReads) {
        return absl::InternalError("Failed to read from InputStream");
      }
    }
    for (const FutureInterrupter& interrupter : interrupters) {
      if (interrupter.IsInterrupted()) {
        return absl::CancelledError("Interrupted after creating input stream.");
      }
    }
  }
  // Update the download progress one more time since there may be more chunks
  // read after the last ticking of kProgressUpdateIntervalLimit interval.
  if (download_progress_info) {
    absl::MutexLock lock(download_progress_info->download_progress_map_mutex);

    absl::Time current_time = ProfilingClock::GetMonotonicClockTime();
    download_progress_info->download_progress_map[string_uri].downloaded_size =
        current_length;
    last_progress_update_time = current_time;
  }

  return cord;
}

}  // namespace imp
