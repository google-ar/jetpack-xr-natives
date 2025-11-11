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

#include "core/split_engine/android/shmem_utils.h"

#include "absl/status/status.h"

#if defined(__ANDROID__) && __ANDROID_API__ >= 26
#include <android/sharedmem.h>
#else
#include <sys/stat.h>
#endif

#include <cstddef>

#include "absl/status/statusor.h"

namespace imp::split_engine {

absl::StatusOr<size_t> GetShmemFdSize(int fd) {
#if defined(__ANDROID__) && __ANDROID_API__ >= 26
  const size_t size = ASharedMemory_getSize(fd);
  if (size == 0) {
    return absl::InternalError("Bad file descriptor.");
  }
  return size;
#else
  // Older Androids, Mac and Linux can use fstat.
  struct stat stat_buf;
  if (fstat(fd, &stat_buf) != 0) {
    return absl::InternalError("Bad file descriptor.");
  }
  return stat_buf.st_size;
#endif
}

}  // namespace imp::split_engine
