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

#include "core/model/mesh/mesh_data_helper.h"

#include <cstddef>
#include <cstdint>

#include "absl/synchronization/mutex.h"

namespace imp::imp_internal {

void MeshDataBufferDescriptorDeleter(void* buffer, size_t size, void* user) {
  auto* copy_counter = static_cast<imp_internal::MeshDataCopyCounter*>(user);
  size_t remaining_copies;
  {
    absl::MutexLock lock(copy_counter->mu);
    remaining_copies = --copy_counter->copies;
  }

  // Deletes the underlying data once it's done being uploaded to filament (or
  // the BufferDescriptor is destroyed) as long as nothing else is referencing
  // it.
  if (remaining_copies == 0) {
    delete copy_counter;
    delete[] static_cast<uint8_t*>(buffer);
  }
}

}  // namespace imp::imp_internal
