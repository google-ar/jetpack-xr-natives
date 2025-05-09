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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_DATA_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_DATA_HELPER_H_

#include <cstddef>
#include <cstdint>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"

namespace imp::imp_internal {

// Used to track the number of copies of a MeshData::BufferDescriptor, used to
// prevent the underlying data from being deleted while it's still being
// uploaded to filament.
struct MeshDataCopyCounter {
  absl::Mutex mu;
  size_t copies ABSL_GUARDED_BY(mu) = 1;
};

void MeshDataBufferDescriptorDeleter(void* buffer, size_t size, void* user);

}  // namespace imp::imp_internal

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_DATA_HELPER_H_
