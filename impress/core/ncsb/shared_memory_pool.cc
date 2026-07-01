/*
 * Copyright 2026 Google LLC
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

#include "core/ncsb/shared_memory_pool.h"

#include "core/common/paged_pointer_array.h"
#include "core/common/uniform_block_pool.h"

namespace imp_internal {

imp::UniformBlockPool& GetPagedPointerArrayBlockPool() {
  thread_local imp::UniformBlockPool memory_pool =
      imp::UniformBlockPool(imp::PagedPointerArray<void>::GetPageBytes());
  return memory_pool;
}

}  // namespace imp_internal
