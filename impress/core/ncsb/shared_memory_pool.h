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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_SHARED_PMR_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_SHARED_PMR_H_

#include "core/common/uniform_block_pool.h"

namespace imp_internal {

// Returns a thread-local memory pool sized for PagedPointerArray allocations.
//
// This is used for component lookups by BaseComponentPool and NodeController
// lookups by NodeAttachmentManager.
//
// Given that Impress Components & Nodes are always interacted with on the
// foreground, an unsynchronized UniformBlockPool is appropriate here.
imp::UniformBlockPool& GetPagedPointerArrayBlockPool();

}  // namespace imp_internal

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_SHARED_PMR_H_
