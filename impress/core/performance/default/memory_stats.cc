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

#include "core/performance/memory_stats.h"

#include <cstddef>

namespace imp {

size_t MemoryStats::GetMemoryBytesAllocatedOnThisThread() { return 0; }
size_t MemoryStats::GetAllocationsCountOnThisThread() { return 0; }
void MemoryStats::ResetMemoryCountersForThisThread() {}
size_t MemoryStats::GetMemoryUsageBytes() { return 0; }
size_t MemoryStats::GetAllocationsCountTotal() { return 0; }
void MemoryStats::IncrementMemoryCounters(size_t size) {}
void MemoryStats::DecrementMemoryCounters(size_t size) {}

}  // namespace imp
