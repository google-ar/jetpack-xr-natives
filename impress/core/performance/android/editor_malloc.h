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

#ifndef THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_ANDROID_EDITOR_MALLOC_H_
#define THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_ANDROID_EDITOR_MALLOC_H_

#include <cstddef>

// This provides a way for MemoryStats to use the real malloc/free.
// This is needed to avoid recursion since thread_local variables allocate
// in Android. Without access to these functions, a stackoverflow would occur.
// e.g. malloc calls IncrementMemoryCounters which adds to a thread_local
// variable which calls malloc and calls IncrementMemoryCounters etc.
namespace imp::imp_malloc {

void Free(void* ptr);
void* Malloc(size_t size);

}  // namespace imp::imp_malloc

#endif  // THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_ANDROID_EDITOR_MALLOC_H_
