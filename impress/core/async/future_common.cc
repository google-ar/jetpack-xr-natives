/*
 * Copyright 2025 Google LLC
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

#include "core/async/future_common.h"

#include "absl/base/no_destructor.h"
#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"

namespace imp {
namespace {
absl::NoDestructor<absl::Mutex> g_future_flags_mu;
bool g_enable_synchronous_future_cancellation
    ABSL_GUARDED_BY(*g_future_flags_mu) = false;
}  // namespace

void FutureFlags::EnableSynchronousFutureCancellation() {
  absl::MutexLock lock(*g_future_flags_mu);
  g_enable_synchronous_future_cancellation = true;
}

bool FutureFlags::IsSynchronousFutureCancellationEnabled() {
  absl::MutexLock lock(*g_future_flags_mu);
  return g_enable_synchronous_future_cancellation;
}

}  // namespace imp
