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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_ATOMIC_FUTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_ATOMIC_FUTURE_H_

#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"

namespace imp {

// AtomicFuture is a thread-safe container for an imp::Future<T> variable.
//
// Standard imp::Future is a value type wrapper (internally holding a shared
// pointer to the shared state). Copying a Future (e.g., passing it to another
// thread) is completely safe and thread-safe because the underlying shared
// state is synchronized internally, and reference counting is atomic.
//
// However, like any standard C++ variable (e.g., std::shared_ptr or
// std::string), a Future variable itself is NOT thread-safe for concurrent
// read (copying) and write (reassignment).
//
// AtomicFuture solves this by providing internal synchronization (via a mutex)
// for reassigning (Store/=) and retrieving (Load) the held Future.
//
// To prevent concurrent races, AtomicFuture is non-copyable and non-movable,
// mimicking std::atomic semantics.
//
// Usage Example:
//   class MyClass {
//    public:
//     void SetFuture(imp::Future<int> f) {
//       my_shared_future_ = std::move(f); // Safe write (calls Store)
//     }
//
//     void Process() {
//       my_shared_future_.Then([](int val) { ... }); // Safe read + Then
//     }
//
//    private:
//     imp::AtomicFuture<int> my_shared_future_;
//   };
template <typename T>
class AtomicFuture {
 public:
  AtomicFuture() = default;

  // Constructs an AtomicFuture holding the given future.
  explicit AtomicFuture(Future<T> f) : future_(std::move(f)) {}

  // AtomicFuture is non-copyable and non-movable to prevent concurrent races
  // on the variable itself.
  AtomicFuture(const AtomicFuture&) = delete;
  AtomicFuture& operator=(const AtomicFuture&) = delete;
  AtomicFuture(AtomicFuture&&) = delete;
  AtomicFuture& operator=(AtomicFuture&&) = delete;

  // Thread-safely assigns a new future to this AtomicFuture.
  AtomicFuture& operator=(Future<T> desired) {
    Store(std::move(desired));
    return *this;
  }

  // Thread-safely replaces the held future with a new one.
  void Store(Future<T> new_future) {
    absl::MutexLock lock(mu_);
    std::swap(future_, new_future);
  }

  // Thread-safely returns a copy of the held future.
  // Returning by value is required to ensure the caller operates on a stable
  // snapshot of the future after the lock is released.
  Future<T> Load() const {
    absl::ReaderMutexLock lock(mu_);
    return future_;
  }

  // Syntactic sugar: Thread-safely loads the future and chains a callback.
  template <typename Fn>
  auto Then(Fn&& fn,
            Executor::Type executor_type = Executor::Type::kForeground) const {
    // Call Load() instead of calling Then under the lock to avoid
    // holding the lock while Then is running.
    return Load().Then(std::forward<Fn>(fn), executor_type);
  }

  template <typename Fn>
  auto Then(Fn&& fn, FutureThenOptions then_options) const {
    // Call Load() instead of calling Then under the lock to avoid
    // holding the lock while Then is running.
    return Load().Then(std::forward<Fn>(fn), then_options);
  }

  // Thread-safely checks if the held future is ready.
  bool Ready() const {
    absl::ReaderMutexLock lock(mu_);
    return future_.Ready();
  }

  // Thread-safely cancels the held future.
  void Cancel() const {
    // Call Load() instead of calling Cancel under the lock to avoid
    // holding the lock while Cancel is running.
    Load().Cancel();
  }

 private:
  mutable absl::Mutex mu_;
  Future<T> future_ ABSL_GUARDED_BY(mu_);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_ATOMIC_FUTURE_H_
