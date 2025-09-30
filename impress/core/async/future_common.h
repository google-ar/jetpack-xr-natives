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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_COMMON_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_COMMON_H_

#include <cstdint>
#include <optional>

#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "core/async/executor.h"

namespace imp {

// Determines what happens when an error is returned to a future when KeptBy has
// been called.
//
// Often, user code should handle errors explicitly in the final Future in a
// chain of futures. However, this functionality handles errors automatically
// for simple cases & debugging.
//
// If the error is explicitly handled, then this behavior will not happen.
//
// Examples:
//
// // The error is *not* handled so the KeptBy error handling *will* occur.
// future.Then([](Foo foo) { }).KeptBy(this);
//
// // The error is explicitly handled and *not* propagated so the KeptBy error
// // handling *will not* occur.
// future.Then([](absl::StatusOr<Foo> foo) { /* handle error*/ }).KeptBy(this);
//
// // The error is explicitly handled and propagated so the KeptBy error
// // handling *will* occur.
// future.Then([](absl::StatusOr<Foo> foo) {
//   return foo.status();
// }).KeptBy(this);
//
enum class FutureKeptByMode : uint8_t {
  // Default behavior.
  //
  // Indicates that the error will automatically be logged as a WARNING. This
  // makes it easier to debug errors when no explicit error handling has been
  // added for the future.
  //
  // Note: The log is treated as a warning because this may not indicate an
  // actual problem. For example, a future could be cancelled upon exit as an
  // expected part of the apps flow.
  kLogOnError,
  // Indicates that the program should crash when an error occurs.
  kDieOnError,
  // Indicates that nothing will happen when an error occurs, silently swallows
  // the error.
  kDoNothingOnError
};

// Determines how Future's run the functors passed into Future<T>::Then and
// Future<T>::Schedule.
enum class FutureExecutorMode {
  // If the current executor is not the future's target executor, then the
  // future's functor is scheduld on the executor. Otherwise, the functor is
  // called immediately.
  kScheduleIfNotOnExecutorThread,
  // Always schedules the future's functor on the target executor regardless of
  // if the current executor is already the target executor.
  kScheduleAlways
};

using ExecutorTypeOrExecutor = absl::variant<Executor::Type, Executor*>;

// Options used to configure calls to Future<T>::Schedule.
struct FutureScheduleOptions {
  // Controls which executor the functor is run on. Defaults to foreground.
  ExecutorTypeOrExecutor executor = Executor::Type::kForeground;

  // Controls how a future future runs the functor passed into
  // Future<T>::Schedule. Defaults to always scheduling the functor on the
  // executor.
  FutureExecutorMode executor_mode = FutureExecutorMode::kScheduleAlways;

  // The priority of this future, which is only used when the future is
  // scheduled. Whether or not the future is scheduled depends on the executor
  // mode.
  //
  // The value here is the "self priority" of the future. The actual priority
  // used for this future - the "active priority" - is calculated as the maximum
  // value across (1) the active priorities of its direct children*, if any, and
  // (2) its own self priority, if specified. To mark the future as having an
  // unspecified self priority, which is also the default, use std::nullopt.
  //
  // *In terms of priority, there are three cases where a future is considered a
  // child of another future:
  // 1. A future returned by Then() is considered a child of the future that
  //    Then() was called on.
  // 2. A future with a lambda that returns an inner future is considered a
  //    child of the inner future.
  // 3. The future returned by Combine(), Merge(), or variants thereof is
  //    considered a child of all of the Futures passed in.
  std::optional<int> task_priority = std::nullopt;
};

// Options used to configure calls to Future<T>::Then.
struct FutureThenOptions {
  // Controls which executor the functor is run on. Defaults to foreground.
  ExecutorTypeOrExecutor executor = Executor::Type::kForeground;

  // Controls how a future runs the functor passed into Future<T>::Then.
  // Defaults to always scheduling the functor on the executor if the current
  // executor isn't the target executor. Otherwise, calls the functor
  // immediately.
  //
  // Be aware that because of this default behavior, if a future is scheduled
  // with Then and runs on the current executor, it will run immediately, and
  // potentially before futures that have been scheduled with higher
  // task_priority values. This is because task_priority is only considered when
  // the future is scheduled.
  //
  // If you want ensure a Then future runs according to task_priority (note that
  // task age also matters), set this to kScheduleAlways.
  FutureExecutorMode executor_mode =
      FutureExecutorMode::kScheduleIfNotOnExecutorThread;

  // The priority of this future, which is only used when the future is
  // scheduled. Whether or not the future is scheduled depends on the executor
  // mode.
  //
  // The value here is the "self priority" of the future. The actual priority
  // used for this future - the "active priority" - is calculated as the maximum
  // value across (1) the active priorities of its direct children*, if any, and
  // (2) its own self priority, if specified. To mark the future as having an
  // unspecified self priority, which is also the default, use std::nullopt.
  //
  // *In terms of priority, there are three cases where a future is considered a
  // child of another future:
  // 1. A future returned by Then() is considered a child of the future that
  //    Then() was called on.
  // 2. A future with a lambda that returns an inner future is considered a
  //    child of the inner future.
  // 3. The future returned by Combine(), Merge(), or variants thereof is
  //    considered a child of all of the Futures passed in.
  std::optional<int> task_priority = std::nullopt;
};

// Forward declaration of Future.
template <typename T>
class Future;

// Holds a non-owning reference to a Future. Semantically, this is similar to
// using std::weak_ptr.
//
// This makes it possible to refer to a future without keeping it alive.
template <typename T>
class WeakFuture {
 public:
  WeakFuture() {};

  // NOLINTNEXTLINE: Implicit conversion allowed.
  WeakFuture(const Future<T>& future) : impl_wrapper_(future.impl_wrapper_) {}

  // Returns the Future referenced by the WeakFuture, creating a hard reference
  // to it.
  //
  // If the referenced future has already been destroyed, then returns nullopt.
  //
  // Similar to std::weak_ptr::lock
  absl::optional<Future<T>> Lock() const {
    if (auto impl_wrapper = impl_wrapper_.lock()) {
      return Future<T>(impl_wrapper);
    }
    return absl::nullopt;
  }

 private:
  std::weak_ptr<typename Future<T>::ImplWrapper> impl_wrapper_;
};

// Helper function for creating a WeakFuture from a given future.
//
// This is the same as creating a WeakFuture directly, except that make_weak can
// implicitly deduce the type of WeakFuture from the future passed in.
template <typename T>
WeakFuture<T> make_weak(Future<T> f) {
  return WeakFuture<T>(f);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_COMMON_H_
