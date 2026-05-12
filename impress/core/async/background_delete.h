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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_BACKGROUND_DELETER_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_BACKGROUND_DELETER_H_

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

#include "absl/base/optimization.h"
#include "core/common/log.h"
#include "core/async/executor.h"
#include "core/common/invocable.h"

namespace imp {

template <typename T>
void DestructOnBackgroundExecutor(T& t) = delete;

// Takes ownership of t and deletes it in the background.
//
// The deletion may happen in the current thread in some unit tests where the
// background executor hasn't been set.
template <typename T,
          typename = std::enable_if_t<std::is_class_v<std::decay_t<T>>>>
void DestructOnBackgroundExecutor(T&& t) {
  // Using std::forward to make linter happy here, but not to support lvalue
  // references.
  imp::Invocable<void()> delete_t = [t = std::forward<T>(t)]() mutable {
    // `t` will be destroyed after this lambda goes out of scope.
  };

  if (ABSL_PREDICT_TRUE(Executor::BackgroundExecutor() != nullptr)) {
    // Offload memory deallocation to the background thread to not block
    // the main thread.
    Executor::BackgroundExecutor()->ScheduleInvocable(std::move(delete_t));

    return;
  }

  // This might happen in some unit tests where the background executor is not
  // set or already destroyed by the time of this call.
  IMP_LOG(imp::ERROR) << "Background executor is null. Deletion will happen on the "
                "current thread.";
}

template <typename T>
struct BackgroundDeleter {
  // Matching std::default_delete.
  static_assert(!std::is_function_v<T>,
                "BackgroundDeleter cannot be instantiated for function types");

  constexpr BackgroundDeleter() noexcept = default;

  template <typename U,
            typename = std::enable_if_t<std::is_convertible_v<U, T>>>
  constexpr BackgroundDeleter(const BackgroundDeleter<U>&) noexcept {};

  void operator()(T* t) noexcept {
    // If type is forward declared, `delete` can be undefined behavior if the
    // type has non-trivial destructor.
    static_assert(sizeof(T) >= 0,
                  "cannot delete an incomplete type: forward declared type?");
    // Special case for void (which is defined as `incomplete type`): some
    // compilers allow sizeof(void) to be 1 and that will bypass previous
    // assert.
    static_assert(
        !std::is_void_v<T>,
        "cannot delete an incomplete type: delete void* is undefined behavior");

    if (t == nullptr) return;

    // Using std::unique_ptr here to capture the pointer to make sure that
    // memory is free'd even if the executor was destroyed before executing this
    // lambda.
    imp::Invocable<void()> delete_t = [ptr = std::unique_ptr<T>(t)]() {};

    if (ABSL_PREDICT_TRUE(Executor::BackgroundExecutor() != nullptr)) {
      // Offload memory deallocation to the background thread to not block
      // the main thread.
      Executor::BackgroundExecutor()->ScheduleInvocable(std::move(delete_t));

      return;
    }

    // This might happen in some unit tests where the background executor is not
    // set or already destroyed by the time of this call.
    IMP_LOG(imp::ERROR) << "Background executor is null. Deletion will happen on the "
                  "current thread.";
  }
};

template <typename T>
struct BackgroundDeleter<T[]> {
  // Matching std::default_delete.
  constexpr BackgroundDeleter() noexcept = default;

  template <typename U, typename = std::enable_if_t<
                            std::is_convertible_v<U (*)[], T (*)[]>>>
  constexpr BackgroundDeleter(const BackgroundDeleter<U[]>&) noexcept {};

  template <typename U, typename = std::enable_if_t<
                            std::is_convertible_v<U (*)[], T (*)[]>>>
  void operator()(U* u) {
    // If type is forward declared, `delete` can be undefined behavior if the
    // type has non-trivial destructor. There's no need to have a special case
    // for `void` here, because there's no `void[]` in C++.
    static_assert(sizeof(U) >= 0,
                  "cannot delete an incomplete type: forward declared type?");

    if (u == nullptr) return;

    // Using std::unique_ptr here to capture the pointer to make sure that
    // memory is free'd even if the executor was destroyed before executing this
    // lambda.
    imp::Invocable<void()> delete_t = [ptr = std::unique_ptr<U[]>(u)]() {};

    if (ABSL_PREDICT_TRUE(Executor::BackgroundExecutor() != nullptr)) {
      // Offload memory deallocation to the background thread to not block
      // the main thread.
      Executor::BackgroundExecutor()->ScheduleInvocable(std::move(delete_t));

      return;
    }

    // This might happen in some unit tests where the background executor is not
    // set or already destroyed by the time of this call.
    IMP_LOG(imp::ERROR) << "Background executor is null. Deletion will happen on the "
                  "current thread.";
  }
};

template <typename T, typename... Args>
std::enable_if_t<!std::is_array_v<T>, std::shared_ptr<T>>
MakeSharedWithBackgroundDeleter(Args&&... args) {
  return std::shared_ptr<T>(new T(std::forward<Args>(args)...),
                            BackgroundDeleter<T>());
}

template <typename T>
std::enable_if_t<std::is_array_v<T>
                     // Make sure we have T[], not T[34].
                     && std::extent_v<T> == 0,
                 // NDK r25 libc++ has buggy implementation of
                 // shared_ptr<TYPE[]>. We have to return shared_ptr<TYPE>
                 // instead.
                 std::shared_ptr<std::remove_extent_t<T>>>
MakeSharedWithBackgroundDeleter(size_t size) {
  // T is TYPE[], U will be TYPE.
  using U = std::remove_extent_t<T>;

  // Again, T is TYPE[], so :
  //  - shared_ptr<T> will be shared_ptr<TYPE[]>
  //  - BackgroundDeleter<T> will be BackgroundDeleter<TYPE[]>.
  //
  // NDK r25 libc++ has buggy implementation of shared_ptr<TYPE[]>.
  // This will create shared_ptr<TYPE> with a deleter for TYPE[].
  using U = std::remove_extent_t<T>;
  return std::shared_ptr<U>(new U[size](), BackgroundDeleter<T>());
}
template <typename T>
std::enable_if_t<std::is_array_v<T>
                     // Make sure we have T[], not T[34].
                     && std::extent_v<T> == 0,
                 // NDK r25 libc++ has buggy implementation of
                 // shared_ptr<TYPE[]>. We have to return shared_ptr<TYPE>
                 // instead.
                 std::shared_ptr<std::remove_extent_t<T>>>
MakeSharedWithBackgroundDeleter(std::remove_extent_t<T>* ptr) {
  // T is TYPE[], so :
  //  - shared_ptr<T> will be shared_ptr<TYPE[]>
  //  - BackgroundDeleter<T> will be BackgroundDeleter<TYPE[]>.
  //
  // NDK r25 libc++ has buggy implementation of shared_ptr<TYPE[]>.
  // This will create shared_ptr<TYPE> with a deleter for TYPE[].
  using U = std::remove_extent_t<T>;
  return std::shared_ptr<U>(ptr, BackgroundDeleter<T>());
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_BACKGROUND_DELETER_H_
