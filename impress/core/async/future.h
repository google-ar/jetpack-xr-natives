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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/base/attributes.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/blocking_counter.h"
#include "core/async/executor.h"
#include "core/async/future_common.h"
#include "core/async/future_impl.h"
#include "core/async/future_traits.h"
#include "core/async/task_priority.h"
#include "core/common/holdable.h"
#include "mediapipe/framework/port/status_builder.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

// Future<T> represents an asynchronous operation and it's result.
//
// The result of a Future is always an absl::StatusOr<T> or an absl::Status.
//
// A future can be declared as Future<T>, for a result of absl::StatusOr<T>, or
// as Future<absl::Status> for a result of absl::Status.
//
// Copying a future creates a shallow copy that refers to the same underlying
// future, and are cheap to copy.
//
// Futures will automatically be cancelled if they go out of scope. To keep a
// future in scope, you can either store it in a field or call Future::KeptBy.
//
// This is used instead of google3's thread::Future for the following reasons:
// * //thread is not part of portable google3 and we need to run on non-prod
//   platforms including Android, iOS, and Windows.
// * We plan to open source, and //thread is not available outside of google3.
// * Extend the interface to simplify composing Futures (Future::Then).
// * To better control which Executor callbacks are invoked on.
template <typename T>
class ABSL_MUST_USE_RESULT Future {
 public:
  // Result is the actual type held by the Future.
  // It will always be either absl::StatusOr<T> or absl::Status.
  using Result = internal::future_traits::ResultT<T>;
  // Value is T if Result is absl::StatusOr<T>, otherwise it is absl::Status.
  using Value = internal::future_traits::ValueT<T>;

  // Constructs an unready future.
  // Future::Return must be called at some later point to make the future ready.
  Future();

  // Constructs a ready future with the result passed in.
  //
  // This constructor handles types that are convertible to Result (which is
  // absl::StatusOr<T>) but are not exactly Result, Future, or absl::Status. It
  // allows for efficient perfect forwarding.
  //
  // Examples:
  //   Future<std::string> f("hello"); // const char* -> std::string
  //   Future<int> f(42L);             // long -> int
  //
  // This constructor uses SFINAE to disable itself in cases where other
  // constructors should be preferred or when the type is incompatible:
  // 1. absl::Status: Prefer the Future(absl::Status) constructor.
  // 2. Future: Prefer the copy/move constructors (Perfect Forwarding Guard).
  //    Without this, Future(Future&) would bind here instead of the copy ctor.
  // 3. Incompatible types: Ensure Result is constructible from U.
  template <typename R, typename std::enable_if_t<
                            !std::is_same_v<std::decay_t<R>, absl::Status> &&
                                !std::is_same_v<std::decay_t<R>, Future> &&
                                !std::is_same_v<std::decay_t<R>, Result> &&
                                !std::is_same_v<std::decay_t<R>, Value> &&
                                std::is_constructible_v<Result, R>,
                            int> = 0>
  explicit Future(R&& result);

  // Constructs a ready future with the Result (absl::StatusOr<T>) passed in.
  //
  // This constructor is strictly for the exact Result type. It is required to
  // support brace initialization for Result, which fails template argument
  // deduction for the forwarding constructor above.
  //
  // Example:
  //   Future<int> f({absl::OkStatus(), 42}); // Brace init for StatusOr<int>
  template <typename R = Result,
            typename = std::enable_if_t<std::is_same_v<R, Result> &&
                                        !std::is_same_v<R, absl::Status>>>
  explicit Future(R result);

  // Constructs a ready future with the Value (T) passed in.
  //
  // This constructor is strictly for the exact Value type. It is required to
  // support brace initialization for T, as the forwarding constructor cannot
  // deduce the type from a braced list.
  //
  // Example:
  //   Future<std::vector<int>> f({1, 2, 3});
  template <typename V = Value,
            typename = std::enable_if_t<std::is_same_v<V, Value> &&
                                        !std::is_same_v<V, absl::Status> &&
                                        !std::is_same_v<V, Result>>>
  explicit Future(V result, int = 0);

  // Constructs a ready future with the status passed in.
  //
  // The constructor is non-explicit to allow for returning an absl::Status
  // from a function that returns a Future without having to explicitly wrap
  // the Status in a Future.
  //
  // Example:
  //   Future<int> MyFunc() {
  //     return absl::InternalError("Failed");
  //   }
  Future(absl::Status result);

  // Allow implicit construction from StatusBuilder to support MP_RETURN_IF_ERROR.
  //
  // Example:
  //   Future<int> MyFunc() {
  //     MP_RETURN_IF_ERROR(DoSomething());
  //     return 42;
  //   }
  Future(const mediapipe::StatusBuilder& builder);

  ~Future();

  Future(const Future&) = default;
  Future& operator=(const Future&) = default;
  Future(Future&&) = default;
  Future& operator=(Future&&) = default;

  // Returns true if the result is ready.
  bool Ready() const;

  // Returns the result of the future if it is ready.
  // If the future is cancelled, it will return a CancelledError status.
  // If the future is not ready, fatals.
  const Result& Get() const;

  // Same as Get(), but returns a moveable.
  // WARNING!!!  Moving a result out of a future should only be done once.
  Result Move() const;

  // Waits for this future to become ready, then invokes the given
  // callback and returns a new Future for the result. The new future must
  // be stored, or KeptBy must be called to prevent the future from
  // being destroyed before the functor is called.
  //
  // If the functor returns a plain value or nothing, the future returned will
  // be ready immediately after functor completes. This will be immediate unless
  // the future switched executors.
  //
  // If the functor returns nothing, the future returned will be a
  // Future<absl::Status>.
  //
  // If the functor returns a future, the future returned by Then will not be
  // ready until that future is ready.
  //
  // Note, if the functor returns a future, the value of that future is
  // *moved* into the future returned by Then, and it is not safe to access
  // the value via the original future (if any such references exist).
  //
  // If an executor is specified, the functor will be scheduled on the provided
  // executor. If not, the functor will be scheduled on the foreground executor.
  template <typename Fn>
  auto Then(Fn&& fn,
            Executor::Type executor_type = Executor::Type::kForeground) const;
  template <typename Fn>
  auto Then(Fn&& fn, FutureThenOptions then_options) const;

  // Sets the value and makes the future ready.
  // If this future was created from a call to Then, the Then callback will not
  // be invoked as the future has already been returned.
  void Return(Result result) const;

  // Cancels this future.
  // If this future was created from a call to Then, the Then callback will be
  // invoked. Cancellation may not be immediate if the callback is invoked on a
  // different executor.
  void Cancel();

  // Ensure the specified objects are retained in memory until the future
  // completes.
  //
  // This works for the following types of objects:
  //   - shared_ptr
  //   - reference counted objects (i.e. imp::Future).
  //   - unique_ptr
  //   - move only objects
  //
  // DependsOn takes ownership of the object passed in. Therefore, move only
  // objects will be destroyed when the future completes, and reference counted
  // objects will be destroyed if it's the last reference to the object.
  //
  // This will not work for raw pointers, it will fail to compile.
  template <typename... Args>
  void DependsOn(Args&&... args) const;

  // Keep the future alive until it is ready or the rememberer is cleared.
  //
  // This is used to keep a future alive as an alternative to storing it in a
  // field. It is typically only called on the final future in a chain. For
  // example, a method that returns a future usually doesn't call KeptBy to
  // retain the future. It is the caller of the method that should be
  // responsible for keeping the future alive by calling KeptBy or holding the
  // future in a field.
  //
  // The passed rememberer is required to be a pointer-like object that has
  // a 'Remember' method which returns  a callable object that is used to
  // forget the future.
  //
  // The standard implementation of this is the following class:
  // third_party/impress/core/common/rememberer.h
  // This class can be inherited/composed to add support for calling KeptBy with
  // a type.
  template <typename Rememberer>
  void KeptBy(Rememberer rememberer, FutureKeptByMode kept_by_mode =
                                         FutureKeptByMode::kLogOnError) const;

  // Combines this Future's status with another Future's status.
  //
  // If any one of the futures fails, then the entire combined future fails
  // without waiting for the rest to become ready. In that case, the first error
  // encountered is returned as the status.
  template <typename... FutureTypes>
  Future<absl::Status> Combine(FutureTypes&&... futures) const;

  // Combines this Future's status with another Future's status.
  //
  // Unlike Combine, CombineWaitForAll will wait until every future
  // becomes ready even when there are failures. The first error encountered
  // is returned as the status.
  template <typename... FutureTypes>
  Future<absl::Status> CombineWaitForAll(FutureTypes&&... futures) const;

  // Merges this future with the set of other futures passed in.
  //
  // Returns a future that becomes ready once all futures passed in are ready.
  // The future will be of type Future<std::tuple<N...>>.
  // The elements of the tuple will correspond to the results of the futures.
  // The elements are in the order of the futures passed in, starting with
  // this future.
  //
  // If any one of the futures fails, then the entire merged future immediately
  // fails (like Combine). In that case, the first error encountered is returned
  // as the status.
  //
  // If any one of the futures is a move only type, then the entire merged
  // future is move only.
  //
  // Example usage:
  //   Future<int> fa;
  //   Future<bool> fb;
  //   Future<std::tuple<int, bool>> combined = fa.Merge(fb);
  //   auto future = combined.Then([](std::tuple<int, bool> tuple) {
  //     auto [fa_result, fb_result] = tuple;
  //     // Do something with the results.
  //   });
  // TODO: Add support for including Future<absl::Status> futures
  // in a merge. Low priority, since in most cases it would make more sense to
  // use combine if doing that.
  // TODO: Make it so Future::Merge fails immediately if any one
  // future fails instead of waiting for all to finish.
  template <typename... FutureTypes>
  auto Merge(FutureTypes... futures) const;

  // Returns true if this is the only copy of this Future.
  // WARNING: Be very careful about using this, it is only guaranteed to be
  // accurate if this future is only copied within a single thread.
  // TODO: Remove this method since it isn't safe.
  bool IsUnique() const { return impl_wrapper_.use_count() == 1; }

  // Schedules a future.
  //
  // Takes a functor that returns the result to be assigned to the future. The
  // function can return T, absl::Status, or absl::StatusOr<T> as the result.
  //
  // Optionally, the function can take an absl::Status as a parameter. In that
  // case, the function will be called even if the future was cancelled before
  // the function was called. absl::Status will have a kCancelled status.
  //
  // If an executor is specified, the functor will be scheduled on the provided
  // executor. If not, the functor will be scheduled on the foreground executor.
  template <typename Fn>
  static Future<T> Schedule(
      Fn&& fn, Executor::Type executor_type = Executor::Type::kForeground);
  template <typename Fn>
  static Future<T> Schedule(Fn&& fn, FutureScheduleOptions schedule_options);

  // Combines a list of futures into a Future<absl::Status>. Once every future
  // in the list succeeds, the future will become ready.
  //
  // If any one of the futures fails, then the entire combined future fails
  // without waiting for the rest to become ready. In that case, the first error
  // encountered is returned as the status.
  template <typename ListT>
  static Future<absl::Status> CombineList(const ListT& futures);

  // Combines a list of futures into a Future<absl::Status>. Once every future
  // in the list succeeds, the future will become ready.
  //
  // Unlike CombineList, CombineListWaitForAll will also wait until every future
  // becomes ready even when there are failures. The first error encountered is
  // returned as the status.
  template <typename ListT>
  static Future<absl::Status> CombineListWaitForAll(const ListT& futures);

  // Transforms a List<Future<T>> into a Future<List<T>> that collects
  // the results from all the individual futures.
  //
  // This only works with `Future<T>`s, not `Future<absl::Status>`s.
  //
  // For example, you can use this to combine a series of individual requests
  // into a single Future that awaits the results of all the requests:
  //
  // Future<std::vector<Item>> FetchItems(std::vector<std::string> item_ids) {
  //   std::vector<Future<Item>> item_requests;
  //   for (const auto& item_id : item_ids) {
  //     item_requests.push_back(FetchItem(item_id));
  //   }
  //   return Future<Item>::MergeList(item_requests);
  // }
  //
  // If any of the individual Futures fails, the whole future will fail
  // immediately, without waiting for all the other futures to finish.
  template <typename ListT>
  static Future<std::vector<Value>> MergeList(const ListT& futures);

  // Updates the priority of this future, which is only used when the future is
  // scheduled. See FutureThenOptions and FutureScheduleOptions for more
  // details.
  //
  // The value set here is the "self priority" of the future. The actual
  // priority used for this future - the "active priority"- is calculated as the
  // maximum value across (1) the active priorities of its direct children*, if
  // any, and (2) its own self priority, if specified. To mark the future as
  // having an unspecified self priority, use std::nullopt.
  //
  // *In terms of priority, there are three cases where a future is considered a
  // child of another future:
  // 1. A future returned by Then() is considered a child of the future that
  //    Then() was called on.
  // 2. A future with a lambda that returns an inner future is considered a
  //    child of the inner future.
  // 3. The future returned by Combine(), Merge(), or variants thereof is
  //    considered a child of all of the Futures passed in.
  void UpdatePriority(std::optional<int> priority);

  // Returns the actual priority used for this future. This is the highest
  // priority of itself and all of it's children.
  //
  // Note: once a future becomes ready, it stops referencing its parents. That
  // means that once its ready it will no longer have it's priority influenced
  // by its prior children. Since the future is already ready, this doesn't
  // matter, but it does mean that calling GetActivePriority may not return the
  // priority you expect in this case.
  int GetActivePriority() const;

  // Returns the self priority of this future, which is initially std::nullopt,
  // and can be updated by calling UpdatePriority.
  //
  // If a future's self priority is std::nullopt, the future has no specified
  // priority and will either inherit its priority from a child future, if
  // possible, or use kNormalTaskPriority.
  std::optional<int> GetSelfPriority() const;

  // Returns the Future's depth in the chain. (1 if no chain.)
  int GetDepth() const;

 private:
  using Impl = internal::FutureImpl;
  using ImplWrapper = internal::FutureImpl::FutureImplWrapper;

  template <typename U>
  friend class Future;

  friend class WeakFuture<T>;

  // Declare function as friend so helper can access impl.
  friend void internal::AddFutureToCombineResult(
      const std::shared_ptr<ImplWrapper>& combine_result,
      const std::shared_ptr<ImplWrapper>& future_to_combine,
      absl::BlockingCounter* remaining_futures_counter,
      absl::Status* combined_status);

  // Declare function as friend so helper can access impl.
  template <typename U>
  friend void internal::ReturnResultToFuture(std::shared_ptr<Impl>& then_impl,
                                             Future<U> result_future);

  explicit Future(std::shared_ptr<ImplWrapper> impl_wrapper)
      : impl_wrapper_(impl_wrapper) {}

  auto MoveOrGet();

  std::shared_ptr<Future<T>::ImplWrapper> impl_wrapper_;

  static_assert(internal::future_traits::IsValidFutureTypeV<T>,
                "Futures cannot be declared as Future<StatusOr<T>>, Instead "
                "just declare Future<T>");

  // This is a measure to catch use-after-free errors. The idea is that if the
  // Future is stale or destructed memory, integrity_marker should almost
  // certainly not be equal to kValidIntegrityMarker. Furthermore, if it's equal
  // to kDestructedIntegrityMarker, we are almost certainly seeing a
  // use-after-free.
  //
  // This is not guaranteed to catch all use-after-free errors, because we are
  // testing against undefined behavior. On the other hand, if this check is
  // failing, it strongly indicates a memory issue. We have seen crashes in the
  // past that suggest Futures are used after destruction without immediately
  // crashing ((broken link)). We want to crash earlier in these cases.
  void AssertIntegrity() const;

#if !IMP_DISABLE_FUTURE_VALIDATION
  int integrity_marker_ = kValidIntegrityMarker;
#endif
};

template <typename T>
Future<T>::Future()
    : impl_wrapper_(std::make_shared<ImplWrapper>(std::make_shared<Impl>(
          internal::ResultHolderWithTypeFromStatus<Result>))) {}

template <typename T>
template <
    typename R,
    typename std::enable_if_t<
        !std::is_same_v<std::decay_t<R>, absl::Status> &&
            !std::is_same_v<std::decay_t<R>, Future<T>> &&
            !std::is_same_v<std::decay_t<R>, typename Future<T>::Result> &&
            !std::is_same_v<std::decay_t<R>, typename Future<T>::Value> &&
            std::is_constructible_v<typename Future<T>::Result, R>,
        int>>
Future<T>::Future(R&& result)
    : impl_wrapper_(std::make_shared<ImplWrapper>(std::make_shared<Impl>(
          internal::ResultHolder(Result(std::forward<R>(result)))))) {}

template <typename T>
template <typename R, typename>
Future<T>::Future(R result, int)
    : impl_wrapper_(std::make_shared<ImplWrapper>(std::make_shared<Impl>(
          internal::ResultHolder(Result(std::move(result)))))) {}

template <typename T>
template <typename V, typename>
Future<T>::Future(V result)
    : impl_wrapper_(std::make_shared<ImplWrapper>(
          std::make_shared<Impl>(internal::ResultHolder(std::move(result))))) {}

template <typename T>
Future<T>::Future(absl::Status result)
    : impl_wrapper_(std::make_shared<ImplWrapper>(std::make_shared<Impl>(
          internal::ResultHolder(Result(std::move(result)))))) {}

template <typename T>
Future<T>::Future(const mediapipe::StatusBuilder& builder)
    : Future(absl::Status(builder)) {}

template <typename T>
Future<T>::~Future() {
#if !IMP_DISABLE_FUTURE_VALIDATION
  integrity_marker_ = kDestructedIntegrityMarker;
#endif
}

template <typename T>
bool Future<T>::Ready() const {
  AssertIntegrity();
  return impl_wrapper_->GetImpl()->Ready();
}

template <typename T>
const typename Future<T>::Result& Future<T>::Get() const {
  AssertIntegrity();
  return impl_wrapper_->GetImpl()->Get().template GetAs<Result>();
}

template <typename T>
typename Future<T>::Result Future<T>::Move() const {
  AssertIntegrity();
  return impl_wrapper_->GetImpl()->Get().template MoveAs<Result>();
}

template <typename T>
auto Future<T>::MoveOrGet() {
  AssertIntegrity();
  return impl_wrapper_->GetImpl()->Get().template MoveOrGetAs<Result>();
}

template <typename T>
template <typename Fn>
auto Future<T>::Then(Fn&& fn, Executor::Type executor_type) const {
  AssertIntegrity();
  return Then(std::forward<Fn>(fn),
              FutureThenOptions{.executor = executor_type});
}

template <typename T>
template <typename Fn>
auto Future<T>::Then(Fn&& fn, FutureThenOptions then_options) const {
  AssertIntegrity();
  static_assert(
      !internal::future_traits::InvokingRequiresNoParamV<Fn> ||
          internal::future_traits::IsValidThenFnForFutureNoParamsV<Fn, T>,
      "Cannot call .Then() with provided zero-parameter-function "
      "because the future is not Future<absl::Status> and returns a "
      "value that must be handled.");
  static_assert(internal::future_traits::InvokingRequiresNoParamV<Fn> ||
                    internal::future_traits::IsValidThenFnForFutureV<Fn, T>,
                "Cannot call .Then() with provided function due to type "
                "mismatch. Function does not take result of Future as its "
                "parameter.");

  // The type returned by the functor.
  using FnResult =
      typename internal::future_traits::FnInvokeResult<Fn,
                                                       Value>::FnInvokeResultT;

  // The type of future Then will return based upon the result of the functor.
  // If the functor returns void, this is Future<absl::Status>.
  // If the functor returns a future, that type of future is returned.
  // Otherwise, returns a Future to the type returned.
  using ThenFuture = internal::future_traits::FnResultToFutureT<FnResult>;

  Impl::ResultProducer result_producer =
      [fn = std::forward<Fn>(fn)](std::shared_ptr<Impl>& then_impl,
                                  internal::ResultHolder& result) mutable {
        // If the function doesn't take the Result, and the status is not ok,
        // then we simply propagate the status.
        if (internal::ReturnIfUnhandledError<Fn, Result>(then_impl, result)) {
          return;
        }

        // ReturnResultToFuture below can call synchronously children futures.
        //
        // We have to make sure that `fn` (user supplied lambda) will go out of
        // scope before that to maintain in-order destruction of lambdas and
        // captures.
        //
        auto&& result_value =
            internal::InvokeThenFunctor<Fn, T>(std::move(fn), result);

        // Invoke the functor with the result using helper function
        // InvokeThenFunctor and then return the result to the future using the
        // helper function ReturnResultToFuture.
        //
        // InvokeThenFunctor handles all the combinatorial cases of valid
        // function signatures that the functor could have: Does the functor
        // take the Result or the Value? Does the functor return void? Does the
        // result need to be moved?
        //
        // ReturnResultToFuture handles all the combinatorial cases of valid
        // results. Is it a future? Is it an absl::StatusOr? is it a value?
        internal::ReturnResultToFuture(
            then_impl, std::forward<decltype(result_value)>(result_value));
      };

  // Sets up the then to run the result producer.
  // It's important for the then to own the result producer instead of just
  // calling it when this becomes ready so that the result producer can also
  // be run if the future is independently cancelled.
  Executor* ex = internal::GetExecutor(then_options.executor);

  // Create the future and pass in this future as the parent future.
  ThenFuture then(std::make_shared<ImplWrapper>(std::make_shared<Impl>(
      impl_wrapper_, std::move(result_producer), ex, then_options.executor_mode,
      internal::ResultHolderWithTypeFromStatus<typename ThenFuture::Result>,
      internal::ResultHolderWithTypeFromStatus<Result>)));

  // Schedule the result producer to be invoked when this future becomes ready
  // with the result of this future.
  impl_wrapper_->GetImpl()->AddChild(then.impl_wrapper_->GetImpl());

  if (then_options.task_priority) {
    then.impl_wrapper_->GetImpl()->UpdatePriority(then_options.task_priority);
  } else {
    impl_wrapper_->GetImpl()->BubbleUpPriority(kNormalTaskPriority);
  }

  return then;
}

template <typename T>
void Future<T>::Return(Result result) const {
  AssertIntegrity();
  // Make a local copy of impl_wrapper so that if Returning results in this
  // future being destroyed it won't be destructed too early.
  std::shared_ptr<Future<T>::ImplWrapper> impl_wrapper = impl_wrapper_;
  impl_wrapper->GetImpl()->Return(internal::ResultHolder(std::move(result)));
}

template <typename T>
void Future<T>::Cancel() {
  AssertIntegrity();
  if (!impl_wrapper_->GetImpl()->Ready()) {
    internal::FutureImpl::InvokeResultProducer(
        impl_wrapper_->GetImpl(),
        absl::CancelledError("from Future::Cancel()"));
  }
}

template <typename T>
template <typename... Args>
void Future<T>::DependsOn(Args&&... args) const {
  AssertIntegrity();
  (impl_wrapper_->GetImpl()->DependsOn(Holdable(std::forward<Args>(args))),
   ...);
}

template <typename T>
template <typename Rememberer>
void Future<T>::KeptBy(Rememberer rememberer,
                       FutureKeptByMode kept_by_mode) const {
  AssertIntegrity();
  impl_wrapper_->GetImpl()->AddKeptForgetter(
      rememberer->Remember(Holdable(*this)), kept_by_mode);
}

template <typename T>
template <typename... FutureTypes>
Future<absl::Status> Future<T>::Combine(FutureTypes&&... futures) const {
  Future<absl::Status> result;

  // Add one to include this future in addition to the parameters.
  constexpr std::size_t kNumFutures = sizeof...(FutureTypes) + 1;
  auto remaining_futures_counter =
      std::make_unique<absl::BlockingCounter>(kNumFutures);

  internal::AddFutureToCombineResult(result.impl_wrapper_, impl_wrapper_,
                                     remaining_futures_counter.get(), nullptr);

  (internal::AddFutureToCombineResult(result.impl_wrapper_,
                                      futures.impl_wrapper_,
                                      remaining_futures_counter.get(), nullptr),
   ...);

  result.DependsOn(std::move(remaining_futures_counter));

  return result;
}

template <typename T>
template <typename... FutureTypes>
Future<absl::Status> Future<T>::CombineWaitForAll(
    FutureTypes&&... futures) const {
  Future<absl::Status> result;

  // Add one to include this future in addition to the parameters.
  constexpr std::size_t kNumFutures = sizeof...(FutureTypes) + 1;
  auto remaining_futures_counter =
      std::make_unique<absl::BlockingCounter>(kNumFutures);
  auto combined_status = std::make_unique<absl::Status>(absl::OkStatus());

  internal::AddFutureToCombineResult(result.impl_wrapper_, impl_wrapper_,
                                     remaining_futures_counter.get(),
                                     combined_status.get());

  (internal::AddFutureToCombineResult(
       result.impl_wrapper_, futures.impl_wrapper_,
       remaining_futures_counter.get(), combined_status.get()),
   ...);

  result.DependsOn(std::move(remaining_futures_counter));
  result.DependsOn(std::move(combined_status));

  return result;
}

template <typename T>
template <typename Fn>
Future<T> Future<T>::Schedule(Fn&& fn, Executor::Type executor_type) {
  return Schedule(std::forward<Fn>(fn),
                  FutureScheduleOptions{.executor = executor_type});
}

template <typename T>
template <typename Fn>
Future<T> Future<T>::Schedule(Fn&& fn, FutureScheduleOptions schedule_options) {
  using FnResult = typename internal::future_traits::FnInvokeResult<
      Fn, absl::Status>::FnInvokeResultT;
  if constexpr (std::is_same_v<Result, absl::Status>) {
    static_assert(
        std::is_void_v<FnResult> || std::is_same_v<FnResult, absl::Status> ||
            std::is_same_v<FnResult, Future<absl::Status>>,
        "Future<absl::Status>::Schedule must return void, absl::Status, or "
        "Future<absl::Status>");
  } else {
    static_assert(
        std::is_same_v<FnResult, Result> || std::is_same_v<FnResult, Value> ||
            std::is_same_v<FnResult, Future<Value>>,
        "Future<T>::Schedule must return T, absl::StatusOr<T>, or Future<T>.");
  }

  Impl::ResultProducer result_producer =
      [fn = std::forward<Fn>(fn)](std::shared_ptr<Impl>& scheduled_impl,
                                  internal::ResultHolder& result) mutable {
        // ResultHolder always holds a Status in the case of calling Schedule,
        // because there is no parent future.
        const absl::Status& status = result.GetAs<absl::Status>();
        if constexpr (std::is_invocable_v<Fn, absl::Status>) {
          // ReturnResultToFuture below can call synchronously children futures.
          //
          // We have to make sure that `fn` (user supplied lambda) will go out
          // of scope before that to maintain in-order destruction of lambdas
          // and captures.
          //
          auto&& result_value = [fn = std::move(fn),
                                 &status]() mutable -> decltype(auto) {
            return fn(status);
          }();

          internal::ReturnResultToFuture(
              scheduled_impl,
              std::forward<decltype(result_value)>(result_value));
        } else {
          if (!status.ok()) {
            internal::ReturnResultToFuture(scheduled_impl, status);
          } else {
            // ReturnResultToFuture below can call synchronously children
            // futures.
            //
            // We have to make sure that `fn` (user supplied lambda) will go out
            // of scope before that to maintain in-order destruction of lambdas
            // and captures.
            //
            auto&& result_value =
                [fn = std::move(fn)]() mutable -> decltype(auto) {
              return fn();
            }();

            internal::ReturnResultToFuture(
                scheduled_impl,
                std::forward<decltype(result_value)>(result_value));
          }
        }
      };

  Executor* ex = internal::GetExecutor(schedule_options.executor);

  Future<T> future(std::make_shared<ImplWrapper>(std::make_shared<Impl>(
      std::move(result_producer), ex, schedule_options.executor_mode,
      internal::ResultHolderWithTypeFromStatus<typename Future<T>::Result>,
      internal::ResultHolderWithTypeFromStatus<absl::Status>)));

  if (schedule_options.task_priority) {
    future.impl_wrapper_->GetImpl()->UpdatePriority(
        schedule_options.task_priority);
  }

  internal::FutureImpl::InvokeResultProducer(future.impl_wrapper_->GetImpl(),
                                             absl::OkStatus());
  return future;
}

template <typename T>
template <typename... FutureTypes>
auto Future<T>::Merge(FutureTypes... futures) const {
  using TupleType = std::tuple<Value, typename FutureTypes::Value...>;
  using TupleTypeOr = absl::StatusOr<TupleType>;

  return Combine(futures...)
      .Then(
          [self = *this,
           futures...](absl::Status status) mutable -> TupleTypeOr {
            // One or more of the combined futures failed, return the failure
            // status.
            if (!status.ok()) {
              return status;
            }

            return TupleType{*self.MoveOrGet(), *futures.MoveOrGet()...};
          },
          {.executor = Executor::Type::kImmediate});
}

template <typename T>
template <typename ListT>
Future<absl::Status> Future<T>::CombineList(const ListT& futures) {
  using FutureT = typename ListT::value_type;
  static_assert(std::is_same_v<FutureT, Future<T>>);

  if (futures.empty()) {
    // If there are no futures, return a ready future.
    return absl::OkStatus();
  } else if (futures.size() == 1) {
    if constexpr (std::is_same_v<Result, absl::Status>) {
      // There is only one future and it is already of the correct type so just
      // return it.
      return futures.at(0);
    } else {
      // There is only one future, but it's the wrong type so just convert it
      // without combining anything.
      return futures.at(0).Then(
          [](const Result& result) { return result.status(); });
    }
  }

  Future<absl::Status> result;

  const std::size_t kNumFutures = futures.size();
  auto remaining_futures_counter =
      std::make_unique<absl::BlockingCounter>(kNumFutures);

  for (auto future : futures) {
    internal::AddFutureToCombineResult(
        result.impl_wrapper_, future.impl_wrapper_,
        remaining_futures_counter.get(), nullptr);
  }

  result.DependsOn(std::move(remaining_futures_counter));

  return result;
}

template <typename T>
template <typename ListT>
Future<absl::Status> Future<T>::CombineListWaitForAll(const ListT& futures) {
  using FutureT = typename ListT::value_type;
  static_assert(std::is_same_v<FutureT, Future<T>>);

  // Note: The implementation is mostly duplicated from CombineList.
  // The details are intentionally not refactored into a helper function to
  // avoid increasing the depth of Future stack traces & because of the rule of
  // three.

  if (futures.empty()) {
    // If there are no futures, return a ready future.
    return Future<absl::Status>(absl::OkStatus());
  } else if (futures.size() == 1) {
    if constexpr (std::is_same_v<Result, absl::Status>) {
      // There is only one future and it is already of the correct type so just
      // return it.
      return futures.at(0);
    } else {
      // There is only one future, but it's the wrong type so just convert it
      // without combining anything.
      return futures.at(0).Then(
          [](const Result& result) { return result.status(); });
    }
  }

  Future<absl::Status> result;

  const std::size_t kNumFutures = futures.size();
  auto remaining_futures_counter =
      std::make_unique<absl::BlockingCounter>(kNumFutures);
  auto combined_status = std::make_unique<absl::Status>(absl::OkStatus());

  for (auto future : futures) {
    internal::AddFutureToCombineResult(
        result.impl_wrapper_, future.impl_wrapper_,
        remaining_futures_counter.get(), combined_status.get());
  }

  result.DependsOn(std::move(remaining_futures_counter));
  result.DependsOn(std::move(combined_status));

  return result;
}

template <typename T>
template <typename ListT>
Future<std::vector<typename Future<T>::Value>> Future<T>::MergeList(
    const ListT& futures) {
  using FutureT = typename ListT::value_type;
  static_assert(std::is_same_v<FutureT, Future<T>>);

  Future<absl::Status> combined_future = CombineList(futures);

  // We copy the list into a vector to avoid issues where if it's a Span it is
  // out-of-scope by the time the Future's callback executes.
  return combined_future.Then(
      [futures = std::vector<Future<T>>(futures.begin(), futures.end())]() {
        std::vector<Value> result;
        result.reserve(futures.size());

        // Copy the values over in-order.
        for (const Future<T>& future : futures) {
          if constexpr (std::is_copy_constructible_v<Value>) {
            result.push_back(future.Get().value());
          } else {
            result.push_back(std::move(future.Move().value()));
          }
        }

        return result;
      });
}

template <typename T>
void Future<T>::UpdatePriority(std::optional<int> priority) {
  impl_wrapper_->GetImpl()->UpdatePriority(priority);
}

template <typename T>
int Future<T>::GetActivePriority() const {
  return impl_wrapper_->GetImpl()->GetActivePriority();
}

template <typename T>
std::optional<int> Future<T>::GetSelfPriority() const {
  return impl_wrapper_->GetImpl()->GetSelfPriority();
}

template <typename T>
int Future<T>::GetDepth() const {
  return impl_wrapper_->GetImpl()->GetDepth();
}

template <typename T>
void Future<T>::AssertIntegrity() const {
#if !IMP_DISABLE_FUTURE_VALIDATION
  if (integrity_marker_ == kDestructedIntegrityMarker) {
    IMP_LOG(imp::FATAL) << "Future is marked as destructed.";
  }
  if (integrity_marker_ != kValidIntegrityMarker) {
    IMP_LOG(imp::FATAL) << "Future has corrupted memory.";
  }
#endif
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_H_
