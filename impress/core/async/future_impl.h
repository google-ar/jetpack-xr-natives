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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_IMPL_H_

#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/blocking_counter.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/optional.h"
#include "core/async/executor.h"
#include "core/async/future_common.h"
#include "core/async/future_traits.h"
#include "core/async/task.h"
#include "core/async/task_priority.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"
#include "core/common/type_erased.h"

namespace imp {

namespace internal {

// Used by the ResultHolder to hold the type-erased result.
//
// The inline storage size is 24 bytes because the ResultHolder often stores a
// StatusOr<T>, so we want it to be big enough to store types like
// StatusOr<OwnedPtr<Foo>> which will be 24 bytes. 8 bytes for the status, and
// 16 bytes for the OwnedPtr.
using ErasedResult = SizedTypeErased<24, 8>;

template <typename T>
const absl::Status& RetrieveStatusFromResultHelper(
    const ErasedResult& erased_result) {
  return erased_result.Get<T>().status();
}

template <>
const absl::Status& RetrieveStatusFromResultHelper<absl::Status>(
    const ErasedResult& erased_result);

// Holds and provides access to the type-erased result of a future.
//
// Guarantees at compile time that the held value is either a StatusOr<T> or a
// Status.
//
// Important: Copying ResultHolder creates a shallow copy - the same
// underlying result is stored. If the result is moved out of the holder it
// will also impact the other copies of the ResultHolder.
class ResultHolder {
 public:
  // Constructs a ResultHolder with no result.
  ResultHolder() = default;

  // Constructs a ResultHolder with the result passed in.
  // T must be a Status or a StatusOr.
  template <typename T>
  explicit ResultHolder(T&& result)
      : held_result_(std::forward<T>(result)),
        retrieve_status_fn_(RetrieveStatusFromResultHelper<std::decay_t<T>>) {
    static_assert(future_traits::IsValidResultV<std::decay_t<T>>);
  }

  // Returns true if the ResultHolder is holding a result.
  bool HasResult() const { return held_result_.HasValue(); }

  // Casts the result to T and moves it out of the ResultHolder to return it.
  // It is expected that the caller knows what the real type is.
  template <typename T>
  T MoveAs() {
    return std::move(held_result_.Get<T>());
  }

  // Casts the result to T and to returns it as a const reference.
  // It is expected that the caller knows what the real type is.
  template <typename T>
  const T& GetAs() const {
    return held_result_.Get<T>();
  }

  template <typename T>
  auto MoveOrGetAs() {
    if constexpr (std::is_copy_constructible_v<T>) {
      return GetAs<T>();
    } else {
      return MoveAs<T>();
    }
  }

  // Casts the result to T and returns it.
  //
  // It is determined at compile time if the result should be
  // moved or returned as a const reference.
  //
  // By default, it will be moved if the type itself is move-only.
  // Optionally, you can pass in a function type as a template parameter in
  // which case the result will only be moved if the function requires the
  // result to be moved to be invoked. This is true if the type is move only
  // and the function takes the result as a value instead of a reference.
  template <typename T, typename Fn = void (*)(T),
            future_traits::EnableIfInvokingRequiresMove<Fn, T> = true>
  T GetOrMoveAs() {
    return MoveAs<T>();
  }

  template <typename T, typename Fn = void (*)(T),
            future_traits::EnableIfInvokingDoesNotRequireMove<Fn, T> = true>
  const T& GetOrMoveAs() {
    return GetAs<T>();
  }

  const absl::Status& GetStatus() const {
    return retrieve_status_fn_(held_result_);
  }

 private:
  using RetrieveStatusFn = const absl::Status& (*)(const ErasedResult&);

  ErasedResult held_result_;
  RetrieveStatusFn retrieve_status_fn_;
};

template <typename T>
ResultHolder GetOrMoveResultHolderHelper(ResultHolder& result_holder) {
  return ResultHolder(result_holder.MoveOrGetAs<T>());
}

// Implementation of Future functionality that can be type-erased.
// Also, copies of a Future hodld the same FutureImpl allowing futures to be
// shallow-copied.
class FutureImpl {
 public:
  using ResultProducer =
      Invocable<void(std::shared_ptr<FutureImpl>&, ResultHolder&)>;

  using RetrieveResultFn = ResultHolder (*)(ResultHolder&);

  using StatusToResultFn = ResultHolder (*)(absl::Status);

  // Used to wrap a FutureImpl in a layer of indirection that will then Cancel
  // the FutureImpl (if needed) upon destruction. This allows futures to be
  // reference counted and cancelled when all are gone.
  //
  // The reason this isn't simply done in the FutureImpl destructor is because
  // the Cancel operation may be asynchronous if it's completed on a different
  // executor which means that the FutureImpl must actually continue to live
  // after the destructor exits.
  //
  // *WARNING* FutureImpl should not store weak references to the ImplWrappers
  // of it's children because it can cause temporary strong references to be
  // created when locked that can cause races where the FutureImplWrapper's last
  // reference is dropped during Return in a background thread, preventing the
  // future from being destroyed in the foreground.
  class FutureImplWrapper {
   public:
    explicit FutureImplWrapper(const std::shared_ptr<FutureImpl>& impl);
    ~FutureImplWrapper();

    std::shared_ptr<FutureImpl>& GetImpl();

   private:
    std::shared_ptr<FutureImpl> impl_;
  };

  // Used for a default constructed Future.
  explicit FutureImpl(StatusToResultFn status_to_result_holder_fn);

  // Used for a Future that is created with a result.
  explicit FutureImpl(ResultHolder result_holder);

  // Used for a future created via Future::Schedule.
  FutureImpl(ResultProducer result_producer, Executor* producer_executor,
             FutureExecutorMode future_executor_mode,
             StatusToResultFn status_to_result_holder_fn,
             StatusToResultFn parent_status_to_result_holder_fn);

  // Used for a future created via Future::Then.
  FutureImpl(const std::shared_ptr<FutureImplWrapper>& parent_future,
             ResultProducer result_producer, Executor* producer_executor,
             FutureExecutorMode future_executor_mode,
             StatusToResultFn status_to_result_holder_fn,
             StatusToResultFn parent_status_to_result_holder_fn);

  ~FutureImpl();

  // Returns true iff the FutureImpl if either Return() or Cancel() has been
  // called.
  bool Ready() const;

  // Returns the underlying value. FATAL if the underlying value is nullptr,
  // aka FutureImpl is not ready.
  ResultHolder& Get() ABSL_LOCKS_EXCLUDED(mu_);

  // Registers a callback to be executed when the future is ready.
  // N.B. OnReady callbacks will be called when a Future is cancelled!
  void OnReady(imp::Invocable<void(ResultHolder&)> fn) ABSL_LOCKS_EXCLUDED(mu_);

  // Called through Future::Then, used to add a child future to this future.
  // The child future will be invoked when this future becomes ready.
  void AddChild(std::weak_ptr<FutureImpl> child) ABSL_LOCKS_EXCLUDED(mu_);

  // Called through Future::Combine, used to add a child future to this
  // future. The child future will be invoked when all parent combined futures
  // become ready.
  void AddCombineChild(std::weak_ptr<FutureImpl> child,
                       absl::BlockingCounter* remaining_futures_counter,
                       absl::Status* combined_status) ABSL_LOCKS_EXCLUDED(mu_);

  void AddCombineParent(const std::shared_ptr<FutureImplWrapper>& parent)
      ABSL_LOCKS_EXCLUDED(mu_);

  // Called when a future's result producer returns a future. The nested
  // future will be invoked when this future becomes ready.
  void AddNestedChild(std::weak_ptr<FutureImpl> child,
                      RetrieveResultFn retrieve_result_fn)
      ABSL_LOCKS_EXCLUDED(mu_);

  // Called through Future::KeptBy, used to add a forgetter to this future.
  // The forgetter will be invoked when this future becomes ready.
  void AddKeptForgetter(Invocable<void()> forget_fn,
                        FutureKeptByMode kept_by_mode) ABSL_LOCKS_EXCLUDED(mu_);

  // Ensure the specified Holdable is retained in memory until the future
  // completes. Holdable is used to wrap an object in a type-erased way. This
  // works best with heap allocated objects with automatic lifetimes like
  // unique_ptr and shared_ptr. It can also work with custom types if
  // implemented careful, like Future itself.
  void DependsOn(Holdable holdable) ABSL_LOCKS_EXCLUDED(mu_);

  // Stores the nested future potentially returned from a future's result
  // producer. This is used to retain the memory of the nested future and
  // track it so it can be dropped during cancellation.
  void SetNestedFuture(const std::shared_ptr<FutureImplWrapper>& nested_future)
      ABSL_LOCKS_EXCLUDED(mu_);

  // Returns the value to the future, making it become ready.
  //
  // The ResultHolder must wrap the Future<T>::Result type of the future
  // containing this FutureImpl.
  void Return(ResultHolder value) ABSL_LOCKS_EXCLUDED(mu_);

  // Returns the status to the future, making it become ready.
  //
  // Automatically handles wrapping the status in a ResultHolder, creating the
  // correct type of StatusOr<T> from the status if the Future contains a
  // StatusOr<T>.
  void Return(absl::Status status) ABSL_LOCKS_EXCLUDED(mu_);

  // Invokes the value producer for the impl passed in. This may get invoked
  // async if the current executor is not the same as the executor that the
  // value producer must run on. After the async work completes, it is
  // guaranteed that the future will become ready.
  // This also guarantees that impl is kept alive until invoking the value
  // producer is complete.
  static void InvokeResultProducer(std::shared_ptr<FutureImpl>& impl,
                                   absl::Status result_status)
      ABSL_LOCKS_EXCLUDED(mu_);

  static void InvokeResultProducerFromParent(
      std::weak_ptr<FutureImpl>& weak_impl) ABSL_LOCKS_EXCLUDED(mu_);

  // Updates the self priority of this future, and then updates the active
  // priority of this future and bubbles up the priority of any children that
  // should have their priority bubbled up to by calling BubbleUpPriority.
  void UpdatePriority(std::optional<int> priority) ABSL_LOCKS_EXCLUDED(mu_);

  // Refreshes the active priority of this future and bubbles up the priority
  // of any children that should have their priority bubbled up to.
  //
  // This is different from UpdatePriority, which actually assigns a new self
  // priority to the future.
  void BubbleUpPriority(std::optional<int> changed_priority = std::nullopt)
      ABSL_LOCKS_EXCLUDED(mu_);

  int GetActivePriority() ABSL_LOCKS_EXCLUDED(mu_);

  std::optional<int> GetSelfPriority() ABSL_LOCKS_EXCLUDED(mu_);

  int GetDepth() const;

 private:
  // When Future::KeptBy is used, this is used to track the forgetter and the
  // mode that must be resolved when the future becomes ready.
  struct KeptForgetter {
    Invocable<void()> forget_fn;
    FutureKeptByMode kept_by_mode;
  };

  // When Combine is used, this is used to track the child future, the
  // remaining futures counter, and the combined status that must be resolved
  // when the future becomes ready.
  struct CombineChild {
    std::weak_ptr<FutureImpl> child;
    absl::BlockingCounter* remaining_futures_counter;
    absl::Status* combined_status;
  };

  // When a nested future is returned from a result producer, this is used to
  // track the child future and the function that must be used to retrieve the
  // result from the result holder.
  struct NestedChild {
    std::weak_ptr<FutureImpl> child;
    RetrieveResultFn retrieve_result_fn;
  };

  // Variant for all the relationships to this future that are resolved when
  // it becomes ready.
  //
  // This is stored in a variant instead of in separate vectors because in
  // testing it was found that the variant version uses less memory overall,
  // despite the variant increasing the memory of each element.
  using Relationship =
      std::variant<std::weak_ptr<FutureImpl>, CombineChild, NestedChild,
                   KeptForgetter, Invocable<void(ResultHolder&)>, Holdable>;

  bool ReturnInternal(ResultHolder result) ABSL_EXCLUSIVE_LOCKS_REQUIRED(mu_);
  bool ReturnInternal(absl::Status status) ABSL_EXCLUSIVE_LOCKS_REQUIRED(mu_);
  bool HasResultOrIsExecutingResultProducer()
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mu_);

  // Get the list of futures that should have their priority bubbled up to
  // from this future.
  std::vector<std::shared_ptr<FutureImplWrapper>> GetPriorityBubbleUpTargets()
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mu_);

  // Refreshes the active priority of this future. Returns true if the active
  // priority changed.
  bool RefreshActivePriority(std::optional<int> changed_priority = std::nullopt)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mu_);
  void ResolveCombineChild(const absl::Status& result_status,
                           CombineChild& combine_child)
      ABSL_LOCKS_EXCLUDED(mu_);
  void ResolveNestedChild(ResultHolder& result_holder,
                          NestedChild& nested_child) ABSL_LOCKS_EXCLUDED(mu_);
  void ResolveKeptForgetter(const absl::Status& result_status,
                            KeptForgetter& kept_forgetter)
      ABSL_LOCKS_EXCLUDED(mu_);

  mutable absl::Mutex mu_;

  absl::optional<absl::Status> pending_result_status_ ABSL_GUARDED_BY(mu_);

  // Type-erased value held by the future as a std::shared_ptr.
  ResultHolder result_ ABSL_GUARDED_BY(mu_);

  // Relationships that are resolved when this future becomes ready.
  std::vector<Relationship> relationships_ ABSL_GUARDED_BY(mu_);

  // If set, this is the type-erased parent of this future.
  // Used to hold the parent future in memory until this futures result
  // producer executes.
  std::shared_ptr<FutureImplWrapper> parent_future_ ABSL_GUARDED_BY(mu_);

  // If set, this is the nested future returned by the result producer. This
  // is used to retain the memory of the nested future and track it so it can
  // be dropped during cancellation.
  std::shared_ptr<FutureImplWrapper> nested_future_ ABSL_GUARDED_BY(mu_);

  // If this future was created by a Combine/Merge method, this is the list of
  // parents that must resolve before this future can resolve.
  std::vector<std::shared_ptr<FutureImplWrapper>> combine_parents_
      ABSL_GUARDED_BY(mu_);

  // Invocable that takes a ResultHolder as a parameter and then assigns the
  // result to the Future. By default, this is just a simple pass-through.
  // Calling Future::Schedule or Future::Then will assign the result_producer
  // so that it calls the method passed into Schedule or Then to determine the
  // result.
  ResultProducer result_producer_ ABSL_GUARDED_BY(mu_);

  // The executor that the result_producer will run on.
  Executor* producer_executor_ ABSL_GUARDED_BY(mu_) = nullptr;

  FutureExecutorMode future_executor_mode_ ABSL_GUARDED_BY(mu_) =
      FutureExecutorMode::kScheduleIfNotOnExecutorThread;

  std::optional<int> self_priority_ ABSL_GUARDED_BY(mu_) = std::nullopt;
  int task_priority_ ABSL_GUARDED_BY(mu_) = kNormalTaskPriority;
  TaskId extant_task_id_ ABSL_GUARDED_BY(mu_) = kInvalidTaskId;

  // The type stored by the ResultHolder must be the same as Future::Result.
  // This function pointer is used to convert a Status to a ResultHolder of
  // the correct type, since FutureImpl doesn't know the type of the Result.
  StatusToResultFn status_to_result_for_self_fn_ ABSL_GUARDED_BY(mu_);
  // Same as above, but instead this is for the ResultHolder that is expected
  // to be passed into the result producer. If the future has a parent, then
  // this will be the parent futures Result type. Otherwise, it is just the
  // Status.
  StatusToResultFn status_to_result_for_result_producer_fn_
      ABSL_GUARDED_BY(mu_);
};

// Helpers for accessing status from a StatusOr or Status.
template <typename T>
const absl::Status& GetStatus(const absl::StatusOr<T>& statusor) {
  return statusor.status();
}
const absl::Status& GetStatus(const absl::Status& status);

// Helpers for accessing value from a StatusOr or Status.
template <typename T>
const T& GetValue(const absl::StatusOr<T>& statusor) {
  return *statusor;
}
template <typename T>
T GetValue(absl::StatusOr<T>&& statusor) {
  return std::move(*statusor);
}
const absl::Status& GetValue(const absl::Status& status);

// Helper for returning a status to a future if it's not ok and a functor
// can't take the status as a parameter. This is used by Future::Then to
// propagate errors when the result producer can't handle them.
template <typename Fn, typename T,
          future_traits::EnableIfInvokingRequiresValue<Fn, T> = true>
bool ReturnIfUnhandledError(std::shared_ptr<FutureImpl>& then_impl,
                            ResultHolder& result) {
  absl::Status status = GetStatus(result.GetAs<T>());
  if (!status.ok()) {
    then_impl->Return(std::move(status));
    return true;
  }
  return false;
}

template <typename Fn, typename T,
          future_traits::EnableIfInvokingRequiresResult<Fn, T> = true>
bool ReturnIfUnhandledError(std::shared_ptr<FutureImpl>& then_impl,
                            ResultHolder& result) {
  return false;
}

// Retrieves either the Result or Value from a result holder based on which
// the function signature Fn requires to be invoked.
// Will also automatically move the result from the result holder if needed.
template <typename Fn, typename T,
          future_traits::EnableIfInvokingRequiresResult<Fn, T> = true>
decltype(auto) RetrieveResultOrValue(ResultHolder& result) {
  return result.GetOrMoveAs<future_traits::ResultT<T>, Fn>();
}

template <typename Fn, typename T,
          future_traits::EnableIfInvokingRequiresValue<Fn, T> = true>
decltype(auto) RetrieveResultOrValue(ResultHolder& result) {
  return GetValue(result.GetOrMoveAs<future_traits::ResultT<T>, Fn>());
}

// Invoke the functor with the result.
// Handles all the combinatorial cases of valid function signatures that the
// functor could have:
// Does the functor take the Result or the Value?
// Does the functor return void?
// Does the result need to be moved?
template <typename Fn, typename T,
          future_traits::EnableIfInvokingRequiresNoParam<Fn> = true,
          future_traits::EnableIfInvokeNoParamResultNotVoid<Fn> = true>
decltype(auto) InvokeThenFunctor(Fn fn, ResultHolder& result) {
  return fn();
}

template <typename Fn, typename T,
          future_traits::EnableIfInvokingRequiresNoParam<Fn> = true,
          future_traits::EnableIfInvokeNoParamResultVoid<Fn> = true>
decltype(auto) InvokeThenFunctor(Fn fn, ResultHolder& result) {
  fn();
  return absl::OkStatus();
}

template <typename Fn, typename T,
          future_traits::DisableIfInvokingRequiresNoParam<Fn> = true,
          future_traits::EnableIfInvokeResultVoid<Fn, T> = true>
absl::Status InvokeThenFunctor(Fn fn, ResultHolder& result) {
  fn(RetrieveResultOrValue<Fn, T>(result));
  return absl::OkStatus();
}

template <typename Fn, typename T,
          future_traits::DisableIfInvokingRequiresNoParam<Fn> = true,
          future_traits::EnableIfInvokeResultNotVoid<Fn, T> = true>
decltype(auto) InvokeThenFunctor(Fn fn, ResultHolder& result) {
  return fn(RetrieveResultOrValue<Fn, T>(result));
}

// Helper function for converting any object into a ResultHolder.
// All results are either an absl::StatusOr or an absl::Status, therefore
// ResultHolder can only hold absl::Status or absl::StatusOr. If the type
// passed into ToResultHolder is either of the former types, it'll just
// directly convert it into a ResultHolder. Otherwise, it will wrap the object
// in an absl::StatusOr and pass it to ResultHolder.
template <typename T>
ResultHolder ToResultHolder(T value) {
  return ResultHolder(absl::StatusOr<T>(std::move(value)));
}
template <typename T>
ResultHolder ToResultHolder(absl::StatusOr<T> value) {
  return ResultHolder(std::move(value));
}
template <>
ResultHolder ToResultHolder(absl::Status value);

// Converts a Status into a ResultHolder of the templated type T.
//
// For instance, if T is absl::StatusOr<Foo>, then this will create a
// ResultHolder that holds a StatusOr<Foo> that contains the passed in status.
// This is used to convert a status to the correct type of ResultHolder in a
// type-erased way.
//
// This is different from ToResultHolder, which simply wraps its parameter
// in a ResultHolder directly. It can be used in a similar way as this by
// relying on implicit conversion from the Status to the parameter type, but
// in testing that was found to bloat binary size.
template <typename T>
ResultHolder ResultHolderWithTypeFromStatus(absl::Status value) {
  return ResultHolder(T(std::move(value)));
}

// Full specialization for absl::Status.
template <>
ResultHolder ResultHolderWithTypeFromStatus<absl::Status>(absl::Status value);

// Helper function used by both Future::Then and Future::Schedule to take the
// return value of the result producer and Return it to the future.
//
// If the result is a future, then the result of the result future is returned
// to the future when it becomes ready. Otherwise, the result is returned
// directly to the future.
template <typename T>
void ReturnResultToFuture(std::shared_ptr<FutureImpl>& then_impl, T result) {
  then_impl->Return(internal::ToResultHolder(std::move(result)));
}

template <typename T>
void ReturnResultToFuture(std::shared_ptr<FutureImpl>& then_impl,
                          Future<T> result_future) {
  then_impl->SetNestedFuture(result_future.impl_wrapper_);
  result_future.impl_wrapper_->GetImpl()->AddNestedChild(
      then_impl, GetOrMoveResultHolderHelper<typename Future<T>::Result>);
}

// Specialization for absl::Status that just calls the overload of Return that
// takes a status so that it can get correctly converted into the Result type of
// the future using status_to_result_for_self_fn_.
//
// Otherwise, a ResultHolder would be created from the status directly, which
// would be the wrong type for the future.
template <>
void ReturnResultToFuture(std::shared_ptr<FutureImpl>& then_impl,
                          absl::Status result_status);

Executor* GetExecutor(const ExecutorTypeOrExecutor& executor_type_or_executor);

// Forward declare this helper function so that Future can declare it as a
// friend. This is so this helper can access Future::impl_.
void AddFutureToCombineResult(
    const std::shared_ptr<FutureImpl::FutureImplWrapper>& combine_result,
    const std::shared_ptr<FutureImpl::FutureImplWrapper>& future_to_combine,
    absl::BlockingCounter* remaining_futures_counter,
    absl::Status* combined_status);

}  // namespace internal

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_IMPL_H_
