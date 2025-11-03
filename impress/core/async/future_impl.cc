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

#include "core/async/future_impl.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/synchronization/blocking_counter.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/variant.h"
#include "core/async/executor.h"
#include "core/async/future_common.h"
#include "core/async/task.h"
#include "core/async/task_priority.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"

namespace imp {
namespace internal {

const absl::Status& GetStatus(const absl::Status& status) { return status; }

const absl::Status& GetValue(const absl::Status& status) { return status; }

template <>
const absl::Status& RetrieveStatusFromResultHelper<absl::Status>(
    const ErasedResult& erased_result) {
  return erased_result.Get<absl::Status>();
}

template <>
ResultHolder ToResultHolder(absl::Status value) {
  return ResultHolder(std::move(value));
}

template <>
ResultHolder ResultHolderWithTypeFromStatus<absl::Status>(absl::Status value) {
  return ResultHolder(std::move(value));
}

FutureImpl::FutureImpl(StatusToResultFn status_to_result_holder_fn)
    : status_to_result_for_self_fn_(status_to_result_holder_fn),
      status_to_result_for_result_producer_fn_(status_to_result_holder_fn) {
  result_producer_ = [](std::shared_ptr<FutureImpl>& impl,
                        ResultHolder& result) {
    // It's safe to always move the result here because this default result
    // producer does not get used if the future is created via Future::Then,
    // meaning that this never gets invoked with a reference to a result stored
    // by another future, only by values passed into Future::Return or through
    // cancellation in which case the status goes through the
    // status_to_result_for_result_producer_fn_ and is the correct type.
    impl->Return(std::move(result));
  };
}

// When constructed with a value, we don't need to invoke any callbacks
// or signal the condition variable since nothing else can be watching
// the future yet.
FutureImpl::FutureImpl(ResultHolder result)
    : result_(std::forward<ResultHolder>(result)),
      status_to_result_for_self_fn_(nullptr),
      status_to_result_for_result_producer_fn_(nullptr) {
  // Note: status_to_result_for_self_fn_ isn't needed in this case because the
  // result is already assigned.
}

FutureImpl::FutureImpl(ResultProducer result_producer,
                       Executor* producer_executor,
                       FutureExecutorMode future_executor_mode,
                       StatusToResultFn status_to_result_holder_fn,
                       StatusToResultFn parent_status_to_result_holder_fn)
    : result_producer_(std::move(result_producer)),
      producer_executor_(producer_executor),
      future_executor_mode_(future_executor_mode),
      status_to_result_for_self_fn_(status_to_result_holder_fn),
      status_to_result_for_result_producer_fn_(
          parent_status_to_result_holder_fn) {}

FutureImpl::FutureImpl(const std::shared_ptr<FutureImplWrapper>& parent_future,
                       ResultProducer result_producer,
                       Executor* producer_executor,
                       FutureExecutorMode future_executor_mode,
                       StatusToResultFn status_to_result_holder_fn,
                       StatusToResultFn parent_status_to_result_holder_fn)
    : parent_future_(parent_future),
      result_producer_(std::move(result_producer)),
      producer_executor_(producer_executor),
      future_executor_mode_(future_executor_mode),
      status_to_result_for_self_fn_(status_to_result_holder_fn),
      status_to_result_for_result_producer_fn_(
          parent_status_to_result_holder_fn) {}

FutureImpl::~FutureImpl() {
  AssertIntegrity();
  absl::MutexLock lock(&mu_);
  if (!result_.HasResult()) {
    // Last ditch effort to cancel the future if it isn't ready when being
    // destroyed. It should only be possible for this to happen if
    // InvokeValueProducer scheduled work on an executor, and then the executor
    // is shut down before the work is run. This can happen when a view is being
    // destroyed / cleaned up, and is a valid case for reaching this block of
    // code.
    ReturnInternal(absl::CancelledError("Future is destroyed."));
  }
#if !IMP_DISABLE_FUTURE_VALIDATION
  integrity_marker_ = kDestructedIntegrityMarker;
#endif
}

bool FutureImpl::Ready() const {
  AssertIntegrity();
  absl::ReaderMutexLock lock(&mu_);
  return result_.HasResult();
}

ResultHolder& FutureImpl::Get() {
  AssertIntegrity();
  absl::ReaderMutexLock lock(&mu_);
  AssertResultHasNotBeenMoved();
  if (!result_.HasResult()) {
    IMP_LOG(imp::FATAL)
        << "FutureImpl::Get() called before FutureImpl::Return(), Future not "
           "Ready";
  }
  return result_;
}

void FutureImpl::OnReady(imp::Invocable<void(ResultHolder&)> fn) {
  AssertIntegrity();
  // We want to check the value & possibly manipulate callbacks
  // under the lock, but we don't want to run the function
  // under the lock.

  // Store the result in a pointer to pass into the fn later.
  // TODO: This is kind of scary, because this pointer is
  // essentially used to access the ResultHolder stored by FutureImpl from
  // outside of the lock. In practice, this has never caused a real problem
  // because the way Future's are actually used we are never accessing the
  // result of a future from two threads at the same time. The actual semantics
  // of the API discourage it.
  ResultHolder* result_ptr = nullptr;
  {
    absl::MutexLock lock(&mu_);
    if (!result_.HasResult()) {
      relationships_.emplace_back(std::move(fn));
      return;
    }
    result_ptr = &result_;
  }
  fn(*result_ptr);
}

void FutureImpl::AddChild(std::weak_ptr<FutureImpl> child) {
  AssertIntegrity();
  {
    absl::MutexLock lock(&mu_);
    AssertResultHasNotBeenMoved();
    if (!result_.HasResult()) {
      relationships_.emplace_back(child);
      return;
    }
  }

  // If we didn't return, then the future is already ready so we can invoke
  // the child immediately.
  internal::FutureImpl::InvokeResultProducerFromParent(child);
}

void FutureImpl::AddKeptForgetter(Invocable<void()> forget_fn,
                                  FutureKeptByMode kept_by_mode) {
  AssertIntegrity();
  ResultHolder* result_ptr = nullptr;

  {
    absl::MutexLock lock(&mu_);
    if (!result_.HasResult()) {
      relationships_.emplace_back(KeptForgetter{
          .forget_fn = std::move(forget_fn), .kept_by_mode = kept_by_mode});
      return;
    }
    result_ptr = &result_;
  }

  // If we didn't return, then the future is already ready so we can invoke
  // the forgetter immediately
  KeptForgetter kept_forgetter = {.forget_fn = std::move(forget_fn),
                                  .kept_by_mode = kept_by_mode};
  ResolveKeptForgetter(result_ptr->GetStatus(), kept_forgetter);
}

void FutureImpl::AddCombineChild(
    std::weak_ptr<FutureImpl> child,
    absl::BlockingCounter* remaining_futures_counter,
    absl::Status* combined_status) {
  ResultHolder* result_ptr = nullptr;
  AssertIntegrity();

  {
    absl::MutexLock lock(&mu_);
    AssertResultHasNotBeenMoved();
    if (!result_.HasResult()) {
      relationships_.emplace_back(
          CombineChild{.child = child,
                       .remaining_futures_counter = remaining_futures_counter,
                       .combined_status = combined_status});
      return;
    }
    result_ptr = &result_;
  }

  // If we didn't return, then the future is already ready so we can resolve
  // the child immediately.
  CombineChild combine_child = {
      .child = child,
      .remaining_futures_counter = remaining_futures_counter,
      .combined_status = combined_status};
  ResolveCombineChild(result_ptr->GetStatus(), combine_child);
}

void FutureImpl::AddCombineParent(
    const std::shared_ptr<FutureImplWrapper>& parent) {
  AssertIntegrity();
  int task_priority;
  {
    absl::MutexLock lock(&mu_);
    combine_parents_.push_back(parent);
    task_priority = task_priority_;
  }

  // Immediately bubble up the priority of the combine parent.
  parent->GetImpl()->BubbleUpPriority(task_priority);
}

void FutureImpl::AddNestedChild(std::weak_ptr<FutureImpl> child,
                                RetrieveResultFn retrieve_result_fn) {
  AssertIntegrity();
  ResultHolder* result_ptr = nullptr;

  bool did_add_child = false;
  {
    absl::MutexLock lock(&mu_);
    AssertResultHasNotBeenMoved();
    if (!result_.HasResult()) {
      relationships_.emplace_back(NestedChild{
          .child = child, .retrieve_result_fn = retrieve_result_fn});
      did_add_child = true;
    }
    result_ptr = &result_;
  }

  if (did_add_child) {
    BubbleUpPriority();
    return;
  }

  // If we didn't return, then the future is already ready so we can resolve
  // the child immediately.
  NestedChild nested_child = {.child = child,
                              .retrieve_result_fn = retrieve_result_fn};
  ResolveNestedChild(*result_ptr, nested_child);
}

void FutureImpl::DependsOn(Holdable holdable) {
  AssertIntegrity();
  absl::MutexLock lock(&mu_);
  if (!result_.HasResult()) {
    relationships_.emplace_back(std::move(holdable));
  }
}

void FutureImpl::SetNestedFuture(
    const std::shared_ptr<FutureImplWrapper>& nested_future) {
  AssertIntegrity();
  int task_priority;
  {
    absl::MutexLock lock(&mu_);
    nested_future_ = nested_future;
    task_priority = task_priority_;
  }

  // Immediately bubble up the priority of the nested future.
  nested_future->GetImpl()->BubbleUpPriority(task_priority);
}

void FutureImpl::Return(ResultHolder value) {
  AssertIntegrity();
  absl::MutexLock lock(&mu_);
  ReturnInternal(std::move(value));
}

void FutureImpl::Return(absl::Status status) {
  AssertIntegrity();
  absl::MutexLock lock(&mu_);
  ReturnInternal(status);
}

bool FutureImpl::ReturnInternal(ResultHolder result) {
  AssertIntegrity();
  decltype(relationships_) relationships;
  decltype(nested_future_) nested_future;

  // Grab the callbacks and set the value under the lock.
  if (result_.HasResult()) {
    return false;
  }

  result_ = std::move(result);
  relationships_.swap(relationships);
  nested_future_.swap(nested_future);
  result_producer_ = ResultProducer();
  producer_executor_ = nullptr;

  // Store the result in a pointer to resolve usages later.
  // TODO: This is kind of scary, because this pointer is
  // essentially used to access the ResultHolder stored by FutureImpl from
  // outside of the lock. In practice, this has never caused a real problem
  // because the way Future's are actually used we are never accessing the
  // result of a future from two threads at the same time. The actual semantics
  // of the API discourage it.
  ResultHolder* result_ptr = &result_;

  mu_.Unlock();

  // Loop through and resolve all the relationships.
  // Holdable is skipped, those are simply held until the future is ready so
  // they are resolved by relationships.clear() below.
  for (auto& relationship : relationships) {
    // This is done with a simple switch statement instead of std::visit to
    // avoid adding complexity to the stack trace, making it easier to debug
    // future call stacks.
    switch (relationship.index()) {
      case 0:
        InvokeResultProducerFromParent(
            std::get<std::weak_ptr<FutureImpl>>(relationship));
        break;
      case 1:
        ResolveCombineChild(result_ptr->GetStatus(),
                            std::get<CombineChild>(relationship));
        break;
      case 2:
        ResolveNestedChild(*result_ptr, std::get<NestedChild>(relationship));
        break;
      case 3:
        ResolveKeptForgetter(result_ptr->GetStatus(),
                             std::get<KeptForgetter>(relationship));
        break;
      case 4:
        std::get<Invocable<void(ResultHolder&)>>(relationship)(*result_ptr);
        break;
    }
  }
  relationships.clear();

  nested_future.reset();

  mu_.Lock();

  return true;
}

bool FutureImpl::ReturnInternal(absl::Status status) {
  return ReturnInternal(status_to_result_for_self_fn_(status));
}

bool FutureImpl::HasResultOrIsExecutingResultProducer() {
  return result_.HasResult() || !result_producer_;
}

void FutureImpl::ResolveCombineChild(const absl::Status& result_status,
                                     CombineChild& combine_child) {
  std::shared_ptr<FutureImpl> child = combine_child.child.lock();
  if (!child) {
    return;
  }

  absl::MutexLock lock(&child->mu_);

  if (child->result_.HasResult()) {
    return;
  }

  if (!result_status.ok()) {
    if (combine_child.combined_status != nullptr) {
      combine_child.combined_status->Update(result_status);
    } else {
      child->ReturnInternal(result_status);
      return;
    }
  }

  if (combine_child.remaining_futures_counter->DecrementCount()) {
    child->ReturnInternal(combine_child.combined_status
                              ? *combine_child.combined_status
                              : absl::OkStatus());
  }
}

void FutureImpl::ResolveNestedChild(ResultHolder& result_holder,
                                    NestedChild& nested_child) {
  if (std::shared_ptr<FutureImpl> child = nested_child.child.lock()) {
    child->Return(nested_child.retrieve_result_fn(result_holder));
  }
}

void FutureImpl::ResolveKeptForgetter(const absl::Status& result_status,
                                      KeptForgetter& kept_forgetter) {
  kept_forgetter.forget_fn();

  if (kept_forgetter.kept_by_mode == FutureKeptByMode::kDoNothingOnError) {
    return;
  }

  if (result_status.ok()) {
    return;
  }

  if (kept_forgetter.kept_by_mode == FutureKeptByMode::kLogOnError) {
    IMP_LOG(imp::WARNING) << "Future failed with error " << result_status;
  } else if (kept_forgetter.kept_by_mode == FutureKeptByMode::kDieOnError) {
    IMP_LOG(imp::FATAL) << "Future failed with error " << result_status;
  }
}

void FutureImpl::InvokeResultProducer(std::shared_ptr<FutureImpl>& impl,
                                      absl::Status result_status) {
  Executor* producer_executor = nullptr;
  FutureExecutorMode future_executor_mode;
  int task_priority;
  TaskId reserved_task_id;
  std::shared_ptr<FutureImplWrapper> parent_future;
  std::shared_ptr<FutureImplWrapper> nested_future_to_reset;
  std::vector<std::shared_ptr<FutureImplWrapper>> to_bubble;

  bool should_return_early = false;
  bool should_schedule = false;
  {
    absl::MutexLock lock(&impl->mu_);

    to_bubble = impl->GetPriorityBubbleUpTargets();

    if (impl->HasResultOrIsExecutingResultProducer() ||
        impl->pending_result_status_.has_value()) {
      impl->pending_result_status_ = result_status;
      should_return_early = true;

      // If we have a nested future, it means the result producer has already
      // run and is now waiting on this future. That means we can let the
      // nested future go instead of blocking on it. We don't release it and
      // return immediately so we can release it from outside the lock.
      std::swap(nested_future_to_reset, impl->nested_future_);
    } else {
      impl->pending_result_status_ = result_status;
      producer_executor = impl->producer_executor_;
      future_executor_mode = impl->future_executor_mode_;
      task_priority = impl->task_priority_;
      std::swap(parent_future, impl->parent_future_);
      if (producer_executor &&
          (producer_executor != Executor::CurrentExecutor() ||
           future_executor_mode == FutureExecutorMode::kScheduleAlways)) {
        should_schedule = true;
        reserved_task_id = producer_executor->ReserveTaskId();
        impl->extant_task_id_ = reserved_task_id;
      }
    }
  }

  for (const std::shared_ptr<FutureImplWrapper>& future : to_bubble) {
    future->GetImpl()->BubbleUpPriority();
  }
  to_bubble.clear();

  if (should_return_early) {
    nested_future_to_reset.reset();
    return;
  }

  parent_future.reset();

  // Capturing impl allows us to guaranteed that the impl is not destroyed
  // until after the callback is complete, guaranteeing that it becomes ready
  // before it is destroyed.
  imp::Invocable<void()> fn = [impl]() mutable {
    FutureImpl::ResultProducer result_producer;
    ResultHolder result;
    {
      absl::MutexLock lock(&impl->mu_);
      if (impl->HasResultOrIsExecutingResultProducer()) {
        return;
      }

      std::swap(result_producer, impl->result_producer_);
      result = impl->status_to_result_for_result_producer_fn_(
          std::move(impl->pending_result_status_.value()));
    }

    result_producer(impl, result);
  };

  if (should_schedule) {
    if (!producer_executor->ScheduleWithReservedTaskId(
            reserved_task_id, std::move(fn), task_priority)) {
      impl->Return(
          absl::CancelledError("Cannot schedule result producer. Cancelling "
                               "future immediately."));
    }
  } else {
    fn();
  }
}

void FutureImpl::InvokeResultProducerFromParent(
    std::weak_ptr<FutureImpl>& weak_impl) {
  std::shared_ptr<FutureImpl> impl = weak_impl.lock();
  if (!impl) {
    return;
  }

  Executor* producer_executor = nullptr;
  FutureExecutorMode future_executor_mode;
  int task_priority;
  TaskId reserved_task_id;
  bool should_schedule = false;
  {
    absl::MutexLock lock(&impl->mu_);
    if (impl->HasResultOrIsExecutingResultProducer()) {
      return;
    }

    producer_executor = impl->producer_executor_;
    future_executor_mode = impl->future_executor_mode_;
    task_priority = impl->task_priority_;
    if (producer_executor &&
        (producer_executor != Executor::CurrentExecutor() ||
         future_executor_mode == FutureExecutorMode::kScheduleAlways)) {
      should_schedule = true;
      reserved_task_id = producer_executor->ReserveTaskId();
      impl->extant_task_id_ = reserved_task_id;
    }
  }

  // Capturing impl allows us to guaranteed that the impl is not destroyed
  // until after the callback is complete, guaranteeing that it becomes ready
  // before it is destroyed.
  imp::Invocable<void()> fn = [impl]() mutable {
    FutureImpl::ResultProducer result_producer;
    std::shared_ptr<FutureImplWrapper> parent_future;
    {
      absl::MutexLock lock(&impl->mu_);
      if (impl->HasResultOrIsExecutingResultProducer() ||
          !impl->parent_future_) {
        return;
      }

      std::swap(result_producer, impl->result_producer_);
      std::swap(parent_future, impl->parent_future_);
    }

    result_producer(impl, parent_future->GetImpl()->Get());
  };

  // Make sure that the shared_ptrs are reset prior to invoking the
  // result_producer. This is to prevent a race condition where the impl can
  // outlive where it's being used when the parent future is on a different
  // executor than the impl.
  impl.reset();

  if (should_schedule) {
    if (!producer_executor->ScheduleWithReservedTaskId(
            reserved_task_id, std::move(fn), task_priority)) {
      std::shared_ptr<FutureImpl> impl = weak_impl.lock();
      if (impl) {
        impl->Return(
            absl::CancelledError("Cannot schedule result producer. Cancelling "
                                 "future immediately."));
      }
    }
  } else {
    fn();
  }
}

std::vector<std::shared_ptr<FutureImpl::FutureImplWrapper>>
FutureImpl::GetPriorityBubbleUpTargets() {
  std::vector<std::shared_ptr<FutureImplWrapper>> result;
  // There are multiple types of "parents" that a future's priority can bubble
  // up to. This method aggregates them all into a single list.

  // This is the direct parent of the future, when calling Future::Then.
  if (parent_future_) {
    result.push_back(parent_future_);
  }

  // These are the futures passed into one of the combine / merge methods, they
  // are treated as the parent of the resulting combined future for priority
  // bubbling purposes.
  if (!combine_parents_.empty()) {
    result.insert(result.end(), combine_parents_.begin(),
                  combine_parents_.end());
  }

  // This is the inner future returned by a lambda passed into
  // Schedule/Then, which is considered the parent of the outer future. That
  // way, if you update the priority of the outer future it bubbles to the inner
  // future.
  if (nested_future_) {
    result.push_back(nested_future_);
  }

  return result;
}

void FutureImpl::BubbleUpPriority(std::optional<int> changed_priority) {
  AssertIntegrity();
  std::vector<std::shared_ptr<FutureImplWrapper>> to_bubble;
  int changed_priority_to_bubble = 0;
  {
    absl::MutexLock lock(&mu_);

    // Only continue bubbling if the active priority changed.
    if (RefreshActivePriority(changed_priority)) {
      to_bubble = GetPriorityBubbleUpTargets();
      changed_priority_to_bubble = task_priority_;
    }
  }

  for (const std::shared_ptr<FutureImplWrapper>& future : to_bubble) {
    future->GetImpl()->BubbleUpPriority(changed_priority_to_bubble);
  }
}

bool FutureImpl::RefreshActivePriority(std::optional<int> changed_priority) {
  std::optional<int> max_child_priority = std::nullopt;

  if (changed_priority && *changed_priority >= task_priority_) {
    // changed_priority indicates that the priority is being refreshed because
    // of the priority of a child future changing to the value passed in.
    // If that is the same or higher than the current priority, then we can use
    // that value instead of fully recalculating the priority from the children.
    max_child_priority = *changed_priority;
  } else {
    for (const auto& relationship : relationships_) {
      switch (relationship.index()) {
        case 0: {
          // Direct children of this future, from Future::Then.
          std::weak_ptr<FutureImpl> weak_child =
              std::get<std::weak_ptr<FutureImpl>>(relationship);
          if (std::shared_ptr<FutureImpl> child = weak_child.lock()) {
            absl::MutexLock lock(&child->mu_);
            if (child->HasResultOrIsExecutingResultProducer() ||
                child->pending_result_status_.has_value()) {
              // If this child is in the process of returning a result, i.e.
              // it's been cancelled, then don't count it.
              continue;
            }
            if (!max_child_priority.has_value()) {
              max_child_priority = child->task_priority_;
            } else {
              max_child_priority =
                  std::max(*max_child_priority, child->task_priority_);
            }
          }
          break;
        }
        case 1: {
          /* A child future from a Combine. */
          const CombineChild& combine_child =
              std::get<CombineChild>(relationship);
          if (std::shared_ptr<FutureImpl> child = combine_child.child.lock()) {
            absl::MutexLock lock(&child->mu_);
            if (child->HasResultOrIsExecutingResultProducer() ||
                child->pending_result_status_.has_value()) {
              // If this child is in the process of returning a result, i.e.
              // it's been cancelled, then don't count it.
              continue;
            }
            if (!max_child_priority.has_value()) {
              max_child_priority = child->task_priority_;
            } else {
              max_child_priority =
                  std::max(*max_child_priority, child->task_priority_);
            }
          }
          break;
        }
        case 2: {
          /* A nested child future. */
          const NestedChild& nested_child = std::get<NestedChild>(relationship);
          if (std::shared_ptr<FutureImpl> child = nested_child.child.lock()) {
            absl::MutexLock lock(&child->mu_);
            // Nested futures are handled a bit differently because the result
            // producer has already run to assign the nested future. If the
            // nested future has been cleared (i.e. it's been cancelled), that
            // means the priority should no longer be counted.
            if (!child->nested_future_) {
              continue;
            }
            if (!max_child_priority.has_value()) {
              max_child_priority = child->task_priority_;
            } else {
              max_child_priority =
                  std::max(*max_child_priority, child->task_priority_);
            }
          }
          break;
        }
        default:
          break;
      }
    }
  }

  // If the future has no specified/self priority and no children, then
  // the active priority defaults to kNormalTaskPriority.
  //
  // Otherwise, the active priority is the maximum value across (1) the active
  // priorities of its direct children, if any, and (2) its own self priority,
  // if specified.
  int new_task_priority = kNormalTaskPriority;
  if (self_priority_.has_value() && max_child_priority.has_value()) {
    new_task_priority = std::max(*self_priority_, *max_child_priority);
  } else if (self_priority_.has_value()) {
    new_task_priority = *self_priority_;
  } else if (max_child_priority.has_value()) {
    new_task_priority = *max_child_priority;
  }

  if (new_task_priority == task_priority_) {
    return false;
  }

  task_priority_ = new_task_priority;

  if (extant_task_id_ == kInvalidTaskId || producer_executor_ == nullptr) {
    return true;
  }

  absl::Status status =
      producer_executor_->UpdateTaskPriority(extant_task_id_, task_priority_);
  // Not found is expected if the task has already completed.
  if (!status.ok() && status.code() != absl::StatusCode::kNotFound) {
    IMP_LOG(imp::FATAL) << "Error updating Executor task priority: " << status;
  }

  return true;
}

void FutureImpl::UpdatePriority(std::optional<int> priority) {
  AssertIntegrity();
  std::vector<std::shared_ptr<FutureImplWrapper>> to_bubble;
  int changed_priority_to_bubble;
  {
    absl::MutexLock lock(&mu_);

    if (self_priority_ == priority) {
      return;
    }

    // If the future is already ready, don't update the priority.
    if (result_.HasResult()) {
      return;
    }

    self_priority_ = priority;

    // Only continue bubbling if the active priority changed.
    if (RefreshActivePriority(self_priority_)) {
      to_bubble = GetPriorityBubbleUpTargets();
      changed_priority_to_bubble = task_priority_;
    }
  }

  for (const std::shared_ptr<FutureImplWrapper>& future : to_bubble) {
    future->GetImpl()->BubbleUpPriority(changed_priority_to_bubble);
  }
}

int FutureImpl::GetActivePriority() {
  AssertIntegrity();
  absl::MutexLock lock(&mu_);
  return task_priority_;
}

std::optional<int> FutureImpl::GetSelfPriority() {
  AssertIntegrity();
  absl::MutexLock lock(&mu_);
  return self_priority_;
}

int FutureImpl::GetDepth() const {
  AssertIntegrity();
  int depth = 0;
  const imp::internal::FutureImpl* curr = this;
  do {
    depth++;
    absl::ReaderMutexLock lock(&(curr->mu_));
    if (!curr->parent_future_) {
      break;
    }
    curr = curr->parent_future_->GetImpl().get();
  } while (curr);
  return depth;
}

FutureImpl::FutureImplWrapper::FutureImplWrapper(
    const std::shared_ptr<FutureImpl>& impl)
    : impl_(impl) {}

FutureImpl::FutureImplWrapper::~FutureImplWrapper() {
  if (!impl_->Ready()) {
    internal::FutureImpl::InvokeResultProducer(
        impl_, absl::CancelledError("from ~FutureImplWrapper()"));
  }
}

std::shared_ptr<FutureImpl>& FutureImpl::FutureImplWrapper::GetImpl() {
  return impl_;
}

template <>
void ReturnResultToFuture(std::shared_ptr<FutureImpl>& then_impl,
                          absl::Status result_status) {
  then_impl->Return(std::move(result_status));
}

Executor* GetExecutor(const ExecutorTypeOrExecutor& executor_type_or_executor) {
  // If the variant stores an Executor::Type, get the executor pointer
  // from the type and return it.
  if (const Executor::Type* executor_type =
          absl::get_if<Executor::Type>(&executor_type_or_executor)) {
    return Executor::Get(*executor_type);
  }

  // Otherwise, the variant stores an Executor*, return that directly.
  return absl::get<Executor*>(executor_type_or_executor);
}

void AddFutureToCombineResult(
    const std::shared_ptr<FutureImpl::FutureImplWrapper>& combine_result,
    const std::shared_ptr<FutureImpl::FutureImplWrapper>& future_to_combine,
    absl::BlockingCounter* remaining_futures_counter,
    absl::Status* combined_status) {
  future_to_combine->GetImpl()->AddCombineChild(
      combine_result->GetImpl(), remaining_futures_counter, combined_status);
  combine_result->GetImpl()->AddCombineParent(future_to_combine);
}

void FutureImpl::AssertIntegrity() const {
#if !IMP_DISABLE_FUTURE_VALIDATION
  if (integrity_marker_ == kDestructedIntegrityMarker) {
    IMP_LOG(imp::FATAL) << "FutureImpl is marked as destructed.";
  }
  if (integrity_marker_ != kValidIntegrityMarker) {
    IMP_LOG(imp::FATAL) << "FutureImpl has corrupted memory.";
  }
#endif
}

void FutureImpl::AssertResultHasNotBeenMoved() const {
#if !IMP_DISABLE_FUTURE_VALIDATION
  if (result_.HasBeenMoved()) {
    IMP_LOG(imp::FATAL)
        << "Invalid operation on FutureImpl: Result has already been moved.";
  }
#endif
}

}  // namespace internal
}  // namespace imp
