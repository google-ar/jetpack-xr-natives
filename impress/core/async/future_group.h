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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_GROUP_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_GROUP_H_

#include <sys/stat.h>

#include <memory>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "core/async/task_priority.h"
#include "core/common/robin_map.h"

namespace imp {

namespace internal {
class FutureImpl;
}

// Holds weak references to a group of Futures, allowing them to be addressed
// together. A FutureGroup can be passed into FutureScheduleOptions or
// FutureThenOptions at the time of a Future::Schedule call or a Future::Then
// call, respectively.
//
// Example of updating the priority of a group of Futures:
// ```
//  FutureGroup future_group = FutureGroup(kNormalTaskPriority);
//
//  Future<absl::Status> future_1 = Future<absl::Status>::Schedule(
//      []() {
//        IMP_LOG(imp::INFO) << "Impress 1";
//        return absl::OkStatus();
//      },
//      {.future_group = future_group});
//  Future<absl::Status> future_2 = Future<absl::Status>::Schedule(
//      []() {
//        IMP_LOG(imp::INFO) << "Impress 2";
//        return absl::OkStatus();
//      },
//      {.future_group = future_group});
//
//  future_group.UpdatePriority(kHighTaskPriority);
// ```
//
// FutureGroups can also be used to update the priority of a chain of Futures:
// ```
//  FutureGroup future_group = FutureGroup(kNormalTaskPriority);
//  Future<absl::Status> future_1 = Future<absl::Status>::Schedule(
//                                      []() {
//                                        IMP_LOG(imp::INFO) << "Impress 1";
//                                        return absl::OkStatus();
//                                      },
//                                      {.future_group = .future_group})
//                                      .Then(
//                                          []() {
//                                            IMP_LOG(imp::INFO) << "Impress 2";
//                                            return absl::OkStatus();
//                                          },
//                                          {.future_group = future_group})
//                                      .Then(
//                                          []() {
//                                            IMP_LOG(imp::INFO) << "Impress 3";
//                                            return absl::OkStatus();
//                                          },
//                                          {.future_group = future_group})
//  future_group.UpdatePriority(kHighTaskPriority);
// ```
//
// FutureGroup uses a shared ownership model: a FutureGroup will be destroyed
// after its last reference is destroyed.
//
// This class is thread-safe ((broken link)).
class FutureGroup {
 public:
  FutureGroup(int task_priority = kNormalTaskPriority)
      : impl_(std::make_shared<FutureGroupImpl>(task_priority)) {}

  // Adds a Future to the group, without increasing the reference count.
  void AddFuture(std::shared_ptr<internal::FutureImpl> future);

  // Updates the priority of every Future in this group to the given numeric
  // task priority. Valid range is [kMinimumTaskPriority,kMaximumTaskPriority].
  void UpdatePriority(int task_priority);

  // Returns the last value passed into UpdatePriority().
  int GetTaskPriority() const;

 private:
  struct FutureGroupImpl {
    FutureGroupImpl(int task_priority) : cached_priority(task_priority) {}
    absl::Mutex mutex;
    int cached_priority ABSL_GUARDED_BY(mutex) = kNormalTaskPriority;
    // Store weak pointers using an integer ID for quick deletion.
    imp::RobinMap<int, std::weak_ptr<internal::FutureImpl>>
        future_group_elements_ ABSL_GUARDED_BY(mutex);
    int next_future_group_element_id = 0;
  };
  std::shared_ptr<FutureGroupImpl> impl_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_GROUP_H_
