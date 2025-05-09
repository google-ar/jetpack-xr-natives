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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_CANCELLATION_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_CANCELLATION_HELPER_H_

#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "core/async/future.h"

namespace imp {

// Utility class used to make it possible to interrupt work being done by a
// future in the middle of a callback when the future is cancelled.
//
// Example Usage:
//
//   FutureInterrupter interrupter;
//   Future<absl::Status> interruptible_future =
//      interrupter.MakeInterruptible(Future<absl::Status>::Schedule(
//          [interrupter]() -> absl::Status {
//            // Do some work.
//
//            if (interrupter.IsInterrupted()) {
//              return absl::CancelledError("Interrupted.");
//            }
//
//            // Do some more work.
//
//            return absl::OkStatus();
//          }, Executor::Type::kBackground));
// TODO: In the future we could make Interrupt() automatically
// cancel the future, instead of just being a mechanism for early-out when
// the future is cancelled/running independently.
class FutureInterrupter {
 public:
  FutureInterrupter();

  // Interrupts the future. Called automatically when the future is cancelled if
  // MakeInterruptible is called.
  void Interrupt();

  // Checks to see if the future should be interrupted. Should be called within
  // the callback of a future to check if the callback should return early.
  bool IsInterrupted() const;

  // Creates a future that will interrupt the input future when it is cancelled.
  template <typename T>
  Future<T> MakeInterruptible(Future<T> future) {
    using FutureResult = typename Future<T>::Result;
    return future.Then(
        [interrupt_notification =
             interrupt_notification_](const FutureResult& result) {
          if constexpr (std::is_same_v<FutureResult, absl::Status>) {
            if (result.code() == absl::StatusCode::kCancelled) {
              interrupt_notification->Notify();
            }
          } else {
            if (result.status().code() == absl::StatusCode::kCancelled) {
              interrupt_notification->Notify();
            }
          }

          return result;
        },
        Executor::Type::kImmediate);
  }

 private:
  std::shared_ptr<absl::Notification> interrupt_notification_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_CANCELLATION_HELPER_H_
