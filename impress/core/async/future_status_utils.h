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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_STATUS_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_STATUS_UTILS_H_

#include "absl/status/status.h"
#include "core/async/future.h"

namespace imp {

// Wraps an absl::Status in a Future<absl::Status>
//
// Examples:
//  one line status to future conversion:
//  imp::Future<absl::Status> future = ReturnFuture(status);
//
//  Changing return type (see (broken link)):
//  imp::Future<absl::Status> MyComponent::Setup() {
//    MP_RETURN_IF_ERROR(status).With(ReturnFuture);
//  }
Future<absl::Status> ReturnFuture(const absl::Status& status);

// Wraps an absl::Status in a Future<T>. This is meant for returning statuses in
// functions that return Futures of a different type.
//
// Examples:
//  Future<std::vector> Foo() {
//    MP_ASSIGN_OR_RETURN(std::vector some_vec, BarReturnsStatusOr(),
//    _.With(ReturnTypedFuture<std::vector>));
//    ...
//  }
//
//  For more information: (broken link)
template <typename T>
Future<T> ReturnTypedFuture(const absl::Status& status) {
  return Future<T>(status);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_STATUS_UTILS_H_
