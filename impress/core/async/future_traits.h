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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_TRAITS_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_TRAITS_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace imp {

template <typename T>
class Future;

namespace internal {

// Compile-time helpers for determining type information about Futures.
namespace future_traits {

// Given any type T, Representation provides the types for the Result, Value,
// and Future for that type.
//
// This includes specializations for absl::StatusOr, absl::Status, and Future,
// so the types are always correct regardless of what T  is.
// Example:
// Representation<Future<int>>::AsFutureT == Future<int>,
//                                         not Future<Future<int>>
// Representation<StatusOr<int>>::ResultT == StatusOr<int>,
//                                         not StatusOr<StatusOr<int>>
template <typename T>
struct Representation {
  using ResultT = absl::StatusOr<T>;
  using ValueT = T;
  using AsFutureT = Future<T>;
};

// Specialization for absl::Status.
template <>
struct Representation<absl::Status> {
  using ResultT = absl::Status;
  using ValueT = absl::Status;
  using AsFutureT = Future<absl::Status>;
};

// Specialization for absl::StatusOr.
template <typename T>
struct Representation<absl::StatusOr<T>> {
  using ResultT = absl::StatusOr<T>;
  using ValueT = T;
  using AsFutureT = Future<T>;
};

// Specialization for Future.
template <typename T>
struct Representation<Future<T>> {
  using ResultT = typename Representation<T>::ResultT;
  using ValueT = typename Representation<T>::ValueT;
  using AsFutureT = Future<T>;
};

// Given any type T, gives the correct Result type for a future, which will
// always be a StatusOr or a Status.
template <typename T>
using ResultT = typename Representation<T>::ResultT;

// Given any type T, gives the correct Result type for a future, which will
// always be the value of a StatusOr or a Status.
template <typename T>
using ValueT = typename Representation<T>::ValueT;

// Converts type T into a type of future in the following ways:
// absl::Status -> Future<absl::Status>
// absl::StatusOr<T> -> Future<T> (Drops the StatusOr since it is implied).
// Future<T> -> Future<T> (Passing in a future type returns itself).
// T -> Future<T> (Any other type returns a Future to that type).
template <typename T>
using FnResultToFutureT = typename Representation<
    std::conditional_t<std::is_void_v<T>, absl::Status, T>>::AsFutureT;

// Integral_constant that is true if T is a Status or *not* a StatusOr<T>.
// Used to enforce at compile time that Future<StatusOr<T>> cannot be declared,
// instead, Future<T> should be declared.
template <typename T>
struct IsValidFutureType : std::true_type {};

// Specialization for StatusOr's is false, since that isn't a valid future
// declaration.
template <typename T>
struct IsValidFutureType<absl::StatusOr<T>> : std::false_type {};

// Helper for converting the integral_constant IsValidFutureType into a boolean.
template <typename T>
inline constexpr bool IsValidFutureTypeV = IsValidFutureType<T>::value;

// Integral_constant that is true if T is a Future.
template <typename T>
struct IsFuture : std::false_type {};

template <typename T>
struct IsFuture<Future<T>> : std::true_type {};

// Determines if T is a Future.
// i.e. IsFutureV<Future<int>> is true.
//      IsFutureV<int> is false.
template <typename T>
inline constexpr bool IsFutureV = IsFuture<T>::value;

// Integral_constant to determine if T is a valid future result.
// T is valid if it's a Status or StatusOr.
template <typename T>
struct IsValidResult : std::false_type {};

template <typename T>
struct IsValidResult<absl::StatusOr<T>> : std::true_type {};

template <>
struct IsValidResult<absl::Status> : std::true_type {};

template <typename T>
inline constexpr bool IsValidResultV = IsValidResult<T>::value;

// Determines if a function signature Fn is callable without any value.
template <typename Fn>
inline constexpr bool InvokingRequiresNoParamV = std::is_invocable_v<Fn>;

// Determines if a function signature Fn requires a Result (StatusOr<T>).
// If false, then the function signature should take a Value.
template <typename Fn, typename T>
inline constexpr bool InvokingRequiresResultV =
    std::is_invocable_v<Fn, ResultT<T>>;

// Determines if a function signature Fn requires the result to be moved.
// The result must be moved if the result is move only *and* the function
// signature does not take the result as a reference.
template <typename Fn, typename T>
inline constexpr bool InvokingRequiresMoveV = !std::is_invocable_v<
    Fn, const std::conditional_t<InvokingRequiresResultV<Fn, T>, ResultT<T>,
                                 ValueT<T>>&>;

// Determines if a function signature Fn has a void result.
template <typename Fn>
inline constexpr bool IsInvokeNoParamResultVoidV =
    std::is_void_v<std::invoke_result_t<Fn>>;

// Determines if a function signature Fn has a void result.
template <typename Fn, typename T>
inline constexpr bool IsInvokeResultVoidV =
    std::is_void_v<std::invoke_result_t<Fn, ValueT<T>>>;

// Determines if a function signature Fn is a valid signature to be passed into
// Future::Then for a future of type T.
template <typename Fn, typename T>
inline constexpr bool IsValidThenFnForFutureV =
    std::is_invocable_v<Fn, ResultT<T>> || std::is_invocable_v<Fn, ValueT<T>>;

// A .Then() with no parameters can only be used on a Future<absl::Status>.
template <typename Fn, typename T>
inline constexpr bool IsValidThenFnForFutureNoParamsV =
    std::is_invocable_v<Fn>&& std::is_same_v<T, absl::Status>;

// A struct to contain the FnInvokeResultT using statement, which must branch on
// whether it requires the T parameter (in the case of a .Then function that
// either takes a T or StatusOr<T>) or whether it is a function with no params.
// This version of the struct invokes the function with no parameters.
template <typename Fn, typename T, bool = InvokingRequiresNoParamV<Fn>>
struct FnInvokeResult {
  using FnInvokeResultT = std::invoke_result_t<Fn>;
};

// This version of the struct does invoke the .Then() with the T value.
template <typename Fn, typename T>
struct FnInvokeResult<Fn, T, false> {
  using FnInvokeResultT = std::invoke_result_t<Fn, T>;
};

//////////////////////////
//  EnableIf helpers. Used to enable/disable function templates based on
//  future_traits.
//////////////////////////

template <typename Fn>
using EnableIfInvokingRequiresNoParam =
    std::enable_if_t<InvokingRequiresNoParamV<Fn>, bool>;

template <typename Fn>
using DisableIfInvokingRequiresNoParam =
    std::enable_if_t<!InvokingRequiresNoParamV<Fn>, bool>;

template <typename Fn, typename T>
using EnableIfInvokingRequiresResult =
    std::enable_if_t<InvokingRequiresResultV<Fn, T>, bool>;

template <typename Fn, typename T>
using EnableIfInvokingRequiresValue =
    std::enable_if_t<!InvokingRequiresResultV<Fn, T>, bool>;

template <typename Fn>
using EnableIfInvokeNoParamResultVoid =
    std::enable_if_t<IsInvokeNoParamResultVoidV<Fn>, bool>;

template <typename Fn, typename T>
using EnableIfInvokeResultVoid =
    std::enable_if_t<IsInvokeResultVoidV<Fn, T>, bool>;

template <typename Fn>
using EnableIfInvokeNoParamResultNotVoid =
    std::enable_if_t<!IsInvokeNoParamResultVoidV<Fn>, bool>;

template <typename Fn, typename T>
using EnableIfInvokeResultNotVoid =
    std::enable_if_t<!IsInvokeResultVoidV<Fn, T>, bool>;

template <typename Fn, typename T>
using EnableIfInvokingRequiresMove =
    std::enable_if_t<InvokingRequiresMoveV<Fn, T>, bool>;

template <typename Fn, typename T>
using EnableIfInvokingDoesNotRequireMove =
    std::enable_if_t<!InvokingRequiresMoveV<Fn, T>, bool>;

}  // namespace future_traits
}  // namespace internal
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_TRAITS_H_
