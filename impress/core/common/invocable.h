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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_INVOCABLE_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_INVOCABLE_H_

#include <cassert>
#include <type_traits>

#include "core/common/type_erased.h"

namespace imp {

// Invocable is a move-only general purpose function wrapper. Instances can
// store and invoke lambda expressions and other function objects.
//
// Example Usage:
//
// // An Invocable that takes a float and int as args and returns a string.
// Invocable<std::string(float, int)> my_invocable =
//   [](float arg1, int arg2) -> std::string {
//     return absl::StrFormat("Float %f, Int %i", arg1, arg2);
//   };
//
// Invokes the invocable.
// std::string result = my_invocable(0.5f, 10);
//
// It is similar to std::function, with the following intentional differences:
// * Invocable is move only.
// * Invocable can capture move only types.
// * No conversion between 'compatible' functions.
template <typename Signature>
class Invocable;
template <typename R, typename... Args>
class Invocable<R(Args...)> {
 public:
  // Helper for enabling methods if Fn matches the function signature
  // requirements of the Invocable type.
  template <typename Fn>
  using EnableIfFnMatchesInvocable = std::enable_if_t<
      std::is_invocable_v<Fn, Args...> &&
          std::is_same_v<R, std::invoke_result_t<Fn, Args...>> &&
          !std::is_same_v<std::decay_t<Fn>, Invocable<R(Args...)>>,
      int>;

  // Creates an Invocable that does not contain a functor.
  // Will evaluate to false.
  Invocable() noexcept;

  // Creates an Invocable from the functor passed in.
  template <typename Fn, EnableIfFnMatchesInvocable<Fn> = 0>
  Invocable(Fn&& fn) noexcept;

  Invocable(const Invocable&) = delete;
  Invocable(Invocable&&) noexcept = default;

  Invocable& operator=(const Invocable&) = delete;
  Invocable& operator=(Invocable&&) noexcept = default;

  // Invokes the invocable with the args passed in.
  // If the Invocable is empty, this will assert.
  // OperatorArgs is used to implement perfect forwarding.
  template <typename... OperatorArgs>
  R operator()(OperatorArgs&&... args);
  template <typename... OperatorArgs>
  R operator()(OperatorArgs&&... args) const;

  // Evaluates to true if Invocable contains a functor, false otherwise.
  explicit operator bool() const noexcept;

 private:
  using InvokerFn = R (*)(TypeErased&, Args...);

  // Helper that casts the type erased invocable back into the real type and
  // invokes it.
  template <typename Fn>
  inline static R Invoker(TypeErased& erased_invocable, Args... args);

  // TODO: Fix const-correctness of this class.
  // invocable_ is mutable because the API contract of this class requires that
  // const functors can be invoked with non-const Invocable objects. This should
  // be fixed, which requires fixing a number of Invocable usages spread
  // throughout the code.
  mutable TypeErased invocable_;
  InvokerFn invoker_;
};

template <typename R, typename... Args>
template <typename Fn>
R Invocable<R(Args...)>::Invoker(TypeErased& erased_invocable, Args... args) {
  Fn& typed_invocable = erased_invocable.Get<Fn>();
  if constexpr (std::is_void_v<R>) {
    typed_invocable(std::forward<Args>(args)...);
  } else {
    return typed_invocable(std::forward<Args>(args)...);
  }
}

template <typename R, typename... Args>
Invocable<R(Args...)>::Invocable() noexcept : invoker_(nullptr) {}

template <typename R, typename... Args>
template <typename Fn, typename Invocable<
                           R(Args...)>::template EnableIfFnMatchesInvocable<Fn>>
Invocable<R(Args...)>::Invocable(Fn&& fn) noexcept
    : invocable_(std::forward<Fn>(fn)), invoker_(Invoker<std::decay_t<Fn>>) {}

template <typename R, typename... Args>
template <typename... OperatorArgs>
R Invocable<R(Args...)>::operator()(OperatorArgs&&... args) {
  assert(invoker_ && invocable_);
  return invoker_(invocable_, std::forward<OperatorArgs>(args)...);
}

template <typename R, typename... Args>
template <typename... OperatorArgs>
R Invocable<R(Args...)>::operator()(OperatorArgs&&... args) const {
  assert(invoker_ && invocable_);
  return invoker_(invocable_, std::forward<OperatorArgs>(args)...);
}

template <typename R, typename... Args>
Invocable<R(Args...)>::operator bool() const noexcept {
  return invoker_ != nullptr && invocable_;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_INVOCABLE_H_
