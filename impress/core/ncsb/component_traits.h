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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_TRAITS_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_TRAITS_H_

#include <type_traits>
#include <utility>

#include "core/view/async/future.h"
#include "core/view/utils/frame_time.h"

namespace imp {

template <typename T>
class ComponentHandle;

namespace component_traits {

namespace internal {

// Uses SFINAE to detect if an overload of the Setup method exists.
// This version of the function will only be defined if the class |T| has a
// Setup function that takes in |Args| and returns a value of |Return|
// type.
template <
    typename T, typename Return, typename... Args,
    std::enable_if_t<std::is_same<Return, decltype(std::declval<T>().Setup(
                                              std::declval<Args>()...))>::value,
                     int> = 0>
static constexpr bool HasSetupFunc(int) {
  return true;
}

// Uses SFINAE to detect if an overload of the Setup method exists.
// If the above HasSetupFunc isn't defined for the template parameters,
// the compiler will default to calling this version of HasSetupFunc.
template <typename T, typename Return, typename... Args>
static constexpr bool HasSetupFunc(...) {
  return false;
}

// Uses SFINAE to detect if an overload of the SetupWithState method exists.
// This version of the function will only be defined if the class |T| has a
// SetupWithState function that takes in |Args| and returns a value of |Return|
// type.
template <typename T, typename Return, typename... Args,
          std::enable_if_t<
              std::is_same<Return, decltype(std::declval<T>().SetupWithState(
                                       std::declval<Args>()...))>::value,
              int> = 0>
static constexpr bool HasSetupWithStateFunc(int) {
  return true;
}

// Uses SFINAE to detect if an overload of the SetupWithState method exists.
// If the above HasSetupWithStateFunc isn't defined for the template parameters,
// the compiler will default to calling this version of HasSetupWithStateFunc.
template <typename T, typename Return, typename... Args>
static constexpr bool HasSetupWithStateFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has PreSave function.
template <typename T,
          std::enable_if_t<
              std::is_same<void, decltype(std::declval<T>().PreSave())>::value,
              int> = 0>
static constexpr bool HasPreSaveFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasPreSaveFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has Update function.
template <typename T,
          std::enable_if_t<
              std::is_same_v<void, decltype(std::declval<T>().Update(
                                       std::declval<const FrameTime&>()))>,
              int> = 0>
static constexpr bool HasUpdateFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasUpdateFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has IsfInfo type defined.
template <typename T,
          std::enable_if_t<!std::is_same_v<void, typename T::IsfInfo>, int> = 0>
static constexpr bool IsIsfInfoDefined(int) {
  return true;
}

template <typename T>
static constexpr bool IsIsfInfoDefined(...) {
  return false;
}

// Uses SFINAE to detect if T has System type defined.
template <typename T,
          std::enable_if_t<!std::is_same_v<void, typename T::System>, int> = 0>
static constexpr bool IsComponentSystemDefined(int) {
  return true;
}

template <typename T>
static constexpr bool IsComponentSystemDefined(...) {
  return false;
}

// Uses SFINAE to detect if T has CleanupDependencies type defined.
template <typename T,
          std::enable_if_t<
              !std::is_same_v<void, typename T::CleanupDependencies>, int> = 0>
static constexpr bool AreCleanupDependenciesDefined(int) {
  return true;
}

template <typename T>
static constexpr bool AreCleanupDependenciesDefined(...) {
  return false;
}

// Uses SFINAE to detect if T has CleanupDependents type defined.
template <typename T,
          std::enable_if_t<!std::is_same_v<void, typename T::CleanupDependents>,
                           int> = 0>
static constexpr bool AreCleanupDependentsDefined(int) {
  return true;
}

template <typename T>
static constexpr bool AreCleanupDependentsDefined(...) {
  return false;
}

}  // namespace internal

// Type returned by calling Setup method on component of type T with the
// specified args.
template <typename T, typename... Args>
using SetupMethodReturnType =
    decltype(std::declval<T>().Setup(std::declval<Args>()...));

// Compile-time check to see if a Setup method is asynchronous for a given
// type of component. The Setup overload is determined by the Args.
// If T has a Setup method overlad with the provided Args that returns
// Future<absl::Status> this check will be true, otherwise false.
template <typename T, typename... Args>
static constexpr bool kIsSetupAsync =
    std::is_same<SetupMethodReturnType<T, Args...>,
                 Future<absl::Status>>::value;

// Compile-time check to see if a Setup method returns absl::Status or not for a
// given type of component. The Setup overload is determined by the Args. If T
// has a Setup method overlad with the provided Args that returns absl::Status
// this check will be true, otherwise false.
template <typename T, typename... Args>
static constexpr bool kIsSetupReturnTypeStatus =
    std::is_same<SetupMethodReturnType<T, Args...>, absl::Status>::value;

// Detects is a type |Comp| has a Setup method with any of the possible
// valid return types for the |Args| passed in.
template <typename Comp, typename... Args>
static constexpr bool kHasSetupFunc =
    internal::HasSetupFunc<Comp, void, Args...>(0) ||
    internal::HasSetupFunc<Comp, absl::Status, Args...>(0) ||
    internal::HasSetupFunc<Comp, Future<absl::Status>, Args...>(0);

// Detects is a type |Comp| has a SetupWithState method with any of the possible
// valid return types for the |Args| passed in.
template <typename Comp, typename... Args>
static constexpr bool kHasSetupWithStateFunc =
    internal::HasSetupWithStateFunc<Comp, void, Args...>(0) ||
    internal::HasSetupWithStateFunc<Comp, absl::Status, Args...>(0) ||
    internal::HasSetupWithStateFunc<Comp, Future<absl::Status>, Args...>(0);

// Used to unpack the return type of setting up a component with State.
//
// If the component has a SetupWithState method, the SetupWithState method
// result is unpacked. Otherwise, it falls back to the regular Setup method.
template <typename T, typename... Args>
auto SetupWithStateMethodReturnTypeUnpacker() {
  if constexpr (kHasSetupWithStateFunc<T, Args...>) {
    return decltype(std::declval<T>().SetupWithState(
        std::declval<Args>()...))();
  } else {
    return SetupMethodReturnType<T, Args...>();
  }
}

// Detects the return type of setting up a component with State.
//
// If the component has a SetupWithState method, the SetupWithState method
// result is unpacked. Otherwise, it falls back to the regular Setup method.
template <typename T, typename... Args>
using SetupWithStateMethodReturnType =
    decltype(SetupWithStateMethodReturnTypeUnpacker<T, Args...>());

template <typename T, typename... Args>
static constexpr bool kIsSetupWithStateAsync =
    std::is_same<SetupWithStateMethodReturnType<T, Args...>,
                 Future<absl::Status>>::value;

template <typename T, typename... Args>
static constexpr bool kIsSetupWithStateReturnTypeStatus =
    std::is_same<SetupWithStateMethodReturnType<T, Args...>,
                 absl::Status>::value;

// Compile-time check to see if a component has a 'void PreSave()' function.
template <typename Comp>
static constexpr bool kHasPreSaveFunc = internal::HasPreSaveFunc<Comp>(0);

template <typename Comp>
static constexpr bool kHasUpdateFunc = internal::HasUpdateFunc<Comp>(0);

// The result returned when adding a component if the Setup method called
// is asynchronous.
template <typename T>
using AsyncResult = Future<ComponentHandle<T>>;

// The result returned when adding a component if the Setup method called
// returns an absl::Status.
template <typename T>
using StatusResult = absl::StatusOr<ComponentHandle<T>>;

// See Component::AddResult for documentation.
template <typename T, typename... Args>
using AddResult = typename std::conditional_t<
    kIsSetupAsync<T, Args...>, AsyncResult<T>,
    std::conditional_t<kIsSetupReturnTypeStatus<T, Args...>, StatusResult<T>,
                       ComponentHandle<T>>>;

// See Component::AddWithStateResult for documentation.
template <typename T, typename... Args>
using AddWithStateResult = typename std::conditional_t<
    kIsSetupWithStateAsync<T, Args...>, AsyncResult<T>,
    std::conditional_t<kIsSetupWithStateReturnTypeStatus<T, Args...>,
                       StatusResult<T>, ComponentHandle<T>>>;

// Compile-time check to see if a component has a nested type named IsfInfo
// defined.
template <typename T>
static constexpr bool kIsIsfInfoDefined = internal::IsIsfInfoDefined<T>(0);

// Compile-time check to see if T has a nested type named
// CleanupDependencies defined.
template <typename T>
static constexpr bool kAreCleanupDependenciesDefined =
    internal::AreCleanupDependenciesDefined<T>(0);

// Compile-time check to see if T has a nested type named
// CleanupDependents defined.
template <typename T>
static constexpr bool kAreCleanupDependentsDefined =
    internal::AreCleanupDependentsDefined<T>(0);

// Compile-time check to see if a component has a nested type named
// System defined.
template <typename T>
static constexpr bool kIsComponentSystemDefined =
    internal::IsComponentSystemDefined<T>(0);

// Compile-time check to see if a component should run when in EditMode in the
// Impress editor.
//
// If not, then component-lifecycle methods are not called when in EditMode.
template <typename T>
static constexpr bool kShouldRunInEditMode =
    T::kRunInEditMode || T::kExcludeFromEditor ||
    !component_traits::kIsIsfInfoDefined<T>;

}  // namespace component_traits

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_TRAITS_H_
