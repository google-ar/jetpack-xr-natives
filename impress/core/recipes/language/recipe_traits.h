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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_TRAITS_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_TRAITS_H_

#include <cmath>
#include <tuple>
#include <type_traits>
#include <utility>

#include "core/math/vec.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp {

namespace recipe_traits {

namespace internal {

template <typename InputT, typename = void>
struct IsAbsAvailable : std::false_type {};

// TODO: Look into allowing this expression to support matNf types
// as well
template <typename InputT>
struct IsAbsAvailable<InputT,
                      std::void_t<decltype(abs(std::declval<InputT>()))>>
    : std::integral_constant<bool,
                             std::is_signed_v<InputT> ||
                                 kIsAnyOf<InputT, float2, float3, float4>> {};

template <typename InputT, typename = void>
struct IsSqrtAvailable : std::false_type {};

template <typename InputT>
struct IsSqrtAvailable<InputT,
                       std::void_t<decltype(sqrt(std::declval<InputT>()))>>
    : std::true_type {};

template <typename InputT, typename = void>
struct IsLogAvailable : std::false_type {};

// This will return true for bool, so we explicitly disallow it
template <typename InputT>
struct IsLogAvailable<InputT,
                      std::void_t<decltype(log(std::declval<InputT>()))>>
    : std::integral_constant<bool, !std::is_same_v<InputT, bool>> {};

template <typename InputT, typename = void>
struct IsSinAvailable : std::false_type {};

template <typename InputT>
struct IsSinAvailable<
    InputT, std::void_t<decltype(recipe::Sin(std::declval<InputT>()))>>
    : std::true_type {};

template <typename InputT, typename = void>
struct IsCosAvailable : std::false_type {};

template <typename InputT>
struct IsCosAvailable<
    InputT, std::void_t<decltype(recipe::Cos(std::declval<InputT>()))>>
    : std::true_type {};

template <typename InputT, typename = void>
struct IsTanAvailable : std::false_type {};

template <typename InputT>
struct IsTanAvailable<
    InputT, std::void_t<decltype(recipe::Tan(std::declval<InputT>()))>>
    : std::true_type {};

template <typename InputT, typename = void>
struct IsAsinAvailable : std::false_type {};

template <typename InputT>
struct IsAsinAvailable<
    InputT, std::void_t<decltype(recipe::Asin(std::declval<InputT>()))>>
    : std::true_type {};

template <typename InputT, typename = void>
struct IsAcosAvailable : std::false_type {};

template <typename InputT>
struct IsAcosAvailable<
    InputT, std::void_t<decltype(recipe::Acos(std::declval<InputT>()))>>
    : std::true_type {};

template <typename InputT, typename = void>
struct IsAtanAvailable : std::false_type {};

template <typename InputT>
struct IsAtanAvailable<
    InputT, std::void_t<decltype(recipe::Atan(std::declval<InputT>()))>>
    : std::true_type {};

template <typename InputT, typename = void>
struct IsSignAvailable : std::false_type {};

template <typename InputT>
struct IsSignAvailable<
    InputT, std::void_t<decltype(recipe::Sign(std::declval<InputT>()))>>
    : std::true_type {};

template <typename InputT, typename = void>
struct IsNormalizeAvailable : std::false_type {};

template <typename InputT>
struct IsNormalizeAvailable<
    InputT, std::void_t<decltype(normalize(std::declval<InputT>()))>>
    : std::true_type {};

template <typename InputT, typename = void>
struct IsNotAvailable : std::false_type {};

template <typename InputT>
struct IsNotAvailable<InputT, std::void_t<decltype(!(std::declval<InputT>()))>>
    : std::integral_constant<bool, (std::is_same_v<InputT, bool>)> {};

template <typename InputT, typename = void>
struct IsBitwiseNotAvailable : std::false_type {};

template <typename InputT>
struct IsBitwiseNotAvailable<InputT,
                             std::void_t<decltype(~(std::declval<InputT>()))>>
    : std::integral_constant<bool, (std::is_same_v<InputT, int>)> {};

template <typename LeftT, typename RightT, typename = void>
struct IsAddAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsAddAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() + std::declval<RightT>())>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsSubtractAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsSubtractAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() - std::declval<RightT>())>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsMultiplyAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsMultiplyAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() * std::declval<RightT>())>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsDivideAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsDivideAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() / std::declval<RightT>())>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsModAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsModAvailable<
    LeftT, RightT,
    std::void_t<decltype(fmod(std::declval<LeftT>(), std::declval<RightT>()))>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsEqualsAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsEqualsAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() == std::declval<RightT>())>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsNotEqualsAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsNotEqualsAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() != std::declval<RightT>())>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsGreaterThanAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsGreaterThanAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() > std::declval<RightT>())>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsLessThanAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsLessThanAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() < std::declval<RightT>())>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsGreaterThanOrEqualAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsGreaterThanOrEqualAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() >= std::declval<RightT>())>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsLessThanOrEqualAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsLessThanOrEqualAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() <= std::declval<RightT>())>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsMinAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsMinAvailable<LeftT, RightT,
                      std::void_t<decltype(recipe::Min(
                          std::declval<LeftT>(), std::declval<RightT>()))>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsMaxAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsMaxAvailable<LeftT, RightT,
                      std::void_t<decltype(recipe::Max(
                          std::declval<LeftT>(), std::declval<RightT>()))>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsAndAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsAndAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() && std::declval<RightT>())>>
    : std::integral_constant<bool, (std::is_same_v<LeftT, bool> &&
                                    std::is_same_v<RightT, bool>)> {};

template <typename LeftT, typename RightT, typename = void>
struct IsBitwiseAndAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsBitwiseAndAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() & std::declval<RightT>())>>
    : std::integral_constant<bool, (std::is_same_v<LeftT, int> &&
                                    std::is_same_v<RightT, int>)> {};

template <typename LeftT, typename RightT, typename = void>
struct IsOrAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsOrAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() || std::declval<RightT>())>>
    : std::integral_constant<bool, (std::is_same_v<LeftT, bool> &&
                                    std::is_same_v<RightT, bool>)> {};

template <typename LeftT, typename RightT, typename = void>
struct IsBitwiseOrAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsBitwiseOrAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() | std::declval<RightT>())>>
    : std::integral_constant<bool, (std::is_same_v<LeftT, int> &&
                                    std::is_same_v<RightT, int>)> {};

template <typename LeftT, typename RightT, typename = void>
struct IsXorAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsXorAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() != std::declval<RightT>())>>
    : std::integral_constant<bool, (std::is_same_v<LeftT, bool> &&
                                    std::is_same_v<RightT, bool>)> {};

template <typename LeftT, typename RightT, typename = void>
struct IsBitwiseXorAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsBitwiseXorAvailable<
    LeftT, RightT,
    std::void_t<decltype(std::declval<LeftT>() ^ std::declval<RightT>())>>
    : std::integral_constant<bool, (std::is_same_v<LeftT, int> &&
                                    std::is_same_v<RightT, int>)> {};

template <typename LeftT, typename RightT, typename = void>
struct IsAssignAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsAssignAvailable<
    LeftT, RightT,
    decltype(std::declval<LeftT&>() = std::declval<RightT>(), void())>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsAddAssignAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsAddAssignAvailable<
    LeftT, RightT,
    decltype(std::declval<LeftT&>() += std::declval<RightT>(), void())>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsSubtractAssignAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsSubtractAssignAvailable<
    LeftT, RightT,
    decltype(std::declval<LeftT&>() -= std::declval<RightT>(), void())>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsMultiplyAssignAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsMultiplyAssignAvailable<
    LeftT, RightT,
    decltype(std::declval<LeftT&>() *= std::declval<RightT>(), void())>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsDivideAssignAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsDivideAssignAvailable<
    LeftT, RightT,
    decltype(std::declval<LeftT&>() /= std::declval<RightT>(), void())>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsDotAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsDotAvailable<
    LeftT, RightT,
    std::void_t<decltype(dot(std::declval<LeftT>(), std::declval<RightT>()))>>
    : std::true_type {};

template <typename LeftT, typename RightT, typename = void>
struct IsCrossAvailable : std::false_type {};

template <typename LeftT, typename RightT>
struct IsCrossAvailable<
    LeftT, RightT,
    std::void_t<decltype(cross(std::declval<LeftT>(), std::declval<RightT>()))>>
    : std::true_type {};

}  // namespace internal

template <typename InputT>
constexpr bool kIsAbsAvailable = internal::IsAbsAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsSqrtAvailable = internal::IsSqrtAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsLogAvailable = internal::IsLogAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsSinAvailable = internal::IsSinAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsCosAvailable = internal::IsCosAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsTanAvailable = internal::IsTanAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsAsinAvailable = internal::IsAsinAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsAcosAvailable = internal::IsAcosAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsAtanAvailable = internal::IsAtanAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsSignAvailable = internal::IsSignAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsNormalizeAvailable =
    internal::IsNormalizeAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsNotAvailable = internal::IsNotAvailable<InputT>::value;

template <typename InputT>
constexpr bool kIsBitwiseNotAvailable =
    internal::IsBitwiseNotAvailable<InputT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsAddAvailable = internal::IsAddAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsSubtractAvailable =
    internal::IsSubtractAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsMultiplyAvailable =
    internal::IsMultiplyAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsDivideAvailable =
    internal::IsDivideAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsModAvailable = internal::IsModAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsEqualsAvailable =
    internal::IsEqualsAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsNotEqualsAvailable =
    internal::IsNotEqualsAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsGreaterThanAvailable =
    internal::IsGreaterThanAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsLessThanAvailable =
    internal::IsLessThanAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsGreaterThanOrEqualAvailable =
    internal::IsGreaterThanOrEqualAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsLessThanOrEqualAvailable =
    internal::IsLessThanOrEqualAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsMinAvailable = internal::IsMinAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsMaxAvailable = internal::IsMaxAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsAndAvailable = internal::IsAndAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsBitwiseAndAvailable =
    internal::IsBitwiseAndAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsOrAvailable = internal::IsOrAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsBitwiseOrAvailable =
    internal::IsBitwiseOrAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsXorAvailable = internal::IsXorAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsBitwiseXorAvailable =
    internal::IsBitwiseXorAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsAssignAvailable =
    internal::IsAssignAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsAddAssignAvailable =
    internal::IsAddAssignAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsSubtractAssignAvailable =
    internal::IsSubtractAssignAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsMultiplyAssignAvailable =
    internal::IsMultiplyAssignAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsDivideAssignAvailable =
    internal::IsDivideAssignAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsDotAvailable = internal::IsDotAvailable<LeftT, RightT>::value;

template <typename LeftT, typename RightT>
constexpr bool kIsCrossAvailable =
    internal::IsCrossAvailable<LeftT, RightT>::value;

template <typename Fn, typename Ret, typename... Args>
struct FunctorUnpacker {
  using ReturnT = Ret;
  using ArgsTuple = std::tuple<Args...>;

  explicit FunctorUnpacker(Ret (Fn::*)(Args...));
  explicit FunctorUnpacker(Ret (Fn::*)(Args...) const);
};

// This template deduction guide is necessary to suppress a warning
template <typename Fn, typename Ret, typename... Args>
FunctorUnpacker(Ret (Fn::*)(Args...))
    -> FunctorUnpacker<Ret, std::tuple<Args...>>;

}  // namespace recipe_traits

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_TRAITS_H_
