/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_PTR_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_PTR_HELPERS_H_

#include <type_traits>
namespace imp {

namespace imp_owned_ptr_traits {

// Checks if a OwnedPtr<TypeA, DeleterA> can be upcasted from an
// OwnedPtr<TypeB, DeleterB>.
template <typename TypeA, typename DeleterA, typename TypeB, typename DeleterB>
static constexpr bool kCanUpcastOwnedPtr =
    std::is_base_of_v<TypeA, TypeB> &&
    std::is_convertible_v<DeleterB, DeleterA> && !std::is_same_v<TypeA, TypeB>;

// Used to disable upcast if the types are not compatible.
template <typename TypeA, typename DeleterA, typename TypeB, typename DeleterB>
using EnableIfCanUpcastOwnedPtr =
    std::enable_if_t<kCanUpcastOwnedPtr<TypeA, DeleterA, TypeB, DeleterB>, int>;

// Checks if a BorrowedPtr<TypeA> can be upcasted from a BorrowedPtr<TypeB>.
template <typename TypeA, typename TypeB>
static constexpr bool kCanUpcastBorrowedPtr =
    std::is_base_of_v<TypeA, TypeB> && !std::is_same_v<TypeA, TypeB>;

// Used to disable upcast if the types are not compatible.
template <typename TypeA, typename TypeB>
using EnableIfCanUpcastBorrowedPtr =
    std::enable_if_t<kCanUpcastBorrowedPtr<TypeA, TypeB>, int>;

template <typename TypeA, typename TypeB>
static constexpr bool kCanDowncastBorrowedPtr =
    std::is_base_of_v<TypeB, TypeA> && !std::is_same_v<TypeA, TypeB>;

template <typename TypeA, typename TypeB>
using EnableIfCanDowncastBorrowedPtr =
    std::enable_if_t<kCanDowncastBorrowedPtr<TypeA, TypeB>, int>;

}  // namespace imp_owned_ptr_traits

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_PTR_HELPERS_H_
