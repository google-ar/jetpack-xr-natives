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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_HOLDABLE_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_HOLDABLE_H_

#include "core/common/type_erased.h"

namespace imp {

// A move-only holder of a type-erased object stored as a void* through a
// unique_ptr.
//
// The type-erased object is destructed when the holdable is destructed.
//
// Holdable can store value types, reference-counted types, and move only types.
// It will not deallocate raw pointers passed into Holdable.
// TODO: Consider removing Holdable in favor of using TypeErased
// directly. The layer of indirection is not necessary. Though, having at least
// a using statement for TypeErased would be helpful so that we can control the
// size  of the TypeErased for this use case.
class Holdable {
 public:
  // Constructs a holdable that holds the object passed in.
  template <typename T>
  explicit Holdable(T&& held);

 private:
  TypeErased held_;
};

template <typename T>
Holdable::Holdable(T&& held) : held_(std::forward<T>(held)) {}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_HOLDABLE_H_
