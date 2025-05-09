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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_VECTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_VECTOR_H_

#include <cstddef>
#include <limits>
#include <type_traits>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/types/span.h"
#include "core/common/typed_container_helpers.h"
#include "core/common/typed_id.h"

namespace imp {

// A `TypedVector<Thing>` owns a sequence of `Thing`s, indexed by `ThingId`.
// When encountered, this is generally the idiomatic/primary data referred to by
// the TypedId.  It wraps a std::vector and uses it for all API's besides
// `operator[]`.
template <class T>
class TypedVector
    : public TTypedContainerMethods<TypedVector, T>,
      public TTypedIdProviderMethods<T, TypedVector, T>,
      public TGenericContainerMethods<TypedVector<T>, std::vector<T>> {
 public:
  using iterator = typename TGenericContainerMethods<TypedVector<T>,
                                                     std::vector<T>>::iterator;
  TypedVector() : container_() {}
  explicit TypedVector(std::vector<T> vector) : container_(std::move(vector)) {}
  explicit TypedVector(size_t size) : container_(size) {}
  TypedVector(std::initializer_list<T> init) : container_(std::move(init)) {}

 protected:
  std::vector<T> container_;
  friend class TGenericContainerMethods<TypedVector<T>, std::vector<T>>;
  friend class TTypedContainerMethods<TypedVector, T>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_VECTOR_H_
