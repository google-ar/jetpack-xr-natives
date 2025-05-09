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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_PAIRED_VECTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_PAIRED_VECTOR_H_

#include <cstddef>
#include <vector>

#include "absl/types/span.h"
#include "core/common/paired_span.h"
#include "core/common/typed_container_helpers.h"

namespace imp {

// A PairedVector<ValueType, ReferredType> owns a std::vector of ValueTypes,
// each of which is conceptually 'paired' with a corresponding ReferredType;
// this pairing boils down to the container being indexed with an Id of its
// paired ReferredType (i.e. a TypedId<ReferredType, [storage type]>).

// Primary use case is to allow static data (e.g. a list of Materials parsed
// from a gltf file) to be associated with instance data (e.g. a list of
// Material Instances unique to a Node), because they can both be indexed with
// the same `MaterialId`.
// Secondary use case is to allow structures-of-arrays or to sidecar
// rarely-accessed data (e.g. PairedVector<std::string, BoneId> bone_names).
template <class T, class IdReferredType>
class PairedVector
    : public TPairedContainerMethods<PairedVector, T, IdReferredType>,
      public TTypedIdProviderMethods<IdReferredType, PairedVector, T,
                                     IdReferredType>,
      public TGenericContainerMethods<PairedVector<T, IdReferredType>,
                                      std::vector<T>> {
  static constexpr bool kIsCopyable = absl::is_copy_assignable<T>::value;

 public:
  PairedVector() : container_() {}
  explicit PairedVector(size_t n, T default_value = {})
      : container_(n, default_value) {}
  explicit PairedVector(std::vector<T>&& vector) : container_(vector) {}
  PairedVector(std::initializer_list<T> init) : container_(std::move(init)) {}
  template <typename C>
  void Pair(const C& container, T default_value = T()) {
    static_assert(std::is_same_v<absl::remove_const_t<absl::remove_reference_t<
                                     decltype(container.front())>>,
                                 absl::remove_const_t<IdReferredType>>,
                  "container has wrong type");
    container_.resize(container.size(), default_value);
  }

  PairedSpan<T, IdReferredType> AsSpan() {
    return PairedSpan<T, IdReferredType>(container_.data(), container_.size());
  }

 protected:
  std::vector<T> container_;
  friend class TPairedContainerMethods<PairedVector, T, IdReferredType>;
  friend class TGenericContainerMethods<PairedVector<T, IdReferredType>,
                                        std::vector<T>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_PAIRED_VECTOR_H_
