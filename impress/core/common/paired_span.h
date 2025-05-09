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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_PAIRED_SPAN_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_PAIRED_SPAN_H_

#include <vector>

#include "absl/types/span.h"
#include "core/common/typed_container_helpers.h"

namespace imp {

// A PairedSpan<T> wraps an un-owned container into something indexable by
// a TypedId to a IdReferredType.  It is spiritually identical to a PairedVector
// except it doesn't own the data.
template <class T, class IdReferredType>
class PairedSpan
    : public TPairedContainerMethods<PairedSpan, T, IdReferredType>,
      public TTypedIdProviderMethods<IdReferredType, PairedSpan, T,
                                     IdReferredType>,
      public TGenericContainerMethods<PairedSpan<T, IdReferredType>,
                                      absl::Span<T>> {
 protected:
  using Container = absl::Span<T>;
  using ConstSourceContainer = const std::vector<absl::remove_const_t<T>>;

 public:
  PairedSpan() : container_() {}
  constexpr PairedSpan(T* array, size_t length) noexcept
      : container_(array, length) {}
  explicit PairedSpan(Container view) : container_(view) {}
  explicit PairedSpan(ConstSourceContainer& vector)
      : container_(vector.data(), vector.size()) {}

 protected:
  Container container_;
  friend class TPairedContainerMethods<PairedSpan, T, IdReferredType>;
  friend class TGenericContainerMethods<PairedSpan<T, IdReferredType>,
                                        absl::Span<T>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_PAIRED_SPAN_H_
