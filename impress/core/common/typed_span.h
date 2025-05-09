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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_SPAN_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_SPAN_H_

#include "absl/types/span.h"
#include "core/common/typed_container_helpers.h"
#include "core/common/typed_id.h"

namespace imp {

// A TypedSpan<T> wraps an un-owned container into something indexable by
// TypedId.  It is spiritually identical to a TypedVector except it doesn't own
// the data.
template <class T>
class TypedSpan : public TTypedContainerMethods<TypedSpan, T>,
                  public TTypedIdProviderMethods<T, TypedSpan, T>,
                  public TGenericContainerMethods<TypedSpan<T>, absl::Span<T>> {
 public:
  TypedSpan() : container_() {}
  explicit TypedSpan(absl::Span<T> view) : container_(view) {}
  constexpr TypedSpan(T* array, size_t length) noexcept
      : container_(array, length) {}

  template <typename C = std::vector<T>>
  void Set(C& target) {
    container_ = absl::MakeSpan(target);
  }

 protected:
  absl::Span<T> container_;
  friend class TTypedContainerMethods<TypedSpan, T>;
  friend class TGenericContainerMethods<TypedSpan<T>, absl::Span<T>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_SPAN_H_
