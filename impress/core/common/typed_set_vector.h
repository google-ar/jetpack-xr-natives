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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_SET_VECTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_SET_VECTOR_H_

#include <cstdint>

#include "core/common/log.h"
#include "absl/types/span.h"
#include "filament/libs/utils/include/utils/Allocator.h"
#include "filament/libs/utils/include/utils/StructureOfArrays.h"
#include "core/common/paired_span.h"
#include "core/common/typed_container_helpers.h"
#include "core/common/typed_id.h"

namespace imp {

// For header simplicity, alias filament's StructureOfArrays type.  Expects N
// type arguments, which become the value types of the N field arrays managed by
// the type.
template <typename... Elements>
class StructureOfArrays
    : public utils::StructureOfArraysBase<utils::HeapArena<>, Elements...> {
 public:
  using SoA = utils::StructureOfArraysBase<utils::HeapArena<>, Elements...>;
};

template <typename D>
class IndexableSetProxy {
  using Derived = D;

 public:
};

// IndexableSetVector (used by TypedSetVector and PairedSetVector) wraps a
// StructureOfArrays, and is indexable with a TypedId. A StructureOfArrays,
// broadly, seeks to maximize cache coherence by arranging the human-level
// concept of an array of structures into a set of arrays, one for each field.
//
// Filament's StructureOfArrays type arranges all of these arrays into a single
// block, and is smart about using placement-new/memcpy on the field types when
// assigned.
//
// To make this type easier to use, T is expected to be a type that meets the
// following requirements:
// - The type must locally declare the desired StructureOfArrays type into a
//   type alias called ArrayType.
// - The type must declare a scoped enum with the same number of fields as type
//   arguments to ArrayType.  The StructureOfArrays type has tuple-style get<>()
//   calls for some low level operations, and for the sake of clarity this enum
//   should live next to ArrayType.
// - The type must define a Proxy type, with no storage except for a union of
//   'ArrayType::Field' members, one for each field.
// The Magic Trick used here is twofold:
// - A C++ union where each union element has the same storage type is legal
//   and well defined.
// - This union can be constructed by supplying the single storage type, which
//   allows IndexableSetVector to construct T::Proxy regardless of how many
//   fields it has by constructing via ArrayType::Field.
// Thus index operations will return a Proxy, which by use of the Field API
// acts as a stand-in for the split-apart structure.
template <typename T, typename IdReferredType>
class IndexableSetVector
    : public T::ArrayType,
      public TTypedIdProviderMethods<IdReferredType, IndexableSetVector, T,
                                     IdReferredType> {
  // It's expected that the proxy type has a 'using' statement defining a
  // StructureOfArrays type.
  using Base = typename T::ArrayType;

 public:
  using Base::resize;
  using Base::size;

  IndexableSetVector() noexcept {}
  explicit IndexableSetVector(size_t initial_size) noexcept {
    resize(initial_size);
  }
  template <typename U, typename... Us>
  explicit IndexableSetVector(absl::Span<U> first,
                              absl::Span<Us>... rest) noexcept {
    size_t c = first.size();
    Base::setCapacity(c);
    if (!EnsureSizes(c, rest...)) IMP_LOG(imp::FATAL) << "mismatched sizes";
    for (size_t i = 0; i < c; ++i) {
      Base::push_back(first[i], rest[i]...);
    }
  }

  template <typename U, typename... Us>
  bool EnsureSizes(size_t size, absl::Span<U> first, absl::Span<Us>... rest) {
    bool result = (first.size() == size);
    if constexpr (sizeof...(rest) > 0) {
      result = result && EnsureSizes(size, rest...);
    }
    return result;
  }

  ~IndexableSetVector() noexcept = default;
  IndexableSetVector(IndexableSetVector&& rhs) noexcept = default;
  IndexableSetVector& operator=(IndexableSetVector&& rhs) noexcept = default;

  IndexableSetVector(IndexableSetVector const& rhs) = delete;
  IndexableSetVector& operator=(IndexableSetVector const& rhs) = delete;

  void reserve(size_t capacity) { Base::setCapacity(capacity); }

  bool empty() const noexcept { return !size(); }

  template <size_t FieldIndex>
  using TypeAt = typename Base::template TypeAt<FieldIndex>;
  template <size_t FieldIndex>
  TypeAt<FieldIndex>* data() noexcept {
    return Base::template data<FieldIndex>();
  }
  template <size_t FieldIndex>
  TypeAt<FieldIndex> const* data() const noexcept {
    return Base::template data<FieldIndex>();
  }
  template <size_t FieldIndex>
  absl::Span<TypeAt<FieldIndex>> RawSpan() noexcept {
    return absl::Span<TypeAt<FieldIndex>>(data<FieldIndex>(), size());
  }
  template <size_t FieldIndex>
  absl::Span<const TypeAt<FieldIndex>> RawSpan() const noexcept {
    return absl::Span<const TypeAt<FieldIndex>>(data<FieldIndex>(), size());
  }

  template <size_t FieldIndex>
  PairedSpan<TypeAt<FieldIndex>, IdReferredType> Span() noexcept {
    return PairedSpan<TypeAt<FieldIndex>, IdReferredType>(
        RawSpan<FieldIndex>());
  }
  template <size_t FieldIndex>
  PairedSpan<const TypeAt<FieldIndex>, IdReferredType> Span() const noexcept {
    return PairedSpan<const TypeAt<FieldIndex>, IdReferredType>(
        RawSpan<FieldIndex>());
  }

  template <typename I>
  UTILS_ALWAYS_INLINE typename T::Proxy operator[](I i) noexcept {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");

    return {{*this, static_cast<typename I::ValueType>(i)}};
  }

  template <typename I>
  UTILS_ALWAYS_INLINE const typename T::Proxy operator[](I i) const noexcept {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    // TODO Remove this hacky const casting by changing SoA::Field
    // to be const-aware, and similarly for Proxy; that way, non-const indexing
    // returns a T::Proxy<is_const=false> and const indexing returns a
    // T::Proxy<is_const=true>.
    auto* non_const_this = const_cast<IndexableSetVector*>(this);
    return {{*non_const_this,
             static_cast<uint32_t>(static_cast<typename I::ValueType>(i))}};
  }

  template <typename I>
  bool IsValid(I id) const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    return static_cast<typename I::ValueType>(id) < size() &&
           ((!kIsSigned<typename I::ValueType>) ||
            static_cast<typename I::ValueType>(id) >= 0);
  }

  template <typename I = TypedId<IdReferredType, size_t>, typename... Args>
  I Append(const Args&... args) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    Base::push_back(args...);
    return BackId<I>();
  }
  template <typename I = TypedId<IdReferredType, size_t>, typename... Args>
  I Append(Args&&... args) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    Base::push_back(std::forward<Args>(args)...);
    return BackId<I>();
  }
  template <typename I = TypedId<IdReferredType, size_t>>
  I BackId() const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    return I(static_cast<typename I::ValueType>(size() - 1));
  }
};

// A TypedSetVector uses the Proxy as the referred type of the indexing ID type.
template <typename Proxy>
using TypedSetVector = IndexableSetVector<Proxy, Proxy>;

// A PairedSetVector takes an additional parameter of the allowed index proxy.
template <typename Proxy, typename IdReferredType>
class PairedSetVector : public IndexableSetVector<Proxy, IdReferredType> {
  using Base = IndexableSetVector<Proxy, IdReferredType>;

 public:
  using Base::resize;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_SET_VECTOR_H_
