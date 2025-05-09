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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_ENUM_INDEXED_ARRAY_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_ENUM_INDEXED_ARRAY_H_

#include <array>
#include <type_traits>

namespace imp {

// Utility that wraps an std::array and provides the ability to index it via an
// enum.
template <typename Enum, typename T, std::size_t N>
class EnumIndexedArray {
 private:
  static_assert(std::is_enum_v<Enum>);

  using IndexType = std::underlying_type_t<Enum>;
  using ArrayType = std::array<T, N>;

 public:
  // Forward types provided by std::array in std container style.
  using value_type = typename ArrayType::value_type;
  using reference = typename ArrayType::reference;
  using const_reference = typename ArrayType::const_reference;

  constexpr EnumIndexedArray() : array_() {}

  constexpr auto operator[](Enum idx) noexcept -> reference {
    return array_[static_cast<IndexType>(idx)];
  }

 private:
  ArrayType array_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_ENUM_INDEXED_ARRAY_H_
