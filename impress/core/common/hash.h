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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_HASH_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_HASH_H_

#include <cstddef>
#include <cstdint>

#include "absl/strings/string_view.h"

// String hashing function used by various parts of Imp.  It uses the
// FNV-1a algorithm from:
// https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
namespace imp {

using HashValue = unsigned int;

constexpr HashValue kHashOffsetBasis = 0x84222325;
constexpr HashValue kHashPrimeMultiplier = 0x000001b3;
/// Golden ratio hash value is derived by dividing max int by the golden ratio:
/// 2^32 / 1.61803399.
constexpr HashValue kHashGoldenRatio = 0x9e3779b9;

constexpr HashValue Hash(absl::string_view str);
// Calling Hash(Hash("prefix"), "Suffix") is equivalent to Hash("prefixSuffix").
constexpr HashValue Hash(HashValue basis, absl::string_view str);
constexpr HashValue HashCaseInsensitive(absl::string_view str);
constexpr HashValue HashCombine(HashValue lhs, HashValue rhs);

constexpr HashValue Hash(absl::string_view str) {
  return Hash(kHashOffsetBasis, str);
}

constexpr HashValue Hash(HashValue basis, absl::string_view str) {
  if (str.empty()) {
    return 0;
  }

  // A quick good hash, from:
  // https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
  // Specifically, the FNV-1a function.
  HashValue value = basis != 0 ? basis : kHashOffsetBasis;
  for (auto& c : str) {
    if (!c) {
      break;
    }
    value = (value ^ static_cast<unsigned char>(c)) * kHashPrimeMultiplier;
  }

#ifdef IMP_DEBUG_HASH
  GetUnhashTable().emplace(value, std::string(str));
#endif

  return value;
}

constexpr HashValue HashCaseInsensitive(absl::string_view str) {
  if (str.empty()) {
    return 0;
  }

  HashValue value = kHashOffsetBasis;
  for (auto& c : str) {
    if (!c) {
      break;
    }
    value =
        (value ^ static_cast<unsigned char>(tolower(c))) * kHashPrimeMultiplier;
  }

  return value;
}

constexpr HashValue HashCombine(HashValue lhs, HashValue rhs) {
  // Offset by the golden ratio to avoid mapping all zeros to all zeros.
  return lhs ^ (rhs + kHashGoldenRatio + (lhs << 6) + (lhs >> 2));
}

ABSL_DEPRECATED("Use `Hash()` instead since that is also constexpr now.")
constexpr HashValue ConstHash(absl::string_view str) { return Hash(str); }

// Functor for using hashable types in STL containers.
struct Hasher {
  template <class T>
  size_t operator()(const T& value) const {
    return Hash(value);
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_HASH_H_
