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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_VECTOR_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_VECTOR_HELPERS_H_

#include <cstddef>
#include <memory>
#include <vector>

namespace imp {

// Helper function to compact a vector of unique pointers.
//
// This function is useful for eliminating all empty elements of a vector so
// that it is compact.
//
// The function takes a vector of unique pointers and a callback function. The
// callback function is called whenever an element is moved with the new index
// of the moved element as the argument. This can be used to update indices or
// other bookkeeping information.
//
// NOTE: This algorithm intentionally does *not* maintain the order of the
// vector. It is instead intended to minimize the number of elements moved.
//
// Example usage:
//
// std::vector<std::unique_ptr<MyClass>> my_vector;
// CompactVector(my_vector, [](size_t new_index) {
//   // Update indices or other bookkeeping information.
// });
template <typename T, typename Deleter, typename Fn>
void CompactVector(std::vector<std::unique_ptr<T, Deleter>>& vector, Fn fn) {
  size_t left = 0;
  size_t right = vector.size();

  while (left < right) {
    if (!vector[left]) {
      // Find the rightmost non-null element to swap with.
      while (right > left && !vector[right - 1]) {
        right--;
      }

      // If there's a non-null element on the right side, swap it.
      if (right > left) {
        vector[left] = std::move(vector[right - 1]);
        fn(left);
        right--;
      }
    }
    left++;
  }

  // Resize the vector to remove the empty slots at the end.
  vector.erase(vector.begin() + right, vector.end());
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_VECTOR_HELPERS_H_
