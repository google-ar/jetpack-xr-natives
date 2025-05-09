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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_SMALL_SOURCE_LOCATION_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_SMALL_SOURCE_LOCATION_H_

#include <cstdint>
#include <utility>

namespace imp {

// Similar to std::source_location, but available prior to C++20 and stores a
// more minimal amount of information to prevent binary size bloat. Reportedly,
// std::source_location can (when used heavily) increase binary size by a large
// amount.
//
// This is also hashable.
//
class SmallSourceLocation {
 public:
  SmallSourceLocation() = default;

  // Can be used as the default parameter value for a function that takes a
  // SmallSourceLocation to track the call-site where the function was called.
  //
  // For example:
  //
  //   void Foo(SmallSourceLocation loc = SmallSourceLocation::Current()) {
  //     ...
  //   }
  //
  // NOTE: This class must be implemented in the header file because the
  // __builtin_LINE() and __builtin_FILE() macros must be resolved at compile
  // time, not link time, when used as default arguments.
  //
  // TODO: Ensure this works on all platforms we care about.
  static constexpr SmallSourceLocation Current(
      uint16_t line = __builtin_LINE(), const char* file = __builtin_FILE()) {
    // Find the file name from the full path by stripping everything before the
    // last '/'.
    //
    // This gets evaluated at compile time, so only the final file name is
    // stored in the binary.
    const char* file_name = file;
    while (*file) {
      if (*file++ == '/') {
        file_name = file;
      }
    }

    // Note: An alternative to this would be to hash the location at compile
    // time using imp::Hash. This would make SmallSourceLocation very
    // lightweight. However, it would require us to figure out a way to generate
    // and store a table for mapping the hashes back to useful locations
    // (similar to desymbolication).
    return SmallSourceLocation(line, file_name);
  }

  uint_least32_t GetLineNumber() const { return line_; }
  const char* GetFileName() const { return file_name_; }

  bool operator==(const SmallSourceLocation& rhs) const {
    return line_ == rhs.line_ && file_name_ == rhs.file_name_;
  }
  bool operator!=(const SmallSourceLocation& rhs) const {
    return line_ != rhs.line_ || file_name_ != rhs.file_name_;
  }

  template <typename H>
  friend H AbslHashValue(H h, const SmallSourceLocation& loc) {
    return H::combine(std::move(h), loc.line_, loc.file_name_);
  }

 private:
  constexpr SmallSourceLocation(uint16_t line, const char* file_name)
      : line_(line), file_name_(file_name) {}

  // The line number of the source location.
  // Doesn't work if the source file has more than 65535 lines.
  uint16_t line_ = 0;

  // The name of the file where the source location was created.
  const char* file_name_ = nullptr;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_SMALL_SOURCE_LOCATION_H_
