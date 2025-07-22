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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_DETAILS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_DETAILS_H_

#include <array>
#include <cstddef>

#include "absl/strings/string_view.h"
#include "core/common/type_traits.h"

// IMP_TRACE_SMASH(X,Y) concatenates evaluations of X and Y
#define IMP_TRACE_XSMASH(X, Y) X##Y
#define IMP_TRACE_SMASH(X, Y) IMP_TRACE_XSMASH(X, Y)

namespace imp::trace::details {

constexpr size_t DigitCount(const size_t in) {
  size_t result = 1;
  for (size_t iter = in; iter > 10; ++result, iter /= 10) {
  }
  return result;
}

template <size_t kNameSize>
constexpr std::array<absl::string_view::value_type, kNameSize + 1>
NullTerminatedName(const absl::string_view function_name) {
  std::array<absl::string_view::value_type, kNameSize + 1> result = {};
  size_t cursor = 0;
  for (; cursor < kNameSize; ++cursor) {
    result[cursor] = function_name[cursor];
  }
  // Null-terminate to make it compatible with the filament API
  result[cursor++] = '\0';
  return result;
}

// Constexpr function that concats two strings (left and right) into a
// null-terminated character array. An array return is necessary because we
// can't do heap (std::string) allocations in a constexpr function.
template <size_t kLeftSize, size_t kRightSize>
constexpr std::array<absl::string_view::value_type, kLeftSize + kRightSize + 1>
ConstStrCat(const absl::string_view left, const absl::string_view right) {
  std::array<absl::string_view::value_type, kLeftSize + kRightSize + 1> result =
      {};
  size_t result_cursor = 0;
  for (size_t str_cursor = 0; str_cursor < kLeftSize;
       ++str_cursor, ++result_cursor) {
    result[result_cursor] = left[str_cursor];
  }
  for (size_t str_cursor = 0; str_cursor < kRightSize;
       ++str_cursor, ++result_cursor) {
    result[result_cursor] = right[str_cursor];
  }
  // Null-terminate to make it compatible with the filament API
  result[result_cursor++] = '\0';
  return result;
}

template <size_t kStorageSize, size_t kLine>
constexpr std::array<absl::string_view::value_type, kStorageSize>
BuildTraceBlockStorage(const absl::string_view function_name,
                       const absl::string_view block_context) {
  constexpr size_t digit_count = DigitCount(kLine);
  std::array<absl::string_view::value_type, kStorageSize> result = {};
  size_t cursor = 0;
  for (; cursor < function_name.size(); ++cursor) {
    result[cursor] = function_name[cursor];
  }
  if (!block_context.empty()) {
    result[cursor++] = '(';
    for (size_t i = 0; i < block_context.size(); ++i) {
      result[cursor + i] = block_context[i];
    }
    cursor += block_context.size();
    result[cursor++] = ')';
  }
  result[cursor++] = ':';
  for (int digit = digit_count - 1; digit >= 0; --digit) {
    size_t divisor = 1;
    for (int i = 0; i < digit; i++) {
      divisor *= 10;
    }
    result[cursor++] = '0' + (kLine / divisor) % 10;
  }
  // Null-terminate to make it compatible with the filament API
  result[cursor++] = '\0';
  return result;
}

}  // namespace imp::trace::details

#define IMP_TRACE_PRIVATE()                                             \
  static constexpr absl::string_view kImpressTraceFunctionName =        \
      IMP_FUNCTION_NAME();                                              \
  (void)kImpressTraceFunctionName;                                      \
  static constexpr auto kImpressTraceFunctionNameStorage =              \
      ::imp::trace::details::NullTerminatedName<                        \
          kImpressTraceFunctionName.size()>(kImpressTraceFunctionName); \
  (void)kImpressTraceFunctionNameStorage;                               \
  IMP_TRACE_PRIVATE_IMPL(kImpressTraceFunctionNameStorage.data())

#define IMP_TRACE_PRIVATE_TEMPLATED(types...) \
  IMP_TRACE_PRIVATE_NAME_TEMPLATED(IMP_FUNCTION_NAME(), types)

#define IMP_TRACE_PRIVATE_NAME(name) IMP_TRACE_PRIVATE_IMPL(name)

#define IMP_TRACE_PRIVATE_NAME_TEMPLATED(name, types...)                    \
  static constexpr absl::string_view kImpressTraceName = name;              \
  (void)kImpressTraceName;                                                  \
  static constexpr absl::string_view kImpressTraceTemplateArgs =            \
      ::imp::type_traits::GetTypeList<types>();                             \
  (void)kImpressTraceTemplateArgs;                                          \
  static constexpr auto kImpressTraceNameStorage =                          \
      ::imp::trace::details::ConstStrCat<kImpressTraceName.size(),          \
                                         kImpressTraceTemplateArgs.size()>( \
          kImpressTraceName, kImpressTraceTemplateArgs);                    \
  (void)kImpressTraceNameStorage;                                           \
  IMP_TRACE_PRIVATE_IMPL(kImpressTraceNameStorage.data())

#define IMP_TRACE_PRIVATE_BLOCK(block_context)                               \
  constexpr size_t IMP_TRACE_SMASH(kImpressTraceBlockLine, __LINE__) =       \
      __LINE__;                                                              \
  constexpr absl::string_view IMP_TRACE_SMASH(kImpressTraceBlockContextName, \
                                              __LINE__) = block_context;     \
  /* Build a null-terminated std::array that holds the context string. */    \
  constexpr auto IMP_TRACE_SMASH(kImpressTraceBlockNameStorage, __LINE__) =  \
      ::imp::trace::details::BuildTraceBlockStorage<                         \
          kImpressTraceFunctionName.size() + 2 +                             \
              ::imp::trace::details::DigitCount(                             \
                  IMP_TRACE_SMASH(kImpressTraceBlockLine, __LINE__)) +       \
              (IMP_TRACE_SMASH(kImpressTraceBlockContextName, __LINE__)      \
                       .empty()                                              \
                   ? 0                                                       \
                   : 2 + IMP_TRACE_SMASH(kImpressTraceBlockContextName,      \
                                         __LINE__)                           \
                             .size()),                                       \
          IMP_TRACE_SMASH(kImpressTraceBlockLine, __LINE__)>(                \
          kImpressTraceFunctionName, block_context);                         \
  /* Emit a trace event using the generated string storage. */               \
  IMP_TRACE_PRIVATE_IMPL(                                                    \
      IMP_TRACE_SMASH(kImpressTraceBlockNameStorage, __LINE__).data())

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_DETAILS_H_
