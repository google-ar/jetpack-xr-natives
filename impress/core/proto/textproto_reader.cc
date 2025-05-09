// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/proto/textproto_reader.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "absl/strings/ascii.h"
#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"
#include "core/proto/proto_common.h"
#include "core/view/utils/string_map.h"

namespace imp {
namespace proto {
namespace {

const char* FindStringEnd(const char* ptr, const char* end) {
  while (ptr && ptr < end) {
    if (*ptr == '\\' && (ptr + 1) < end && ptr[1] == '"') {
      ++ptr;
    }
    ++ptr;
    if (ptr < end && *ptr == '"') break;
  }
  return ptr;
}

}  // namespace

StringMap<TextprotoReader::VisitRegisteredAnyFn>*
TextprotoReader::GetVisitRegisteredFnMap() {
  // It's best to store static data structures as a static pointer inside of
  // a static function, per (broken link) and (broken link).
  static StringMap<TextprotoReader::VisitRegisteredAnyFn>*
      visit_registered_fn_map =
          new StringMap<TextprotoReader::VisitRegisteredAnyFn>();
  return visit_registered_fn_map;
}

void TextprotoReader::ClearRegisteredTypes() {
  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  map->clear();
}

const char* TextprotoReader::Consume(const char* ptr, const char* end,
                                     char end_token) {
  std::vector<char> expect(1, end_token);
  while (ptr && ptr < end) {
    if (*ptr == expect.back()) {
      expect.pop_back();
      if (expect.empty()) break;
    }
    if (*ptr == '"') {
      ptr = FindStringEnd(ptr, end);
    } else if (*ptr == '{') {
      expect.push_back('}');
    } else if (*ptr == '[') {
      expect.push_back(']');
    }
    ++ptr;
  }
  return ptr + 1;
}

const char* TextprotoReader::NextToken(const char* ptr, int* token) {
  ptr = SkipWhitespace(ptr);
  if (!ptr || ptr >= end_) {
    *token = 0;
    return end_;
  }
  if (*ptr == '{' || *ptr == '}' || *ptr == '[' || *ptr == ']' || *ptr == ':' ||
      *ptr == ',') {
    *token = *ptr;
    token_ = absl::string_view();
    return SkipWhitespace(ptr + 1);
  } else if (*ptr == '"') {
    *token = '"';
    auto end = FindStringEnd(ptr, end_);
    if (end < end_) {
      token_ = absl::string_view(ptr + 1, end - ptr - 1);
      return SkipWhitespace(end + 1);
    }
    token_ = absl::string_view();
    return end_;
  } else {
    auto end = std::find_if(ptr, end_, [](char c) {
      return absl::ascii_isspace(c) || c == ',' || c == '}' || c == ']' ||
             c == ':';
    });
    *token = '*';
    token_ = absl::string_view(ptr, end - ptr);
    return SkipWhitespace(end);
  }
}

const char* TextprotoReader::ExpectToken(const char* ptr, int expect) {
  int type;
  ptr = NextToken(ptr, &type);
  if (type == expect) {
    return ptr;
  }
  return nullptr;
}

const char* TextprotoReader::Unknown(const char* ptr, int field_id,
                                     int token_type) {
  switch (token_type) {
    case '[':
      ptr_ = Consume(ptr, end_, ']');
      break;
    case '{':
      ptr_ = Consume(ptr, end_, '}');
      break;
    default:
      break;
  }
  return ptr_;
}

template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id, bool* field,
                                   bool* other, int token_type) {
  static_assert(field_type == TYPE_BOOL);
  if (token_ == "true") {
    *field = true;
  } else if (token_ == "false") {
    *field = false;
  } else {
    ptr_ = nullptr;
  }
  return ptr;
}
template const char* TextprotoReader::Visit<TYPE_BOOL>(const char* ptr,
                                                       int field_id,
                                                       bool* field, bool* other,
                                                       int token_type);

template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   int32_t* field, int32_t* other,
                                   int token_type) {
  ReadNumber(field);
  return ptr;
}
template const char* TextprotoReader::Visit<TYPE_INT32>(const char* ptr,
                                                        int field_id,
                                                        int32_t* field,
                                                        int32_t* other,
                                                        int token_type);
template const char* TextprotoReader::Visit<TYPE_SINT32>(const char* ptr,
                                                         int field_id,
                                                         int32_t* field,
                                                         int32_t* other,
                                                         int token_type);
template const char* TextprotoReader::Visit<TYPE_SFIXED32>(const char* ptr,
                                                           int field_id,
                                                           int32_t* field,
                                                           int32_t* other,
                                                           int token_type);
template const char* TextprotoReader::Visit<TYPE_UINT64>(const char* ptr,
                                                         int field_id,
                                                         int32_t* field,
                                                         int32_t* other,
                                                         int token_type);

template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   uint32_t* field, uint32_t* other,
                                   int token_type) {
  ReadNumber(field);
  return 0;
}
template const char* TextprotoReader::Visit<TYPE_UINT32>(const char* ptr,
                                                         int field_id,
                                                         uint32_t* field,
                                                         uint32_t* other,
                                                         int token_type);
template const char* TextprotoReader::Visit<TYPE_FIXED32>(const char* ptr,
                                                          int field_id,
                                                          uint32_t* field,
                                                          uint32_t* other,
                                                          int token_type);

template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   int64_t* field, int64_t* other,
                                   int token_type) {
  ReadNumber(field);
  return 0;
}
template const char* TextprotoReader::Visit<TYPE_INT64>(const char* ptr,
                                                        int field_id,
                                                        int64_t* field,
                                                        int64_t* other,
                                                        int token_type);
template const char* TextprotoReader::Visit<TYPE_SINT64>(const char* ptr,
                                                         int field_id,
                                                         int64_t* field,
                                                         int64_t* other,
                                                         int token_type);
template const char* TextprotoReader::Visit<TYPE_SFIXED64>(const char* ptr,
                                                           int field_id,
                                                           int64_t* field,
                                                           int64_t* other,
                                                           int token_type);

template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   uint64_t* field, uint64_t* other,
                                   int token_type) {
  ReadNumber(field);
  return 0;
}
template const char* TextprotoReader::Visit<TYPE_UINT64>(const char* ptr,
                                                         int field_id,
                                                         uint64_t* field,
                                                         uint64_t* other,
                                                         int token_type);
template const char* TextprotoReader::Visit<TYPE_FIXED64>(const char* ptr,
                                                          int field_id,
                                                          uint64_t* field,
                                                          uint64_t* other,
                                                          int token_type);

template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id, float* field,
                                   float* other, int token_type) {
  ReadNumber(field);
  return 0;
}
template const char* TextprotoReader::Visit<TYPE_FLOAT>(
    const char* ptr, int field_id, float* field, float* other, int token_type);

template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id, double* field,
                                   double* other, int token_type) {
  ReadNumber(field);
  return 0;
}
template const char* TextprotoReader::Visit<TYPE_DOUBLE>(const char* ptr,
                                                         int field_id,
                                                         double* field,
                                                         double* other,
                                                         int token_type);

template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   std::string* field, std::string* other,
                                   int token_type) {
  if constexpr (field_type == TYPE_BYTES) {
    if (token_type != '"' || !absl::Base64Unescape(token_, field)) {
      ptr_ = nullptr;
      return ptr_;
    }
  } else {
    if (token_type == '"') {
      *field = std::string(token_);
    } else {
      ptr_ = nullptr;
      return ptr_;
    }
  }
  return ptr_;
}
template const char* TextprotoReader::Visit<TYPE_STRING>(const char* ptr,
                                                         int field_id,
                                                         std::string* field,
                                                         std::string* other,
                                                         int token_type);
template const char* TextprotoReader::Visit<TYPE_BYTES>(const char* ptr,
                                                        int field_id,
                                                        std::string* field,
                                                        std::string* other,
                                                        int token_type);

template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   absl::string_view* field,
                                   absl::string_view* other, int token_type) {
  if (token_type == '"') {
    *field = token_;
  } else {
    ptr_ = nullptr;
  }
  return ptr_;
}
template const char* TextprotoReader::Visit<TYPE_STRING>(
    const char* ptr, int field_id, absl::string_view* field,
    absl::string_view* other, int token_type);
// Since a string_view can't hold unencoded bytes, don't implement
// TYPE_BYTES for string_view fields.  This will cause a linker error.

}  // namespace proto
}  // namespace imp
