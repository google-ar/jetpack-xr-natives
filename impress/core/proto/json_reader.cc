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

#include "core/proto/json_reader.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/ascii.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_cat.h"
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

StringMap<JsonReader::VisitRegisteredAnyFn>*
JsonReader::GetVisitRegisteredFnMap() {
  // It's best to store static data structures as a static pointer inside of
  // a static function, per (broken link) and (broken link).
  static StringMap<JsonReader::VisitRegisteredAnyFn>* visit_registered_fn_map =
      new StringMap<JsonReader::VisitRegisteredAnyFn>();
  return visit_registered_fn_map;
}

void JsonReader::ClearRegisteredTypes() {
  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  map->clear();
}

const char* JsonReader::Consume(const char* ptr, const char* end,
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

const char* JsonReader::SkipWhitespace(const char* ptr) {
  while (ptr && ptr < end_ && absl::ascii_isspace(*ptr)) {
    if (*ptr == '\n') {
      ++line_number_;
      line_start_ = ptr + 1;
    }
    ++ptr;
  }
  return ptr;
}

const char* JsonReader::NextToken(const char* ptr, int* token) {
  ptr = SkipWhitespace(ptr);
  if (!ptr || ptr >= end_) {
    *token = 0;
    return end_;
  }
  token_line_number_ = line_number_;
  if (*ptr == '{' || *ptr == '}' || *ptr == '[' || *ptr == ']' || *ptr == ':' ||
      *ptr == ',') {
    *token = *ptr;
    token_ = absl::string_view();
    token_line_pos_ = (ptr - line_start_) + 1;
    return SkipWhitespace(ptr + 1);
  } else if (*ptr == '"') {
    *token = '"';
    token_line_pos_ = (ptr - line_start_) + 1;
    auto end = FindStringEnd(ptr, end_);
    if (end < end_) {
      token_ = absl::string_view(ptr + 1, end - ptr - 1);
      return SkipWhitespace(end + 1);
    }
    token_ = absl::string_view();
    return end_;
  } else {
    auto end = std::find_if(ptr, end_, [](char c) {
      return absl::ascii_isspace(c) || c == ',' || c == '}' || c == ']';
    });
    *token = '*';
    token_ = absl::string_view(ptr, end - ptr);
    token_line_pos_ = (ptr - line_start_) + 1;
    return SkipWhitespace(end);
  }
}

const char* JsonReader::ExpectToken(const char* ptr, int expect) {
  int type;
  ptr = NextToken(ptr, &type);
  if (type == expect) {
    return ptr;
  }
  SetTokenError(expect);
  return nullptr;
}

const char* JsonReader::Unknown(const char* ptr, int field_id, int token_type) {
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
const char* JsonReader::Visit(const char* ptr, int field_id, bool* field,
                              bool* other, int token_type) {
  static_assert(field_type == TYPE_BOOL);
  if (!status_.ok()) {
    return ptr_;
  }
  if (token_ == "true") {
    *field = true;
  } else if (token_ == "false") {
    *field = false;
  } else {
    SetBooleanError();
    ptr_ = nullptr;
  }
  return ptr_;
}

template const char* JsonReader::Visit<TYPE_BOOL>(const char* ptr, int field_id,
                                                  bool* field, bool* other,
                                                  int token_type);

template <int field_type>
const char* JsonReader::Visit(const char* ptr, int field_id, int32_t* field,
                              int32_t* other, int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  ReadNumber(field);
  return ptr_;
}
template const char* JsonReader::Visit<TYPE_INT32>(const char* ptr,
                                                   int field_id, int32_t* field,
                                                   int32_t* other,
                                                   int token_type);
template const char* JsonReader::Visit<TYPE_SINT32>(const char* ptr,
                                                    int field_id,
                                                    int32_t* field,
                                                    int32_t* other,
                                                    int token_type);
template const char* JsonReader::Visit<TYPE_SFIXED32>(const char* ptr,
                                                      int field_id,
                                                      int32_t* field,
                                                      int32_t* other,
                                                      int token_type);
template const char* JsonReader::Visit<TYPE_UINT64>(const char* ptr,
                                                    int field_id,
                                                    int32_t* field,
                                                    int32_t* other,
                                                    int token_type);

template <int field_type>
const char* JsonReader::Visit(const char* ptr, int field_id, uint32_t* field,
                              uint32_t* other, int token_type) {
  ReadNumber(field);
  return ptr_;
}
template const char* JsonReader::Visit<TYPE_UINT32>(const char* ptr,
                                                    int field_id,
                                                    uint32_t* field,
                                                    uint32_t* other,
                                                    int token_type);
template const char* JsonReader::Visit<TYPE_FIXED32>(const char* ptr,
                                                     int field_id,
                                                     uint32_t* field,
                                                     uint32_t* other,
                                                     int token_type);

template <int field_type>
const char* JsonReader::Visit(const char* ptr, int field_id, int64_t* field,
                              int64_t* other, int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  ReadNumber(field);
  return ptr_;
}
template const char* JsonReader::Visit<TYPE_INT64>(const char* ptr,
                                                   int field_id, int64_t* field,
                                                   int64_t* other,
                                                   int token_type);
template const char* JsonReader::Visit<TYPE_SINT64>(const char* ptr,
                                                    int field_id,
                                                    int64_t* field,
                                                    int64_t* other,
                                                    int token_type);
template const char* JsonReader::Visit<TYPE_SFIXED64>(const char* ptr,
                                                      int field_id,
                                                      int64_t* field,
                                                      int64_t* other,
                                                      int token_type);

template <int field_type>
const char* JsonReader::Visit(const char* ptr, int field_id, uint64_t* field,
                              uint64_t* other, int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  ReadNumber(field);
  return ptr_;
}
template const char* JsonReader::Visit<TYPE_UINT64>(const char* ptr,
                                                    int field_id,
                                                    uint64_t* field,
                                                    uint64_t* other,
                                                    int token_type);
template const char* JsonReader::Visit<TYPE_FIXED64>(const char* ptr,
                                                     int field_id,
                                                     uint64_t* field,
                                                     uint64_t* other,
                                                     int token_type);

template <int field_type>
const char* JsonReader::Visit(const char* ptr, int field_id, float* field,
                              float* other, int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  ReadNumber(field);
  return ptr_;
}
template const char* JsonReader::Visit<TYPE_FLOAT>(const char* ptr,
                                                   int field_id, float* field,
                                                   float* other,
                                                   int token_type);

template <int field_type>
const char* JsonReader::Visit(const char* ptr, int field_id, double* field,
                              double* other, int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  ReadNumber(field);
  return ptr_;
}
template const char* JsonReader::Visit<TYPE_DOUBLE>(const char* ptr,
                                                    int field_id, double* field,
                                                    double* other,
                                                    int token_type);

template <int field_type>
const char* JsonReader::Visit(const char* ptr, int field_id, std::string* field,
                              std::string* other, int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  if constexpr (field_type == TYPE_BYTES) {
    if (token_type != '"' || !absl::Base64Unescape(token_, field)) {
      SetBase64StringError();
      ptr_ = nullptr;
    }
  } else {
    if (token_type == '"') {
      *field = std::string(token_);
    } else {
      SetStringError();
      ptr_ = nullptr;
    }
  }
  return ptr_;
}
template const char* JsonReader::Visit<TYPE_STRING>(const char* ptr,
                                                    int field_id,
                                                    std::string* field,
                                                    std::string* other,
                                                    int token_type);
template const char* JsonReader::Visit<TYPE_BYTES>(const char* ptr,
                                                   int field_id,
                                                   std::string* field,
                                                   std::string* other,
                                                   int token_type);

template <int field_type>
const char* JsonReader::Visit(const char* ptr, int field_id,
                              absl::string_view* field,
                              absl::string_view* other, int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  if (token_type == '"') {
    *field = token_;
  } else {
    SetStringError();
    ptr_ = nullptr;
  }
  return ptr_;
}
template const char* JsonReader::Visit<TYPE_STRING>(const char* ptr,
                                                    int field_id,
                                                    absl::string_view* field,
                                                    absl::string_view* other,
                                                    int token_type);
// Since a string_view can't hold unencoded bytes, don't implement
// TYPE_BYTES for string_view fields.  This will cause a linker error.

void JsonReader::SetNumberError() {
  status_.Update(absl::InvalidArgumentError(absl::StrCat(
      "Failed to parse number field with value '", token_,
      "' at position: ", token_line_number_, ":", token_line_pos_)));
}

void JsonReader::SetMessageError() {
  status_.Update(absl::InvalidArgumentError(absl::StrCat(
      "Expected '{' but got '", token_,
      "' instead at position: ", token_line_number_, ":", token_line_pos_)));
}

void JsonReader::SetMessageOrLiteralError() {
  status_.Update(absl::InvalidArgumentError(absl::StrCat(
      "Expected an object or number but got '", token_,
      "' instead at position: ", token_line_number_, ":", token_line_pos_)));
}

void JsonReader::SetTokenError(char expected) {
  SetTokenError(expected, token_);
}

void JsonReader::SetTokenError(char expected, std::string_view actual) {
  status_.Update(absl::InvalidArgumentError(absl::StrCat(
      "Expected token '", absl::string_view(&expected, 1), "' but got '",
      actual, "' instead at position: ", token_line_number_, ":",
      token_line_pos_)));
}

void JsonReader::SetStringKeyError() {
  status_.Update(absl::InvalidArgumentError(absl::StrCat(
      "Expected string key but got '", token_,
      "' instead at position: ", token_line_number_, ":", token_line_pos_)));
}

void JsonReader::SetNumberKeyError() {
  status_.Update(absl::InvalidArgumentError(absl::StrCat(
      "Expected number key but got '", token_,
      "' instead at position: ", token_line_number_, ":", token_line_pos_)));
}

void JsonReader::SetChordError() {
  status_.Update(absl::InvalidArgumentError(
      "Unable to process Cord field of type other than TYPE_BYTES"));
}

void JsonReader::SetStringError() {
  status_.Update(absl::InvalidArgumentError(absl::StrCat(
      "Expected string but got '", token_,
      "' instead at position: ", token_line_number_, ":", token_line_pos_)));
}

void JsonReader::SetBase64StringError() {
  status_.Update(absl::InvalidArgumentError(
      absl::StrCat("Invalid base64 string at position: ", token_line_number_,
                   ":", token_line_pos_)));
}

void JsonReader::SetNoHandlerError(const std::string& type_url, int line_number,
                                   ptrdiff_t line_pos) {
  status_.Update(absl::InvalidArgumentError(
      absl::StrCat("No handler for type: '", type_url,
                   "' at position: ", line_number, ":", line_pos)));
}

void JsonReader::SetBooleanError() {
  status_.Update(absl::InvalidArgumentError(absl::StrCat(
      "Expected boolean value but got '", token_,
      "' instead at position: ", token_line_number_, ":", token_line_pos_)));
}

}  // namespace proto
}  // namespace imp
