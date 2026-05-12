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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_TEST_NATIVE_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_TEST_NATIVE_H_

#include <cstdint>
#include <optional>
#include <utility>

#include "core/proto/proto_common.h"

namespace test {

struct NativeStruct {
  int32_t i32;
  uint64_t u64;
  float flt;
};

struct NativeNoCodegenStruct {
  int32_t i32 = -1;
};

class NativeWithPresenceNoCodegen {
 public:
  explicit operator bool() const { return i32_.has_value(); }

  int32_t GetValue() const { return *i32_; }

  void SetValue(int32_t value) { i32_.emplace(value); }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor cursor, NativeWithPresenceNoCodegen* other,
               Args... args) {
    if (!i32_) {
      return cursor;
    }

    int32_t* other_i32 = nullptr;
    if (other && other->i32_) {
      other_i32 = &other->i32_.value();
    }

    return v.template Visit<imp::proto::TYPE_INT32>(
        cursor, 1, &i32_.value(), other_i32, std::forward<Args>(args)...);
  }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor cursor,
                    NativeWithPresenceNoCodegen* other, Args... args) {
    if (!i32_) {
      i32_.emplace();
    }

    int32_t* other_i32 = nullptr;
    if (other && other->i32_) {
      other_i32 = &other->i32_.value();
    }

    return v.template Visit<imp::proto::TYPE_INT32>(
        cursor, 1, &i32_.value(), other_i32, std::forward<Args>(args)...);
  }

 private:
  std::optional<int32_t> i32_;
};

template <typename T>
struct TemplatedNativeStruct {
  template <typename Visitor, typename Cursor>
  Cursor Visit(Visitor* v, Cursor cursor,
               const TemplatedNativeStruct* other) const {
    return cursor;
  }
  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor* v, Cursor cursor,
                    const TemplatedNativeStruct* other, Args... args) {
    return v->Unknown(cursor, std::forward<Args>(args)...);
  }
};

}  // namespace test

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_TEST_NATIVE_H_
