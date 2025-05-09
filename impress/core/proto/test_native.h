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
#include <utility>

namespace test {

struct NativeStruct {
  int32_t i32;
  uint64_t u64;
  float flt;
};

struct NativeNoCodegenStruct {
  int32_t i32 = -1;
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
