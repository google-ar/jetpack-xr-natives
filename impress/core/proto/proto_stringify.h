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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_STRINGIFY_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_STRINGIFY_H_

#include "core/config.h"
#if IMP_RUNTIME(DEV)
#include "core/proto/textproto_writer.h"
#endif

namespace imp {
namespace proto {

// Templated version of AbslStringify for Impress protos. Generated code for
// Impress protos will call this implementation and either print out the
// textproto serialization or the type url depending on whether or not the dev
// runtime is enabled.
//
// This is an implementation detail of the generated code and should not be used
// directly. Instead just use AbslStringify (via LOG, absl::StrCat, etc).
template <typename Sink, typename T>
void AbslStringifyProto(Sink &sink, const T &message) {
#if IMP_RUNTIME(DEV)
  std::string result;

  // ToTextproto expects a non-const pointer, but it never actually modifies the
  // data.
  T *non_const_proto = const_cast<T *>(&message);
  imp::proto::ToTextproto(non_const_proto, &result);

  sink.Append(result);
#else
  sink.Append(T::kTypeUrl);
#endif
}

}  // namespace proto
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_STRINGIFY_H_
