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

#ifndef THIRD_PARTY_IMPRESS_CORE_WEB_MESSAGE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_WEB_MESSAGE_HELPERS_H_

#include <cstddef>
#include <string>

#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_reader.h"
#include "core/proto/proto_writer.h"

namespace imp {
namespace scripting {

using google::protobuf::imp_proto::Any;

// Parses the array into a proto.
template <typename T>
bool ParseFromArray(const void* data, size_t len, T* message) {
  absl::string_view array(static_cast<const char*>(data), len);
  return proto::ParseMessage(array, message);
}

// Checks if the imp Any is of a specific proto type.
template <typename T>
bool Is(const Any& message) {
  return message.type_url.compare(T::kTypeUrl) == 0;
}

// Returns proto as a base-64 encoded string.
template <typename T>
std::string SerializeToBase64(T const& message) {
  std::string serialized;
  proto::SerializeTo(&message, &serialized);
  std::string encoded;
  absl::Base64Escape(serialized, &encoded);
  return encoded;
}

}  // namespace scripting
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_WEB_MESSAGE_HELPERS_H_
