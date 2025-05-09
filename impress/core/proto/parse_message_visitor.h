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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_PARSE_MESSAGE_VISITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_PARSE_MESSAGE_VISITOR_H_

#include "core/common/hash.h"
#include "core/common/type_traits.h"
#include "core/proto/proto_common.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

namespace proto {

// Visitor used to hook into the process of parsing an impress proto when
// calling proto::ParseMessage to modify & inspect the deserialized proto as
// it's deserialized.
//
// This makes it possible to write custom logic to apply when deserializing a
// proto.
class ParseMessageVisitor {
 public:
  // Called automatically during ParseMessage. Takes a type-erased message.
  void Accept(HashValue message_type_hash, void* erased_message);

  // Registers a functor to be called just after a field is deserialized when
  // parsing an impress proto.
  //
  // The functor should always take one parameter of the message type that you
  // want to visit.
  //
  // The functor will be called automatically for every field in the proto of
  // the message type. This does not work with primitives, only messages. It
  // *will* be called for each instance of the type within a map or a repeated
  // field.
  //
  // The functor will not be called if the serialized data does not contain data
  // for the field (i.e. the field is assigned to the default value).
  //
  // Example:
  //
  //   proto::ParseMessageVisitor visitor;
  //   visitor.OnVisit([](NestedMessage& msg) {
  //     // Do Something
  //   });
  //   proto::ParseMessage(&out_msg, &data, &visitor);
  template <typename Fn>
  void OnVisit(Fn fn);

 private:
  // Helper function declaration that is used to extract the type of message
  // parameter the functor passed into OnVisit requires.
  template <typename Fn, typename Arg>
  static Arg GetMessageTypeHelper(void (Fn::*)(Arg&) const);

  // Mutable helper function declaration that is used to extract the type of
  // message parameter the functor passed into OnVisit requires.
  template <typename Fn, typename Arg>
  static Arg GetMessageTypeHelper(void (Fn::*)(Arg&));

  tsl::robin_map<HashValue, std::vector<std::function<void(void*)>>>
      visit_functions_;
};

template <typename Fn>
void ParseMessageVisitor::OnVisit(Fn fn) {
  // Deduce the Message type from fn's function signature.
  using FnType = typename std::remove_reference<Fn>::type;
  using MessageType = decltype(GetMessageTypeHelper(&FnType::operator()));

  // Wrap fn in a function that takes the type-erased version of the message and
  // converts it back into the concrete type. Register the wrapped fn based on
  // the type hash of MessageType.
  visit_functions_[ProtoTypeUrlHash<MessageType>::value].push_back(
      [fn = std::move(fn)](void* erased_message) {
        MessageType* message = static_cast<MessageType*>(erased_message);
        fn(*message);
      });
}

}  // namespace proto

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_PARSE_MESSAGE_VISITOR_H_
