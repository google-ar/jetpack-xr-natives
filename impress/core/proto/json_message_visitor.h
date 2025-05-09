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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_JSON_MESSAGE_VISITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_JSON_MESSAGE_VISITOR_H_

#include <functional>
#include <type_traits>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/hash.h"
#include "core/common/robin_map.h"
#include "core/common/type_traits.h"
#include "core/proto/proto_common.h"

namespace imp::proto {

// Forward declare JsonReader as JsonReader depends on this class.
class JsonReader;

// Visitor used to hook into the process of parsing a json when calling
// proto::ParseJson to modify & inspect the deserialized proto as it's
// deserialized.
//
// This makes it possible to write custom logic to apply when deserializing a
// proto.
class JsonMessageVisitor {
 public:
  // Registers a functor to be called before a message is deserialized when
  // parsing an impress proto.
  //
  // The functor the message type as the first parameter, along with the same
  // visitor that will be used on OnVisit and the position of the cursor.
  //
  // The functor will be called automatically for every message. This does not
  // work with primitives, only messages. It *will* be called for each instance
  // of the type within a map or a repeated field.
  //
  // The functor will not be called if the serialized data does not contain data
  // for the field (i.e. the field is assigned to the default value).
  //
  // Example:
  //
  //   proto::JsonMessageVisitor visitor;
  //   visitor.OnPreVisit([](NestedMessage& msg, proto::JsonReader& visitor,
  //     const char* cursor) {
  //     // Do Something
  //   });
  //   proto::ParseJson(&out_msg, &data, &visitor);
  template <typename Fn>
  void OnPreVisit(Fn fn);

  // Registers a functor to be called just before a field is deserialized when
  // parsing an impress proto.
  //
  // The functor the message type as the first parameter, and effectively
  // becomes the visitor of the message that you want to visit. If you want to
  // handle the visiting for that message, return true so the default visiting
  // logic will be skipped, otherwise return false.
  //
  // The functor will be called automatically for every field in the proto of
  // the message type. This does not work with primitives, only messages. It
  // *will* be called for each instance of the type within a map or a repeated
  // field.
  //
  // The functor will not be called if the serialized data does not contain data
  // for the field (i.e. the field is assigned to the default value).
  //
  // Registering a new functor will overwrite the previous one.
  //
  // Example:
  //
  //   proto::JsonMessageVisitor visitor;
  //   visitor.OnVisit([](NestedMessage& msg, int field_id, proto::JsonReader&
  //   visitor, const char* cursor, int token_type) {
  //     // Do Something
  //   });
  //   proto::ParseJson(&out_msg, &data, &visitor);
  template <typename Fn>
  void OnVisit(Fn fn);

  // Registers a functor to be called after a message is deserialized when
  // parsing an impress proto. Multiple functors for the same message type can
  // be registered.
  //
  // The functor should always take one parameter of the message type that you
  // want to visit.
  //
  // The functor will be called automatically for every message. This only works
  // with messages and vectors of messages. It *will* be called for each
  // instance of the type within a map or a repeated field.
  //
  // The functor will not be called if the serialized data does not contain data
  // for the field (i.e. the field is assigned to the default value).
  //
  // Example:
  //
  //   proto::JsonMessageVisitor visitor;
  //   visitor.OnPostVisit([](std::vector<NestedMessage>& messages) {
  //     // Do Something
  //   });
  //   proto::ParseJson(&out_msg, &data, &visitor);
  template <typename Fn>
  void OnPostVisit(Fn fn);

 private:
  // Called automatically during ParseJson. Takes a type-erased message. This
  // is called before message is read from JSON, which allows some preliminary
  // parsing of the message before the main Visit logic reaches it.
  absl::Status Poll(HashValue message_type_hash, void* erased_message,
                    void* erased_visitor, const char* cursor);

  // Called automatically during ParseJson. Takes a type-erased message. This
  // allows custom logic for visiting the equivalent json when the proto is
  // being deserialized.
  // Returns true if the message has been handled and the visitor has visited
  // the field indicated, false to let the default visitor logic to continue to
  // execute for that message, and a status if there's an issue visiting the
  // field.
  absl::StatusOr<bool> Handle(HashValue message_type_hash, void* erased_message,
                              int field_id, void* erased_visitor,
                              const char* cursor, int token_type);

  // Called automatically during ParseJson. Takes a type-erased message. This
  // is called after the message has been deserialized, and allow modification
  // of it if needed.
  void Accept(HashValue message_type_hash, void* erased_message);

  // Helper function declaration that is used to extract the type of message
  // parameter the functor passed into OnPreVisit requires.
  template <typename Fn, typename Visitor, typename Arg>
  static Arg GetPreVisitMessageTypeHelper(
      absl::Status (Fn::*)(Arg&, const Visitor& v, const char*) const);
  template <typename Fn, typename Visitor, typename Arg>
  static Arg GetPreVisitMessageTypeHelper(absl::Status (Fn::*)(Arg&,
                                                               const Visitor& v,
                                                               const char*));

  // Helper function declaration that is used to extract the type of visitor
  // parameter the functor passed into OnPreVisit requires.
  template <typename Fn, typename Visitor, typename Arg>
  static Visitor GetPreVisitVisitorTypeHelper(
      absl::Status (Fn::*)(Arg&, const Visitor& v, const char*) const);
  template <typename Fn, typename Visitor, typename Arg>
  static Visitor GetPreVisitVisitorTypeHelper(
      absl::Status (Fn::*)(Arg&, const Visitor& v, const char*));

  // Helper function declaration that is used to extract the type of message
  // parameter the functor passed into OnVisit requires.
  template <typename Fn, typename Visitor, typename Arg>
  static Arg GetVisitMessageTypeHelper(absl::StatusOr<bool> (Fn::*)(
      Arg&, int, Visitor& v, const char*, int) const);
  template <typename Fn, typename Visitor, typename Arg>
  static Arg GetVisitMessageTypeHelper(
      absl::StatusOr<bool> (Fn::*)(Arg&, int, Visitor& v, const char*, int));

  // Helper function declaration that is used to extract the type of visitor
  // the functor passed into OnVisit requires.
  template <typename Fn, typename Visitor, typename Arg>
  static Visitor GetVisitVisitorTypeHelper(absl::StatusOr<bool> (Fn::*)(
      Arg&, int, Visitor& v, const char*, int) const);
  template <typename Fn, typename Visitor, typename Arg>
  static Visitor GetVisitVisitorTypeHelper(
      absl::StatusOr<bool> (Fn::*)(Arg&, int, Visitor& v, const char*, int));

  // Helper function declaration that is used to extract the type of message
  // parameter the functor passed into OnPostVisit requires.
  template <typename Fn, typename Arg>
  static Arg GetPostVisitMessageTypeHelper(void (Fn::*)(const Arg&) const);
  template <typename Fn, typename Arg>
  static Arg GetPostVisitMessageTypeHelper(void (Fn::*)(const Arg&));

  // Helper function declaration that is used to extract the type of message
  // in the vector parameter the functor passed into OnPostVisit requires.
  template <typename Fn, typename Arg>
  static Arg GetPostVisitMessageListTypeHelper(
      void (Fn::*)(const std::vector<Arg>&) const);
  template <typename Fn, typename Arg>
  static Arg GetPostVisitMessageListTypeHelper(
      void (Fn::*)(const std::vector<Arg>&));

  RobinMap<
      HashValue,
      std::vector<std::function<absl::Status(void*, const void*, const char*)>>>
      pre_visit_functions_;
  RobinMap<HashValue, std::function<absl::StatusOr<bool>(void*, int, void*,
                                                         const char*, int)>>
      visit_functions_;
  RobinMap<HashValue, std::vector<std::function<void(void*)>>>
      post_visit_functions_;

  friend class JsonReader;
};

template <typename Fn>
void JsonMessageVisitor::OnPreVisit(Fn fn) {
  // Deduce the Message type from fn's function signature.
  using FnType = typename std::remove_reference<Fn>::type;
  using MessageType =
      decltype(GetPreVisitMessageTypeHelper(&FnType::operator()));
  using VisitorType =
      decltype(GetPreVisitVisitorTypeHelper(&FnType::operator()));

  // Wrap fn in a function that takes the type-erased version of the message and
  // converts it back into the concrete type. Register the wrapped fn based on
  // the type hash of MessageType.
  pre_visit_functions_[ProtoTypeUrlHash<MessageType>::value].push_back(
      [fn = std::move(fn)](void* erased_message, const void* erased_visitor,
                           const char* cursor) {
        MessageType* message = static_cast<MessageType*>(erased_message);
        const VisitorType* visitor =
            static_cast<const VisitorType*>(erased_visitor);
        return fn(*message, *visitor, cursor);
      });
}

template <typename Fn>
void JsonMessageVisitor::OnVisit(Fn fn) {
  // Deduce the Message type from fn's function signature.
  using FnType = typename std::remove_reference<Fn>::type;
  using MessageType = decltype(GetVisitMessageTypeHelper(&FnType::operator()));
  using VisitorType = decltype(GetVisitVisitorTypeHelper(&FnType::operator()));

  // Wrap fn in a function that takes the type-erased version of the message and
  // converts it back into the concrete type. Register the wrapped fn based on
  // the type hash of MessageType.
  visit_functions_[ProtoTypeUrlHash<MessageType>::value] =
      [fn = std::move(fn)](void* erased_message, int field_id,
                           void* erased_visitor, const char* cursor,
                           int token_type) {
        MessageType* message = static_cast<MessageType*>(erased_message);
        VisitorType* visitor = static_cast<VisitorType*>(erased_visitor);

        return fn(*message, field_id, *visitor, cursor, token_type);
      };
}

template <typename Fn>
void JsonMessageVisitor::OnPostVisit(Fn fn) {
  // Deduce the Message type from fn's function signature.
  using FnType = typename std::remove_reference<Fn>::type;
  using InputType =
      decltype(GetPostVisitMessageTypeHelper(&FnType::operator()));

  // Wrap fn in a function that takes the type-erased version of the message and
  // converts it back into the concrete type. Register the wrapped fn based on
  // the type hash of InputType.
  post_visit_functions_[ProtoTypeUrlHash<InputType>::value].push_back(
      [fn = std::move(fn)](void* erased_message) {
        // Checks if the parameter passed in is an std::vector
        if constexpr (type_traits::IsTemplateType<InputType,
                                                  std::vector>::value) {
          using MessageType =
              decltype(GetPostVisitMessageListTypeHelper(&FnType::operator()));

          const std::vector<MessageType>* messages =
              static_cast<std::vector<MessageType>*>(erased_message);
          fn(*messages);
        } else {
          const InputType* message = static_cast<InputType*>(erased_message);
          fn(*message);
        }
      });
}

}  // namespace imp::proto

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_JSON_MESSAGE_VISITOR_H_
