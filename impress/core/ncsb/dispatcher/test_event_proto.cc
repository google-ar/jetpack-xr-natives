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

#include <iostream>
#include <unordered_map>
#include <utility>

#include "google/protobuf/any.pb.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "absl/strings/str_cat.h"
#include "core/common/hash.h"
#include "core/common/test_helpers.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/dispatcher/test_event.proto.imp.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_reader.h"
#include "core/proto/proto_writer.h"
#include "testing.h"

namespace imp {

namespace {
using ::testing::Eq;

class EventProtoTest : public testing::ViewFixture {};

// Main concept. create an "context" class and pass it to both
// AddListener and RemoveListener
struct HandlerContext {
  imp::View* view;
  std::map<int, Dispatcher::ScopedConnection> connection_map;
  int connection_guid = 0;
};

// Functor adds a connection every time operator() is called.
// returns a unique id (not the connection id)
class AddHandler {
 public:
  explicit AddHandler(HandlerContext* context) : context_(context) {}

  int operator()(google::protobuf::imp_proto::Any any_from_event) {
    Dispatcher& d = context_->view->GetDispatcher();
    Dispatcher::ScopedConnection sc =
        d.Connect(imp::Hash(any_from_event.type_url),
                  [&](const Event& event) { events_received_++; });
    context_->connection_map.insert(
        std::make_pair(context_->connection_guid, std::move(sc)));
    return context_->connection_guid++;
  }

  int events_received_ = 0;
  HandlerContext* context_;
};

// Destroys a connection every time the operator() is called with a valid
// connection guid.
class RemoveHandler {
 public:
  explicit RemoveHandler(HandlerContext* context) : context_(context) {}

  void operator()(int connection_id) {
    context_->connection_map.erase(
        context_->connection_map.find(connection_id));
  }

  HandlerContext* context_;
};

TEST_F(EventProtoTest, GeneratedTypeUrlMatches) {
  ::test::EventFromProto source_event;
  google::protobuf::imp_proto::Any any_from_event;
  source_event.ToAny(&any_from_event);

  EXPECT_THAT(::test::EventFromProto::kTypeUrl, Eq(any_from_event.type_url));
}

TEST_F(EventProtoTest, ConnectDisconnectFlowTest) {
  std::map<int, Dispatcher::ScopedConnection> connection_map;
  HandlerContext context_{view_};

  ::test::EventFromProto source_event;
  google::protobuf::imp_proto::Any any_from_event;
  source_event.ToAny(&any_from_event);

  AddHandler add_connection_functor(&context_);
  RemoveHandler remove_connection_functor(&context_);

  EXPECT_THAT(view_->GetDispatcher().GetHandlerCount(
                  imp::Hash(any_from_event.type_url)),
              Eq(0));

  int connection_id = add_connection_functor(any_from_event);
  EXPECT_THAT(view_->GetDispatcher().GetHandlerCount(
                  imp::Hash(any_from_event.type_url)),
              Eq(1));

  view_->GetDispatcher().Send(source_event);
  remove_connection_functor(connection_id);

  EXPECT_THAT(view_->GetDispatcher().GetHandlerCount(
                  imp::Hash(any_from_event.type_url)),
              Eq(0));
  EXPECT_THAT(add_connection_functor.events_received_, Eq(1));
}

TEST_F(EventProtoTest, ConnectToEventUsingAnyProto) {
  Dispatcher& d = view_->GetDispatcher();
  ::test::EventFromProto source_event;
  source_event.value = 12;
  google::protobuf::imp_proto::Any any_from_event;
  bool result_from_toany = source_event.ToAny(&any_from_event);

  // Uses type_url from an any to create connection to imp::Event
  int events_received = 0;
  auto c = d.Connect(
      imp::Hash(any_from_event.type_url), [&](const Event& event) mutable {
        EXPECT_EQ(static_cast<const test::EventFromProto&>(event).value,
                  source_event.value);
        ++events_received;
      });

  d.Send(source_event);

  EXPECT_THAT(result_from_toany, Eq(true));
  EXPECT_THAT(events_received, Eq(1));
}

TEST_F(EventProtoTest, PackEventToAnyAndUnpackIt) {
  ::test::EventFromProto source_event;
  source_event.value = -987;
  google::protobuf::imp_proto::Any any_from_event;
  ::test::EventFromProto event_from_any;

  bool result_from_toany = source_event.ToAny(&any_from_event);
  imp::proto::ParseMessage(any_from_event.value.Flatten(), &event_from_any);

  EXPECT_THAT(result_from_toany, Eq(true));
  EXPECT_THAT(event_from_any.value, Eq(source_event.value));
  EXPECT_THAT(any_from_event.type_url, Eq(source_event.kTypeUrl));
  EXPECT_THAT(Hash(any_from_event.type_url),
              Eq(EventTypeHelper::GetEventTypeHash<test::EventFromProto>()));
}

TEST_F(EventProtoTest, TestEventSending) {
  Dispatcher d;
  int32_t event_value = 0;
  auto c = d.Connect([&](const ::test::EventFromProto& event) mutable {
    event_value = event.value;
  });

  EXPECT_THAT(0, Eq(event_value));

  ::test::EventFromProto event_proto;
  event_proto.value = -987;
  d.Send(event_proto);

  EXPECT_THAT(event_proto.value, Eq(event_value));
}

TEST_F(EventProtoTest, EventTypeNameIsCorrect) {
  absl::string_view type_name = type_traits::kTypeName<test::EventFromProto>;
  EXPECT_EQ(type_name,
            absl::string_view("type.googleapis.com/test.EventFromProto"));
}

TEST_F(EventProtoTest, EventTypeNameMatchesTypeUrl) {
  absl::string_view type_name = type_traits::kTypeName<test::EventFromProto>;
  absl::string_view type_url = test::EventFromProto::kTypeUrl;
  EXPECT_EQ(type_name, type_url);
}

TEST_F(EventProtoTest, EventTypeNameMemoryAddressIsConstant) {
  absl::string_view type_name_1 = type_traits::kTypeName<test::EventFromProto>;
  absl::string_view type_name_2 = type_traits::kTypeName<test::EventFromProto>;
  EXPECT_EQ(type_name_1.data(), type_name_2.data());
}

}  // namespace

}  // namespace imp
