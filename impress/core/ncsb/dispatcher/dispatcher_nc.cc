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

#include "core/ncsb/dispatcher/dispatcher.h"

// This file is named "nc" for "negative compilation" test and will fail to
// compile.

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "testing.h"

namespace imp {

// Events need to inherit from imp::Event.
struct InvalidEvent {
  int i;
};

struct ValidEvent : public Event {
  int i;
};

}  // namespace imp

namespace imp {

using ViewFixture = ::imp::testing::ViewFixture;

#ifdef TEST_SEND_INVALID_EVENT
TEST_F(ViewFixture, SendInvalidEvent) {
  view_->GetDispatcher().Send(InvalidEvent());
}
#endif

#ifdef TEST_WARN_UNUSED_RESULT
TEST_F(ViewFixture, WarnUnusedResult) {
  view_->GetDispatcher().Connect([](const ValidEvent&) {});
}
#endif

#ifdef TEST_CONNECT_INVALID_EVENT
TEST_F(ViewFixture, ConnectInvalidEvent) {
  auto c = view_->GetDispatcher().Connect([](const InvalidEvent&) {});
}
#endif

#ifdef TEST_OWNER_VALUE
class MyClass {};

TEST_F(ViewFixture, OwnerValue) {
  // ConnectionOwner can be any pointer, not values or references.
  MyClass c;
  view_->GetDispatcher().Connect([](const ValidEvent&) {}, c);
}
#endif

#ifdef TEST_OWNER_NOT_REMEMBERER
class MyClass {};

TEST_F(ViewFixture, OwnerNotRememberer) {
  // ConnectionOwner can be any pointer, not values or references.
  MyClass c;
  view_->GetDispatcher().Connect([](const ValidEvent&) {}, &c);
}
#endif

}  // namespace imp
