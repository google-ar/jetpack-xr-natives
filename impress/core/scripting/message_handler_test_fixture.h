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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_MESSAGE_HANDLER_TEST_FIXTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_MESSAGE_HANDLER_TEST_FIXTURE_H_

#include <memory>
#include <utility>

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/context.h"
#include "core/ncsb/node_handle.h"
#include "core/scripting/base_message_handler.h"
#include "core/scripting/scripting_system.h"
#include "testing/test_view.h"
#include "testing/view_fixture.h"

// TODO: Don't use the "testing" namespace.
namespace imp::scripting::testing {

using Response = ::imp::scripting::BaseMessageHandler::Response;

// Generic fixture for testing message handlers.
//
// Note: if a custom View is not needed, use MessageHandlerTestFixture instead.
//
// Concrete tests should inherit from this fixture and implement the
// CreateHandler() method to create the handler to be tested.
//
// The fixture also creates a scripting system and a MockWebView. The mock web
// view is used to test responses sent back to the script client.
//
template <typename TView, typename THandler>
class GenericMessageHandlerTestFixture
    : public imp::testing::GenericViewFixture<TView> {
 protected:
  GenericMessageHandlerTestFixture()
      : imp::testing::GenericViewFixture<TView>(std::make_unique<Context>()) {
    scripting_system_ = std::make_unique<ScriptingSystem>(*this->GetView());
  }

  explicit GenericMessageHandlerTestFixture(std::unique_ptr<Context> context)
      : imp::testing::GenericViewFixture<TView>(std::move(context)) {}

  void SetUp() override {
    imp::testing::ViewFixture::SetUp();
    handler_ = CreateHandler();
  }

  virtual std::unique_ptr<THandler> CreateHandler() = 0;

  NodeHandle GetInvalidNode() {
    return imp::NodeHandle(utils::Entity::import(1234));
  }

  template <typename T>
  void ExpectMessageWithInvalidNodeTargetCausesFailure(T& message_with_target) {
    message_with_target.target = GetInvalidNode();
    auto future =
        handler_->HandleAnyMessage(*proto::PackAny(message_with_target));
    EXPECT_FALSE(future.Get().ok());
  }

  std::unique_ptr<ScriptingSystem> scripting_system_;
  std::unique_ptr<THandler> handler_;
};

template <typename THandler>
using MessageHandlerTestFixture =
    GenericMessageHandlerTestFixture<imp::testing::TestView, THandler>;

}  // namespace imp::scripting::testing

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_MESSAGE_HANDLER_TEST_FIXTURE_H_
