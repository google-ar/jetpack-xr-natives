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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_LOAD_MODEL_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_LOAD_MODEL_HANDLER_H_

#include "core/async/future.h"
#include "core/config.h"
#include "core/scripting/multi_message_handler.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/base_view.h"

#if IMP_PLATFORM(ANDROID)
#include "core/scripting/message_handler.h"
#include "core/view/scripting/script_message_handler.h"
#endif

namespace imp::scripting {

// A MessageHandler to load models and scenes and return them as nodes.
class LoadModelHandler : public MultiMessageHandler {
 public:
  explicit LoadModelHandler(BaseView& base_view) : view_(base_view) {
    AddHandler<LoadModelRequest, NodeHandle>(this);
    AddHandler<LoadSceneRequest, NodeHandle>(this);
  }

  Future<NodeHandle> HandleMessage(const LoadModelRequest& message);
  Future<NodeHandle> HandleMessage(const LoadSceneRequest& message);

 private:
  BaseView& view_;
};

#if IMP_PLATFORM(ANDROID)
// TODO: Merge this into LoadModelHandler.
// A MessageHandler to load models from an input stream.
class LoadModelFromInputStreamHandler
    : public MessageHandler<LoadModelFromInputStreamRequest, NodeHandle> {
 public:
  explicit LoadModelFromInputStreamHandler(BaseView& base_view)
      : view_(base_view) {}

  Future<NodeHandle> HandleMessage(
      const LoadModelFromInputStreamRequest& message) override;

  Future<NodeHandle> HandleMessage(
      const LoadModelFromInputStreamRequest& message,
      const PlatformArgs& args) override;

 private:
  BaseView& view_;
};
#endif

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_LOAD_MODEL_HANDLER_H_
