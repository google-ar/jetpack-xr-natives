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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_ADD_OR_UPDATE_COMPONENT_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_ADD_OR_UPDATE_COMPONENT_HANDLER_H_

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/scripting/multi_message_handler.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/base_view.h"

namespace imp::scripting {

// A MessageHandler to add any registered Component to a node.
class AddOrUpdateComponentHandler : public MultiMessageHandler {
 public:
  explicit AddOrUpdateComponentHandler(BaseView& view) : view_(view) {
    AddHandler<AddComponentRequest, absl::Status>(this);
    AddHandler<UpdateComponentRequest, absl::Status>(this);
  }

  Future<absl::Status> HandleMessage(const AddComponentRequest& message);
  Future<absl::Status> HandleMessage(const UpdateComponentRequest& message);

 private:
  BaseView& view_;
};

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_ADD_OR_UPDATE_COMPONENT_HANDLER_H_
