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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_TRANSFORM_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_TRANSFORM_HANDLER_H_

#include "core/scripting/multi_message_handler.h"
#include "core/scripting/proto/api.proto.imp.h"

namespace imp::scripting {

// A MessageHandler for transform-related requests.
class TransformHandler : public MultiMessageHandler {
 public:
  TransformHandler() {
    AddHandler<GetTransformRequest, GetTransformResponse>(this);
    AddHandler<SetTransformRequest, absl::Status>(this);
    AddHandler<GetTransformMatrixRequest, GetTransformMatrixResponse>(this);
    AddHandler<SetTransformMatrixRequest, absl::Status>(this);
  }

  Future<GetTransformResponse> HandleMessage(
      const GetTransformRequest& message);
  Future<absl::Status> HandleMessage(const SetTransformRequest& message);
  Future<GetTransformMatrixResponse> HandleMessage(
      const GetTransformMatrixRequest& message);
  Future<absl::Status> HandleMessage(const SetTransformMatrixRequest& message);
};

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_TRANSFORM_HANDLER_H_
