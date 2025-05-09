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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_LOAD_FILE_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_LOAD_FILE_HANDLER_H_

#include "core/async/future.h"
#include "core/ncsb/node_handle.h"
#include "core/scripting/message_handler.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/base_view.h"

namespace imp::scripting {

// A MessageHandler for LoadFileHandler requests. This MessageHandler is only
// available in dev mode.
class LoadFileHandler : public MessageHandler<LoadFileRequest, absl::Status> {
 public:
  explicit LoadFileHandler(BaseView& base_view);

  Future<absl::Status> HandleMessage(const LoadFileRequest& message) override;

 private:
  BaseView& view_;
};

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_LOAD_FILE_HANDLER_H_
