/*
 * Copyright 2026 Google LLC
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

#include "core/scripting/message_handlers/set_view_config_handler.h"

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/base_view.h"

namespace imp::scripting {

SetViewConfigHandler::SetViewConfigHandler(BaseView& base_view)
    : view_(base_view) {}

Future<absl::Status> SetViewConfigHandler::HandleMessage(
    const SetViewConfigRequest& message) {
  view_.SetConfig(message.config);
  return Future<absl::Status>(absl::OkStatus());
}

}  // namespace imp::scripting
