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

#include "core/scripting/multi_message_handler.h"

#include <algorithm>
#include <iterator>
#include <vector>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/scripting/base_message_handler.h"
#include "core/view/scripting/script_message_handler.h"

namespace imp {
namespace scripting {

Future<BaseMessageHandler::Response> MultiMessageHandler::HandleAnyMessage(
    const Any& message, const PlatformArgs& args) {
  auto iter = handler_forwarders_.find(message.type_url);
  if (iter == handler_forwarders_.end()) {
    IMP_LOG(imp::ERROR) << "Unable to handle message of type " << message.type_url
               << ", unsupported.";
    return Future<BaseMessageHandler::Response>();
  }

  if (!args.empty()) {
    IMP_LOG(imp::FATAL) << "MultiMessageHandler does not support platform args.";
  }

  auto& forwarder = iter->second;
  return forwarder->HandleAnyMessage(message, args);
}

std::vector<absl::string_view>
MultiMessageHandler::GetSupportedRequestTypeUrls() {
  std::vector<absl::string_view> result;
  std::transform(
      handler_forwarders_.begin(), handler_forwarders_.end(),
      std::back_inserter(result),
      [](const HandlersMap::value_type& pair) { return pair.first; });
  return result;
}

}  // namespace scripting
}  // namespace imp
