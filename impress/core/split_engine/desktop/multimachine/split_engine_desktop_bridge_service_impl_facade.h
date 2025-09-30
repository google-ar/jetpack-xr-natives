// Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_ROUTER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_ROUTER_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// "Facade" for actual implementation to support local and proxy use-cases.
class SplitEngineMMDesktopBridgeServiceImplFacade {
 public:
  using MessageGroupCompletionHandler = std::function<void(MessageGroupId)>;
  using ResponseHandler = std::function<void(const std::vector<uint8_t>&)>;
  virtual ~SplitEngineMMDesktopBridgeServiceImplFacade() = default;

  virtual absl::Status CreateBridge(
      BridgeId bridge_id,
      MessageGroupCompletionHandler&& message_group_completion_handler) = 0;

  virtual absl::Status DestroyBridge(BridgeId bridge_id) = 0;

  virtual absl::Status SendRequest(BridgeId bridge_id,
                                   absl::Span<const uint8_t> data,
                                   ResponseHandler response_handler) = 0;

  virtual absl::Status SendMessageGroupPart(
      BridgeId bridge_id, MessageGroupId group_id, size_t max_group_size_bytes,
      absl::Span<const uint8_t> partial_data) = 0;

  virtual absl::Status CloseMessageGroup(BridgeId bridge_id,
                                         MessageGroupId group_id) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_ROUTER_H_
