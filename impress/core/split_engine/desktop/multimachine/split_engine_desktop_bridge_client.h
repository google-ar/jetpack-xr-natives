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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_MM_BRIDGE_CLIENT_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_MM_BRIDGE_CLIENT_H_

#include <cstddef>
#include <cstdint>
#include <functional>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "core/common/invocable.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// To be used in Multi-machine Split Engine Desktop Environment.
class SplitEngineMMDesktopBridgeClient {
 public:
  virtual ~SplitEngineMMDesktopBridgeClient() = default;

  using MessageGroupSentCallback = std::function<void(MessageGroupId)>;
  virtual absl::Status SetOnMessageGroupSentCallback(
      MessageGroupSentCallback&& callback) = 0;

  virtual absl::Status SendRequest(
      absl::Span<const uint8_t> data,
      imp::Invocable<void(absl::Span<const uint8_t>)> callback) = 0;

  // `data` span shall be valid until MessageGroupProcessed callback is called.
  virtual absl::Status SendMessage(MessageGroupId message_group_id,
                                   size_t message_group_max_size_bytes,
                                   absl::Span<const uint8_t> data) = 0;

  virtual absl::Status EndMessageGroup(MessageGroupId message_group_id) = 0;

  virtual MessageGroupId GenerateMessageGroupId() = 0;
  virtual ClientId GetClientId() const = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_MM_BRIDGE_CLIENT_H_
