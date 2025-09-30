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

#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_service_impl_proxy.h"

#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

absl::Status SplitEngineMMDesktopBridgeServiceImplProxy::CreateBridge(
    BridgeId bridge_id,
    MessageGroupCompletionHandler&& message_group_completion_handler) {
  return absl::UnimplementedError("Not implemented");
}

absl::Status SplitEngineMMDesktopBridgeServiceImplProxy::DestroyBridge(
    BridgeId bridge_id) {
  return absl::UnimplementedError("Not implemented");
}

absl::Status SplitEngineMMDesktopBridgeServiceImplProxy::SendRequest(
    BridgeId bridge_id, absl::Span<const uint8_t> data,
    ResponseHandler response_handler) {
  return absl::UnimplementedError("Not implemented");
}

absl::Status SplitEngineMMDesktopBridgeServiceImplProxy::SendMessageGroupPart(
    BridgeId bridge_id, MessageGroupId group_id, size_t max_group_size_bytes,
    absl::Span<const uint8_t> partial_data) {
  return absl::UnimplementedError("Not implemented");
}

absl::Status SplitEngineMMDesktopBridgeServiceImplProxy::CloseMessageGroup(
    BridgeId bridge_id, MessageGroupId group_id) {
  return absl::UnimplementedError("Not implemented");
}

}  // namespace imp::split_engine
