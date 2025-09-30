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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_SERVICE_IMPL_PROXY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_SERVICE_IMPL_PROXY_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_service_impl_facade.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_bridge_sender.h"

namespace imp::split_engine {

// Proxy rendering use case:
// App (serializer) is running on the desktop
// Service is running on AXR device and acts as a proxy to the renderer.
class SplitEngineMMDesktopBridgeServiceImplProxy
    : public SplitEngineMMDesktopBridgeServiceImplFacade {
 public:
  // TODO: (broken link) - implement.
  absl::Status CreateBridge(BridgeId bridge_id,
                            MessageGroupCompletionHandler&&
                                message_group_completion_handler) override;

  absl::Status DestroyBridge(BridgeId bridge_id) override;

  absl::Status SendRequest(BridgeId bridge_id, absl::Span<const uint8_t> data,
                           ResponseHandler response_handler) override;

  absl::Status SendMessageGroupPart(
      BridgeId bridge_id, MessageGroupId group_id, size_t max_group_size_bytes,
      absl::Span<const uint8_t> partial_data) override;

  absl::Status CloseMessageGroup(BridgeId bridge_id,
                                 MessageGroupId group_id) override;

 private:
  std::unique_ptr<SplitEngineAndroidBridge> bridge_;
  std::unique_ptr<SplitEngineBridgeSender> bridge_sender_;
  std::unique_ptr<SplitEngineBridgeSender> one_shot_bridge_sender_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_SERVICE_IMPL_PROXY_H_
