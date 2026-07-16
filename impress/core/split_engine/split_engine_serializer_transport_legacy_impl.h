// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_LEGACY_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_LEGACY_IMPL_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "core/common/invocable.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_bridge_sender.h"
#include "core/split_engine/split_engine_serializer_transport.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

// Legacy implementation of SplitEngineSerializerTransport that uses the
// SplitEngineAndroidBridge and SplitEngineBridgeSender.
class SplitEngineSerializerTransportLegacyImpl
    : public SplitEngineSerializerTransport {
 public:
  SplitEngineSerializerTransportLegacyImpl(
      /*absl_nullable*/  std::unique_ptr<SplitEngineAndroidBridge> bridge,
      /*absl_nonnull*/  std::unique_ptr<SplitEngineBridgeSender> bridge_sender);
  ~SplitEngineSerializerTransportLegacyImpl() override = default;

  imp::OwnedPtr<MessageBuilder> CreateBuilder(
      MessageGroupId message_group_id, size_t initial_size_bytes) override;

  absl::StatusOr<MessageGroupId> BeginFrameUpdate(
      size_t max_message_size_bytes) override;

  absl::StatusOr<MessageGroupId> BeginOneShot(
      size_t max_message_size_bytes) override;

  absl::Status AddMessage(
      MessageGroupId message_group_id, imp::OwnedPtr<MessageBuilder> builder,
      const flatbuffers::Offset<android_xr::schemas::Command>& offset) override;

  absl::Status AddMessage(MessageGroupId message_group_id,
                          imp::OwnedPtr<MessageBuilder> builder,
                          OffsetProducer offset_fn) override;

  absl::Status End(MessageGroupId message_group_id) override;

  absl::StatusOr<std::reference_wrapper<SplitEngineAndroidBridge>> GetBridge()
      override;

  void Schedule(imp::Invocable<absl::Status()> fn) override;

  absl::StatusOr<int32_t> GetActiveFrameUpdatesCount() const override;
  absl::StatusOr<int32_t> GetActiveOneShotsCount() const override;

  void ClearReleasedMessageGroups() override;

 private:
  // Hold the split engine bridge and ensures the lifetime of the bridge is the
  // lifetime of the serializer.
  // NOTE: it is critical that the bridge is destroyed after the senders are
  // destroyed, so the order of these fields is important.
  const /*absl_nullable*/  std::unique_ptr<SplitEngineAndroidBridge> bridge_;

  // The sender used for sending messages to the split engine renderer.
  const /*absl_nonnull*/  std::unique_ptr<SplitEngineBridgeSender> bridge_sender_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_LEGACY_IMPL_H_
