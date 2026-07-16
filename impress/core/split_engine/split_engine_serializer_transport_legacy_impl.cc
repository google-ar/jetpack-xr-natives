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

#include "core/split_engine/split_engine_serializer_transport_legacy_impl.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/invocable.h"
#include "core/common/owned_ptr.h"
#include "core/common/pass_key.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_bridge_sender.h"
#include "core/split_engine/split_engine_serializer_transport.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

namespace {
class Wrapper : public SplitEngineSerializerTransport::MessageBuilder {
 public:
  Wrapper(imp::OwnedPtr<flatbuffers::FlatBufferBuilder> fbb,
          imp::Invocable<PassKey<SplitEngineSerializerTransport>()> keygen)
      : MessageBuilder(keygen(), *fbb), fbb_(std::move(fbb)) {}

  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> ReleaseBuilder() {
    return std::move(fbb_);
  }

 private:
  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> fbb_;
};
}  // namespace

SplitEngineSerializerTransportLegacyImpl::
    SplitEngineSerializerTransportLegacyImpl(
        /*absl_nullable*/  std::unique_ptr<SplitEngineAndroidBridge> bridge,
        /*absl_nonnull*/  std::unique_ptr<SplitEngineBridgeSender> bridge_sender)
    : bridge_(std::move(bridge)), bridge_sender_(std::move(bridge_sender)) {}

imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder>
SplitEngineSerializerTransportLegacyImpl::CreateBuilder(
    MessageGroupId message_group_id, size_t initial_size_bytes) {
  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> fbb =
      bridge_sender_->CreateFlatBufferBuilder(message_group_id,
                                              initial_size_bytes);
  if (fbb) {
    return imp::OwnedPtr<MessageBuilder>(
        new Wrapper(std::move(fbb), [this]() { return GetPassKey(); }));
  } else {
    return nullptr;
  }
}

absl::StatusOr<MessageGroupId>
SplitEngineSerializerTransportLegacyImpl::BeginFrameUpdate(
    size_t max_message_size_bytes) {
  return bridge_sender_->BeginMessageGroup(
      max_message_size_bytes,
      SplitEngineBridgeSender::MessageType::kFrameUpdate);
}

absl::StatusOr<MessageGroupId>
SplitEngineSerializerTransportLegacyImpl::BeginOneShot(
    size_t max_message_size_bytes) {
  return bridge_sender_->BeginMessageGroup(
      max_message_size_bytes, SplitEngineBridgeSender::MessageType::kOneShot);
}

absl::Status SplitEngineSerializerTransportLegacyImpl::AddMessage(
    MessageGroupId message_group_id, imp::OwnedPtr<MessageBuilder> builder,
    const flatbuffers::Offset<android_xr::schemas::Command>& offset) {
  (*builder)->Finish(offset);

  auto wrapper =
      imp::OwnedPtr<Wrapper>(static_cast<Wrapper*>(builder.Release()));

  return bridge_sender_->SendMessage(message_group_id,
                                     wrapper->ReleaseBuilder());
}

absl::Status SplitEngineSerializerTransportLegacyImpl::AddMessage(
    MessageGroupId message_group_id, imp::OwnedPtr<MessageBuilder> builder,
    OffsetProducer offset_fn) {
  auto wrapper =
      imp::OwnedPtr<Wrapper>(static_cast<Wrapper*>(builder.Release()));

  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> fbb = wrapper->ReleaseBuilder();

  Schedule(
      [builder = fbb.Borrow(), offset_fn = std::move(offset_fn)]() mutable {
        builder->Finish(offset_fn(*builder));
        return absl::OkStatus();
      });

  return bridge_sender_->SendMessage(message_group_id, std::move(fbb));
}

absl::Status SplitEngineSerializerTransportLegacyImpl::End(
    MessageGroupId message_group_id) {
  return bridge_sender_->EndMessageGroup(message_group_id);
}

absl::StatusOr<std::reference_wrapper<SplitEngineAndroidBridge>>
SplitEngineSerializerTransportLegacyImpl::GetBridge() {
  if (!bridge_) {
    return absl::UnavailableError("Bridge is not available");
  }

  return *bridge_;
}

void SplitEngineSerializerTransportLegacyImpl::Schedule(
    imp::Invocable<absl::Status()> fn) {
  bridge_sender_->Schedule(std::move(fn));
}

absl::StatusOr<int32_t>
SplitEngineSerializerTransportLegacyImpl::GetActiveFrameUpdatesCount() const {
  return bridge_sender_->GetActiveMessageGroupCount(
      SplitEngineBridgeSender::MessageType::kFrameUpdate);
}

absl::StatusOr<int32_t>
SplitEngineSerializerTransportLegacyImpl::GetActiveOneShotsCount() const {
  return bridge_sender_->GetActiveMessageGroupCount(
      SplitEngineBridgeSender::MessageType::kOneShot);
}

void SplitEngineSerializerTransportLegacyImpl::ClearReleasedMessageGroups() {
  bridge_sender_->ClearReleasedMessageGroups();
}

}  // namespace imp::split_engine
