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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_SENDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_SENDER_H_

#include "flatbuffers/flatbuffer_builder.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_sender_base.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_client.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// To be used in Multi-machine Split Engine Desktop Environment.
class SplitEngineMMDesktopBridgeSender
    : public SplitEngineSharedMemoryBridgeSenderBase {
 public:
  SplitEngineMMDesktopBridgeSender(SplitEngineMMDesktopBridgeClient& client,
                                   bool recycle_buffers);

  // TODO: (broken link) - remove this once (broken link) is merged.
  MessageGroupId GenerateMessageGroupId() override;

  ClientId GetClientId() const override;

  BufferHandleFactory& GetBufferHandleFactory() override;

  FlatbufferArenaAllocator& GetAllocator() override;

  void SendMessage(const flatbuffers::FlatBufferBuilder& message) override;
  void EndMessageGroup() override;

 private:
  SplitEngineMMDesktopBridgeClient& client_;
  SizePrefixedFlatbufferArenaAllocator arena_allocator_;
  BufferHandleFactory buffer_handle_factory_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_SENDER_H_
