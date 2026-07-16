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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_SHMEM_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_SHMEM_H_

#include <cstdint>
#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "core/async/background_scheduler.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/transport/basic_transport.h"
#include "core/split_engine/transport/concepts/concepts.h"
#include "core/split_engine/transport/concepts/shared_memory_transport.h"
#include "core/split_engine/transport/shared_memory_region_allocator.h"
#include "core/split_engine/transport/transport.h"
#include "core/split_engine/transport/transport_session_memory_manager_impl.h"

namespace imp::split_engine {

// BasicSharedMemoryTransport is a transport that uses shared memory regions to
// send data between processes.
//
// BasicSharedMemoryTransport specializes BasicTransport to use
// SharedMemoryRegionAllocator as the arena allocator.
//
// BasicSharedMemoryTransport has more knowledge than BasicTransport about the
// underlying transport and uses that knowledge to send messages and requests
// using legacy approach:
// - Messages are allocated from session memory blocks and sent using
// ProcessRegion.
// - Requests are allocated from the heap and sent using SendRequest.
//
// Use cases:
// - Split Engine on Android using SplitEngineBridge.
// - Split Engine on Desktop using gRPC in Single Machine mode.
template <CONCEPT(SharedMemoryTransport) TSharedMemoryTransport>
class BasicSharedMemoryTransport
    : public BasicTransport<TSharedMemoryTransport> {
 public:
  using Base = BasicTransport<TSharedMemoryTransport>;
  using BufferHead = Base::BufferHead;
  using SessionID = Base::SessionID;

  BasicSharedMemoryTransport(
      std::unique_ptr<TSharedMemoryTransport> underlying_transport,
      std::unique_ptr<imp::ArenaAllocator> arena_allocator,
      imp::BorrowedPtr<SharedMemoryRegionAllocator> region_allocator,
      imp::BorrowedPtr<BackgroundScheduler> scheduler)
      : Base(
            std::move(underlying_transport),
            std::make_unique<TransportSessionMemoryManagerImpl>(
                std::move(arena_allocator), region_allocator.WithNewLocation()),
            scheduler.WithNewLocation()),
        scheduler_(scheduler.WithNewLocation()),
        region_allocator_(region_allocator.WithNewLocation()) {}

 private:
  absl::Status SendMessageImpl(SessionID session_id, BufferHead buffer_head,
                               absl::Span<const uint8_t> data,
                               Transport::MessageCallback callback) override {
    if (session_id == Transport::kPermanentSessionID) {
      // Requests are going through the permanent session, because they are
      // allocated from the heap.
      return this->underlying_transport_->SendRequest(data,
                                                      std::move(callback));
    } else {
      // Messages are going through dynamic sessions, because they are allocated
      // from the session memory blocks.
      return this->underlying_transport_->ProcessRegion(
          this->region_allocator_->GetBufferHandle(buffer_head),
          data.data() - buffer_head, data.size());
    }
  }

  const imp::BorrowedPtr<BackgroundScheduler> scheduler_;
  const imp::BorrowedPtr<SharedMemoryRegionAllocator> region_allocator_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_SHMEM_H_
