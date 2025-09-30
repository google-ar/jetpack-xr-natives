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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_SERVICE_IMPL_LOCAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_SERVICE_IMPL_LOCAL_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/rememberer.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_service_impl_facade.h"
#include "core/split_engine/desktop/utils/buffer_factory.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"

namespace imp::split_engine {

// `Local` means that the service is running on the same machine as the service.
class SplitEngineMMDesktopBridgeServiceImplLocal
    : public SplitEngineMMDesktopBridgeServiceImplFacade,
      public Rememberer {
 public:
  SplitEngineMMDesktopBridgeServiceImplLocal(BaseView& view,
                                             Executor& executor);
  ~SplitEngineMMDesktopBridgeServiceImplLocal() override = default;

  using MessageGroupCompletionHandler =
      SplitEngineMMDesktopBridgeServiceImplFacade::
          MessageGroupCompletionHandler;
  using ResponseHandler =
      SplitEngineMMDesktopBridgeServiceImplFacade::ResponseHandler;

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
  // Total amount of memory that can be allocated for all pending message groups
  // for each bridge.
  static constexpr size_t kMemoryAllocationQuotaBytes =
      512 * 1024 * 1024;  // 512MB
  BaseView& view_;
  Executor& foreground_executor_;

  absl::Mutex bridge_status_mutex_;
  absl::flat_hash_map<BridgeId, absl::Status> bridge_status_
      ABSL_GUARDED_BY(bridge_status_mutex_);

  void SetBridgeStatus(BridgeId bridge_id, absl::Status status);
  absl::Status GetBridgeStatus(BridgeId bridge_id);

  absl::Mutex bridge_release_message_group_functions_mutex_;
  absl::flat_hash_map<BridgeId, MessageGroupCompletionHandler>
      bridge_release_message_group_functions_
          ABSL_GUARDED_BY(bridge_release_message_group_functions_mutex_);

  // Client sends next message group when the previous one was `sent`, but
  // not `delivered` to speed things up a little. This may cause a race: if next
  // message group is smaller, it can be delivered before the previous one.
  // In turn, there's a need to have an entity that executes message groups in
  // order and `SequentialPipeline` is such an entity.
  //
  // SequentialPipeline assumes that client sends message groups with
  // continuously increasing IDs without skipping any IDs.
  //
  // TODO: (broken link) - Review this approach once the task is done.
  class SequentialPipeline {
   public:
    SequentialPipeline(Executor& executor);
    ~SequentialPipeline();

    void Schedule(MessageGroupId group_id, std::function<void()> func);

   private:
    void MaybeExecuteNextMessageGroup();

    Executor& executor_;
    Future<absl::Status> task_;

    absl::Mutex mutex_;
    MessageGroupId next_message_group_id_ ABSL_GUARDED_BY(mutex_) = 1;
    absl::flat_hash_map<MessageGroupId, std::function<absl::Status()>>
        pending_message_groups_ ABSL_GUARDED_BY(mutex_);
  };

  class MessageGroupCompletionTracker {
   public:
    using CompletionHandler = std::function<void()>;
    MessageGroupCompletionTracker(size_t commands_count,
                                  CompletionHandler&& on_message_group_complete)
        : commands_count_(commands_count),
          on_message_group_complete_(std::move(on_message_group_complete)) {}
    MessageGroupCompletionTracker(const MessageGroupCompletionTracker&) =
        delete;
    MessageGroupCompletionTracker& operator=(
        const MessageGroupCompletionTracker&) = delete;
    MessageGroupCompletionTracker(MessageGroupCompletionTracker&&) = default;
    MessageGroupCompletionTracker& operator=(MessageGroupCompletionTracker&&) =
        default;

    void ReportCompletion();

   private:
    const size_t commands_count_;
    const CompletionHandler on_message_group_complete_;
    size_t completed_commands_count_ = 0;
  };

  struct MessageGroupData {
    std::unique_ptr<Buffer> buffer;
  };

  class BridgeData {
   public:
    BridgeData(size_t memory_allocation_quota_bytes,
               std::unique_ptr<SequentialPipeline> pipeline);

    // BridgeData is not copyable or movable because of absl::Mutex field.
    BridgeData(BridgeData&&) = delete;
    BridgeData& operator=(BridgeData&&) = delete;
    BridgeData(const BridgeData&) = delete;
    BridgeData& operator=(const BridgeData&) = delete;

    std::unique_ptr<BufferFactory> buffer_factory;
    absl::Mutex mutex;
    absl::flat_hash_map<MessageGroupId, MessageGroupData> message_groups
        ABSL_GUARDED_BY(mutex);
    std::unique_ptr<SequentialPipeline> pipeline_;
  };

  absl::Mutex bridge_data_mutex_;
  absl::flat_hash_map<BridgeId, std::unique_ptr<BridgeData>> bridge_data_
      ABSL_GUARDED_BY(bridge_data_mutex_);

  absl::StatusOr<std::reference_wrapper<BridgeData>> GetBridgeData(
      BridgeId bridge_id);

  // Returns number of commands in the buffer or error if the buffer is invalid.
  absl::StatusOr<size_t> VerifyBuffer(const Buffer& buffer,
                                      MessageGroupId group_id);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_SERVICE_IMPL_LOCAL_H_
