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

#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_service_impl_local.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "flatbuffers/base.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/verifier.h"
#include "core/async/executor.h"
#include "core/common/enum_flags.h"
#include "core/common/registry.h"
#include "core/split_engine/desktop/utils/buffer_factory.h"
#include "core/split_engine/desktop/utils/buffer_factory_heap.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

namespace {
// Utility class to temporarily switch the active executor to schedule future.
class ExecutorSetup {
 public:
  explicit ExecutorSetup(Executor& executor, Executor::Type type)
      : type_(type) {
    switch (type) {
      case Executor::Type::kForeground:
        original_executor_ = Executor::ForegroundExecutor();
        Executor::SetForegroundExecutor(&executor);
        break;
      case Executor::Type::kBackground:
        original_executor_ = Executor::BackgroundExecutor();
        Executor::SetBackgroundExecutor(&executor);
        break;
      default:
        IMP_LOG(imp::FATAL) << "Unsupported executor type: " << static_cast<int>(type);
        break;
    }
  }

  ~ExecutorSetup() {
    switch (type_) {
      case Executor::Type::kForeground:
        Executor::SetForegroundExecutor(original_executor_);
        break;
      case Executor::Type::kBackground:
        Executor::SetBackgroundExecutor(original_executor_);
        break;
      default:
        IMP_LOG(imp::FATAL) << "Unsupported executor type: " << static_cast<int>(type_);
        break;
    }
  }

 private:
  const Executor::Type type_;
  Executor* original_executor_;
};
}  // namespace

SplitEngineMMDesktopBridgeServiceImplLocal::SequentialPipeline::
    SequentialPipeline(Executor& executor)
    : executor_(executor), task_(absl::OkStatus()) {
  // Constructor is called from the gRPC Thread. Foreground executor is thread
  // local static and expected to be null.
  
}

SplitEngineMMDesktopBridgeServiceImplLocal::SequentialPipeline::
    ~SequentialPipeline() {
  // Destructor is called from the executor
  

  // Clear pending message groups.
  {
    absl::MutexLock lock(mutex_);
    pending_message_groups_.clear();
  }

  // Cancel pending task if any
  task_.Cancel();
}

void SplitEngineMMDesktopBridgeServiceImplLocal::SequentialPipeline::Schedule(
    MessageGroupId group_id, std::function<void()> func) {
  // `Schedule` is called from the gRPC Thread. Foreground executor is thread
  // local static and expected to be null.
  

  // Create a wrapper for the function that will execute the `func` and
  // schedule the next message group if needed.

  auto execute_func = [this, func = std::move(func)]() {
    // `execute_func` is called from the executor.
    

    func();
    MaybeExecuteNextMessageGroup();

    return absl::OkStatus();
  };

  absl::MutexLock lock(mutex_);
  if (group_id == next_message_group_id_) {
    // Hijack ForegroundExecutor in this scope.
    ExecutorSetup setup(executor_, Executor::Type::kForeground);
    

    // The group is the one we expected, so schedule it right away.
    next_message_group_id_++;

    // `Then` will schedule the lambda on the ForegroundExecutor (`executor_`).
    task_ = task_.Then(std::move(execute_func));
  } else {
    // The group came early, save it for later execution.
    
  }

  // Ensure that we leave the foreground executor as it was before.
  
}

void SplitEngineMMDesktopBridgeServiceImplLocal::SequentialPipeline::
    MaybeExecuteNextMessageGroup() {
  // `MaybeExecuteNextMessageGroup` is called from the executor.
  

  absl::MutexLock lock(mutex_);
  if (pending_message_groups_.empty()) {
    return;
  }
  auto it = pending_message_groups_.find(next_message_group_id_);
  if (it == pending_message_groups_.end()) {
    // We have not received the next message group yet.
    // Nothing to do, exit early.
    return;
  }

  // We got the next group, schedule its execution and remove it from the
  // pending list.
  task_ = task_.Then(std::move(it->second));
  pending_message_groups_.erase(it);
  next_message_group_id_++;
}

SplitEngineMMDesktopBridgeServiceImplLocal::BridgeData::BridgeData(
    size_t memory_allocation_quota_bytes,
    std::unique_ptr<SequentialPipeline> pipeline)
    : buffer_factory(
          std::make_unique<HeapBufferFactory>(memory_allocation_quota_bytes)),
      pipeline_(std::move(pipeline)) {}

SplitEngineMMDesktopBridgeServiceImplLocal::
    SplitEngineMMDesktopBridgeServiceImplLocal(BaseView& view,
                                               Executor& executor)
    : view_(view), foreground_executor_(executor) {}

absl::Status SplitEngineMMDesktopBridgeServiceImplLocal::CreateBridge(
    BridgeId bridge_id,
    MessageGroupCompletionHandler&& message_group_completion_handler) {
  {
    // Create a release function for the bridge and ensure it is unique.
    absl::MutexLock lock(bridge_release_message_group_functions_mutex_);
    if (!bridge_release_message_group_functions_
             .emplace(bridge_id, std::move(message_group_completion_handler))
             .second) {
      return absl::AlreadyExistsError("Bridge already exists.");
    }
  }

  {
    absl::MutexLock lock(bridge_status_mutex_);
    bridge_status_.emplace(bridge_id, absl::OkStatus());
  }

  {
    absl::MutexLock lock(bridge_data_mutex_);
    bridge_data_.emplace(
        bridge_id,
        std::make_unique<BridgeData>(
            kMemoryAllocationQuotaBytes,
            std::make_unique<SequentialPipeline>(foreground_executor_)));
  }

  // Mimic the Android behavior of granting unrestricted system access to all
  // apps on the device.
  foreground_executor_.Schedule([this, bridge_id]() {
    view_.GetRegistry()
        .Get<imp::split_engine::SplitEngineRenderer>()
        ->get()
        .AddAppPermission(
            bridge_id,
            imp::ToFlags(AppPermissionTypes::kHasUnrestrictedSystemAccess));
  });

  return absl::OkStatus();
}

absl::Status SplitEngineMMDesktopBridgeServiceImplLocal::DestroyBridge(
    BridgeId bridge_id) {
  foreground_executor_.ScheduleInvocable([this, bridge_id]() {
    SplitEngineRenderer& renderer =
        view_.GetRegistry().Get<SplitEngineRenderer>()->get();
    if (const absl::Status status = renderer.ClearAppContext(bridge_id);
        !status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to clear app context: " << status;
    }

    {
      absl::MutexLock lock(bridge_release_message_group_functions_mutex_);
      bridge_release_message_group_functions_.erase(bridge_id);
    }

    {
      absl::MutexLock lock(bridge_status_mutex_);
      bridge_status_.erase(bridge_id);
    }

    {
      absl::MutexLock lock(bridge_data_mutex_);
      bridge_data_.erase(bridge_id);
    }
  });
  return absl::OkStatus();
}

namespace {
std::vector<uint8_t> SerializeStatusResponse(absl::Status status) {
  flatbuffers::FlatBufferBuilder fbb;
  if (!status.ok()) {
    return SerializeResponse(fbb, android_xr::schemas::CreateErrorResponse(
                                      fbb, StatusToErrorCode(status),
                                      fbb.CreateString(status.message())));
  } else {
    // Create an empty response to signal an OK status.
    return SerializeTable(fbb, android_xr::schemas::CreateResponse(fbb));
  }
}
}  // namespace

// TODO: (broken link) - Unify logic with Binder/Single Machine implementations.
absl::Status SplitEngineMMDesktopBridgeServiceImplLocal::SendRequest(
    BridgeId bridge_id, absl::Span<const uint8_t> data,
    ResponseHandler response_handler) {
  MP_RETURN_IF_ERROR(GetBridgeStatus(bridge_id));

  flatbuffers::Verifier verifier(data.data(), data.size());
  if (!verifier.VerifyBuffer<android_xr::schemas::Request>()) {
    return absl::InvalidArgumentError(
        "Invalid flatbuffer schema passed to SendRequest()");
  }

  foreground_executor_.Schedule(
      [this, bridge_id, data = std::vector<uint8_t>(data.begin(), data.end()),
       response_handler = std::move(response_handler)]() {
        const android_xr::schemas::Request* request =
            flatbuffers::GetRoot<android_xr::schemas::Request>(data.data());
        absl::StatusOr<std::reference_wrapper<SplitEngineRenderer>> renderer =
            view_.GetRegistry().Get<imp::split_engine::SplitEngineRenderer>();
        if (!renderer.ok()) {
          return absl::FailedPreconditionError(
              "SendRequest requires a SplitEngineRenderer in the registry.");
        }
        renderer->get()
            .HandleRequest(bridge_id, *request)
            .Then([response_handler =
                       std::move(response_handler)](absl::Status status) {
              response_handler(SerializeStatusResponse(status));
            })
            .KeptBy(this);
        return absl::OkStatus();
      });

  return absl::OkStatus();
}

absl::Status SplitEngineMMDesktopBridgeServiceImplLocal::SendMessageGroupPart(
    BridgeId bridge_id, MessageGroupId group_id, size_t max_group_size_bytes,
    absl::Span<const uint8_t> partial_data) {
  MP_RETURN_IF_ERROR(GetBridgeStatus(bridge_id));
  MP_ASSIGN_OR_RETURN(std::reference_wrapper<BridgeData> bridge_data,
                   GetBridgeData(bridge_id));
  BridgeData& bridge_data_ref = bridge_data.get();

  absl::MutexLock lock(bridge_data_ref.mutex);
  auto message_group_data_iterator =
      bridge_data_ref.message_groups.find(group_id);
  if (message_group_data_iterator == bridge_data_ref.message_groups.end()) {
    MP_ASSIGN_OR_RETURN(
        std::unique_ptr<Buffer> buffer,
        bridge_data_ref.buffer_factory->CreateBuffer(max_group_size_bytes));
    message_group_data_iterator =
        bridge_data_ref.message_groups.emplace(group_id, std::move(buffer))
            .first;
    if (message_group_data_iterator->second.buffer == nullptr) {
      return absl::InternalError(absl::StrCat(
          "Bridge: ", bridge_id, ". Failed to create buffer of size ",
          max_group_size_bytes, " for message group: ", group_id));
    }
  }

  return message_group_data_iterator->second.buffer->Write(partial_data);
}

namespace {
// Utility class to read flatbuffers from a buffer.
//
// "SizePrefixed" prefix comes from the fact that client send a flatbuffer
// prefixed with its size.
//
// MessageGroup is represented by one buffer with multiple flatbuffers.
// Each flatbuffer is prefixed with its size.
//
// The class reads those flatbuffers one by one.
class SizePrefixedBufferReader {
 public:
  SizePrefixedBufferReader(absl::Span<const uint8_t> data) : data_(data) {}

  explicit operator bool() const { return !data_.empty(); }

  // Checks if the next flatbuffer in the buffer is of type `T`.
  // Does not advance the reading position.
  template <typename T>
  absl::StatusOr<const T*> Peek() {
    MP_ASSIGN_OR_RETURN(const flatbuffers::uoffset_t size, GetPrefixedSize());
    const uint8_t* flatbuffer_data = data_.data() + sizeof(size);
    flatbuffers::Verifier verifier(flatbuffer_data, size);
    if (!verifier.VerifyBuffer<T>()) {
      return absl::InvalidArgumentError(
          "Flatbuffer schema verification failed.");
    }

    return flatbuffers::GetRoot<T>(flatbuffer_data);
  }

  // Verifies that the next flatbuffer in the buffer is of type `T`, reads it
  // and advances the reading position.
  template <typename T>
  absl::StatusOr<const T*> Read() {
    MP_ASSIGN_OR_RETURN(const T* result, Peek<T>());

    // `Peek` already verified the size of the buffer.
    const flatbuffers::uoffset_t size =
        flatbuffers::GetPrefixedSize(data_.data());

    data_ = absl::MakeSpan(data_.data() + size + sizeof(size),
                           data_.size() - size - sizeof(size));

    return result;
  }

  absl::StatusOr<flatbuffers::Verifier> GetVerifierWithoutPrefixedSize() const {
    MP_ASSIGN_OR_RETURN(const flatbuffers::uoffset_t size, GetPrefixedSize());
    return flatbuffers::Verifier(data_.data() + sizeof(size), size);
  }

 private:
  absl::StatusOr<flatbuffers::uoffset_t> GetPrefixedSize() const {
    if (data_.empty()) {
      return absl::ResourceExhaustedError("No data left to read.");
    }

    if (data_.size() < sizeof(flatbuffers::uoffset_t)) {
      return absl::InvalidArgumentError(
          "Data is too small to contain a flatbuffer.");
    }

    const flatbuffers::uoffset_t size =
        flatbuffers::GetPrefixedSize(data_.data());
    if (size == 0) {
      return absl::InvalidArgumentError("Got 0 size flatbuffer.");
    }
    if (size > data_.size() - sizeof(size)) {
      return absl::OutOfRangeError(
          absl::StrCat("Flatbuffer size ", size, " exceeds buffer size."));
    }

    return size;
  }

  absl::Span<const uint8_t> data_;
};
}  // namespace

// Prints the buffer to the log, used only in debug builds.
absl::Status PrintBuffer(const Buffer& buffer) {
  constexpr std::string_view kPrefix = "--";
  IMP_LOG(imp::INFO) << "Buffer: " << buffer.GetWrittenSize() << " bytes {";
  SizePrefixedBufferReader reader(
      absl::MakeConstSpan(buffer.Data(), buffer.GetWrittenSize()));
  while (reader) {
    if (const absl::StatusOr<const android_xr::schemas::MessageGroupOperation*>
            message_group_operation_status =
                reader.Read<android_xr::schemas::MessageGroupOperation>();
        message_group_operation_status.ok()) {
      const auto message_group_operation =
          message_group_operation_status.value();
      IMP_LOG(imp::INFO) << kPrefix << " Message group operation type ("
                 << EnumNameMessageGroupOperationTypes(
                        message_group_operation->operation_type())
                 << ", group id " << (int)message_group_operation->group_id();
    } else if (const absl::StatusOr<const android_xr::schemas::Command*>
                   command_status = reader.Read<android_xr::schemas::Command>();
               command_status.ok()) {
      IMP_LOG(imp::INFO) << kPrefix << kPrefix << " "
                 << EnumNameCommandTypes(
                        command_status.value()->command_type());
    } else {
      return absl::InvalidArgumentError(
          absl::StrCat("Message group contains invalid data. (Data size =",
                       buffer.GetWrittenSize(), ")"));
    }
  }
  IMP_LOG(imp::INFO) << "} // Buffer.";
  return absl::OkStatus();
}

// Returns number of commands in the buffer or error if the buffer is invalid.
absl::StatusOr<size_t> SplitEngineMMDesktopBridgeServiceImplLocal::VerifyBuffer(
    const Buffer& buffer, MessageGroupId group_id) {
  // The `PrintBuffer` will be called only in debug builds.
  

  SizePrefixedBufferReader reader(
      absl::MakeConstSpan(buffer.Data(), buffer.GetWrittenSize()));

  // The very first flatbuffer in the buffer must be MessageGroupOperation with
  // BeginMessageGroup operation type with the same group id as the one
  // provided.
  MP_ASSIGN_OR_RETURN(
      const android_xr::schemas::MessageGroupOperation* message_group_operation,
      reader.Read<android_xr::schemas::MessageGroupOperation>());

  if (message_group_operation->operation_type() !=
      android_xr::schemas::MessageGroupOperationTypes::BeginMessageGroup) {
    return absl::InvalidArgumentError(
        absl::StrCat("Message group operation type (",
                     message_group_operation->operation_type(), ", group id ",
                     message_group_operation->group_id(),
                     ") is not BeginMessageGroup of group id ", group_id));
  }
  if (message_group_operation->group_id() != group_id) {
    return absl::InvalidArgumentError(
        absl::StrCat("Message group id ", message_group_operation->group_id(),
                     " does not match expected id ", group_id));
  }

  size_t commands_count = 0;

  while (reader) {
    if (const absl::StatusOr<const android_xr::schemas::MessageGroupOperation*>
            message_group_operation_status =
                reader.Read<android_xr::schemas::MessageGroupOperation>();
        message_group_operation_status.ok()) {
      const android_xr::schemas::MessageGroupOperation*
          message_group_operation = message_group_operation_status.value();

      // Verify that the message group operation is EndMessageGroup of the
      // same group id.
      if (message_group_operation->operation_type() !=
          android_xr::schemas::MessageGroupOperationTypes::EndMessageGroup) {
        return absl::InvalidArgumentError(
            absl::StrCat("Message group operation type (",
                         message_group_operation->operation_type(),
                         ", group id ", message_group_operation->group_id(),
                         ") is not EndMessageGroup of group id ", group_id));
      }
      if (message_group_operation->group_id() != group_id) {
        return absl::InvalidArgumentError(absl::StrCat(
            "Message group id ", message_group_operation->group_id(),
            " does not match expected id ", group_id));
      }

      // Verify that `EndMessageGroup` is the last message in the buffer.
      if (reader) {
        return absl::InvalidArgumentError(
            "Message group operation is not the last message in the group.");
      }
    } else if (const absl::StatusOr<const android_xr::schemas::Command*>
                   command_status = reader.Read<android_xr::schemas::Command>();
               command_status.ok()) {
      commands_count++;
      continue;
    } else {
      return absl::InvalidArgumentError("Message group contains invalid data.");
    }
  }

  return commands_count;
}

// TODO: (broken link) - Unify logic with Binder/Single Machine implementations.
void SplitEngineMMDesktopBridgeServiceImplLocal::SetBridgeStatus(
    BridgeId bridge_id, absl::Status status) {
  {
    absl::MutexLock lock(bridge_status_mutex_);

    // The bridge may have already been shut down.
    if (!bridge_status_.contains(bridge_id)) {
      return;
    }
    // If the bridge is already in an error state, don't update the status.
    absl::Status& current_status = bridge_status_.at(bridge_id);
    if (!current_status.ok()) return;
    current_status = status;
  }
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Bridge error for ID " << bridge_id
               << " - no further messages will be processed: "
               << status.ToString();
  }
}

absl::Status SplitEngineMMDesktopBridgeServiceImplLocal::GetBridgeStatus(
    BridgeId bridge_id) {
  absl::MutexLock lock(bridge_status_mutex_);
  auto it = bridge_status_.find(bridge_id);
  if (it == bridge_status_.end()) {
    return absl::NotFoundError(absl::StrCat("Bridge not found: ", bridge_id));
  }
  return it->second;
}

absl::Status SplitEngineMMDesktopBridgeServiceImplLocal::CloseMessageGroup(
    BridgeId bridge_id, MessageGroupId group_id) {
  MP_RETURN_IF_ERROR(GetBridgeStatus(bridge_id));
  MP_ASSIGN_OR_RETURN(std::reference_wrapper<BridgeData> bridge_data,
                   GetBridgeData(bridge_id));
  BridgeData& bridge_data_ref = bridge_data.get();

  std::shared_ptr<Buffer> buffer;
  std::shared_ptr<MessageGroupCompletionTracker> tracker;
  {
    absl::MutexLock lock(bridge_data_ref.mutex);
    const auto message_group_iterator =
        bridge_data_ref.message_groups.find(group_id);
    if (message_group_iterator == bridge_data_ref.message_groups.end()) {
      return absl::NotFoundError(absl::StrCat(
          "Bridge: ", bridge_id, ". Message group not found: ", group_id));
    }

    // MessageGroup's buffer is now `shared_ptr` and will be moved to the
    // foreground executor and later copied to every command handler's
    // on_finished callback in MessageGroup to make sure that data stays alive
    // until all commands are processed.
    buffer = std::move(message_group_iterator->second.buffer);

    MP_ASSIGN_OR_RETURN(size_t commands_count, VerifyBuffer(*buffer, group_id));

    // We no longer need the message group data.
    bridge_data_ref.message_groups.erase(message_group_iterator);

    // Lifecycle of the message group completion tracker:
    //  1. Moved to the foreground executor
    //  2. Copied to every command handler's on_finished callback in
    //  MessageGroup.
    //  3. Deleted after the last command is processed.
    tracker = std::make_shared<MessageGroupCompletionTracker>(
        commands_count, [this, bridge_id, group_id]() {
          IMP_LOG(imp::INFO) << "Message group " << group_id
                     << " was processed by Impress.";
          {
            absl::MutexLock lock(bridge_release_message_group_functions_mutex_);
            auto it = bridge_release_message_group_functions_.find(bridge_id);
            if (it == bridge_release_message_group_functions_.end()) {
              IMP_LOG(imp::ERROR) << "Bridge " << bridge_id
                         << " not found while trying to release message group "
                         << group_id;
              return;
            }
            it->second(group_id);
          }
        });
  }

  // Schedule processing of the message group.
  bridge_data_ref.pipeline_->Schedule(
      group_id,
      [this,  // Access to `view_`,
              // `SetBridgeStatus`,
              // `GetBridgeStatus`
       bridge_id, group_id, buffer = std::move(buffer),
       tracker = std::move(tracker)]() {
        if (!GetBridgeStatus(bridge_id).ok()) {
          IMP_LOG(imp::ERROR) << "Bridge " << bridge_id << " is not found.";
          return;
        }

        SizePrefixedBufferReader reader(
            absl::MakeConstSpan(buffer->Data(), buffer->GetWrittenSize()));

        // `VerifyBuffer` already verified that the first flatbuffer is
        // MessageGroupOperation and it's valid BeginMessageGroup. So, we are
        // skipping it.
        if (const auto status =
                reader.Read<android_xr::schemas::MessageGroupOperation>();
            !status.ok()) {
          IMP_LOG(imp::ERROR) << "Message group(" << group_id
                     << "): Failed to read BeginMessageGroup.";
          SetBridgeStatus(bridge_id, status.status());
          return;
        }

        while (reader) {
          if (const absl::StatusOr<
                  const android_xr::schemas::MessageGroupOperation*>
                  message_group_operation_status =
                      reader.Read<android_xr::schemas::MessageGroupOperation>();
              message_group_operation_status.ok()) {
            // `VerifyBuffer` already verified that the only
            // MessageGroupOperation left is EndMessageGroup. So, we are done
            // processing the message group.
            break;
          } else if (absl::StatusOr<const android_xr::schemas::Command*>
                         command_status =
                             reader.Peek<android_xr::schemas::Command>();
                     command_status.ok()) {
            absl::StatusOr<flatbuffers::Verifier> verifier =
                reader.GetVerifierWithoutPrefixedSize();
            if (!verifier.ok()) {
              SetBridgeStatus(bridge_id, verifier.status());
              return;
            }

            // Advance the reader.
            command_status = reader.Read<android_xr::schemas::Command>();
            if (!command_status.ok()) {
              // This should never happen: `VerifyBuffer` already verified the
              // buffer.
              IMP_LOG(imp::ERROR) << "Message group(" << group_id
                         << "): Failed to read Command.";
              SetBridgeStatus(bridge_id, command_status.status());
              return;
            }

            SplitEngineRenderer& renderer =
                view_.GetRegistry()
                    .Get<imp::split_engine::SplitEngineRenderer>()
                    ->get();

            if (const absl::Status status = renderer.HandleCommand(
                    bridge_id, verifier.value(), *command_status.value(),
                    [tracker,
                     // capturing `buffer` shared_ptr by value to make sure that
                     // message group data stays alive until all commands are
                     // processed.
                     buffer]() mutable { tracker->ReportCompletion(); });
                !status.ok()) {
              IMP_LOG(imp::ERROR) << "Message group(" << group_id
                         << "): Failed to handle Command: "
                         << status.ToString();
              SetBridgeStatus(bridge_id, status);
              return;
            }
          } else {
            // This should never happen, because `VerifyBuffer` already verified
            // that the message group contains only MessageGroupOperation and
            // Command messages.
            IMP_LOG(imp::ERROR) << "Message group(" << group_id
                       << "): Invalid flatbuffer found.";
            SetBridgeStatus(
                bridge_id, absl::InternalError("Invalid flatbuffer found while "
                                               "processing message group."));
            return;
          }
        }

        return;
      });

  return absl::OkStatus();
}

void SplitEngineMMDesktopBridgeServiceImplLocal::MessageGroupCompletionTracker::
    ReportCompletion() {
  
  if (++completed_commands_count_ == commands_count_) {
    on_message_group_complete_();
  }
}

absl::StatusOr<std::reference_wrapper<
    SplitEngineMMDesktopBridgeServiceImplLocal::BridgeData>>
SplitEngineMMDesktopBridgeServiceImplLocal::GetBridgeData(BridgeId bridge_id) {
  absl::MutexLock lock(bridge_data_mutex_);
  auto it = bridge_data_.find(bridge_id);
  if (it == bridge_data_.end()) {
    return absl::NotFoundError(absl::StrCat("Bridge not found: ", bridge_id));
  }
  return *it->second;
}

}  // namespace imp::split_engine
