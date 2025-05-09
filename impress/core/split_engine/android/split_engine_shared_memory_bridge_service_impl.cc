// Copyright 2024 Google LLC
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

#include "core/split_engine/android/split_engine_shared_memory_bridge_service_impl.h"

#include <asm-generic/mman-common.h>
#include <jni.h>
#include <linux/mman.h>
#include <sys/mman.h>

#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

SplitEngineSharedMemoryBridgeServiceImpl::
    SplitEngineSharedMemoryBridgeServiceImpl(BaseView& view, Executor* executor)
    : view_(&view), foreground_executor_(executor) {}

void SplitEngineSharedMemoryBridgeServiceImpl::Update(BaseView& view,
                                                      Executor* executor) {
  view_ = &view;
  foreground_executor_ = executor;
}

absl::Status SplitEngineSharedMemoryBridgeServiceImpl::InitializeBridge(
    BridgeId bridge_id,
    ReleaseMessageGroupFunction release_message_group_function) {
  {
    // Create a release function for the bridge and ensure it is unique.
    absl::MutexLock lock(&bridge_release_message_group_functions_mutex_);
    if (!bridge_release_message_group_functions_
             .insert({bridge_id, release_message_group_function})
             .second) {
      return absl::AlreadyExistsError("Bridge already exists.");
    }
  }
  {
    // Create a status record for the bridge.
    absl::MutexLock lock(&bridge_status_mutex_);
    bridge_status_[bridge_id] = absl::OkStatus();
  }
  return absl::OkStatus();
}

absl::Status SplitEngineSharedMemoryBridgeServiceImpl::RegisterBuffer(
    BridgeId bridge_id, BufferId buffer_id, int file_descriptor,
    size_t size_bytes) {
  // Check if the bridge is in an error state.
  MP_RETURN_IF_ERROR(GetBridgeStatus(bridge_id));

  {
    // Ensure the bridge exists, since otherwise the buffer will never be
    // released.
    absl::MutexLock lock(&bridge_release_message_group_functions_mutex_);
    if (bridge_release_message_group_functions_.find(bridge_id) ==
        bridge_release_message_group_functions_.end()) {
      return absl::InvalidArgumentError(
          "Attempt to register buffer for unknown bridge.");
    }
  }

  if (buffers_.find(buffer_id) != buffers_.end()) {
    return absl::AlreadyExistsError("Attempt to register existing buffer.");
  }

  // Map the shared memory buffer to the process address space.
  void* buffer = ::mmap(nullptr, size_bytes, PROT_READ | PROT_WRITE, MAP_SHARED,
                        file_descriptor, 0);
  if (buffer == MAP_FAILED) {
    return absl::InternalError("Failed to mmap shared memory buffer. " +
                               std::string(strerror(errno)));
  }

  // Create a record for the buffer - this associates the buffer with the bridge
  // and ensures the buffer is released when the bridge is released.
  buffers_.insert({buffer_id, RenderBridgeBuffer{
                                  .bridge_id = bridge_id,
                                  .buffer_ptr = static_cast<uint8_t*>(buffer),
                                  .buffer_size_bytes = size_bytes,
                              }});
  return absl::OkStatus();
}

void SplitEngineSharedMemoryBridgeServiceImpl::CleanupBuffer(
    BufferId buffer_id) {
  // Unmap the shared memory buffer and delete the record.
  if (auto it = buffers_.find(buffer_id); it != buffers_.end()) {
    auto& record = it->second;
    ::munmap(record.buffer_ptr, record.buffer_size_bytes);
    buffers_.erase(buffer_id);
  }
}

absl::Status SplitEngineSharedMemoryBridgeServiceImpl::GetBridgeStatus(
    BridgeId bridge_id) {
  absl::MutexLock lock(&bridge_status_mutex_);
  auto it = bridge_status_.find(bridge_id);
  if (it == bridge_status_.end()) {
    return absl::NotFoundError(absl::StrCat("Bridge not found: ", bridge_id));
  }
  return it->second;
}

void SplitEngineSharedMemoryBridgeServiceImpl::SetBridgeStatus(
    BridgeId bridge_id, absl::Status status) {
  {
    absl::MutexLock lock(&bridge_status_mutex_);

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

absl::Status SplitEngineSharedMemoryBridgeServiceImpl::ProcessRegion(
    const BufferId buffer_id, std::shared_ptr<MessageGroupStorage> storage,
    int32_t offset_bytes, size_t region_length_bytes) {
  // Find the buffer containing the region. This also gets the bridge id.
  auto buffer_it = buffers_.find(buffer_id);
  if (buffer_it == buffers_.end()) {
    return absl::InvalidArgumentError("Attempt to process unknown buffer.");
  }
  const RenderBridgeBuffer& buffer = buffer_it->second;

  // Check if the bridge is in an error state.
  MP_RETURN_IF_ERROR(GetBridgeStatus(buffer.bridge_id));

  // Ensure the region is within the buffer bounds.
  if (offset_bytes + region_length_bytes > buffer.buffer_size_bytes) {
    absl::Status status =
        absl::OutOfRangeError("Offset + region length exceeds buffer size.");
    SetBridgeStatus(buffer.bridge_id, status);
    return status;
  }

  // Check if the message is a MessageGroup message or Command message.
  // MessageGroup messages are used to group multiple messages together.
  // Command messages are used to send commands to the SplitEngineRenderer.
  // TODO: Improve flatbuffer message type checking.
  // This reliance on VerifyBuffer is fragile. It will not perform any
  // explicit type checks, and relies on the message schema providing a
  // "fingerprint" of the type. eg: VerifyBuffer<A> will return true for type
  // B if they have the same schema.
  absl::Status status = absl::OkStatus();
  const uint8_t* message_start = buffer.buffer_ptr + offset_bytes;
  flatbuffers::Verifier verifier(message_start, region_length_bytes);
  if (verifier.VerifyBuffer<android_xr::schemas::MessageGroup>()) {
    status = OnMessageGroupMessage(buffer.bridge_id, buffer_id, message_start,
                                   region_length_bytes, storage);
  } else if (verifier.VerifyBuffer<android_xr::schemas::Command>()) {
    // If this is a Command message, cache it for later processing. All Command
    // messages must be processed contiguously to avoid breaking work across
    // frame boundaries.
    MessageGroupTracker* message_group_tracker = nullptr;
    {
      // Ensure there is an active message group for this buffer.
      absl::MutexLock lock(&message_groups_mutex_);
      auto message_group = message_groups_.find(buffer_id);
      if (message_group == message_groups_.end()) {
        status = absl::InvalidArgumentError(
            absl::StrCat("Message group not found: ", buffer_id));
      } else {
        message_group_tracker = message_group->second.get();
      }
    }
    if (message_group_tracker) {
      // Cache the Command message for later processing.
      message_group_tracker->AddMessage(offset_bytes, region_length_bytes);
    }
  } else {
    // Bad messages are fatal as part of our "zero-tolerance policy".
    IMP_LOG(imp::ERROR) << "[SplitEngineRenderer] SplitEngineBridgeReceiver received "
                  "invalid message.";
    status = absl::InvalidArgumentError("Invalid message.");
  }

  SetBridgeStatus(buffer.bridge_id, status);
  return status;
}

absl::Status SplitEngineSharedMemoryBridgeServiceImpl::HandleAllMessagesInGroup(
    BufferId buffer_id, MessageGroupId message_group_id) {
  // We don't want to block the client waiting for the result of the async
  // ProcessRegion, so we schedule the work on the foreground executor.
  //
  // Note that ProcessRegion runs on the binder thread, but the "work" of the
  // message is done on the foreground executor that lives in the system side.
  // This means that if the previous status was not OK, we receive the result
  // one call later.
  //
  // Note also that we don't want to hold the lock on message_group_messages_
  // while we schedule the work on the foreground executor, because that lock
  // is also held while we're processing the message. Copying the vector of
  // MessageOffsets is cheap, and avoids a potential deadlock.
  // Find the buffer record for the buffer id.
  auto buffer_it = buffers_.find(buffer_id);
  if (buffer_it == buffers_.end()) {
    return absl::InvalidArgumentError(
        "Attempt to process unknown bridge buffer.");
  }
  const RenderBridgeBuffer& buffer = buffer_it->second;

  // Extract the message group from the map so that we can pass it to the
  // foreground executor without holding the lock.
  std::unique_ptr<MessageGroupTracker> message_group = nullptr;
  {
    absl::MutexLock lock(&message_groups_mutex_);
    auto it = message_groups_.find(buffer_id);
    if (it == message_groups_.end()) {
      absl::Status status = absl::InvalidArgumentError(absl::StrCat(
          "Attempt to process Command for unknown buffer - BeginMessageGroup "
          "not received for buffer_id: ",
          buffer_id));
      SetBridgeStatus(buffer.bridge_id, status);
      return status;
    }
    if (it->second->GetMessageGroupId() != message_group_id) {
      return absl::InvalidArgumentError(
          "EndMessageGroup received but group id mismatch.");
    }
    // Extract the message group from the map so that we can pass it to the
    // foreground executor without holding the lock.
    message_group = std::move(message_groups_.extract(it).mapped());
  }

  // Process all messages in the group on the Impress foreground executor, where
  // Nodes, Renderables, etc. live.
  foreground_executor_->ScheduleInvocable([this, buffer_id, buffer,
                                           message_group = std::move(
                                               message_group)]() mutable {
    MessageGroupTracker* message_group_ptr = message_group.get();
    // The foreground executor now owns the message group. This vector of groups
    // is only accessed on the foreground executor and maintained so groups can
    // be closed asynchronously, for example if a texture needs to be uploaded
    // to the GPU.
    message_groups_processing_[buffer_id] = std::move(message_group);

    for (const MessageOffset& message_offset :
         message_group_ptr->GetMessageOffsets()) {
      if (message_offset.offset_bytes + message_offset.size_bytes >
          buffer.buffer_size_bytes) {
        absl::Status status = absl::InvalidArgumentError(
            "Offset + region length exceeds buffer size.");
        SetBridgeStatus(buffer.bridge_id, status);
        return;
      }

      // Check if the message is a MessageGroup message or Command message.
      // MessageGroup messages are used to group multiple messages together.
      // Command messages are used to send commands to the
      // SplitEngineRenderer.
      // TODO: Improve flatbuffer message type checking.
      // This reliance on VerifyBuffer is fragile. It will not perform any
      // explicit type checks, and relies on the message schema providing a
      // "fingerprint" of the type. eg: VerifyBuffer<A> will return true for
      // type B if they have the same schema.
      const uint8_t* message_start =
          buffer.buffer_ptr + message_offset.offset_bytes;
      flatbuffers::Verifier verifier(message_start, message_offset.size_bytes);
      if (!verifier.VerifyBuffer<android_xr::schemas::Command>()) {
        // Bad messages are logged and ignored. There's not much else we can
        // do other than treat it as fatal and take down the process.
        IMP_LOG(imp::ERROR)
            << "[SplitEngineRenderer] SplitEngineBridgeReceiver received "
               "invalid message.";
        absl::Status status = absl::InvalidArgumentError("Invalid message.");
        SetBridgeStatus(buffer.bridge_id, status);
        return;
      }

      // Handle the individual Command message. If the result status is not OK,
      // the bridge will be shut down and the last error status will be
      // returned indefinitely.
      //
      // This is the current "zero-tolerance policy" since it is the simplest
      // to implement. In the future, we may want to consider a more
      // sophisticated policy.
      if (absl::Status status =
              HandleMessage(buffer.bridge_id, verifier, message_start,
                            [this, bridge_id = buffer.bridge_id, buffer_id,
                             message_group_ptr] {
                              // Notify the message group that a message has
                              // been finished. If this was the last unfinished
                              // message in the group, release the group.
                              message_group_ptr->OnMessageFinished();
                              ReleaseMessageGroupIfFinished(
                                  bridge_id, buffer_id,
                                  message_group_ptr->GetMessageGroupId());
                            });
          !status.ok()) {
        SetBridgeStatus(buffer.bridge_id, status);
        return;
      }
    }
    // All messages may synchronously fully process, so possibly release the
    // message group.
    ReleaseMessageGroupIfFinished(buffer.bridge_id, buffer_id,
                                  message_group_ptr->GetMessageGroupId());
  });
  return absl::OkStatus();
}

void SplitEngineSharedMemoryBridgeServiceImpl::CleanupBridge(
    BridgeId bridge_id) {
  foreground_executor_->Schedule([this, bridge_id]() {
    // Destroy all 3D content associated with this app before releasing memory.
    SplitEngineRenderer& renderer =
        view_->GetRegistry()
            .Get<imp::split_engine::SplitEngineRenderer>()
            ->get();
    if (absl::Status status = renderer.ClearAppContext(bridge_id);
        !status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to clear app context: " << status;
    }
    // Clear all external texture surfaces associated with this bridge.
    surface_factory_.Clear(bridge_id);

    // Clear all message groups associated with this bridge.
    //
    // First, find the buffer release function for this bridge.
    ReleaseMessageGroupFunction release_function = {};
    {
      absl::MutexLock lock(&bridge_release_message_group_functions_mutex_);
      auto release_function_it =
          bridge_release_message_group_functions_.find(bridge_id);
      if (release_function_it ==
          bridge_release_message_group_functions_.end()) {
        IMP_LOG(imp::ERROR) << "No release function found for bridge: " << bridge_id;
      } else {
        release_function = release_function_it->second;
      }
    }
    // Next, release all message groups associated with this bridge.
    if (release_function) {
      {
        // Release all message groups that are currently being processed.
        for (auto it = message_groups_processing_.begin(),
                  end = message_groups_processing_.end();
             it != end;) {
          auto copy_it = it++;
          if (copy_it->second->GetBridgeId() == bridge_id) {
            release_function(copy_it->second->GetMessageGroupId());
            message_groups_processing_.erase(copy_it);
          }
        }
      }
      {
        // Release all message groups that have been started but not ended.
        absl::MutexLock lock(&message_groups_mutex_);
        for (auto it = message_groups_.begin(), end = message_groups_.end();
             it != end;) {
          auto copy_it = it++;
          if (copy_it->second->GetBridgeId() == bridge_id) {
            release_function(it->second->GetMessageGroupId());
            message_groups_.erase(copy_it);
          }
        }
      }
    }

    // Finally, clear the release function and bridge status.
    {
      absl::MutexLock lock(&bridge_release_message_group_functions_mutex_);
      if (bridge_release_message_group_functions_.find(bridge_id) !=
          bridge_release_message_group_functions_.end()) {
        bridge_release_message_group_functions_.erase(bridge_id);
      }
    }
    {
      absl::MutexLock lock(&bridge_status_mutex_);
      bridge_status_.erase(bridge_id);
    }
  });
}

absl::Status SplitEngineSharedMemoryBridgeServiceImpl::OnMessageGroupMessage(
    BridgeId bridge_id, BufferId buffer_id, const uint8_t* message, size_t size,
    std::shared_ptr<MessageGroupStorage> storage) {
  const android_xr::schemas::MessageGroup* message_group =
      flatbuffers::GetRoot<android_xr::schemas::MessageGroup>(message);
  switch (message_group->message_group_type()) {
    case android_xr::schemas::MessageGroupTypes::BeginMessageGroup:
      return OnBeginMessageGroupMessage(bridge_id, buffer_id, message_group,
                                        storage);
    case android_xr::schemas::MessageGroupTypes::EndMessageGroup:
      return OnEndMessageGroupMessage(bridge_id, buffer_id, message_group,
                                      storage);
    default:
      return absl::InvalidArgumentError("Unknown message group type.");
  }
}

absl::Status
SplitEngineSharedMemoryBridgeServiceImpl::OnBeginMessageGroupMessage(
    BridgeId bridge_id, BufferId buffer_id,
    const android_xr::schemas::MessageGroup* message_group,
    std::shared_ptr<MessageGroupStorage> storage) {
  absl::MutexLock lock(&message_groups_mutex_);
  if (message_groups_.find(buffer_id) != message_groups_.end()) {
    return absl::InvalidArgumentError(
        "BeginMessageGroup received but message group already exists.");
  }
  message_groups_.insert(std::make_pair(
      buffer_id, std::make_unique<MessageGroupTracker>(
                     bridge_id, message_group->group_id(), storage)));
  return absl::OkStatus();
}

absl::Status SplitEngineSharedMemoryBridgeServiceImpl::OnEndMessageGroupMessage(
    BridgeId bridge_id, BufferId buffer_id,
    const android_xr::schemas::MessageGroup* message_group,
    std::shared_ptr<MessageGroupStorage> storage) {
  // To process Commands in a MessageGroup contiguously and part of one frame,
  // process all of the messages in the MessageGroup when EndMessageGroup is
  // received.
  return HandleAllMessagesInGroup(buffer_id, message_group->group_id());
}

absl::Status SplitEngineSharedMemoryBridgeServiceImpl::HandleMessage(
    BridgeId bridge_id, flatbuffers::Verifier& verifier, const uint8_t* message,
    SplitEngineRenderer::OnFinishedCallback on_finished) {
  SplitEngineRenderer& renderer =
      view_->GetRegistry().Get<imp::split_engine::SplitEngineRenderer>()->get();
  return renderer.HandleMessage(
      bridge_id, verifier,
      *flatbuffers::GetRoot<android_xr::schemas::Command>(message),
      std::move(on_finished));
}

void SplitEngineSharedMemoryBridgeServiceImpl::ReleaseMessageGroupIfFinished(
    BridgeId bridge_id, BufferId buffer_id, MessageGroupId message_group_id) {
  // To be eligible for release, the message group must have
  // num_messages_received == num_messages_finished AND not be the currently
  // active group.
  auto it = message_groups_processing_.find(buffer_id);
  if (it == message_groups_processing_.end() || !it->second->IsFinished()) {
    return;
  }
  message_groups_processing_.erase(it);

  absl::MutexLock lock(&bridge_release_message_group_functions_mutex_);
  auto release_function_it =
      bridge_release_message_group_functions_.find(bridge_id);
  if (release_function_it == bridge_release_message_group_functions_.end()) {
    IMP_LOG(imp::ERROR) << "No release function found for bridge: " << bridge_id
               << " with message group id: " << message_group_id
               << " - This represents a bookkeeping error in the "
                  "SplitEngineSharedMemoryBridgeServiceImpl.";
    return;
  }
  release_function_it->second(message_group_id);
}

absl::Status
SplitEngineSharedMemoryBridgeServiceImpl::CreateExternalTextureSurface(
    BridgeId bridge_id, const std::vector<TextureId>& in_texture_ids,
    jobject& out_surface) {
  absl::Notification notify;
  foreground_executor_->Schedule(
      [this, &notify, bridge_id, in_texture_ids, &out_surface]() mutable {
        out_surface = surface_factory_.CreateExternalTextureSurface(
            *view_, bridge_id, in_texture_ids);
        notify.Notify();
      });
  notify.WaitForNotification();
  return absl::OkStatus();
}

absl::Status
SplitEngineSharedMemoryBridgeServiceImpl::SetExternalTextureSurfaceSize(
    BridgeId bridge_id, TextureId in_texture_id, int32_t width,
    int32_t height) {
  if (width <= 0 || height <= 0) {
    return absl::InvalidArgumentError("Width and height must be positive.");
  }
  absl::Notification notify;
  absl::Status result;
  foreground_executor_->Schedule([this, &notify, &result, bridge_id,
                                  in_texture_id, width, height]() mutable {
    result = surface_factory_.SetExternalTextureSurfaceSize(
        *view_, bridge_id, in_texture_id, {width, height});
    notify.Notify();
  });
  notify.WaitForNotification();
  return result;
}

flatbuffers::Offset<android_xr::schemas::ErrorResponse> CreateErrorResponse(
    flatbuffers::FlatBufferBuilder& fbb, absl::Status status) {
  return android_xr::schemas::CreateErrorResponse(
      fbb, StatusToErrorCode(status), fbb.CreateString(status.message()));
}

std::vector<uint8_t> SerializeErrorResponse(absl::Status status) {
  flatbuffers::FlatBufferBuilder fbb;
  return SerializeResponse(fbb, CreateErrorResponse(fbb, status));
}

std::vector<uint8_t> SerializeStatusResponse(absl::Status status) {
  if (!status.ok()) {
    return SerializeErrorResponse(status);
  }

  // Create an empty response to signal an OK status.
  flatbuffers::FlatBufferBuilder fbb;
  flatbuffers::Offset<android_xr::schemas::Response> response =
      android_xr::schemas::CreateResponse(fbb);
  return SerializeTable(fbb, response);
}

absl::Status SplitEngineSharedMemoryBridgeServiceImpl::SendRequest(
    BridgeId bridge_id, const std::vector<uint8_t>& data,
    ResponseHandlerFunction response_handler) {
  flatbuffers::Verifier verifier(data.data(), data.size());
  if (!verifier.VerifyBuffer<android_xr::schemas::Request>()) {
    return absl::InvalidArgumentError(
        "Invalid flatbuffer schema passed to SendRequest()");
  }
  // Copy the data into the foreground executor.
  foreground_executor_->Schedule([this, bridge_id,
                                  response_handler =
                                      std::move(response_handler),
                                  data_storage = data]() {
    const android_xr::schemas::Request* request =
        flatbuffers::GetRoot<android_xr::schemas::Request>(data_storage.data());
    absl::StatusOr<std::reference_wrapper<SplitEngineRenderer>> renderer =
        view_->GetRegistry().Get<imp::split_engine::SplitEngineRenderer>();
    if (!renderer.ok()) {
      return absl::FailedPreconditionError(
          "CreateMaterial requires a SplitEngineRenderer in the registry.");
    }
    switch (request->request_type()) {
      case android_xr::schemas::RequestTypes::BuiltInMaterialRequest: {
        const android_xr::schemas::BuiltInMaterialRequest* material_request =
            request->request_as<android_xr::schemas::BuiltInMaterialRequest>();
        renderer->get()
            .CreateBuiltInMaterial(bridge_id, *material_request)
            .Then([response_handler =
                       std::move(response_handler)](absl::Status status) {
              response_handler(SerializeStatusResponse(status));
            })
            .KeptBy(this);
        break;
      }
      default: {
        response_handler(SerializeErrorResponse(absl::UnimplementedError(
            absl::StrFormat("There is no handler for request type: %d",
                            request->request_type()))));
        break;
      }
    }
    return absl::OkStatus();
  });
  return absl::OkStatus();
}

}  // namespace imp::split_engine
