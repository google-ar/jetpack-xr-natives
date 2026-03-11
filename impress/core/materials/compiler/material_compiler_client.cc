/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "core/materials/compiler/material_compiler_client.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/ipc/message_pipe.h"
#include "core/materials/compiler/schemas/material_compiler_ipc_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace {
inline absl::Status GetAbslStatus(schemas::ErrorStatusCode error_status_code,
                                  absl::string_view message) {
  switch (error_status_code) {
    case schemas::ErrorStatusCode::InvalidArgument:
      return absl::InvalidArgumentError(message);
    case schemas::ErrorStatusCode::Internal:
      return absl::InternalError(message);
    case schemas::ErrorStatusCode::Unsupported:
      return absl::UnimplementedError(message);
      // Note: Omit default so that the compiler can catch any missing
      // schemas::ErrorStatusCode if it's extended.
  }
  return absl::UnknownError(message);
}
}  // namespace

MaterialCompilerClient::MaterialCompilerClient(int fd)
    : pipe_(
          fd,
          [](std::unique_ptr<uint8_t[]> data, size_t size,
             void* user) -> ipc::MessagePipe::OnMessageResult {
            MaterialCompilerClient* client =
                static_cast<MaterialCompilerClient*>(user);
            client->OnMessage(std::move(data), size);
            return ipc::MessagePipe::OnMessageResult::kKeepAlive;
          },
          [](void* user) {
            MaterialCompilerClient* client =
                static_cast<MaterialCompilerClient*>(user);
            client->OnPipeClosed();
          },
          this, "MaterialCompilerClientPipe") {}

MaterialCompilerClient::~MaterialCompilerClient() { Close(); }

// Execution order:
// 1) GetCompiledMaterialResponse in CompileMaterial grabs the lock.
// 2) Releases the lock at processing_message_, waiting on the server response
// 3) OnMessage grabs the lock and signals processing_message_, releases the
// lock as it exits the function.
// 4) Since processing_message_ is signalled, GetCompiledShaderResponse resumes
// the execution and grabs the lock again.
// 5) After it moves/copies the last_message_ and its size, it signals
// can_process_new_message_ and releases the lock.
// 6) Now the lock is released, OnMessage can grab the lock if there's any new
// response from the server.
absl::StatusOr<FlatBufferAccess<const schemas::CompileResponse>>
MaterialCompilerClient::CompileMaterial(absl::string_view material_string,
                                        schemas::Platform platform,
                                        schemas::TargetApi target_api) {
  uint64_t operation_id = ++last_operation_id_;
  flatbuffers::FlatBufferBuilder builder;
  auto command_offset = schemas::CreateCompileRequest(
      builder,
      builder.CreateString(material_string.data(), material_string.size()),
      platform, target_api);
  auto request_offset =
      schemas::CreateRequest(builder, schemas::RequestType::CompileRequest,
                             command_offset.Union(), operation_id);
  builder.Finish(request_offset);

  MP_RETURN_IF_ERROR(SendRequest(builder));

  // This blocks until the processing is done.
  return GetCompiledMaterialResponse(operation_id);
}

void MaterialCompilerClient::Close() {
  {
    absl::MutexLock lock(lock_);
    if (connection_state_ == ConnectionState::kClosed ||
        connection_state_ == ConnectionState::kClosing) {
      IMP_LOG(imp::INFO) << "Connection is already closed or closing";
      return;
    }
    connection_state_ = ConnectionState::kClosing;
  }

  // Send a CloseRequest to the service to close the pipe. This allows the
  // service to process any last request before closing.
  flatbuffers::FlatBufferBuilder builder;
  auto request_offset =
      schemas::CreateRequest(builder, schemas::RequestType::CloseRequest,
                             schemas::CreateCloseRequest(builder).Union());
  builder.Finish(request_offset);
  if (absl::Status status = SendRequest(builder); !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to send close request: " << status.ToString();
    // We failed to send a CloseRequest, in such a case we close the pipe here.
    if (!pipe_.IsClosed()) {
      // This will trigger OnPipeClosed() and unblock any waiting threads.
      pipe_.Close();
    }
  }
}

void MaterialCompilerClient::OnPipeClosed() {
  absl::MutexLock lock(lock_);
  if (connection_state_ == ConnectionState::kClosed) {
    // It's already closed, no further work is needed.
    return;
  }
  connection_state_ = ConnectionState::kClosed;
}

void MaterialCompilerClient::OnMessage(std::unique_ptr<std::uint8_t[]> message,
                                       size_t size) {
  absl::MutexLock lock(lock_);
  const uint8_t* message_buffer = message.get();
  flatbuffers::Verifier verifier(message_buffer, size);
  if (!verifier.VerifyBuffer<schemas::Response>()) {
    IMP_LOG(imp::ERROR) << "Failed to validate response.";
    return;
  }
  const schemas::Response* response =
      flatbuffers::GetRoot<schemas::Response>(message_buffer);

  if (response == nullptr) {
    IMP_LOG(imp::ERROR) << "Response is null.";
    return;
  }

  auto try_emplace_response =
      [this](uint64_t operation_id,
             absl::StatusOr<FlatBufferAccess<const schemas::CompileResponse>>
                 response) {
        lock_.AssertHeld();
        auto [_, inserted] =
            processed_messages_.try_emplace(operation_id, std::move(response));
        if (!inserted) {
          IMP_LOG(imp::ERROR) << "Duplicate response for operation id: " << operation_id;
        }
      };

  uint64_t operation_id = response->operation_id();
  if (operation_id == 0) {
    IMP_LOG(imp::ERROR) << "Operation id must be greater than 0, ignoring the response.";
    return;
  }

  switch (response->response_type()) {
    case schemas::ResponseType::ErrorResponse: {
      auto error = response->response_as<schemas::ErrorResponse>();
      if (error && error->message()) {
        try_emplace_response(operation_id,
                             GetAbslStatus(error->error_status_code(),
                                           error->message()->string_view()));
        return;
      }
      try_emplace_response(
          operation_id, absl::InternalError("Unable to process ErrorResponse"));
      return;
    }
    case schemas::ResponseType::CompileResponse: {
      try_emplace_response(
          operation_id, FlatBufferAccess<const schemas::CompileResponse>(
                            response->response_as<schemas::CompileResponse>(),
                            BufferAccess{std::move(message), size}));
      return;
    }
    default:
      try_emplace_response(
          operation_id,
          absl::UnimplementedError(absl::StrFormat("Unknown response type: %d",
                                                   response->response_type())));
      return;
  }
}

absl::Status MaterialCompilerClient::SendRequest(
    const flatbuffers::FlatBufferBuilder& builder) {
  if (builder.GetSize() > std::numeric_limits<uint32_t>::max()) {
    return absl::InvalidArgumentError("Request too large");
  }
  // Lock to avoid sending multiple requests at the same time. This is
  // needed because the pipe is not thread-safe.
  absl::MutexLock lock(lock_);
  bool sent = pipe_.Send(builder.GetBufferPointer(),
                         static_cast<uint32_t>(builder.GetSize()));
  if (!sent) {
    return absl::InternalError("Failed to send request");
  }
  return absl::OkStatus();
}

absl::StatusOr<FlatBufferAccess<const schemas::CompileResponse>>
MaterialCompilerClient::GetCompiledMaterialResponse(uint64_t operation_id) {
  absl::MutexLock lock(lock_);
  auto has_response_or_pipe_closed = [this, operation_id]() -> bool {
    lock_.AssertReaderHeld();
    return processed_messages_.contains(operation_id) ||
           connection_state_ != ConnectionState::kConnected;
  };
  if (!lock_.AwaitWithTimeout(absl::Condition(&has_response_or_pipe_closed),
                              absl::Seconds(30))) {
    return absl::DeadlineExceededError("Timed out waiting for response");
  }

  auto it = processed_messages_.find(operation_id);
  if (it != processed_messages_.end()) {
    absl::StatusOr<FlatBufferAccess<const schemas::CompileResponse>> response =
        std::move(it->second);
    processed_messages_.erase(it);
    return response;
  }
  return absl::NotFoundError("Operation id not found");
}
}  // namespace imp
