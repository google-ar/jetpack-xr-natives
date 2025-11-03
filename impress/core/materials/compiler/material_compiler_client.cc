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
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "flatbuffers/verifier.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/ipc/message_pipe.h"
#include "core/materials/compiler/schemas/material_compiler_ipc_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

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
  flatbuffers::FlatBufferBuilder builder;
  auto command_offset = schemas::CreateCompileRequest(
      builder,
      builder.CreateString(material_string.data(), material_string.size()),
      platform, target_api);
  auto request_offset = schemas::CreateRequest(
      builder, schemas::RequestType::CompileRequest, command_offset.Union());
  builder.Finish(request_offset);

  MP_RETURN_IF_ERROR(SendRequest(builder));

  // This blocks until the processing is done.
  return GetCompiledMaterialResponse();
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

  // Unblock any waiting threads.
  processing_message_.SignalAll();
  can_process_new_message_.SignalAll();
}

void MaterialCompilerClient::OnMessage(std::unique_ptr<std::uint8_t[]> message,
                                       size_t size) {
  absl::MutexLock lock(lock_);
  // Wait for the last message to be processed.
  while (last_message_ != nullptr) {
    // Releases the lock until it gets the signal.
    can_process_new_message_.Wait(&lock_);
  }
  last_message_ = std::move(message);
  last_message_size_ = size;
  processing_message_.Signal();
}

absl::Status MaterialCompilerClient::SendRequest(
    const flatbuffers::FlatBufferBuilder& builder) {
  if (builder.GetSize() > std::numeric_limits<uint32_t>::max()) {
    return absl::InvalidArgumentError("Request too large");
  }
  bool sent = pipe_.Send(builder.GetBufferPointer(),
                         static_cast<uint32_t>(builder.GetSize()));
  if (!sent) {
    return absl::InternalError("Failed to send request");
  }
  return absl::OkStatus();
}

absl::StatusOr<FlatBufferAccess<const schemas::CompileResponse>>
MaterialCompilerClient::GetCompiledMaterialResponse() {
  {
    absl::MutexLock lock(lock_);
    if (pipe_.IsClosed()) {
      return absl::InternalError("Pipe is already closed");
    }

    while (last_message_ == nullptr) {
      // Releases the lock until the service responds. OnMessage acquires the
      // lock, then releases once ready. Then the main thread wakes up and
      // re-acquires the lock.
      processing_message_.Wait(&lock_);
    }

    std::unique_ptr<uint8_t[]> message = std::move(last_message_);
    size_t message_size = last_message_size_;
    last_message_size_ = 0;
    // Signal that now it's available for processing the next messages.
    can_process_new_message_.Signal();

    const uint8_t* message_buffer = message.get();
    flatbuffers::Verifier verifier(message_buffer, message_size);
    if (!verifier.VerifyBuffer<schemas::Response>()) {
      return absl::InternalError("Failed to validate response.");
    }
    const schemas::Response* response =
        flatbuffers::GetRoot<schemas::Response>(message_buffer);

    // If there is an error, return it as an Error to continue error
    // propagation.
    if (response->response_type() == schemas::ResponseType::ErrorResponse) {
      auto error = response->response_as<schemas::ErrorResponse>();
      if (error && error->message()) {
        return absl::InvalidArgumentError(error->message()->c_str());
      }
      return absl::InternalError("Unable to process ErrorResponse");
    }

    if (response->response_type() != schemas::ResponseType::CompileResponse) {
      return absl::InvalidArgumentError("Not a compile response");
    }

    return FlatBufferAccess<const schemas::CompileResponse>(
        response->response_as<schemas::CompileResponse>(),
        BufferAccess{std::move(message), message_size});
  }
}
}  // namespace imp
