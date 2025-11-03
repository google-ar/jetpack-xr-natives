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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_CLIENT_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_CLIENT_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "absl/base/thread_annotations.h"
#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/ipc/message_pipe.h"
#include "core/materials/compiler/schemas/material_compiler_ipc_generated.h"

namespace imp {
// Tracks the connection state to manage the shutdown sequence correctly.
// This ensures that no new requests are sent after a close has been
// initiated.
//
// A shutdown can be started by the client (moving to kClosing) or
// by the service/pipe (moving directly to kClosed).
enum class ConnectionState {
  // Connection is established.
  kConnected,
  // Client is closed and has sent a CloseRequest to the service. Any further
  // requests are blocked.
  kClosing,
  // Connection is closed. The client and pipe are both closed.
  kClosed
};

// Native client for requesting material compilation at runtime.
class MaterialCompilerClient {
 public:
  explicit MaterialCompilerClient(int fd);
  ~MaterialCompilerClient();

  // Compiles the given source material (.mat) and returns the compiled material
  // (.cmat) binary that can be used to build a runtime Material representation.
  // Expects the material input to be already inlined.
  // The platform and target_api parameters are used to determine the compiled
  // shader output.
  absl::StatusOr<FlatBufferAccess<const schemas::CompileResponse>>
  CompileMaterial(absl::string_view source_material_string,
                  schemas::Platform platformm, schemas::TargetApi target_api);

  // Initiates close by the client. This sends a CloseRequest to the service,
  // ask the service to close the pipe. This closes the pipe itself if it fails
  // to send a CloseRequest.
  void Close();

 private:
  // Called when the pipe is closed by the service.
  void OnPipeClosed();

  void OnMessage(std::unique_ptr<uint8_t[]> message, size_t size);

  absl::Status SendRequest(const flatbuffers::FlatBufferBuilder& builder);

  absl::StatusOr<FlatBufferAccess<const schemas::CompileResponse>>
  GetCompiledMaterialResponse();

  ConnectionState connection_state_ = ConnectionState::kConnected;

  // For now, we handle one request at a time.
  std::unique_ptr<uint8_t[]> last_message_;
  size_t last_message_size_ = 0;

  absl::Mutex lock_;
  absl::CondVar can_process_new_message_ ABSL_GUARDED_BY(lock_);
  absl::CondVar processing_message_ ABSL_GUARDED_BY(lock_);

  ipc::MessagePipe pipe_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_CLIENT_H_
