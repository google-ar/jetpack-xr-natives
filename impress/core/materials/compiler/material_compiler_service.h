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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_SERVICE_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_SERVICE_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "filament/libs/filamat/include/filamat/MaterialBuilder.h"
#include "filament/libs/utils/include/utils/JobSystem.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/invocable.h"
#include "core/ipc/message_pipe.h"
#include "core/materials/compiler/schemas/material_compiler_ipc_generated.h"

namespace imp {

// Native service for requesting material compilation at runtime.
class MaterialCompilerService {
 public:
  explicit MaterialCompilerService(int fd,
                                   imp::Invocable<void()> on_close = {});
  ~MaterialCompilerService();

 private:
  ipc::MessagePipe::OnMessageResult OnMessage(
      std::unique_ptr<uint8_t[]> message, size_t size);

  void HandleCompileRequest(uint64_t operation_id,
                            const schemas::CompileRequest* request);

  absl::StatusOr<std::string> CompileMaterial(
      absl::string_view source_material_string,
      filamat::MaterialBuilder::Platform platform,
      filamat::MaterialBuilder::TargetApi target_api);

  absl::Status SendResponse(const flatbuffers::FlatBufferBuilder& builder);

  void SendErrorResponse(uint64_t operation_id, absl::Status error_status);

  void Close();

  // We need this since `init` should be called before initializing `pipe_`,
  // and `shutdown` should be called after `pipe_` is destroyed.

  // Initialization order:
  // 1) MaterialBuilderGuard is initialized, `init` is called.
  // 2) `pipe_` is initialized, establishing IPC connection, this expects
  // `MaterialBuilder::init` is already called.

  // Destruction order:
  // 1) `Close` is called in destructor, which signals the `pipe_` to stop.
  // 2) JobSystem is destroyed (could wait the remaining work to be done)
  // 3) `pipe_` is destroyed. (at this point, it's possible that the finishing
  // job attempts to send response, but we check if the pipe is closed.)
  // 4) `MaterialBuilder::shutdown` is called.
  struct MaterialBuilderGuard {
    MaterialBuilderGuard() { filamat::MaterialBuilder::init(); }
    ~MaterialBuilderGuard() { filamat::MaterialBuilder::shutdown(); }
  };

  MaterialBuilderGuard material_builder_guard_;

  // Declared before `pipe_` to ensure it is initialized first. This avoids
  // a race condition where `pipe_` starts receiving messages before
  // `job_system_` is ready to handle them.
  //
  // We use a `std::unique_ptr` so we can explicitly destroy `job_system_`
  // in the destructor *before* `pipe_` is destroyed. This ensures all
  // background jobs finish while `pipe_` is still valid, preventing crashes
  // if a job attempts to send a response during shutdown.
  std::unique_ptr<utils::JobSystem> job_system_;

  // A callback when the Service closes (optional).
  imp::Invocable<void()> on_close_;

  // Pipe can be touched by multiple threads from JobSystem and the worker
  // thread.
  absl::Mutex lock_;
  ipc::MessagePipe pipe_ ABSL_GUARDED_BY(lock_);
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_SERVICE_H_
