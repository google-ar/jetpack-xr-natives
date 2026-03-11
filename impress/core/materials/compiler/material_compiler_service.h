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

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/ipc/message_pipe.h"
#include "core/materials/compiler/schemas/material_compiler_ipc_generated.h"

namespace imp {

// Native service for requesting material compilation at runtime.
class MaterialCompilerService {
 public:
  explicit MaterialCompilerService(int fd);
  ~MaterialCompilerService();

 private:
  ipc::MessagePipe::OnMessageResult OnMessage(
      std::unique_ptr<uint8_t[]> message, size_t size);

  absl::Status HandleCompileRequest(uint64_t operation_id,
                                    const schemas::CompileRequest* request);

  absl::StatusOr<std::string> CompileMaterial(
      absl::string_view source_material_string, schemas::Platform platform,
      schemas::TargetApi target_api);

  absl::Status SendResponse(const flatbuffers::FlatBufferBuilder& builder);

  void SendErrorResponse(uint64_t operation_id, absl::Status error_status);

  void Close();

  ipc::MessagePipe pipe_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_SERVICE_H_
