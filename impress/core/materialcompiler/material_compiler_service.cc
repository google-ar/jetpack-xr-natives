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
#include "core/materialcompiler/material_compiler_service.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/libs/filamat/include/filamat/MaterialBuilder.h"
#include "filament/libs/filamat/include/filamat/Package.h"
#include "filament/libs/filament-matp/include/filament-matp/Config.h"
#include "filament/libs/filament-matp/include/filament-matp/MaterialParser.h"
#include "filament/libs/utils/include/utils/JobSystem.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/ipc/message_pipe.h"
#include "core/materialcompiler/runtime_material_compiler_config.h"
#include "core/materialcompiler/schemas/material_compiler_ipc_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp_material_compiler {
MaterialCompilerService::MaterialCompilerService(int fd)
    : pipe_(
          fd,
          [](std::unique_ptr<uint8_t[]> data, size_t size,
             void* user) -> imp::ipc::MessagePipe::OnMessageResult {
            MaterialCompilerService* service =
                static_cast<MaterialCompilerService*>(user);
            return service->OnMessage(std::move(data), size);
          },
          [](void* user) {}, this, "MaterialCompilerServicePipe") {}

MaterialCompilerService::~MaterialCompilerService() { Close(); }

imp::ipc::MessagePipe::OnMessageResult MaterialCompilerService::OnMessage(
    std::unique_ptr<uint8_t[]> message, size_t size) {
  const uint8_t* message_buf = message.get();

  auto verifier = flatbuffers::Verifier(message_buf, size);
  if (!verifier.VerifyBuffer<imp::schemas::Request>()) {
    if (absl::Status status = SendErrorResponse("Failed to validate request");
        !status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to send error response: " << status.ToString();
    }
    return imp::ipc::MessagePipe::OnMessageResult::kInitiateClose;
  }

  absl::StatusOr<imp::ipc::MessagePipe::OnMessageResult> result =
      OnRequest(flatbuffers::GetRoot<imp::schemas::Request>(message_buf));
  if (!result.ok()) {
    if (absl::Status status = SendErrorResponse(result.status().message());
        !status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to send error response: " << status.ToString();
    }
    return imp::ipc::MessagePipe::OnMessageResult::kInitiateClose;
  }

  return result.value();
}

absl::StatusOr<imp::ipc::MessagePipe::OnMessageResult>
MaterialCompilerService::OnRequest(const imp::schemas::Request* request) {
  if (request == nullptr) {
    return absl::InvalidArgumentError("Request is null");
  }

  switch (request->request_type()) {
    case imp::schemas::RequestType::CompileRequest:
      MP_RETURN_IF_ERROR(HandleCompileRequest(
          request->request_as<imp::schemas::CompileRequest>()));
      return imp::ipc::MessagePipe::OnMessageResult::kKeepAlive;
    default:
      return absl::InvalidArgumentError("Unexpected request type");
  }
}

absl::Status MaterialCompilerService::HandleCompileRequest(
    const imp::schemas::CompileRequest* request) {
  if (request == nullptr) {
    return absl::InvalidArgumentError("CompileRequest is null");
  }
  MP_ASSIGN_OR_RETURN(std::string compiled_shader,
                   CompileMaterial(request->source_material()->string_view()));

  flatbuffers::FlatBufferBuilder builder;

  flatbuffers::Offset<imp::schemas::CompileResponse> compile_response =
      imp::schemas::CreateCompileResponse(
          builder, builder.CreateVector(
                       reinterpret_cast<const uint8_t*>(compiled_shader.data()),
                       compiled_shader.size()));
  flatbuffers::Offset<imp::schemas::Response> response_offset =
      imp::schemas::CreateResponse(builder,
                                   imp::schemas::ResponseType::CompileResponse,
                                   compile_response.Union());
  builder.Finish(response_offset);

  return SendResponse(builder);
}

absl::StatusOr<std::string> MaterialCompilerService::CompileMaterial(
    absl::string_view source_material_string) {
  matp::MaterialParser parser;
  filamat::MaterialBuilder builder;
  std::ostringstream compiler_output;

  RuntimeMaterialCompilerConfig config(source_material_string, compiler_output);
  matp::Config::Input* input = config.getInput();
  if (input == nullptr) {
    return absl::InvalidArgumentError("Input is null");
  }
  ssize_t size = input->open();
  if (size <= 0) {
    return absl::InvalidArgumentError("Input file is empty");
  }
  std::unique_ptr<const char[]> buffer = input->read();

  bool template_sub_succeed =
      parser.processTemplateSubstitutions(config, size, buffer);
  if (!template_sub_succeed) {
    return absl::InvalidArgumentError(
        "Failed to process template substitutions");
  }

  builder.init();
  if (!parser.parse(builder, config, size, buffer)) {
    return absl::InvalidArgumentError("Failed to parse material");
  }

  utils::JobSystem js;
  js.adopt();

  filamat::Package package = builder.build(js);

  js.emancipate();
  filamat::MaterialBuilder::shutdown();

  if (!package.isValid()) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Failed to compile shader %s", input->getName()));
  }

  if (!config.getOutput()->write(package.getData(), package.getSize())) {
    return absl::InvalidArgumentError("Failed to write material package");
  }

  return compiler_output.str();
}

absl::Status MaterialCompilerService::SendResponse(
    const flatbuffers::FlatBufferBuilder& builder) {
  if (builder.GetSize() > std::numeric_limits<uint32_t>::max()) {
    return absl::InternalError("Response too large");
  }
  bool sent = pipe_.Send(builder.GetBufferPointer(),
                         static_cast<uint32_t>(builder.GetSize()));
  if (!sent) {
    return absl::InternalError("Failed to send response");
  }

  return absl::OkStatus();
}

absl::Status MaterialCompilerService::SendErrorResponse(
    absl::string_view error_message) {
  flatbuffers::FlatBufferBuilder builder;
  flatbuffers::Offset<imp::schemas::ErrorResponse> error_response =
      imp::schemas::CreateErrorResponse(builder,
                                        builder.CreateString(error_message));
  flatbuffers::Offset<imp::schemas::Response> response_offset =
      imp::schemas::CreateResponse(builder,
                                   imp::schemas::ResponseType::ErrorResponse,
                                   error_response.Union());
  builder.Finish(response_offset);

  return SendResponse(builder);
}

void MaterialCompilerService::Close() {
  if (!pipe_.IsClosed()) {
    pipe_.Close();
  }
}

}  // namespace imp_material_compiler
