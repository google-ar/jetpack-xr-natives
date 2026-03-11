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
#include "core/materials/compiler/material_compiler_service.h"

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
#include "core/common/filament_status_helpers.h"
#include "core/ipc/message_pipe.h"
#include "core/materials/compiler/runtime_material_compiler_config.h"
#include "core/materials/compiler/schemas/material_compiler_ipc_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace {
// Convert the target API enum from the IPC schema to the Filament material
// builder enum.
inline filamat::MaterialBuilder::TargetApi UnpackTargetApi(
    schemas::TargetApi target_api) {
  switch (target_api) {
    case schemas::TargetApi::NONE:
      return filamat::MaterialBuilder::TargetApi::ALL;
    case schemas::TargetApi::OpenGL:
      return filamat::MaterialBuilder::TargetApi::OPENGL;
    case schemas::TargetApi::Vulkan:
      return filamat::MaterialBuilder::TargetApi::VULKAN;
    case schemas::TargetApi::Metal:
      return filamat::MaterialBuilder::TargetApi::METAL;
    case schemas::TargetApi::WebGPU:
      return filamat::MaterialBuilder::TargetApi::WEBGPU;
    case schemas::TargetApi::ANY:
      return filamat::MaterialBuilder::TargetApi::ALL;
  }
}

// Convert the platform enum from the IPC schema to the Filament material
// builder enum.
inline filamat::MaterialBuilder::Platform UnpackPlatform(
    schemas::Platform platform) {
  switch (platform) {
    case schemas::Platform::Desktop:
      return filamat::MaterialBuilder::Platform::DESKTOP;
    case schemas::Platform::Mobile:
      return filamat::MaterialBuilder::Platform::MOBILE;
    case schemas::Platform::All:
      return filamat::MaterialBuilder::Platform::ALL;
  }
}

// Convert the absl::StatusCode to the IPC schema ErrorStatusCode.
inline schemas::ErrorStatusCode PackStatusCode(absl::StatusCode status_code) {
  switch (status_code) {
    case absl::StatusCode::kInvalidArgument:
      return schemas::ErrorStatusCode::InvalidArgument;
    case absl::StatusCode::kInternal:
      return schemas::ErrorStatusCode::Internal;
    case absl::StatusCode::kUnimplemented:
      return schemas::ErrorStatusCode::Unsupported;
    default:
      IMP_LOG(imp::FATAL) << "Unexpected Abseil StatusCode: " << status_code;
  }
}
}  // namespace

MaterialCompilerService::MaterialCompilerService(int fd)
    : pipe_(
          fd,
          [](std::unique_ptr<uint8_t[]> data, size_t size,
             void* user) -> ipc::MessagePipe::OnMessageResult {
            MaterialCompilerService* service =
                static_cast<MaterialCompilerService*>(user);
            return service->OnMessage(std::move(data), size);
          },
          [](void* user) {}, this, "MaterialCompilerServicePipe") {}

MaterialCompilerService::~MaterialCompilerService() { Close(); }

ipc::MessagePipe::OnMessageResult MaterialCompilerService::OnMessage(
    std::unique_ptr<uint8_t[]> message, size_t size) {
  const uint8_t* message_buf = message.get();

  auto verifier = flatbuffers::Verifier(message_buf, size);
  if (!verifier.VerifyBuffer<schemas::Request>()) {
    IMP_LOG(imp::ERROR) << "Failed to validate request flatbuffer.";
    // We will keep the pipe alive, basically ignoring the invalid request.
    return ipc::MessagePipe::OnMessageResult::kKeepAlive;
  }

  const schemas::Request* request =
      flatbuffers::GetRoot<schemas::Request>(message_buf);
  // In theory this can't be null if it passes the verifier block above.
  if (request == nullptr) {
    IMP_LOG(imp::ERROR) << "Request is null.";
    return ipc::MessagePipe::OnMessageResult::kKeepAlive;
  }

  // Check the request type.
  absl::Status status;
  ipc::MessagePipe::OnMessageResult result =
      ipc::MessagePipe::OnMessageResult::kKeepAlive;
  switch (request->request_type()) {
    case schemas::RequestType::CompileRequest:
      status =
          HandleCompileRequest(request->operation_id(),
                               request->request_as<schemas::CompileRequest>());
      break;
    case schemas::RequestType::CloseRequest:
      // This will close the pipe on the correct thread, thus there no need to
      // explicitly call `Close()` here. Actually we shouldn't, as it will close
      // the pipe on the worker thread.
      result = ipc::MessagePipe::OnMessageResult::kInitiateClose;
      break;
    default:
      status = absl::InvalidArgumentError("Unexpected request type");
      break;
  }

  if (!status.ok()) {
    SendErrorResponse(request->operation_id(), status);
  }

  return result;
}

absl::Status MaterialCompilerService::HandleCompileRequest(
    uint64_t operation_id, const schemas::CompileRequest* request) {
  if (operation_id == 0) {
    return absl::InvalidArgumentError("Operation id must be greater than 0");
  }
  if (request == nullptr) {
    return absl::InvalidArgumentError("CompileRequest is null");
  }
  MP_ASSIGN_OR_RETURN(std::string compiled_shader,
                   CompileMaterial(request->source_material()->string_view(),
                                   request->platform(), request->target_api()));

  flatbuffers::FlatBufferBuilder builder;

  flatbuffers::Offset<schemas::CompileResponse> compile_response =
      schemas::CreateCompileResponse(
          builder, builder.CreateVector(
                       reinterpret_cast<const uint8_t*>(compiled_shader.data()),
                       compiled_shader.size()));
  flatbuffers::Offset<schemas::Response> response_offset =
      schemas::CreateResponse(builder, schemas::ResponseType::CompileResponse,
                              compile_response.Union(), operation_id);
  builder.Finish(response_offset);

  return SendResponse(builder);
}

absl::StatusOr<std::string> MaterialCompilerService::CompileMaterial(
    absl::string_view source_material_string, schemas::Platform platform,
    schemas::TargetApi target_api) {
  matp::MaterialParser parser;
  filamat::MaterialBuilder builder;
  std::ostringstream compiler_output;

  // TODO: Consider exposing config in the schema. We need the
  // client to be able to set some config including platform and target api.
  RuntimeMaterialCompilerConfig config(source_material_string, compiler_output);
  config.SetPlatform(UnpackPlatform(platform));
  config.SetTargetApi(UnpackTargetApi(target_api));

  matp::Config::Input* input = config.getInput();
  if (input == nullptr) {
    return absl::InvalidArgumentError("Input is null");
  }
  ssize_t size = input->open();
  if (size <= 0) {
    return absl::InvalidArgumentError("Input file is empty");
  }
  std::unique_ptr<const char[]> buffer = input->read();

  if (absl::Status template_sub_status = FilamentStatusToAbslStatus(
          parser.processTemplateSubstitutions(config, size, buffer));
      !template_sub_status.ok()) {
    return template_sub_status;
  }

  builder.init();
  if (absl::Status parse_status = FilamentStatusToAbslStatus(
          parser.parse(builder, config, size, buffer));
      !parse_status.ok()) {
    return parse_status;
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

void MaterialCompilerService::SendErrorResponse(uint64_t operation_id,
                                                absl::Status error_status) {
  flatbuffers::FlatBufferBuilder builder;
  flatbuffers::Offset<schemas::ErrorResponse> error_response =
      schemas::CreateErrorResponse(
          builder, PackStatusCode(error_status.code()),
          builder.CreateString(error_status.message()));
  flatbuffers::Offset<schemas::Response> response_offset =
      schemas::CreateResponse(builder, schemas::ResponseType::ErrorResponse,
                              error_response.Union(), operation_id);
  builder.Finish(response_offset);

  if (absl::Status status = SendResponse(builder); !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to send error response: " << status.ToString();
  }
}

void MaterialCompilerService::Close() {
  if (!pipe_.IsClosed()) {
    pipe_.Close();
  }
}

}  // namespace imp
