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

#include "core/loader/ipc/loader_service.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/globals.h"
#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "flatbuffers/base.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/platform_helpers.h"
#include "core/common/resource_helpers.h"
#include "core/ipc/message_pipe.h"
#include "core/loader/data/embedded_placeholder_textures.h"
#include "core/loader/ipc/schemas/loader_ipc_generated.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/provider.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::ipc {

namespace {
using OnMessageResult = MessagePipe::OnMessageResult;
}  // namespace

LoaderService::LoaderService(int fd)
    : pipe_(
          fd,
          [](std::unique_ptr<uint8_t[]> data, size_t size,
             void* user) -> OnMessageResult {
            auto* service = reinterpret_cast<LoaderService*>(user);
            return service->OnMessage(std::move(data), size);
          },
          [](void* user) {}, this, "ServicePipe") {
  RegisterPackagedResources(embedded_placeholder_textures_create());
  absl::SetGlobalVLogLevel(1);
}

LoaderService::~LoaderService() { Close(); }

void LoaderService::Close() {
  if (!pipe_.IsClosed()) {
    pipe_.Close();
  }
}

absl::StatusOr<OnMessageResult> LoaderService::OnRequest(
    const schemas::Request* request, std::unique_ptr<uint8_t[]> storage) {
  switch (request->request_type()) {
    case schemas::RequestTypes::Start:
      MP_RETURN_IF_ERROR(HandleStart(request->request_as<schemas::Start>(),
                                  std::move(storage)));
      return OnMessageResult::kKeepAlive;
    case schemas::RequestTypes::TryLoad:
      MP_RETURN_IF_ERROR(HandleTryLoad(request->request_as<schemas::TryLoad>(),
                                    std::move(storage)));
      return OnMessageResult::kKeepAlive;
    case schemas::RequestTypes::AddResource:
      MP_RETURN_IF_ERROR(HandleAddResource(
          request->request_as<schemas::AddResource>(), std::move(storage)));
      return OnMessageResult::kKeepAlive;
    case schemas::RequestTypes::GetLoadedModel:
      MP_RETURN_IF_ERROR(HandleGetLoadedModel(
          request->request_as<schemas::GetLoadedModel>(), std::move(storage)));
      return OnMessageResult::kKeepAlive;
    case schemas::RequestTypes::Done:
      MP_RETURN_IF_ERROR(
          HandleDone(request->request_as<schemas::Done>(), std::move(storage)));
      return OnMessageResult::kInitiateClose;
    default:
      return Error("Unexpected request");
  }
}

OnMessageResult LoaderService::OnMessage(std::unique_ptr<uint8_t[]> message,
                                         size_t size) {
  auto message_buf = reinterpret_cast<const uint8_t*>(message.get());

  auto verifier = flatbuffers::Verifier(message_buf, size);
  if (!verifier.VerifyBuffer<schemas::Request>()) {
    SendErrorResponse("Failed to validate request");
    provider_.reset();
    pipe_.Close();

    // Consider flatbuffer errors as unrecoverable.
    IMP_LOG(imp::FATAL) << "Failed to validate request";
    return OnMessageResult::kInitiateClose;
  }
  absl::StatusOr<OnMessageResult> result = OnRequest(
      flatbuffers::GetRoot<schemas::Request>(message_buf), std::move(message));
  if (!result.ok()) {
    IMP_LOG(imp::WARNING) << "Service error getting request: " << result.status();
    SendErrorResponse(result.status().message());
    return OnMessageResult::kInitiateClose;
  }

  return *result;
}

OptionalError LoaderService::HandleStart(const schemas::Start* request,
                                         std::unique_ptr<uint8_t[]> storage) {
  if (provider_) {
    return Error("Loading already started");
  }

  MP_ASSIGN_OR_RETURN(
      provider_,
      Provider::Create(
          absl::string_view(
              reinterpret_cast<const char*>(request->uri()->Data()),
              request->uri()->Length()),
          BufferAccess(std::move(storage), absl::Span<const uint8_t>(
                                               reinterpret_cast<const uint8_t*>(
                                                   request->data()->Data()),
                                               request->data()->Length())),
          LoaderOptions{
              .use_lite_materials = request->use_lite_materials(),
              .exclude_excess_nodes = request->exclude_excess_nodes(),
              .remove_shadow_planes = request->remove_shadow_planes()}));

  flatbuffers::FlatBufferBuilder builder;
  auto offset = schemas::CreateStartResponse(builder);
  auto response_offset = schemas::CreateResponse(
      builder, schemas::ResponseTypes::StartResponse, offset.Union());
  builder.Finish(response_offset);

  SendMessage(builder);
  IMP_LOG(imp::INFO) << "LoaderService processing started...";

  return NoError();
}

OptionalError LoaderService::HandleTryLoad(const schemas::TryLoad* request,
                                           std::unique_ptr<uint8_t[]> storage) {
  if (!provider_) {
    return Error("Loading not started");
  }

  std::vector<std::string> missing_resource_paths;
  bool loaded;
  auto error = provider_->TryLoad(&missing_resource_paths, &loaded);
  if (missing_resource_paths.empty() && !error.ok()) {
    return error;
  }

  flatbuffers::FlatBufferBuilder builder;
  auto offset = schemas::CreateTryLoadResponse(
      builder, builder.CreateVectorOfStrings(missing_resource_paths), loaded);
  auto response_offset = schemas::CreateResponse(
      builder, schemas::ResponseTypes::TryLoadResponse, offset.Union());
  builder.Finish(response_offset);

  SendMessage(builder);
  return NoError();
}

OptionalError LoaderService::HandleAddResource(
    const schemas::AddResource* request, std::unique_ptr<uint8_t[]> storage) {
  if (!provider_) {
    return Error("Loading not started");
  }

  absl::string_view path =
      absl::string_view(reinterpret_cast<const char*>(request->path()->Data()),
                        request->path()->Length());
  BufferAccess access(
      std::move(storage),
      absl::Span<const uint8_t>(
          reinterpret_cast<const uint8_t*>(request->data()->Data()),
          request->data()->Length()));

  if (request->was_missing()) {
    MP_RETURN_IF_ERROR(provider_->AddMissingResource(path, std::move(access)));
  } else {
    provider_->AddResource(path, std::move(access));
  }

  flatbuffers::FlatBufferBuilder builder;
  auto offset = schemas::CreateAddResourceResponse(builder);
  auto response_offset = schemas::CreateResponse(
      builder, schemas::ResponseTypes::AddResourceResponse, offset.Union());
  builder.Finish(response_offset);

  SendMessage(builder);
  return NoError();
}

OptionalError LoaderService::HandleGetLoadedModel(
    const schemas::GetLoadedModel* request,
    std::unique_ptr<uint8_t[]> storage) {
  if (!provider_) {
    return Error("Loading not started");
  }

  FlatBufferAccess<imp::schemas::LoadedModel> loaded_model;
  MP_RETURN_IF_ERROR(provider_->GetLoadedModel(&loaded_model));
  flatbuffers::FlatBufferBuilder fbb;
  auto typed_response = schemas::CreateGetLoadedModelResponse(
      fbb, fbb.CreateVector<uint8_t>(loaded_model.Buffer().Data(),
                                     loaded_model.Buffer().Size()));

  auto response = schemas::CreateResponse(
      fbb, schemas::ResponseTypes::GetLoadedModelResponse,
      typed_response.Union());
  fbb.Finish(response);

  SendMessage(fbb);
  return NoError();
}

OptionalError LoaderService::HandleDone(const schemas::Done* request,
                                        std::unique_ptr<uint8_t[]> storage) {
  if (!provider_) {
    return Error("Loading not started");
  }

  flatbuffers::FlatBufferBuilder fbb;
  auto typed_response = schemas::CreateDoneResponse(fbb);
  auto response = schemas::CreateResponse(
      fbb, schemas::ResponseTypes::DoneResponse, typed_response.Union());
  fbb.Finish(response);

  SendMessage(fbb);
  return NoError();
}

void LoaderService::SendErrorResponse(absl::string_view message) {
  flatbuffers::FlatBufferBuilder builder;
  auto error_offset = schemas::CreateErrorResponse(
      builder, builder.CreateString(message.data(), message.size()));
  auto response_offset = schemas::CreateResponse(
      builder, schemas::ResponseTypes::ErrorResponse, error_offset.Union());
  builder.Finish(response_offset);

  SendMessage(builder);
}

void LoaderService::SendMessage(const flatbuffers::FlatBufferBuilder& builder) {
  if (builder.GetSize() > std::numeric_limits<uint32_t>::max()) {
    IMP_LOG(imp::FATAL) << "Response too large";
  }
  pipe_.Send(reinterpret_cast<const uint8_t*>(builder.GetBufferPointer()),
             static_cast<uint32_t>(builder.GetSize()));
}

}  // namespace imp::loader::ipc
